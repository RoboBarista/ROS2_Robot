#-*- coding: UTF-8 -*-
import sys
import asyncio
import websockets
import json
import time
import socketio
import playsound as ps
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import fair_drip.control_v2 as rc
from sock import sio_flask, sio_vision

# ----- 전역 변수: 비전에서 받은 좌표 데이터를 담을 리스트 -----
dripper_data = []
cup_data = []

# ----- 서버 주소(주문 서버/비전 서버 등) -----
FLASK_SERVER_URI = 'ws://192.168.58.27:5555'
VISION_SERVER_URI = 'ws://192.168.58.24:9999'

# ----- 전역 플래그: 비전 데이터가 수신되었는지 여부 확인 -----
vision_ok = 0


# ----- TTS(음성 재생) 매니저 클래스 -----
class TTSManager:
    def __init__(self):
        self.lock = threading.Lock()

    def speak(self, file):
        with self.lock:
            ps.playsound(file)


# ----- 리스너 노드 정의 -----
class Listener(Node):
    step = 0
    
    def __init__(self):
        super().__init__('listener')
        
        self.publisher = self.create_publisher(String, '/robot', 10)

        # ROS2 구독 설정
        self.sub_check = self.create_subscription(
            String, 'pos', self.listener_callback, 10
        )
        self.sub_check
        
        self.subscription_order = self.create_subscription(
            String, 'order', self.listener_callback, 10
        )
        self.subscription_vision = self.create_subscription(
            String, 'vision', self.listener_vision_callback, 10
        )
        self.subscription_order
        self.subscription_vision

        global vision_ok
        global dripper_data
        global cup_data

        self.lock = asyncio.Lock()
        self.websocket_uri = "ws://192.168.58.27:9090"
        self.tts_manager = TTSManager()

        try:
            self.loop = asyncio.get_event_loop()
        except RuntimeError:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

        # 웹소켓 연결
        self.loop.create_task(self.connect_websocket())

    async def connect_websocket(self):
        self.websocket = await websockets.connect(self.websocket_uri)
        if self.websocket.open:
            print("Websocket Connect Successfully")
            await sio_flask.connect(FLASK_SERVER_URI)
            await sio_vision.connect(VISION_SERVER_URI)
            await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_init.mp3")

    async def speaking(self, file):
        asyncio.create_task(self._play_sound(file))

    async def _play_sound(self, file):
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.tts_manager.speak, file)

    async def listener_vision_callback(self, msg):
        print("CALLBACK : GOT VISION TOPIC")
        data = msg['data'].strip()
        data = data.replace("'", '"')
        data = json.loads(data)
        self.get_logger().info(f"Received raw data: {data}")

        # ----- dripper 데이터 처리 -----
        dripper_data.clear()
        for dripper in data['dripper']:
            single_dripper_data = {}
            single_dripper_data['order'] = dripper.get('order')
            single_dripper_data['coordinate'] = dripper.get('coordinate') or []
            single_dripper_data['center'] = dripper.get('center') or []
            single_dripper_data['exist_dripper'] = dripper.get('exist_dripper')
            single_dripper_data['exist_coffee_beans'] = dripper.get('exist_coffee_beans')
            dripper_data.append(single_dripper_data)

        # ----- cup 데이터 처리 -----
        cup_data.clear()
        for cup in data['cup']:
            single_cup_data = {}
            single_cup_data['order'] = cup.get('order')
            single_cup_data['coordinate'] = cup.get('coordinate') or []
            single_cup_data['center'] = cup.get('center') or []
            single_cup_data['exist_cup'] = cup.get('exist_cup')
            cup_data.append(single_cup_data)

        self.get_logger().info(f"Processed dripper data: {dripper_data}")
        self.get_logger().info(f"Processed cup data: {cup_data}")

    async def listener_callback(self, msg):
        async with self.lock:
            data = msg['data'].strip()
            data = data.replace("'", '"')
            data = json.loads(data)
            print(type(data))

            msg_Coffee = data['recipe'].get('coffee')
            msg_Type = data['recipe'].get('drip_type')
            msg_Temp = data['recipe'].get('water_temp')
            msg_WTotal = data['recipe'].get('water_total')
            msg_WM = data['recipe'].get('water_method')
            msg_TM = data['recipe'].get('time_method')

            recipe_w = msg_WM.split(', ') if msg_WM else []
            recipe_t = msg_TM.split(', ') if msg_TM else []
            self.get_logger().info(f"Received raw data: {data}")

            if None in [msg_Coffee]:
                print("Failed to get Coffee Data")
            else:
                await self.recipe_dripper(msg_Coffee, msg_Type, msg_Temp, msg_WTotal, msg_WM, len(recipe_w))

    async def check_drip_point(self, dripper_data):
        print("CHECK_DRIP_POINT")
        valid_drippers = [
            dripper['order'] for dripper in dripper_data
            if dripper.get('exist_dripper') or dripper.get('exist_coffee_beans')
        ]
        print("---------------------------------------------------------------")
        print(valid_drippers)
        return min(valid_drippers) if valid_drippers else None

    async def check_cup_point(self, cup_data):
        valid_cups = [
            cup['order'] for cup in cup_data
            if cup.get('exist_cup')
        ]
        if valid_cups is None:
            print("NO valid Cup")
        print(f'valid_cup : {min(valid_cups)}')
        return min(valid_cups) if valid_cups else None

    async def recipe_dripper(self, coffee, type, temp, wtotal, wm, wmc):
        current_step = 0
        self.progress_info(current_step)

        # 비전 데이터 요청
        asyncio.run_coroutine_threadsafe(self.request_vision_current_data(), self.loop)

        i = 0
        while vision_ok != 1:
            if i < 10:
                print('.', end='', flush=True)
                await asyncio.sleep(1)
                i += 1
            else:
                i = 0
        print()

        drip_point, cup_point = await asyncio.gather(
            self.check_drip_point(dripper_data),
            self.check_cup_point(cup_data)
        )

        if vision_ok:
            current_step = 4
            self.progress_info(current_step)

            if (drip_point and cup_point) is None:
                print("Drip point and Cup Point is not chosen.\nPlease Check one more time")
                await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_2.mp3")
                time.sleep(1)
                return await self.recipe_dripper(coffee, type, temp, wtotal, wm, wmc)
            else:
                if coffee == 1:  # 전주연 레시피
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_1.mp3")
                    print(f'Drip Point: {dripper_data}\nCup Point: {cup_data}')

                    # 전주연 레시피 설명
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_jjy1.mp3")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_jjy2.mp3")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_jjy3.mp3")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_jjy4.mp3")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_jjy5.mp3")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_jjy6.mp3")
                    await rc.set_home()

                    # 원두컵 집고 드립퍼에 붓기
                    await self.coffee_drop(drip_point, cup_point)
                    current_step = 10
                    self.progress_info(current_step)

                    # 주전자 집기
                    await rc.kettle_pick()
                    current_step = 25
                    self.progress_info(current_step)

                    # 물 붓기
                    await rc.pouring_water()
                    current_step = 45
                    self.progress_info(current_step)

                    # 레시피에 따른 드립
                    await rc.pouring_water_home()
                    await rc.spiral_dripper(drip_point, coffee)
                    current_step = 65
                    self.progress_info(current_step)

                    # 주전자 원위치
                    await rc.kettle_back()
                    current_step = 75
                    self.progress_info(current_step)

                    current_step = 90
                    self.progress_info(current_step)

                    # 로봇 Home
                    await rc.set_home()
                    current_step = 95
                    self.progress_info(current_step)
                    print("finish")

                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_end.mp3")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_vote.mp3")
                    current_step = 100
                    self.progress_info(current_step)

                elif coffee == 2:  # 테츠 카츠야 레시피
                    print("테츠 카츠야 레시피")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output1.mp3")
                    print(f'Drip Point: {dripper_data}\nCup Point: {cup_data}')

                    # 테츠 카츠야 레시피 설명
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_tsu1.mp3")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_tsu2.mp3")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_tsu3.mp3")
                    await rc.set_home()

                    # 원두컵 집고 드립퍼에 붓기
                    await self.coffee_drop(drip_point, cup_point)
                    current_step = 10
                    self.progress_info(current_step)

                    # 주전자 집기
                    await rc.kettle_pick()
                    current_step = 25
                    self.progress_info(current_step)

                    # 물 붓기
                    await rc.pouring_water()
                    current_step = 45
                    self.progress_info(current_step)

                    # 레시피에 따른 드립
                    await rc.pouring_water_home()
                    await rc.spiral_dripper(drip_point, coffee)
                    current_step = 65
                    self.progress_info(current_step)

                    # 주전자 원위치
                    await rc.kettle_back()
                    current_step = 75
                    self.progress_info(current_step)

                    current_step = 90
                    self.progress_info(current_step)

                    # 로봇 Home
                    await rc.set_home()
                    current_step = 95
                    self.progress_info(current_step)
                    print("finish")

                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_end.mp3")
                    await self.speaking("/root/ros2_ws/src/Ajou_Drip_Project/ROS2_Foxy/fair_drip/voice/output_vote.mp3")
                    current_step = 100
                    self.progress_info(current_step)

    def progress_info(self, step):
        # 진행률 전송
        asyncio.run_coroutine_threadsafe(self.send_websocket_progress(step), self.loop)

    # ----- Socket.IO 이벤트 핸들러 -----
    @sio_flask.event
    async def connect():
        print("Connected to Flask server")

    @sio_vision.event
    async def connect():
        print("Connected to Vision server")

    @sio_flask.event
    async def disconnect():
        print("Disconnected from Flask server")

    @sio_vision.event
    async def disconnect():
        print("Disconnected from Vision server")

    # ----- Vision 서버로부터 받은 신호 -----
    @sio_vision.on('vision_get')
    async def on_vision_get():
        global vision_ok
        vision_ok = 1
        print("Received vision_get message")

    # ----- 진행률 WebSocket 전송 함수 -----
    async def send_websocket_progress(self, progress):
        try:
            message = {
                "op": "progress_update",
                "progress": progress
            }
            await sio_flask.emit('progress_update', message)
            print(f'Progress sent to Flask Server: {message}')
        except Exception as e:
            print(f"Failed to send progress to Flask Server: {str(e)}")

    async def request_vision_current_data(self):
        try:
            await sio_vision.emit('data plz')
            print('Requested vision data to vision server')
        except Exception as e:
            print(f"Failed to request to Vision server : {str(e)}")

    async def coffee_drop(self, drop_point, cup_point):
        await rc.beancup_pick(cup_point)
        await rc.beancup_dropbean_ready()
        await rc.beancup_dropbean(drop_point)
        await rc.beancup_back(cup_point)
        await rc.new_preparing_pick_dripper()
        await rc.shaking_dripper(drop_point)
        await rc.new_preparing_pick_dripper()


# ----- 주문(ROS 토픽) 수신 비동기 함수 -----
async def listen_order(listener_node):
    uri = "ws://192.168.58.27:9090"
    async with websockets.connect(uri) as websocket:
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/order",
            "type": "std_msgs/String"
        }
        await websocket.send(json.dumps(subscribe_msg))
        while True:
            message = await websocket.recv()
            data = json.loads(message)
            print(data)
            if 'msg' in data:
                await listener_node.listener_callback(data['msg'])


# ----- 비전(ROS 토픽) 수신 비동기 함수 -----
async def listen_vision(listener_node):
    uri = "ws://192.168.58.27:9090"
    async with websockets.connect(uri) as websocket:
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/vision",
            "type": "std_msgs/String"
        }
        await websocket.send(json.dumps(subscribe_msg))
        while True:
            message = await websocket.recv()
            data = json.loads(message)
            print(data)
            if 'msg' in data:
                await listener_node.listener_vision_callback(data['msg'])


# ----- 메인 함수 -----
def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    asyncio.get_event_loop().run_until_complete(
        asyncio.gather(listen_order(listener), listen_vision(listener))
    )
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
