import time
import asyncio
from sock import sio_vision
import conf_file as cf
import frrpc as frrpc

# 로봇 정보
ROBOT_IP = "192.168.58.3"
robot = frrpc.RPC(ROBOT_IP)

EP = [0.000, 0.000, 0.000, 0.000]
DP = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]
EP2 = [0.000, 0.000, 0.000, 0.000]


# --- 공통 유틸 함수들 ---
async def movecheck():
    """로봇 모션 완료 대기."""
    while True:
        motion_state = robot.GetRobotMotionDone()
        if motion_state[1] == 1:
            break
        await asyncio.sleep(0.1)


async def SetSpeed(speed):
    robot.SetSpeed(speed)


async def Gripper_open():
    robot.MoveGripper(1, 80, 50, 50, 10000, 0)


async def Gripper_close():
    robot.MoveGripper(1, 25, 50, 50, 10000, 0)


def get_current_joint_positions():
    """현재 조인트 위치 확인."""
    joint_positions = robot.GetJointState()
    print(f"현재 조인트 위치: {joint_positions}")
    return joint_positions


def transform_pose(joint):
    """Forward Kinematics: Joint -> Cartesian Pose."""
    pose = robot.GetForwardKin(joint)
    cartesian_pose = [pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]]
    return cartesian_pose


def transform_joint(pose):
    """Inverse Kinematics: Cartesian Pose -> Joint."""
    joint = robot.GetInverseKin(0, pose, -1)
    joint_pose = [joint[1], joint[2], joint[3], joint[4], joint[5], joint[6]]
    return joint_pose


async def PTP(J=None, SPEED=70.0, BLEND=-1, P=None, retry=2):
    """
    MoveJ 방식으로 로봇을 이동.
    J(조인트)나 P(포즈) 중 하나만 주어져도 작동 가능.
    """
    try:
        SPEED = float(SPEED)
        BLEND = float(BLEND)
        if P is None:
            P = transform_pose(J)
        if J is None:
            J = transform_joint(P)
        robot.MoveJ(J, P, 0, 0, SPEED, 100.0, BLEND, EP, -1.0, 0, DP)
    except:
        if retry > 0:
            await PTP(J, 70.0, BLEND, P, retry - 1)
            print("execpt : retry PTP")


async def newSPIRAL(J1, SPEED, Pa, DP):
    """
    NewSpiral 함수: 지정된 자리(J1)에서 나선형 패턴을 수행.
    """
    P1 = transform_pose(J1)
    robot.NewSpiral(J1, P1, 0, 0, float(SPEED), 0.0, EP2, 50.0, 2, DP, Pa)

    while True:
        motion_state = robot.GetRobotMotionDone()
        if motion_state[1] == 1:
            break
        await asyncio.sleep(0.1)


async def movegripper(index, pos, vel, force, max_time, last_arg=0):
    """
    그리퍼 제어: 모션 완료까지 대기.
    """
    robot.MoveGripper(index, pos, vel, force, max_time, last_arg)
    start_time = time.time()
    while time.time() - start_time < 1.0:
        gripper_state = robot.GetGripperMotionDone()
        if gripper_state[2] == 1:
            break
        await asyncio.sleep(0.1)


# --- 로봇 동작 시퀀스 함수들 ---
async def set_home():
    print("home위치로 이동합니다.")
    robot.MoveGripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.home_point["J"], 70, -1)
    while True:
        motion_state = robot.GetRobotMotionDone()
        if motion_state[1] == 1:
            break
        await asyncio.sleep(0.1)


async def kettle_pick():
    print("물을 채운 주전자를 집어듭니다.")
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.new_pick_fullwater_kettle["J91"], 100, -1)
    await PTP(cf.new_pick_fullwater_kettle["J92"], 100, -1)
    await PTP(cf.new_pick_fullwater_kettle["J93"], 100, -1)
    await PTP(cf.new_pick_fullwater_kettle["J94"], 50, -1)
    await PTP(cf.new_pick_fullwater_kettle["J95"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J96"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J97"], 30, -1)
    await movegripper(1, 15, 50, 50, 10000, 0)
    await PTP(cf.new_pick_fullwater_kettle["J98"], 30, -1)


async def kettle_back():
    await PTP(cf.pouring_water["J2"], 30, -1)
    await PTP(cf.pouring_water["J1"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J99"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J98"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J97"], 30, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.new_pick_fullwater_kettle["J96"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J95"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J94"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J93"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J92"], 30, -1)
    await PTP(cf.new_pick_fullwater_kettle["J91"], 30, -1)


async def pouring_water():
    await PTP(cf.pouring_water["J1"], 30, -1)
    await PTP(cf.pouring_water["J2"], 20, -1)


async def pouring_water_home():
    speed = 100
    robot.SetSpeed(speed)
    await PTP(cf.pouring_water["J2"], 20, -1)  # 물붓기 home


# --- 예시 스파이럴/드립 동작(1,2,3번 위치) ---
async def spiral_dripper1():
    await PTP(cf.spiral_dripper1["J1"], 30, -1)
    await PTP(cf.spiral_dripper1["J2"], 30, -1)


async def spiral_dripper2():
    await PTP(cf.spiral_dripper2["J1"], 30, -1)
    await PTP(cf.spiral_dripper2["J2"], 30, -1)


async def spiral_dripper3():
    await PTP(cf.spiral_dripper3["J1"], 30, -1)
    await PTP(cf.spiral_dripper3["J2"], 30, -1)


# --- 원두컵 집고/되돌리기 ---
async def beancup_pick1():
    speed = 80
    robot.SetSpeed(speed)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.pick_bean_cup1['J1'], 100, -1)
    await PTP(cf.pick_bean_cup1['J2'], 100, -1)
    await PTP(cf.pick_bean_cup1['J3'], 100, -1)
    await movegripper(1, 0, 50, 50, 10000, 0)
    await PTP(cf.pick_bean_cup1['J1'], 100, -1)
    await PTP(cf.pick_bean_cup1['J4'], 100, -1)
    speed = 100
    robot.SetSpeed(speed)


async def beancup_pick2():
    speed = 80
    robot.SetSpeed(speed)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.pick_bean_cup2['J1'], 100, -1)
    await PTP(cf.pick_bean_cup2['J2'], 100, -1)
    await PTP(cf.pick_bean_cup2['J3'], 100, -1)
    await movegripper(1, 5, 50, 50, 10000, 0)
    await PTP(cf.pick_bean_cup2['J1'], 100, -1)
    await PTP(cf.pick_bean_cup2['J4'], 100, -1)
    speed = 100
    robot.SetSpeed(speed)


async def beancup_pick3():
    speed = 80
    robot.SetSpeed(speed)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.pick_bean_cup3['J1'], 100, -1)
    await PTP(cf.pick_bean_cup3['J2'], 100, -1)
    await PTP(cf.pick_bean_cup3['J3'], 100, -1)
    await movegripper(1, 10, 50, 50, 10000, 0)
    await PTP(cf.pick_bean_cup3['J1'], 100, -1)
    await PTP(cf.pick_bean_cup3['J4'], 100, -1)
    speed = 100
    robot.SetSpeed(speed)


async def beancup_back1():
    await PTP(cf.pick_bean_cup1['J4'], 100, -1)
    await PTP(cf.pick_bean_cup1['J1'], 100, -1)
    await PTP(cf.pick_bean_cup1['J2'], 100, -1)
    await PTP(cf.pick_bean_cup1['J3'], 100, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.pick_bean_cup1['J1'], 100, -1)
    await PTP(cf.pick_bean_cup1['J4'], 100, -1)


async def beancup_back2():
    await PTP(cf.pick_bean_cup2['J4'], 100, -1)
    await PTP(cf.pick_bean_cup2['J1'], 100, -1)
    await PTP(cf.pick_bean_cup2['J2'], 100, -1)
    await PTP(cf.pick_bean_cup2['J3'], 100, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.pick_bean_cup2['J1'], 100, -1)
    await PTP(cf.pick_bean_cup2['J4'], 100, -1)


async def beancup_back3():
    await PTP(cf.pick_bean_cup3['J4'], 100, -1)
    await PTP(cf.pick_bean_cup3['J1'], 100, -1)
    await PTP(cf.pick_bean_cup3['J2'], 100, -1)
    await PTP(cf.pick_bean_cup3['J3'], 100, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.pick_bean_cup3['J1'], 100, -1)
    await PTP(cf.pick_bean_cup3['J4'], 100, -1)


# --- 그라인딩(예: 미사용) ---
async def beancup_grinding_bean_in():
    await PTP(cf.grinding_coffee_bean['J1'], 100, -1)


async def beancup_grinding_bean_out():
    await PTP(cf.grinding_coffee_bean['J2'], 100, -1)


# --- 드리퍼 쪽으로 원두 이동 ---
async def beancup_dropbean_ready():
    await PTP(cf.moving_coffee_bean['J1'], 100, -1)


async def beancup_dropbean1():
    await PTP(cf.set_coffee_bean_dripper1['J1'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J2'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J4'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J4'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J4'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J2'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper1['J1'], 100, -1)


async def beancup_dropbean2():
    await PTP(cf.set_coffee_bean_dripper2['J1'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J2'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J4'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J4'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J4'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J2'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper2['J1'], 100, -1)


async def beancup_dropbean3():
    await PTP(cf.set_coffee_bean_dripper3['J1'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J2'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J4'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J4'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J4'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J3'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J2'], 100, -1)
    await PTP(cf.set_coffee_bean_dripper3['J1'], 100, -1)


async def beancup_dropbean_end():
    await PTP(cf.moving_coffee_bean['J3'], 60, -1)
    await PTP(cf.moving_coffee_bean['J2'], 60, -1)
    await PTP(cf.moving_coffee_bean['J1'], 60, -1)


# --- 컵 옮기기(예: pick_the_cup 등) ---
async def pick_the_cup():
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.home_point["J"], 100, -1)
    await PTP(cf.new_pick_the_cup["J1"], 100, -1)
    await PTP(cf.new_pick_the_cup["J2"], 100, -1)
    await PTP(cf.new_pick_the_cup["J3"], 100, -1)
    await PTP(cf.new_pick_the_cup["J4"], 100, -1)
    await PTP(cf.new_pick_the_cup["J5"], 100, -1)
    await movegripper(1, 80, 50, 50, 10000, 0)
    await movegripper(1, 30, 50, 50, 10000, 0)
    await PTP(cf.new_pick_the_cup["J6"], 100, -1)
    await PTP(cf.new_pick_the_cup["J7"], 100, -1)
    await PTP(cf.new_pick_the_cup["J8"], 100, -1)
    await PTP(cf.new_pick_the_cup["J9"], 100, -1)


# --- 컵 배치 (예: new_set_cup1, 2, 3) ---
async def new_set_cup1():
    print("컵을 첫 번째 자리에 위치합니다.")
    await PTP(cf.new_set_cup1["J1"], 100, -1)
    await PTP(cf.new_set_cup1["J2"], 30, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.new_set_cup1["J3"], 30, -1)
    await PTP(cf.new_set_cup1["J4"], 70, -1)
    await PTP(cf.home_point["J"], 100, -1)


async def new_set_cup2():
    print("컵을 두 번째 자리에 위치합니다.")
    await PTP(cf.new_set_cup2["J1"], 100, -1)
    await PTP(cf.new_set_cup2["J2"], 30, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.new_set_cup2["J3"], 30, -1)
    await PTP(cf.new_set_cup2["J4"], 70, -1)
    await PTP(cf.home_point["J"], 100, -1)


async def new_set_cup3():
    print("컵을 세 번째 자리에 위치합니다.")
    await PTP(cf.new_set_cup3["J1"], 100, -1)
    await PTP(cf.new_set_cup3["J2"], 70, -1)
    await PTP(cf.new_set_cup3["J3"], 30, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.new_set_cup3["J4"], 30, -1)
    await PTP(cf.new_set_cup3["J5"], 70, -1)
    await PTP(cf.new_set_cup3["J6"], 100, -1)
    await PTP(cf.home_point["J"], 100, -1)


# --- 드리퍼 집기/놓기 ---
async def new_pick_dripper1():
    print("첫 번째 드리퍼를 집습니다.")
    await PTP(cf.new_pick_dripper1["J1"], 100, -1)
    await movegripper(1, 80, 50, 50, 10000, 0)
    await PTP(cf.new_pick_dripper1["J2"], 30, -1)
    await movegripper(1, 15, 50, 50, 10000, 0)
    await PTP(cf.new_pick_dripper1["J3"], 30, -1)
    await PTP(cf.new_pick_dripper1["J4"], 70, -1)
    await PTP(cf.new_pick_dripper1["J5"], 70, -1)


# (다른 드리퍼 pick 함수들 동일 구조라 생략 가능, 여기선 그대로 둠)
async def new_pick_dripper2():
    print("두 번째 드리퍼를 집습니다.")
    await PTP(cf.new_pick_dripper2["J1"], 100, -1)
    await movegripper(1, 80, 50, 50, 10000, 0)
    await PTP(cf.new_pick_dripper2["J2"], 30, -1)
    await movegripper(1, 15, 50, 50, 10000, 0)
    await PTP(cf.new_pick_dripper2["J3"], 30, -1)
    await PTP(cf.new_pick_dripper2["J4"], 70, -1)


async def new_pick_dripper3():
    print("세 번째 드리퍼를 집습니다.")
    await PTP(cf.new_pick_dripper3["J1"], 100, -1)
    await movegripper(1, 80, 50, 50, 10000, 0)
    await PTP(cf.new_pick_dripper3["J2"], 30, -1)
    await movegripper(1, 15, 50, 50, 10000, 0)
    await PTP(cf.new_pick_dripper3["J3"], 30, -1)
    await PTP(cf.new_pick_dripper3["J4"], 70, -1)
    await PTP(cf.new_pick_dripper3["J5"], 70, -1)


# ... (이하 new_pick_dripper4 ~ new_pick_dripper9 생략 가능 / 동일 구조이므로 유지)


async def new_ready_for_set_1st_floor_dripper():
    print("1층 드리퍼 준비 완료.")
    await PTP(cf.new_ready_for_set_1st_floor_dripper["J1"], 100, -1)
    await PTP(cf.new_ready_for_set_1st_floor_dripper["J2"], 100, -1)
    await PTP(cf.new_ready_for_set_1st_floor_dripper["J3"], 100, -1)
    await PTP(cf.new_ready_for_set_1st_floor_dripper["J4"], 100, -1)


async def new_ready_for_set_234_floor_dripper():
    print("2-4층 드리퍼 준비 완료.")
    await PTP(cf.new_ready_for_set_234_floor_dripper["J1"], 100, -1)
    await PTP(cf.new_ready_for_set_234_floor_dripper["J2"], 100, -1)
    await PTP(cf.new_ready_for_set_234_floor_dripper["J3"], 100, -1)


# ... (back_pick_dripperX 계열 함수 동일 구조라 필요하다면 그대로 유지)


async def new_set_dripper_1st_pos():
    print("첫 번째 위치에 드리퍼를 놓습니다.")
    await PTP(cf.new_set_dripper_1st_pos["J1"], 100, -1)
    await PTP(cf.new_set_dripper_1st_pos["J2"], 70, -1)
    await PTP(cf.new_set_dripper_1st_pos["J3"], 30, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.new_set_dripper_1st_pos["J4"], 30, -1)


# ... (new_set_dripper_2nd_pos, 3rd_pos도 동일 구조)


async def new_preparing_pick_dripper():
    print("드리퍼를 집기 위한 준비를 합니다.")
    await PTP(cf.new_preparing_pick_dripper["J1"], 100, -1)


# --- 드리퍼 원두 평탄화(흔들기) ---
async def shaking_dripper1():
    print("1번 드리퍼의 원두를 평탄화합니다.")
    await PTP(cf.new_shaking_dripper1["J1"], 50, -1)
    await PTP(cf.new_shaking_dripper1["J2"], 50, -1)
    await movegripper(1, 0, 50, 50, 10000, 0)
    await PTP(cf.new_shaking_dripper1["J3"], 100, -1)
    await PTP(cf.new_shaking_dripper1["J6"], 100, 500)
    await PTP(cf.new_shaking_dripper1["J7"], 100, 500)
    await PTP(cf.new_shaking_dripper1["J6"], 100, 500)
    await PTP(cf.new_shaking_dripper1["J7"], 100, 500)
    await PTP(cf.new_shaking_dripper1["J3"], 100, 500)
    await PTP(cf.new_shaking_dripper1["J2"], 100, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.new_shaking_dripper1["J1"], 50, -1)


async def shaking_dripper2():
    print("2번 드리퍼의 원두를 평탄화합니다.")
    await PTP(cf.new_shaking_dripper2["J1"], 50, -1)
    await PTP(cf.new_shaking_dripper2["J2"], 50, -1)
    await movegripper(1, 15, 50, 50, 10000, 0)
    await PTP(cf.new_shaking_dripper2["J3"], 100, -1)
    await PTP(cf.new_shaking_dripper2["J4"], 100, 500)
    await PTP(cf.new_shaking_dripper2["J5"], 100, 500)
    await PTP(cf.new_shaking_dripper2["J4"], 100, 500)
    await PTP(cf.new_shaking_dripper2["J5"], 100, 500)
    await PTP(cf.new_shaking_dripper2["J3"], 100, -1)
    await PTP(cf.new_shaking_dripper2["J2"], 100, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.new_shaking_dripper2["J1"], 50, -1)


async def shaking_dripper3():
    print("3번 드리퍼의 원두를 평탄화합니다.")
    await PTP(cf.new_shaking_dripper3["J1"], 50, -1)
    await PTP(cf.new_shaking_dripper3["J2"], 50, -1)
    await movegripper(1, 15, 50, 50, 10000, 0)
    await PTP(cf.new_shaking_dripper3["J3"], 100, -1)
    await PTP(cf.new_shaking_dripper3["J4"], 100, 500)
    await PTP(cf.new_shaking_dripper3["J5"], 100, 500)
    await PTP(cf.new_shaking_dripper3["J4"], 100, 500)
    await PTP(cf.new_shaking_dripper3["J5"], 100, 500)
    await PTP(cf.new_shaking_dripper3["J3"], 100, -1)
    await PTP(cf.new_shaking_dripper3["J2"], 100, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.new_shaking_dripper3["J1"], 50, -1)


# --- 실제 물붓기(Spiral) ---
async def standard_spiral1(SPEED, ang, ang_sub1, ang_sub2, ang_sub3, ang_sub4, ang_sub5, pa0, ext):
    """
    1번 드리퍼에 나선형 물붓기.
    """
    repos = None
    print("1번 드리퍼에 Spiral")
    DP1 = [0.000, 0.000, 0.000, 0.0, 0.000, 0.000]
    Pa1 = [pa0, 0.0, 0.0, 30.0, 0.0, 0.0]

    await PTP(cf.pouring_water_dripper1["J1"], 100, -1)
    repos = cf.pouring_water_dripper1["J2"]
    repos[5] = ang
    repos[4] = ang_sub1
    repos[3] = ang_sub2
    repos[2] = ang_sub3
    repos[1] = ang_sub4
    repos[0] = ang_sub5
    print("Spiral 좌표 : ", repos, "pa0 : ", pa0, "speed : ", SPEED)
    await PTP(repos, 20, -1)
    print("Spiral... ")
    await newSPIRAL(repos, SPEED, Pa1, DP1)

    await PTP(cf.pouring_water_dripper1["J1"], 100, -1)
    # Vision 서버에 메시지 (예시)
    try:
        print("Sending message to Vision server before sleep.")
        await sio_vision.emit('success plz')
    except Exception as e:
        print(f"Failed to send message to Vision server: {e}")

    await asyncio.sleep(ext)


async def spiral1(recipe_num):
    """
    1번 드리퍼에 대해 레시피별 물붓기 시퀀스.
    """
    for i in range(1, 6):
        if recipe_num == 1:
            pa0 = [0.0, 1.8, 1.6, 2.0, 1.6, 2.0]
            spd = [0, 80, 90, 80, 90, 80]
            ext = [0, 45, 25, 25, 18, 1]
        elif recipe_num == 2:
            pa0 = [0.0, 1.8, 1.8, 1.8, 1.8, 1.8]
            spd = [0, 100, 100, 80, 90, 70]
            ext = [0, 45, 45, 45, 45, 1]

        ang = list(cf.pouring_water_dripper1.values())[i][5]
        ang_sub1 = list(cf.pouring_water_dripper1.values())[i][4]
        ang_sub2 = list(cf.pouring_water_dripper1.values())[i][3]
        ang_sub3 = list(cf.pouring_water_dripper1.values())[i][2]
        ang_sub4 = list(cf.pouring_water_dripper1.values())[i][1]
        ang_sub5 = list(cf.pouring_water_dripper1.values())[i][0]

        await standard_spiral1(spd[i], ang, ang_sub1, ang_sub2, ang_sub3, ang_sub4, ang_sub5, pa0[i], ext[i])


async def standard_spiral2(SPEED, ang, ang_sub1, ang_sub2, ang_sub3, ang_sub4, ang_sub5, pa0, ext):
    """
    2번 드리퍼에 나선형 물붓기.
    """
    repos = None
    print("2번 드리퍼에 Spiral")
    DP1 = [0.000, 0.000, 0.000, 0.0, 0.000, 0.000]
    Pa1 = [pa0, 0.0, 0.0, 30.0, 0.0, 0.0]

    await PTP(cf.pouring_water_dripper2["J1"], 30, -1)
    repos = cf.pouring_water_dripper2["J2"]
    repos[5] = ang
    repos[4] = ang_sub1
    repos[3] = ang_sub2
    repos[2] = ang_sub3
    repos[1] = ang_sub4
    repos[0] = ang_sub5

    print("Spiral 좌표 : ", repos, "pa0 : ", pa0, "speed : ", SPEED)
    await PTP(repos, 20, -1)
    print("Spiral... ")
    await newSPIRAL(repos, SPEED, Pa1, DP1)
    await PTP(cf.pouring_water_dripper2["J1"], 100, -1)
    await asyncio.sleep(ext)


async def spiral2(recipe_num):
    """
    2번 드리퍼에 대해 레시피별 물붓기 시퀀스.
    """
    for i in range(1, 6):
        pa0 = 1.8
        if recipe_num == 1:
            spd = [0, 70, 90, 70, 90, 80]
            ext = [0, 45, 25, 25, 18, 1]
        elif recipe_num == 2:
            spd = [0, 100, 100, 80, 90, 70]
            ext = [0, 45, 45, 45, 45, 1]

        ang = list(cf.pouring_water_dripper2.values())[i][5]
        ang_sub1 = list(cf.pouring_water_dripper2.values())[i][4]
        ang_sub2 = list(cf.pouring_water_dripper2.values())[i][3]
        ang_sub3 = list(cf.pouring_water_dripper2.values())[i][2]
        ang_sub4 = list(cf.pouring_water_dripper2.values())[i][1]
        ang_sub5 = list(cf.pouring_water_dripper2.values())[i][0]

        await standard_spiral2(spd[i], ang, ang_sub1, ang_sub2, ang_sub3, ang_sub4, ang_sub5, pa0, ext[i])


async def standard_spiral3(SPEED, ang, ang_sub1, ang_sub2, ang_sub3, ang_sub4, ang_sub5, pa0, ext):
    """
    3번 드리퍼에 나선형 물붓기.
    """
    repos = None
    print("3번 드리퍼에 Spiral")
    DP1 = [0.000, 0.000, 0.000, 0.0, 0.000, 0.000]
    Pa1 = [pa0, 0.0, 0.0, 30.0, 0.0, 0.0]

    await PTP(cf.pouring_water_dripper3["J1"], 100, -1)
    repos = cf.pouring_water_dripper3["J2"]
    repos[5] = repos[5] + ang
    repos[4] = repos[4] + ang_sub1
    repos[3] = repos[3] + ang_sub2
    repos[2] = repos[2] + ang_sub3
    repos[1] = repos[1] + ang_sub4
    repos[0] = repos[0] + ang_sub5

    print("Spiral 좌표 : ", repos, "pa0 : ", pa0, "speed : ", SPEED)
    await PTP(repos, 20, -1)
    print("Spiral... ")
    await newSPIRAL(repos, SPEED, Pa1, DP1)
    await PTP(cf.pouring_water_dripper3["J1"], 100, -1)
    await asyncio.sleep(ext)


async def spiral3(recipe_num):
    """
    3번 드리퍼에 대해 레시피별 물붓기 시퀀스.
    """
    for i in range(2, 7):
        if i == 2:
            await PTP(cf.pouring_water_dripper3["J0"], 100, -1)

        pa0 = 1.8
        if recipe_num == 1:
            spd = [0, 0, 50, 75, 50, 85, 65]
            ext = [0, 0, 45, 25, 25, 18, 1]
        elif recipe_num == 2:
            spd = [0, 0, 100, 100, 80, 90, 70]
            ext = [0, 0, 45, 45, 45, 45, 1]

        if i % 2 == 1:
            ang = 5
            ang_sub1 = 0.896
            ang_sub2 = 0.261
            ang_sub3 = -0.428
            ang_sub4 = 1.152
            ang_sub5 = 0.893
        else:
            ang = 0
            ang_sub1 = 0
            ang_sub2 = 0
            ang_sub3 = 0
            ang_sub4 = 0
            ang_sub5 = 0

        await standard_spiral3(spd[i], ang, ang_sub1, ang_sub2, ang_sub3, ang_sub4, ang_sub5, pa0, ext[i])


# --- 최종 커피 전달 동작 (delivery1,2,3) ---
async def delivery1():
    robot.SetSpeed(60)
    await PTP(cf.delivery_home["J1"], 100, -1)
    await PTP(cf.delivery_cup1["J1"], 100, -1)
    await PTP(cf.delivery_cup1["J2"], 100, -1)
    await PTP(cf.delivery_cup1["J3"], 100, -1)
    await movegripper(1, 0, 50, 50, 10000, 0)
    await PTP(cf.delivery_cup1["J4"], 100, -1)
    await PTP(cf.delivery_cup1["J5"], 100, -1)
    await PTP(cf.delivery_cup1["J6"], 100, -1)
    await PTP(cf.delivery_cup1["J7"], 100, -1)
    await PTP(cf.delivery_cup1["J8"], 100, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.delivery_cup1["J7"], 100, -1)
    await PTP(cf.delivery_cup1["J6"], 100, -1)
    await PTP(cf.delivery_home["J1"], 100, -1)


async def delivery2():
    robot.SetSpeed(60)
    await PTP(cf.delivery_home["J1"], 100, -1)
    await PTP(cf.delivery_cup2["J1"], 100, -1)
    await PTP(cf.delivery_cup2["J2"], 100, -1)
    await PTP(cf.delivery_cup2["J3"], 100, -1)
    await movegripper(1, 0, 50, 50, 10000, 0)
    await PTP(cf.delivery_cup2["J4"], 100, -1)
    await PTP(cf.delivery_cup2["J5"], 100, -1)
    await PTP(cf.delivery_cup2["J6"], 100, -1)
    await PTP(cf.delivery_cup2["J7"], 100, -1)
    await PTP(cf.delivery_cup2["J8"], 100, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.delivery_cup2["J7"], 100, -1)
    await PTP(cf.delivery_cup2["J6"], 100, -1)
    await PTP(cf.delivery_home["J1"], 100, -1)


async def delivery3():
    robot.SetSpeed(60)
    await PTP(cf.delivery_home["J1"], 100, -1)
    await PTP(cf.delivery_cup3["J1"], 100, -1)
    await PTP(cf.delivery_cup3["J2"], 100, -1)
    await PTP(cf.delivery_cup3["J3"], 100, -1)
    await movegripper(1, 0, 50, 50, 10000, 0)
    await PTP(cf.delivery_cup3["J4"], 100, -1)
    await PTP(cf.delivery_cup3["J5"], 100, -1)
    await PTP(cf.delivery_cup3["J6"], 100, -1)
    await PTP(cf.delivery_cup3["J7"], 100, -1)
    await PTP(cf.delivery_cup3["J8"], 100, -1)
    await movegripper(1, 100, 50, 50, 10000, 0)
    await PTP(cf.delivery_cup3["J7"], 100, -1)
    await PTP(cf.delivery_cup3["J6"], 100, -1)
    await PTP(cf.delivery_home["J1"], 100, -1)


async def hello_drip():
    """
    간단한 데모 동작 (인사 동작처럼).
    """
    await PTP(cf.hello_drip_pos["J0"], 100, -1)
    await PTP(cf.hello_drip_pos["J1"], 100, -1)
    await PTP(cf.hello_drip_pos["J0"], 100, -1)
    await PTP(cf.hello_drip_pos["J1"], 100, -1)
    await PTP(cf.hello_drip_pos["J0"], 100, -1)
    await PTP(cf.hello_drip_pos["J1"], 100, -1)
    await PTP(cf.home_point["J"], 70, -1)


# --- 드립 과정에서 호출할 편의 함수 모음 ---
async def beancup_pick(cup_point):
    if cup_point == 1:
        await beancup_pick1()
    elif cup_point == 2:
        await beancup_pick2()
    elif cup_point == 3:
        await beancup_pick3()


async def beancup_back(cup_point):
    if cup_point == 1:
        await beancup_back1()
    elif cup_point == 2:
        await beancup_back2()
    elif cup_point == 3:
        await beancup_back3()


async def beancup_dropbean(drip_point):
    if drip_point == 1:
        await beancup_dropbean1()
    elif drip_point == 2:
        await beancup_dropbean2()
    elif drip_point == 3:
        await beancup_dropbean3()


async def shaking_dripper(drip_point):
    if drip_point == 1:
        await shaking_dripper1()
    elif drip_point == 2:
        await shaking_dripper2()
    elif drip_point == 3:
        await shaking_dripper3()


async def spiral_dripper(drip_point, recipe_num):
    if drip_point == 1:
        await spiral1(recipe_num)
    elif drip_point == 2:
        await spiral2(recipe_num)
    elif drip_point == 3:
        await spiral3(recipe_num)
