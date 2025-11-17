from endeffector import endeffectorCTL
from neuromeka import IndyDCP2
import time
import math

class indyCTL():
    def __init__(self):
        
        self.robot_ip = "192.168.0.xx"               # 로봇 컨트롤 박스의 IP 주소
        self.robot_name = 'NRMK-Indy7'              # 로봇 이름
        self.indy = IndyDCP2(server_ip=self.robot_ip, robot_name=self.robot_name)
        self.endeffector = endeffectorCTL()


        self.indy.connect()                         # 연결
        self.indy.reset_robot()
        self.indy.set_joint_vel_level(1)            # 로봇 속도 조절, 기본 : 3
        self.indy.set_task_vel_level(1)             # [안전:1,2], [보통:3,4], [위험:5,6], [매우 위험:7,8,9]

        print(self.indy.get_task_vel_level())
        print(self.indy.get_task_vel_level())
        
        self.indy.joint_move_to([0,0,-90,0,0,0])
        self.indy.wait_for_move_finish()
        print("Ready")

    def run(self, x=0, y=0, z=0, angle=0):
        self.endeffector.home()
        mx = -float(format(x-0.32, ".4f"))
        my = +float(format(y, ".4f"))

        print(angle)

        X = -0.26*math.cos(math.radians(angle))
        Y = 0.26*math.sin(math.radians(angle))
        Z = (z-0.32)+0.08

        print("mx: ", mx, " my: ", my, "z: ", z)
        print("X:", X, "Y: ", Y, "Angle: ", angle)

        move_count = 0
        mode = "home"

        self.indy.joint_move_to([0,0,-90,0,-90,0])
        self.indy.wait_for_move_finish()
        if z < -0.24:
            self.indy.task_move_by([mx+X,my+Y,-0.24,0,0,angle])
        else:
            self.indy.task_move_by([mx+X,my+Y,-Z,0,0,angle])
        self.indy.wait_for_move_finish()
        status = self.indy.get_robot_status().get('busy')

        while True:
            if mode == "home":
                tdtl = self.endeffector.get_TDTL()

                if tdtl == 1:
                    print('stop')
                    self.indy.stop_motion()
                    self.endeffector.grasp_stem()
                    mode = "grasp"
                    time.sleep(1)

                if status == 0 and move_count == 0:
                    print("Move")
                    self.indy.task_move_by([-X,-Y,0,0,0,0])
                    move_count=1

            if mode == "grasp":
                [micro1, micro2] = self.endeffector.get_micro_photo()

                if micro1 == 0 or micro2 == 0:
                    self.indy.stop_motion()
                    self.endeffector.pull_wire()
                    time.sleep(1)
                    self.endeffector.release_stem()
                    time.sleep(1)
                    self.indy.task_move_by([-0.07,0,0,0,0,0])
                    self.indy.wait_for_move_finish()
                    self.indy.joint_move_to([0,0,-90,0,-90,0])
                    self.indy.wait_for_move_finish()
                    break

                if move_count == 1:
                    self.indy.task_move_by([0,0,0.1,0,0,0])
                    move_count = 2

            # status = self.indy.get_robot_status().get('busy')
            

        # self.indy.task_move_by([-float(format(y, ".4f"))+0.22,-float(format(x, ".4f")),0,0,0,0])


        # self.indy.task_move_by([-0.13+0.22,0.09,0,0,0,0])

        # self.indy.wait_for_move_finish()
        time.sleep(0.5)

    def go_home(self):
        self.endeffector.home()
        self.indy.joint_move_to([0,0,-90,0,0,0])
        self.indy.wait_for_move_finish()
        time.sleep(2)
        print("Ready")

    def close(self):
        self.go_home()
        self.indy.go_home()
        self.indy.wait_for_move_finish()
        print("Task Pos: ", self.indy.get_task_pos())
        time.sleep(0.5)
        self.indy.disconnect()                   # 연결 해제
        print("Close")


if __name__ == "__main__":
    indy = indyCTL()
    indy.run()
    indy.close()


# pos = indy.get_task_pos()
# print(pos[2])
# mv_z = -0.11

# if pos[2] + mv_z >= 0.4:
#     indy.task_move_by([0, 0, mv_z, 0, 0, 0])
#     indy.wait_for_move_finish()
#     print(indy.get_task_pos())

# print(indy.get_robot_status())
# print(indy.get_robot_status().get('ready'))


# indy.go_home()                      # 영 위치로 이동
# indy.wait_for_move_finish()         # 동작이 끝날 때까지 대기status = indy.direct_teaching(False)    # True : 직접 조종 가능, False : 직접 조종 불가


# ====================================================================================

# status = indy.direct_teaching(False)    # True : 직접 조종 가능, False : 직접 조종 불가


# indy.go_home()                          # 홈 위치로 이동
# indy.wait_for_move_finish()             # 동작이 끝날 때까지 대기
# print(indy.get_control_torque())        # 각 관절들의 현재 토크값 가져오기

# indy.go_zero()                          # 영 위치로 이동
# indy.wait_for_move_finish()             # 동작이 끝날 때까지 대기
# print(indy.get_control_torque())        # 각 관절들의 현재 토크값 가져오기

# indy.joint_move_by([0,0,0,0,0,0])       # 현재 위치에서 정해진 만큼 관절 이동

# indy.joint_move_to([0,0,0,0,0,0])       # 영 위치에서 정해진 만큼 관절 이동, 영위치 : [0,0,0,0,0,0]

# indy.get_joint_vel_level()              # 현재 설정된 joint move 속도값 가져오기
# indy.set_joint_vel_level(3)             # joint move 속도 조절, 기본 : 3
#                                         # [안전:1,2], [보통:3,4], [위험:5,6], [매우 위험:7,8,9]

# indy.get_task_vel_level()               # 현재 설정된 task move 속도값 가져오기

# indy.task_move_by([0,0,0,0,0,0])        # 현재 위치에서 도구 좌표계 기준으로 이동
#                                         # [0,0,0, : x(+ : 앞, - : 뒤), y(+ : 왼, - : 오), z(+ : 위, - : 아래), 기본 0.1씩 이동
#                                         # 0,  :  각도 (+ : 오른쪽 바라봄, - : 왼쪽 바라봄)
#                                         # 0,  :  각도 (+ : 아래쪽 바라봄, - : 위쪽 바라봄)
#                                         # 0]  :  각도 (+ : 시계방향, - : 반시계방향)
