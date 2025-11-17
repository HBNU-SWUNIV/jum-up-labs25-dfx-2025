from libraries.peripheral._opencr import OpenCRSerial
import time

class endeffectorCTL():
    def __init__(self):
        self.BAUDRATE                    = 57600
        self.DEVICENAME                  = '/dev/ttyACM0'
        self.timeout                     = 1.0
        self.devaddr                     = 0xf0

        self.oc = OpenCRSerial(self.DEVICENAME, self.BAUDRATE, self.timeout, self.devaddr)

        # 속도 0~1023
        dxl11_velocity     = 200
        dxl11_acceleration = 0
        dxl13_velocity     = 200
        dxl13_acceleration = 0

        self.oc.dxl_init(11, dxl11_velocity, dxl11_acceleration)
        self.oc.dxl_init(13, dxl13_velocity, dxl13_acceleration)

    def home(self):
        self.oc.dxl_goalPosition(11, 2048)
        time.sleep(2)
        self.oc.dxl_goalPosition(13, 2500)

    def grasp_stem(self):
        self.oc.dxl_goalPosition(11, 950)  # 음수 = 시계방향, 양수 = 반시계방향
        
    def release_stem(self):
        self.oc.dxl_goalPosition(11, 2048)  # 음수 = 시계방향, 양수 = 반시계방향

    def pull_wire(self):
        self.oc.dxl_goalPosition(13, 3000)

    def release_wire(self):
        self.oc.dxl_goalPosition(13, 2500)

    def get_micro_photo(self):
        photo1 = self.oc.microphoto_1()[0]
        photo2 = self.oc.microphoto_2()[0]

        return [photo1, photo2]
    
    def get_TDTL(self):
        return self.oc.tdtl()[0]

    def run(self):
        # home, grasp, pull_wire
        mode = "home"

        self.home()
        time.sleep(2)
        print("Start")

        while True:
            try:
                tdtl = self.get_TDTL()
                [micro1, micro2] = self.get_micro_photo()
                print(micro1)
                print(micro2)
                print(tdtl)

                if mode == 'home':
                    if tdtl == 1:
                        self.grasp_stem()
                        time.sleep(1)
                        mode = "grasp"
                elif mode == 'grasp':
                    if micro1 == 0 or micro2 == 0:
                        self.pull_wire()
                        time.sleep(1)
                        self.release_stem()
                        time.sleep(1)
                        mode = "pull_wire"
                elif mode == "pull_wire":
                    cha = input('a 입력: ')
                    if cha == "a":
                        self.release_wire()
                        time.sleep(1)
                        mode = "home"
                    
                time.sleep(0.1)
            except KeyboardInterrupt:
                print("초기화")
                self.home()
                time.sleep(2)
                self.oc.dxl_torqueOff(11)
                self.oc.dxl_torqueOff(13)
                time.sleep(1)
                print("종료")
                break
        
        # self.oc.dxl_torqueOn(11)
        # self.oc.dxl_torqueOn(13)

        # # # self.release13()
        # # # self.up11()
        # # # time.sleep(2)

        # # current_11 = self.oc.dxl_getPresentPositionData(11)
        # # current_13 = self.oc.dxl_getPresentPositionData(13)

        # # print(current_11)
        # # self.oc.dxl_goalPosition(11, 950)
        # # time.sleep(3)
        # # print(current_13)
        # # self.oc.dxl_goalPosition(13, 3000)
        # # time.sleep(3)

        # # self.home()
        # # time.sleep(3)

        # print(self.oc.dxl_getPresentPositionData(11))
        # print(self.oc.dxl_getPresentPositionData(13))
        
        # self.oc.dxl_torqueOff(11)
        # self.oc.dxl_torqueOff(13)


if __name__ == '__main__':
    to = endeffectorCTL()
    to.run()
