
from pymycobot import MyCobotSocket
# 默认使用9000端口
#其中"172.20.10.14"为机械臂IP，请自行输入你的机械臂IP
mc = MyCobotSocket("192.168.137.238",9000) 
print("mycobot连接成功！")
def gripper_he():
    mc.set_gripper_state(1, 15)
def gripper_kai():
    mc.set_gripper_state(0, 15)

def off_lock():
    mc.release_all_servos()

def on_lock():
    mc.power_on()
  
def menu():
    print("=============================")
    print(" 1:回到零点\n 2:获取当前位姿\n 3:移动到指定坐标\n 4:移动到指定角度\n 5:关电\n 6:开电\n 7:quit\n")
    print("=============================") 

if __name__ == "__main__":
    menu()
    
    # print(mc.get_reference_frame())
    # print(mc.get_end_type())
    
    while(True):
        a = input("输入选项: ")
        if a == "1":
            mc.send_angles([0, 0, 0, 0, 0, 0], 30)
            print("\u56de\u96f6 ok\n")
            menu()
        elif a == "2":
            print("coords:", mc.get_coords())
            print("\n")
            print("angles:", mc.get_angles())
            print("\n")
            menu()
        elif a == "3":
            # [ 11.16   53.88 300]
            # [69.3, 53.7, 272.9, -159.36, 1.26, -3.86]

            mc.send_coords([23.2, 118.13, 47.2, -93.77, -1.08, 6.49],10,0)
            print("\u5b9e\u9645\u5230\u8fbe\u7684\u4f4d\u7f6e", mc.get_coords())
            print("3 ok\n")
            menu()
        elif a == "4":
            # 解 1: [-107.6, 70.85, -0.0, -66.35, 107.95, 4.58]
            # 解 2: [-107.6, 70.85, -0.0, -66.35, 107.95, 4.58]
            # 解 3: [102.95, -71.92, 0.0, 78.21, -102.73, -5.95]
            # 解 4: [102.95, -71.92, 0.0, 78.21, -102.73, -5.95]

            mc.send_angles([89.92, -59.97, -12.0, 74.3, 5.9, -0.47],10)
            print("\u5b9e\u9645\u5230\u8fbe\u7684\u89d2\u5ea6", mc.get_coords())
            print("4 ok\n")
            menu()

        elif a == "5":
            mc.power_off()
            print("5 ok\n")
            menu()
        elif a == "6":
            mc.power_on()
            print("6 ok\n")
            menu()
        elif a == "7":
            print("quit!\n")
            break
    