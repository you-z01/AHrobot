

import cv2

from detection import *
from cobot.kinematics_solution import *
from cobot.socket_client import *

def camera_coordinate():
    """计算相机坐标系下的xyz"""
    camera_xyz_list = [] # 相机坐标系下的xyz 二维列表
    camera_xyz_list_mm = []
    if xyxy_list:
        for i in range(len(xyxy_list)):
            ux = int((xyxy_list[i][0]+xyxy_list[i][2])/2)  # 计算像素坐标系的x
            uy = int((xyxy_list[i][1]+xyxy_list[i][3])/2)  # 计算像素坐标系的y

            # 获取深度值
            dis = aligned_depth_frame.get_distance(ux, uy)

            # 将像素坐标和深度值转换为相机坐标系下的XYZ（米）
            camera_xyz = rs.rs2_deproject_pixel_to_point(
                depth_intrin, (ux, uy), dis)  # 计算相机坐标系的xyz
            camera_xyz = np.round(np.array(camera_xyz), 5)  # 转成5位小数

            # 将numpy数组转换为列表
            camera_xyz = camera_xyz.tolist()

            # 将坐标转换为毫米并保留两位小数
            camera_xyz_mm = [round(coord * 1000, 2) for coord in camera_xyz]

            cv2.circle(canvas, (ux,uy), 4, (255, 255, 255), 5)  # 标出中心点
            cv2.putText(canvas, str(camera_xyz_mm), (ux+20, uy+10), 0, 1,
                        [225, 255, 255], thickness=2, lineType=cv2.LINE_AA) # 标出坐标
            
            camera_xyz_list.append(camera_xyz)
            camera_xyz_list_mm.append(camera_xyz_mm)
    fps = int(1 / (time.time() - t_start))
    cv2.putText(canvas, f'FPS: {fps}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('ObjectDetection', canvas)

    return camera_xyz_list, camera_xyz_list_mm



if __name__ == '__main__':
    print("[INFO] 启动RealSense相机与YoloV5目标检测")
    # 初始化相机和模型
    realsense_camera = RealSenseCamera()
    print("[INFO] 完成realsense相机启动")

    yolo_model = YoloV5(yolov5_yaml_path='config/strawberry_maturity.yaml')
    print("[INFO] 完成YoloV5模型加载")

    client = TCPClient('192.168.137.238', 9000)
    client.connect()

    # 初始化机械臂运动学解类
    kinematics_solution = KinematicsSolution('cobot/dh_parameters.yaml')

    camera_xyz = []         # 相机坐标系下的xyz 一维列表
    camera_xyz_mm = []      # 相机坐标系下的xyz(mm) 一维列表
    target_xyz = []         # 要抓取的目标坐标
    target_rotation = []    # 要抓取的目标旋转
    target_xyzRxyz = []     # 相机坐标系下的xyzRxyz 一维列表
    target_robot = []       # 机械臂坐标系下的目标坐标
    target_rotation = [-87.63, -0.71, 5.77] # 末端固定位姿 rx ry rz
    tool_length = 100       # 夹具长度
    flag = False # 标志位 是否抓取
    try:
        while True:
            # 获取对齐的图像
            intr, depth_intrin, color_image, depth_image, aligned_depth_frame = realsense_camera.get_aligned_images()
            if not depth_image.any() or not color_image.any():
                continue

            t_start = time.time()  # 开始计时
            # 执行目标检测
            canvas, class_id_list, xyxy_list, conf_list = yolo_model.detect(color_image)
            t_end = time.time()  # 结束计时
            
            # 计算相机坐标系下的xyz
            camera_xyz, camera_xyz_mm = camera_coordinate()

            # 计算目标坐标，合并为要发送的相机坐标系下的目标坐标
            if camera_xyz:
                target_xyz = camera_xyz[0]
                target_xyzRxyz = target_xyz + target_rotation

                print(f"目标坐标为：{target_xyzRxyz}")

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            
            
            if  camera_xyz and target_xyzRxyz[2] < 23:
                 # 发送目标坐标
                client.send_message(str(target_xyzRxyz)) # 发送坐标信息到服务器
                flag = True
                # while flag:
                time.sleep(1) 
                client.send_message("[1]") # 发送查询指令到服务器
                recive_data = client.receive_message()  # 接收到的数据

                # 尝试解析接收到的数据
                par_data = parse_data(recive_data)
                if par_data is None:
                    print("解析失败，无法处理数据")
                if len(par_data) < 3:
                    print("数据长度不足，无法提取前三个坐标值")

                # 提取前三位数据
                xyz_data_m = par_data[:3]

                # 将米转换为毫米，并保留两位小数
                xyz_data_mm = [round(coord * 1000, 2) for coord in xyz_data_m]

                # z减去夹据具的长度
                if  xyz_data_mm[-1] > tool_length:
                    xyz_data_mm[-1] = xyz_data_mm[-1] - tool_length
                else:
                    xyz_data_mm[-1] = 0
                # 合并为新的列表
                target_list = xyz_data_mm + target_rotation
                print(f"转换后的坐标和旋转列表为：{target_xyzRxyz}")

                IS_results = kinematics_solution.inverse_kinematics(target_list)    # 逆运动学解
                if IS_results:
                    print("有效的关节角度解:")
                    for i, solution in enumerate(IS_results):
                        formatted_solution = np.round(solution, 2)        # 保留两位小数
                        solution_list = formatted_solution.tolist()  # 转换为列表
                        print(f"解 {i+1}:", formatted_solution.tolist())  # 转换为列表并打印  

                    # client = TCPClient('192.168.137.238', 9000)
                    # client.connect()

                        
                        
                    # client.send_message("[6,100,15,9]") # 打开夹爪
                    # print("等待中")
                    # time.sleep(10) # 等待移动到指定点
                    # # 有逆运动学解 solution_list = [89.92, -59.97, -12.0, 74.3, 5.9, -0.47]
                    # # 在列表开头插入6，末尾添加9，添加移动速度
                        send_list = [6, 30] + solution_list + [9]
                        # 将列表转换成字符串
                        solution_str = str(send_list)
                        angle_data = f"{solution_str}"
                        print(f"发送的关节角度列表为：{angle_data}")
                        client.send_message(angle_data) # 发送关节角度信息到服务器，控制机械臂移动到指定点
                        time.sleep(1) # 等待移动到指定点
                        client.send_message("[6,50,15,9]") # 关闭夹爪
                        while(True):
                            pass
                    #flag = False


    finally:
        client.close()
        realsense_camera.stop()
        cv2.destroyAllWindows()
