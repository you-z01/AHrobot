
import cv2
from detection import *
from cobot.camera_to_armend import *
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


translation = [-0.0291947, -0.0481932, 0.0133379]  # 单位: m
quaternion = [0.999574831685977, 0.016861922075053517, -0.0012176168900470685, 0.023756027719189287]

if __name__ == '__main__':
    print("[INFO] 启动RealSense相机与YoloV5目标检测")

    # 初始化相机和模型
    realsense_camera = RealSenseCamera()
    print("[INFO] 完成realsense相机启动")

    yolo_model = YoloV5(yolov5_yaml_path='config/strawberry_maturity.yaml')
    print("[INFO] 完成YoloV5模型加载")

    # 初始化手眼转换类
    #hand_eye_transfor = HandEyeTransformation(translation, quaternion)

    # 初始化机械臂运动学解类
    kinematics_solution = KinematicsSolution('cobot/dh_parameters.yaml')

    camera_xyz = [] # 相机坐标系下的xyz 一维列表
    camera_xyz_mm = [] # 相机坐标系下的xyz 一维列表
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

            # 计算机械臂末端执行器下的坐标
            # arm_end = hand_eye_transfor.camera_to_end(camera_xyz) 
            # arm_end_six = [arm_end[0],arm_end[1],arm_end[2], -37.08, 2.1, -2.02]


            # # 逆运动学解
            # IS_results = kinematics_solution.inverse_kinematics(arm_end_six)
            # if IS_results:
            #     print("有效的关节角度解:")
            #     for i, solution in enumerate(IS_results):
            #         formatted_solution = np.round(solution, 2)        # 保留两位小数
            #         print(f"解 {i+1}:", formatted_solution.tolist())  # 转换为列表并打印


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if cv2.waitKey(1) & 0xFF == ord('1'):
                print(camera_xyz)
                print(camera_xyz_mm)

    finally:
        realsense_camera.stop()
        cv2.destroyAllWindows()
