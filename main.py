
from detection import *
import cv2

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

            # 将坐标转换为毫米并保留两位小数
            camera_xyz_mm = [round(coord * 1000, 2) for coord in camera_xyz]

            #camera_xyz = camera_xyz.tolist()
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
            
            camera_xyz, camera_xyz_mm = camera_coordinate() # 计算相机坐标系下的xyz

           
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if cv2.waitKey(1) & 0xFF == ord('1'):
                print(camera_xyz)
                print(camera_xyz_mm)
                continue

    finally:
        realsense_camera.stop()
        cv2.destroyAllWindows()
