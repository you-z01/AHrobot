# coding=gbk 
import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs

"""Send all coords to robot arm.
        Args:
           coords: a list of coords value(List[float]).
                        for mycobot / mecharm / myArm: [x(mm), y, z, rx(angle), ry, rz]\n
                        for mypalletizer: [x, y, z, 胃]
            speed : (int) 1 ~ 100
            mode : (int) 0 - angluar, 1 - linear (mypalletizer 340 does not require this parameter)
"""

"""
计算手眼矩阵
传入为四元数和平移向量
"""
def calculate_hand_eye_matrix(qt, ts):
    w, x, y, z = qt

    # 计算旋转矩阵
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])

    # 平移向量
    t = np.array(ts)

    # 创建齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    print("手眼标定矩阵：")
    print(T)
    # print(type(T))
    return T

"""
眼在手上标定方式
将相机坐标系下所获得的目标物体(x,y,z)转换成机械臂基坐标系下的（x,y,z）
"""
def transform_camera_to_armend(camera_xyz, hand_eye_matrix):

    # 将相机坐标系下的物体坐标(x y z)转换为齐次坐标
    camera_xyz_homogeneous = np.append(camera_xyz, 1)
 
    # 使用手眼矩阵将相机坐标转换为机械臂末端坐标(4x4齐次矩阵乘四维向量[x,y,z,1])
    arm_xyz_homogeneous = np.dot(hand_eye_matrix, camera_xyz_homogeneous)
 
    # 从齐次坐标中取出xyz坐标
    arm_xyz = arm_xyz_homogeneous[:3]
    return arm_xyz

"""
配置realsense参数并开启pipleline
start_realsense启动D435i摄像头并对齐rgb和深度图
"""
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)
def start_realsense():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # 获取realsense内参
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    # 内参矩阵
    intr_matrix = np.array([
        [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
    ])
    # 16位深度图
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    # 8位深度图
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
    pos = np.where(depth_image_8bit == 0)
    depth_image_8bit[pos] = 255
    # rgb图
    color_image = np.asanyarray(color_frame.get_data())
  
    # 返回 rgb图，深度图，相机内参，相机畸变系数(intr.coeffs)
    return color_image, depth_image, intr_matrix, np.array(intr.coeffs)

"""
识别aruco标并返回旋转向量rvec和平移向量tvec, 传入rgb, depth, intr_matrix, intr_coeffs
"""
def get_aruco_coord( rgb, depth, intr_matrix, intr_coeffs):

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)   # 获取dictionary
    parameters = aruco.DetectorParameters()                                 # 创建detector parameters

    # 检测aruco
    corners, ids, rejectImapoint = aruco.detectMarkers(rgb, aruco_dict, parameters=parameters)
    if len(corners) > 0:
        # 估计出aruco位姿,0.1对应markerLength参数，单位m, rvect旋转向量, tvec平移向量
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.1, intr_matrix, intr_coeffs)
        # aruco.drawDetectedMarkers(rgb, corners) # 在图片上标注aruco位置
        # # 标注aruco姿态
        # for i in range(rvec.shape[0]):
        #     cv2.drawFrameAxes(rgb, intr_matrix, intr_coeffs, rvec[i, :, :], tvec[i, :, :], 0.05)
        #     aruco.drawDetectedMarkers(rgb, corners) 
        #     cv2.imshow('RGB image', rgb)
        return rvec, tvec, corners
    else:
        return  None, None, None


if __name__ == "__main__":

    # 计算手眼矩阵
    quaternion = [0.999574831685977, 0.016861922075053517, -0.0012176168900470685, 0.023756027719189287]
    translation = [-0.0291947, -0.0481932, 0.0133379]
    hand_eye_matrix = calculate_hand_eye_matrix(quaternion, translation)
    while 1:

        rgb, depth, intr_matrix, intr_coeffs = start_realsense() # 启动相机获取rgb相关参数
        rvec, tvec, corners = get_aruco_coord( rgb, depth, intr_matrix, intr_coeffs) # 传入相机图像并识别arcuo, 获取旋转向量rvec和平移向量tvec

        # 识别到aruco
        if corners is not None:
            aruco.drawDetectedMarkers(rgb, corners) # 在图片上标注aruco位置
            # 标注aruco姿态
            for i in range(rvec.shape[0]):
                cv2.drawFrameAxes(rgb, intr_matrix, intr_coeffs, rvec[i, :, :], tvec[i, :, :], 0.05)
                aruco.drawDetectedMarkers(rgb, corners) 
            for i in range(len(tvec)):
                # 相机坐标系下某点三维坐标
                camera_arcuo_xyz = np.array([tvec[i][0][0], tvec[i][0][1], tvec[i][0][2]])
            
                # 将点转移至机械臂末端坐标系下
                point_end_effector = transform_camera_to_armend(camera_arcuo_xyz, hand_eye_matrix)
            

        cv2.imshow('RGB image', rgb)

        key = cv2.waitKey(1)
        if key == ord('1'):
            print("旋转向量rvec:")
            print(rvec)
            print("平移向量tvec:")
            print(tvec)
        if key == ord('2'):
            print("相机中aruco坐标:")
            print(camera_arcuo_xyz)
        if key == ord('3'):
            print("机械臂末端执行器坐标系下的坐标:", point_end_effector)

        elif key == ord('q') or key == 27:
            pipeline.stop()
            break

    cv2.destroyAllWindows()