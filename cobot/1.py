import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_transformations
from mycobot import MyCobot  # 需要根据你的实际情况导入控制机械臂的库

# 手眼标定矩阵
translation = [-0.0291947, -0.0481932, 0.0133379]  # 平移 (x, y, z)
rotation_euler = [1.92847, -0.185372, 2.71977]  # 欧拉角 (rx, ry, rz)

# 将欧拉角转换为旋转矩阵
rotation_matrix = tf_transformations.euler_matrix(rotation_euler[0], rotation_euler[1], rotation_euler[2])[:3, :3]

# 创建齐次变换矩阵
H_eye_cam = np.eye(4)
H_eye_cam[:3, :3] = rotation_matrix
H_eye_cam[:3, 3] = translation

# 启动 Realsense 相机
def start_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    intr_matrix = np.array([
        [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
    ])
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
    pos = np.where(depth_image_8bit == 0)
    depth_image_8bit[pos] = 255
    color_image = np.asanyarray(color_frame.get_data())
    
    return color_image, depth_image, intr_matrix, np.array(intr.coeffs)

# 识别 ArUco 标记并计算位姿
def get_aruco_coord(rgb, depth, intr_matrix, intr_coeffs):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters()
    corners, ids, _ = aruco.detectMarkers(rgb, aruco_dict, parameters=parameters)
    if len(corners) > 0:
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.1, intr_matrix, intr_coeffs)
        return rvec, tvec, corners, ids
    else:
        return None, None, None, None

# 将 ArUco 标记的位置从相机坐标系转换到机械臂末端坐标系
def transform_aruco_to_robot_end(rvec, tvec, H_eye_cam):
    # 将旋转向量转换为旋转矩阵
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    
    # 形成 4x4 的位姿矩阵
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = rotation_matrix
    pose_matrix[:3, 3] = tvec[0]
    
    # 转换到机械臂末端坐标系
    aruco_pos_camera = np.dot(H_eye_cam, pose_matrix)
    
    # 提取转换后的位置信息
    return aruco_pos_camera[:3, 3]

# 控制机械臂的函数
def send_coords_to_robot(coords, speed=100, mode=1):
    # 这里使用一个假设的控制库 MyCobot 进行示例
    # 实际代码需根据你的机械臂控制库调整
    mc = MyCobot('/dev/ttyUSB0')  # 假设的连接端口
    mc.move_to(coords[0], coords[1], coords[2], speed, mode)

def main():
    rospy.init_node('aruco_tf_broadcaster')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    while not rospy.is_shutdown():
        rgb, depth, intr_matrix, intr_coeffs = start_realsense()
        rvec, tvec, corners, ids = get_aruco_coord(rgb, depth, intr_matrix, intr_coeffs)

        if corners is not None:
            aruco.drawDetectedMarkers(rgb, corners)
            for i in range(rvec.shape[0]):
                cv2.drawFrameAxes(rgb, intr_matrix, intr_coeffs, rvec[i, :, :], tvec[i, :, :], 0.05)
                marker_id = ids[i][0]
                
                # 将 ArUco 标记的位置从相机坐标系转换到机械臂末端坐标系
                aruco_pos_robot_end = transform_aruco_to_robot_end(rvec[i], tvec[i], H_eye_cam)
                
                rospy.loginfo(f"Moving to position: {aruco_pos_robot_end}")

                # 将坐标发送给机械臂
                send_coords_to_robot(aruco_pos_robot_end)
        
        cv2.imshow('RGB image', rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
