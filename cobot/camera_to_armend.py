

# 眼在手上标定方式
# 将相机坐标系下所获得的目标物体(x,y,z)转换成机械臂基坐标系下的（x,y,z）
import numpy as np
 
# 将相机坐标系下的物体坐标转移至机械臂末端坐标系下
def transform_camera_to_armend(camera_xyz, hand_eye_matrix):
    # Append 1 to the camera coordinates to make them homogeneous
    camera_xyz_homogeneous = np.append(camera_xyz, 1)
 
    # Transform camera coordinates to arm coordinates using hand-eye matrix
    arm_xyz_homogeneous = np.dot(hand_eye_matrix, camera_xyz_homogeneous)
 
    # Remove the homogeneous component and return arm coordinates
    arm_xyz = arm_xyz_homogeneous[:3]
    return arm_xyz
 
# 将机械臂末端坐标系下的物体坐标转移至机械臂基坐标系下
def transform_point_to_base(point, T_end_effector_to_base):
    # 将点从末端坐标系转移到基座标系
    point_homogeneous = np.append(point, 1)  # 转换为齐次坐标
    point_base_homogeneous = T_end_effector_to_base @ point_homogeneous
    point_base = point_base_homogeneous[:3] / point_base_homogeneous[3]  # 转换回非齐次坐标
 
    return point_base
 
# 深度相机坐标系到机械臂末端坐标系的位姿转换矩阵
hand_eye_matrix = np.array([[ 7.03178404e-01, -7.10991045e-01, -5.64506277e-03,  2.17625306e-02],
 [ 7.11002245e-01,  7.03189737e-01, -3.22089035e-05, -6.90680181e-02],
 [ 3.99245045e-03, -3.99100370e-03,  9.99984066e-01,  4.01377097e-02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
 
# 机械臂末端的坐标系到机械臂基坐标系下的位姿转换矩阵
T_end_effector_to_base = np.array([[0.665037, -0.746393, 0.0249527, 0.442627],
                                   [-0.746141, -0.662656, 0.0645062, -0.242624],
                                   [-0.0316119, -0.0615173, -0.997605, 0.731147],
                                   [0, 0, 0, 1]])
 
# 相机坐标系下某点三维坐标
# camera_xyz = np.array([0.0858, 0.00472, 0.47100])
camera_xyz = np.array([6.482825, -4.570933, 304.000000])
# 将点转移至机械臂末端坐标系下
point_end_effector = transform_camera_to_armend(camera_xyz, hand_eye_matrix)
 
# 将点转移到基座标系下
point_base = transform_point_to_base(point_end_effector, T_end_effector_to_base)
 
# 输出点在机械臂基坐标系下的坐标
print("Point in Base Coordinates:", point_base)