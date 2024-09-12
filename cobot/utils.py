# coding=gbk 
import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs

"""Send all coords to robot arm.
        Args:
           coords: a list of coords value(List[float]).
                        for mycobot / mecharm / myArm: [x(mm), y, z, rx(angle), ry, rz]\n
                        for mypalletizer: [x, y, z, θ]
            speed : (int) 1 ~ 100
            mode : (int) 0 - angluar, 1 - linear (mypalletizer 340 does not require this parameter)
"""

"""
�������۾���
����Ϊ��Ԫ����ƽ������
"""
def calculate_hand_eye_matrix(qt, ts):
    w, x, y, z = qt

    # ������ת����
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])

    # ƽ������
    t = np.array(ts)

    # ������α任����
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    print("���۱궨����")
    print(T)
    # print(type(T))
    return T

"""
�������ϱ궨��ʽ
���������ϵ������õ�Ŀ������(x,y,z)ת���ɻ�е�ۻ�����ϵ�µģ�x,y,z��
"""
def transform_camera_to_armend(camera_xyz, hand_eye_matrix):

    # ���������ϵ�µ���������(x y z)ת��Ϊ�������
    camera_xyz_homogeneous = np.append(camera_xyz, 1)
 
    # ʹ�����۾����������ת��Ϊ��е��ĩ������(4x4��ξ������ά����[x,y,z,1])
    arm_xyz_homogeneous = np.dot(hand_eye_matrix, camera_xyz_homogeneous)
 
    # �����������ȡ��xyz����
    arm_xyz = arm_xyz_homogeneous[:3]
    return arm_xyz

"""
����realsense����������pipleline
start_realsense����D435i����ͷ������rgb�����ͼ
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

    # ��ȡrealsense�ڲ�
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    # �ڲξ���
    intr_matrix = np.array([
        [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
    ])
    # 16λ���ͼ
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    # 8λ���ͼ
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
    pos = np.where(depth_image_8bit == 0)
    depth_image_8bit[pos] = 255
    # rgbͼ
    color_image = np.asanyarray(color_frame.get_data())
  
    # ���� rgbͼ�����ͼ������ڲΣ��������ϵ��(intr.coeffs)
    return color_image, depth_image, intr_matrix, np.array(intr.coeffs)

"""
ʶ��aruco�겢������ת����rvec��ƽ������tvec, ����rgb, depth, intr_matrix, intr_coeffs
"""
def get_aruco_coord( rgb, depth, intr_matrix, intr_coeffs):

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)   # ��ȡdictionary
    parameters = aruco.DetectorParameters()                                 # ����detector parameters

    # ���aruco
    corners, ids, rejectImapoint = aruco.detectMarkers(rgb, aruco_dict, parameters=parameters)
    if len(corners) > 0:
        # ���Ƴ�arucoλ��,0.1��ӦmarkerLength��������λm, rvect��ת����, tvecƽ������
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.1, intr_matrix, intr_coeffs)
        # aruco.drawDetectedMarkers(rgb, corners) # ��ͼƬ�ϱ�עarucoλ��
        # # ��עaruco��̬
        # for i in range(rvec.shape[0]):
        #     cv2.drawFrameAxes(rgb, intr_matrix, intr_coeffs, rvec[i, :, :], tvec[i, :, :], 0.05)
        #     aruco.drawDetectedMarkers(rgb, corners) 
        #     cv2.imshow('RGB image', rgb)
        return rvec, tvec, corners
    else:
        return  None, None, None


if __name__ == "__main__":

    # �������۾���
    quaternion = [0.999574831685977, 0.016861922075053517, -0.0012176168900470685, 0.023756027719189287]
    translation = [-0.0291947, -0.0481932, 0.0133379]
    hand_eye_matrix = calculate_hand_eye_matrix(quaternion, translation)
    while 1:

        rgb, depth, intr_matrix, intr_coeffs = start_realsense() # ���������ȡrgb��ز���
        rvec, tvec, corners = get_aruco_coord( rgb, depth, intr_matrix, intr_coeffs) # �������ͼ��ʶ��arcuo, ��ȡ��ת����rvec��ƽ������tvec

        # ʶ��aruco
        if corners is not None:
            aruco.drawDetectedMarkers(rgb, corners) # ��ͼƬ�ϱ�עarucoλ��
            # ��עaruco��̬
            for i in range(rvec.shape[0]):
                cv2.drawFrameAxes(rgb, intr_matrix, intr_coeffs, rvec[i, :, :], tvec[i, :, :], 0.05)
                aruco.drawDetectedMarkers(rgb, corners) 
            for i in range(len(tvec)):
                # �������ϵ��ĳ����ά����
                camera_arcuo_xyz = np.array([tvec[i][0][0], tvec[i][0][1], tvec[i][0][2]])
            
                # ����ת������е��ĩ������ϵ��
                point_end_effector = transform_camera_to_armend(camera_arcuo_xyz, hand_eye_matrix)
            

        cv2.imshow('RGB image', rgb)

        key = cv2.waitKey(1)
        if key == ord('1'):
            print("��ת����rvec:")
            print(rvec)
            print("ƽ������tvec:")
            print(tvec)
        if key == ord('2'):
            print("�����aruco����:")
            print(camera_arcuo_xyz)
        if key == ord('3'):
            print("��е��ĩ��ִ��������ϵ�µ�����:", point_end_effector)

        elif key == ord('q') or key == 27:
            pipeline.stop()
            break

    cv2.destroyAllWindows()