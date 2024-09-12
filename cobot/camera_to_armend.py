import numpy as np
import transforms3d


class HandEyeTransformation:
    def __init__(self, translation, quaternion):
        """
        初始化手眼转换类，计算手眼矩阵
        
        参数:
        translation: 手眼标定的平移向量 [x, y, z] (m)
        quaternion: 手眼标定的四元数 [w, x, y, z]
        """
        # 将平移向量从米转换为毫米
        self.translation = np.array(translation) * 1000

        # 四元数转换为旋转矩阵
        self.rotation_matrix = transforms3d.quaternions.quat2mat(quaternion)

        # 构建 4x4 齐次变换矩阵 T_camera_to_end_effector
        self.T_camera_to_end_effector = np.eye(4)
        self.T_camera_to_end_effector[:3, :3] = self.rotation_matrix
        self.T_camera_to_end_effector[:3, 3] = self.translation
        print("手眼矩阵: \n", self.T_camera_to_end_effector)
        print("=====================================")

    def camera_to_end(self, camera_xyz):
        """
        将相机坐标系下的物体位置(mm)转换为机械臂末端执行器坐标系下的位置(mm)
        
        参数:
        x_camera, y_camera, z_camera: 物体在相机坐标系下的3D位置 (mm)

        返回:
        物体在末端执行器坐标系下的3D位置 (mm 两位小数)
        """
        # 目标物体在相机坐标系下的位置 (齐次坐标)
        P_camera = np.append(camera_xyz, 1)
        
        # 使用手眼矩阵将相机坐标系下的目标物体坐标转换为末端执行器坐标系下的坐标
        P_end_effector = np.dot(self.T_camera_to_end_effector, P_camera)

        # 返回末端执行器坐标系下的目标物体位置，保留两位小数
        return np.round(P_end_effector[:3], 2)
    
if __name__ == "__main__":
    

    # 平移向量 x, y, z (米)
    translation = [-0.0435494, -0.0569944, 0.0108573]
    # 四元数 w, x, y, z
    quaternion = [0.9992468756961643, 0.018581772994452937, 0.028317631977733293, -0.018934382562174293]

    hand_eye_calib = HandEyeTransformation(translation, quaternion)
    camera_xyz = [23.7, -43.83, 242.0]
    # 调用类方法进行坐标转换
    end_effector_position = hand_eye_calib.camera_to_end(camera_xyz)

    print("目标物体在末端执行器坐标系下的坐标 (单位: 毫米): ", end_effector_position)
