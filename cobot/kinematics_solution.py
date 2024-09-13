import numpy as np
import sympy as sp
from math import *
import yaml
import os

class KinematicsSolution:
    def __init__(self, dh_parameters_file):
        """
        初始化函数，读取并加载 D-H 参数表

        输入参数:
        dh_parameters_file (str): 存放 D-H 参数的 YAML 文件路径
        """
        # 读取 D-H 参数
        with open(dh_parameters_file, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
        dh_parameters_list = data['dh_parameters']
        
        # 转换为 NumPy 数组
        self.dh_parameters = np.array([
            [param['a'], param['alpha'], param['d'], param['theta'], param['offset']]
            for param in dh_parameters_list
        ], dtype=np.float16)

        # 关节角度范围
        self.motion_range = np.array([
            [-159, 159],
            [-139, 140],
            [-150, 150],
            [-150, 145],
            [-160, 160],
            [-160, 160]
        ], dtype=np.float16)

    def rotation_matrix_to_euler_angles(self, R):
        """
        将旋转矩阵转换为欧拉角（ZYX 顺序）

        参数:
        R (numpy.ndarray): 3x3 旋转矩阵

        返回:
        numpy.ndarray: 对应的欧拉角 (roll, pitch, yaw) 
        """
        yaw = atan2(R[1, 0], R[0, 0])   # yaw表示绕Z轴的旋转角度
        pitch = atan2(-R[2, 0], sqrt(R[2, 1]**2 + R[2, 2]**2))   # pitch表示绕Y轴的旋转角度
        roll = atan2(R[2, 1], R[2, 2])  # roll表示绕X轴的旋转角度

        yaw = degrees(yaw)
        pitch = degrees(pitch) 
        roll = degrees(roll)

        return np.array([roll, pitch, yaw])


    def forward_kinematics(self, joint_angles):
        """
        使用 D-H 参数进行正运动学求解

        参数:
        joint_angles (list): 输入的关节角度

        返回:
        numpy.ndarray: 末端执行器的位姿（位置 + 欧拉角）
        """
        transformation_matrix = np.eye(4)

        for i, params in enumerate(self.dh_parameters):
            a, alpha, d, theta, offset = params
            theta = joint_angles[i]*pi/180 + offset

            Ti = np.array([
                [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]
            ])
            transformation_matrix = transformation_matrix @ Ti
        
        T = transformation_matrix
        end_effector_position = T[:3, 3]
        end_effector_orientation = self.rotation_matrix_to_euler_angles(T[:3, :3])
        coords = np.concatenate((end_effector_position*1000, end_effector_orientation))

        return coords


    def myRPY2R_robot(self, x, y, z):
        """
        根据欧拉角构建旋转矩阵(ZYX 顺序）

        参数:
        x, y, z (float): 分别表示绕 X, Y, Z 轴的旋转角度（弧度）

        返回:
        numpy.ndarray: 旋转矩阵
        """
        Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
        Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
        Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        return R


    def pose_robot(self, x, y, z, Tx, Ty, Tz):
        """
        根据位置和欧拉角构建 4x4 齐次变换矩阵

        参数:
        x, y, z (float): 末端执行器的位置（单位：毫米）
        Tx, Ty, Tz (float): 末端执行器的欧拉角（单位：度）

        返回:
        numpy.ndarray: 4x4 齐次变换矩阵
        """
        thetaX = Tx / 180 * pi
        thetaY = Ty / 180 * pi
        thetaZ = Tz / 180 * pi
        R = self.myRPY2R_robot(thetaX, thetaY, thetaZ)
        t = np.array([[x/1000], [y/1000], [z/1000]])
        RT1 = np.column_stack([R, t])  # 列合并
        RT1 = np.row_stack((RT1, np.array([0, 0, 0, 1])))
        return RT1


    def inverse_kinematics(self, end_effector_pose):
        """
        逆运动学求解，计算末端执行器的逆解

        参数:
        end_effector_pose (list): 末端执行器的位姿（位置 + 欧拉角）

        返回:
        list: 有效的关节角度解集
        """
         # 初始化dh参数
        [a1, alpha1, d1, theta1, offset1] = self.dh_parameters[0]
        [a2, alpha2, d2, theta2, offset2] = self.dh_parameters[1]
        [a3, alpha3, d3, theta3, offset3] = self.dh_parameters[2]
        [a4, alpha4, d4, theta4, offset4] = self.dh_parameters[3]
        [a5, alpha5, d5, theta5, offset5] = self.dh_parameters[4]
        [a6, alpha6, d6, theta6, offset6] = self.dh_parameters[5]

        theta = np.zeros((8, 6))  # 八组解，每组解六个角

        RT = self.pose_robot(end_effector_pose[0], end_effector_pose[1], end_effector_pose[2], 
                        end_effector_pose[3], end_effector_pose[4], end_effector_pose[5])

        [Nx, Ox, Ax, Px] = RT[0,:]
        [Ny, Oy, Ay, Py] = RT[1,:]
        [Nz, Oz, Az, Pz] = RT[2,:]
        
        # 求解theta1, 有两个解
        M = d6*Ay - Py
        N = d6*Ax - Px

        # 返回弧度值  
        theta_1 = atan2(M, N) - atan2(d4, sqrt(M*M + N*N - d4*d4))
        theta_1_2 = atan2(M, N) - atan2(d4, -sqrt(M*M + N*N - d4*d4))

        # 赋给一到四组
        theta[0:4, 0] = theta_1
        theta[4:8, 0] = theta_1_2

        # 计算theta5, 有四个解
        for i in range(4):
            S1 = sin(theta[i*2, 0])
            C1 = cos(theta[i*2, 0])
            theta_5 = acos(Ax*S1 - Ay*C1)
            theta_5_2 = -acos(Ax*S1 - Ay*C1)
            theta[i*2, 4] = theta_5
            theta[i*2+1, 4] = theta_5_2
        
        # 计算theta6, 有四组解
        for i in range(8):
            S1 = sin(theta[i, 0])
            C1 = cos(theta[i, 0])
            M = (Nx*S1 - C1*Ny)  
            N = (Ox*S1 - C1*Oy)
            S5 = sin(theta[i, 4])
            # print(np.float16(M*M + N*N - S5*S5))
            theta_6 = atan2(M, N) - atan2(S5, 0) 

            theta[i, 5] = theta_6

        # 计算theta2
        for i in range(0, 8):
            if i == 2 or i ==3:
                continue
            if i == 6 or i ==7:
                break
            S1 = sin(theta[i, 0])
            C1 = cos(theta[i, 0])
            S6 = sin(theta[i, 5])
            C6 = cos(theta[i, 5])

            M = d5*(C6*(C1*Ox + Oy*S1) + S6*(C1*Nx + Ny*S1)) -d6*(Ax*C1 + Ay*S1) + C1*Px + Py*S1
            N = d5*(C6*Oz + Nz*S6) - d6*Az + Pz - d1
            A = 2*M*a2 
            B = -2*N*a2 
            C = M*M + N*N + a2*a2 - a3*a3
            D = np.float16(A*A + B*B - C*C)
            # print(D)
            if D < 0: D = 0
            theta_2 = atan2(A, B) - atan2(C, sqrt(D))
            theta_2_2 = atan2(A, B) - atan2(C, -sqrt(D))
            theta[i, 1] = theta_2
            theta[i + 2, 1] = theta_2_2

        # 计算theta3、theta4
        for i in range(0, 8):
            S1 = sin(theta[i, 0])
            C1 = cos(theta[i, 0])
            S5 = sin(theta[i, 4])
            C5 = cos(theta[i, 4])
            S6 = sin(theta[i, 5])
            C6 = cos(theta[i, 5])

            S234 = C5*(C6*Nz - Oz*S6) - Az*S5
            C234 = C6*Oz + Nz*S6
            theta_234 = atan2(S234, C234)
            # print(theta_234, theta_234*180/pi)

            M = d5*(C6*(C1*Ox + Oy*S1) + S6*(C1*Nx + Ny*S1)) -d6*(Ax*C1 + Ay*S1) + C1*Px + Py*S1
            N = d5*(C6*Oz + Nz*S6) - d6*Az + Pz - d1

            S2 = sin(theta[i, 1])
            C2 = cos(theta[i, 1])

            S23 = (N - a2*S2)/a3  
            C23 = (M - a2*C2)/a3  
            theta_23 = atan2(S23, C23)

            theta[i, 2] = theta_23 - theta[i, 1]

            theta[i, 3] = theta_234 - theta_23

        # offset theta偏移量修正
        for i in range(8):
            for j in range(6):
                theta[i, j] = theta[i, j] - self.dh_parameters[j][4]

        # 输出
        # for i in range(8):
        #     print("第{}组解：".format(i + 1))
        #     for j in range(6):
        #         # print(theta[i, j] * 180 / pi)
        #         print("theta{} = {:.2f}".format(j + 1, theta[i, j] * 180 / np.pi), end="  ")
        #     print()
        #     print()
            
        # 弧度(radian)转换为角度(angle), 保留2位小数
        theta_angle = np.around(theta * 180 / pi, 2)
        # 角度修正, 并记录角度满足机械臂每个关节的执行范围
        valid_theta_angle = []
        valid_flag = 1 # 记录每组解是否合理
        for i in range(8):
            valid_flag = 1
            for j in range(6):
                if theta_angle[i, j] > self.motion_range[j, 1] or theta_angle[i, j] < self.motion_range[j, 0]:
                    if theta_angle[i, j] < 0:
                        theta_angle[i, j] = theta_angle[i, j] + 360 #角度加上或减去2*pi其cos和sin值都不会变化
                    elif theta_angle[i, j] > 0:
                        theta_angle[i, j] = theta_angle[i, j] - 360 
                if theta_angle[i, j] < self.motion_range[j, 1] and theta_angle[i, j] > self.motion_range[j, 0]:
                    continue
                else:
                    valid_flag = 0
            if valid_flag == 1:
                valid_theta_angle.append(theta_angle[i])

        return valid_theta_angle


    def _dh_transform(self, a, alpha, d, theta):
        """
        计算 D-H 变换矩阵

        输入参数:
        a (float): 连杆长度
        alpha (float): 连杆扭角
        d (float): 连杆偏移
        theta (float): 关节角度

        返回:
        np.ndarray: 4x4 D-H 变换矩阵
        """
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ], dtype=np.float16)

    def compute_end_effector_transform(self, joint_angles):
        """
        计算末端执行器相对于基坐标系的变换矩阵

        输入参数:
        joint_angles (list or np.ndarray): 关节角度（单位：弧度）

        返回:
        np.ndarray: 4x4 末端执行器相对于基坐标系的变换矩阵
        """
        assert len(joint_angles) == len(self.dh_parameters), "关节角度的数量应与 D-H 参数的数量匹配"
        
        T = np.eye(4, dtype=np.float16)
        for i in range(len(joint_angles)):
            a, alpha, d, theta, offset = self.dh_parameters[i]
            theta += joint_angles[i]  # 关节角度
            T_i = self._dh_transform(a, alpha, d, theta)
            T = T @ T_i
        
        return T

    def get_end_effector_to_base_transform(self, joint_angles):
        """
        获取末端执行器到基坐标系的变换矩阵

        输入参数:
        joint_angles (list or np.ndarray): 关节角度（单位：弧度）

        返回:
        np.ndarray: 4x4 末端执行器到基坐标系的变换矩阵
        """
        # 计算末端执行器相对于基坐标系的变换矩阵
        T_base_to_end_effector = self.compute_end_effector_transform(joint_angles)
        return T_base_to_end_effector

if __name__ == "__main__":

    robot = KinematicsSolution('cobot/dh_parameters.yaml')

    joint_angles = [-132.89, -29.35, 4.92, -61.52, -1.23, 45.96]  # 实际关节角度
    # robot_real_pose = [-1.28, -20.35, 133.44, -163.09, 6.57, 0.7]   # 位姿
    robot_real_pose = [-8.51, 48.63, 188.0, -91.4, 0.25, -89.74]   # 位姿
    inverse_solution_results = robot.inverse_kinematics(robot_real_pose)
    if inverse_solution_results:
        print("有效的关节角度解:")
        for i, solution in enumerate(inverse_solution_results):
            formatted_solution = np.round(solution, 2)        # 保留两位小数
            print(f"解 {i+1}:", formatted_solution.tolist())  # 转换为列表并打印
            
        # 验证正运动学结果
        positive_solution_pose = robot.forward_kinematics(inverse_solution_results[0])
        print("对应的正运动学位姿:\n", np.round(positive_solution_pose, 2).tolist())  # 保留两位小数并转换为列表

    else:
        print("没有找到有效的解")

