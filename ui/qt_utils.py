import socket
# import ast
# import json
import time
import cv2
import pyrealsense2 as rs
import numpy as np
import re

class TCPClient:
    def __init__(self, server_ip, server_port):
        """初始化客户端并设置服务器 IP 和端口"""
        self.server_ip = server_ip
        self.server_port = server_port
        self.client_socket = None

    def connect(self):
        """连接到服务器"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.server_ip, self.server_port))
            # print(f"成功连接到服务器 {self.server_ip}:{self.server_port}")
        except Exception as e:
            self.client_socket = None
            print(f"连接错误: {e}")
            return f"连接错误: {e}"

    def send_message(self, data):
        """发送数据到服务器"""
        if self.client_socket:
            try:
                message = data.encode('utf-8')  # 转换为字节格式
                self.client_socket.sendall(message)
                print(f"已发送数据: {data}")
                return f"已发送数据: {data}"
            except Exception as e:
                print(f"发送数据时发生错误: {e}")
                return f"发送数据时发生错误: {e}"
        else:
            print("发送消息-尚未连接到服务器")
            return "发送消息-尚未连接到服务器"


    def receive_message(self, buffer_size=1024):
        """从服务器接收数据"""
        if self.client_socket:
            try:
                data = self.client_socket.recv(buffer_size)
                decoded_data = data.decode('utf-8')  # 将接收到的字节数据解码为字符串
                print(f"收到数据: {decoded_data}")
                return f"收到数据: {decoded_data}"
            
            except Exception as e:
                print(f"接收数据时发生错误: {e}")
                return f"接收数据时发生错误: {e}"
        else:
            print("接收消息-尚未连接到服务器")
            return "接收消息-尚未连接到服务器"

    def close(self):
        """关闭客户端连接"""
        if self.client_socket:
            self.client_socket.close()
            print("已关闭与服务器的连接")
            return "已关闭与服务器的连接"
        else:
            print("客户端尚未连接")
            return "客户端尚未连接"


# def parse_data(data_str):
#     """
#     通用数据解析函数, 能够处理JSON、列表格式的数据。
    
#     参数:
#     data_str (str): 待解析的数据字符串。
    
#     返回:
#     list: 解析后的数据列表。如果解析失败, 返回None。
#     """
#     # 去除前后的空白符或无效字符
#     data_str = data_str.strip()
    
#     # 如果数据字符串为空，返回None
#     if not data_str:
#         print("收到的数据为空")
#         return "收到的数据为空"

#     # 尝试解析为JSON格式
#     try:
#         return json.loads(data_str)
#     except json.JSONDecodeError:
#         pass  # 如果失败，继续尝试其他方式

#     # 尝试解析为Python列表
#     try:
#         return ast.literal_eval(data_str)
#     except (ValueError, SyntaxError):
#         pass  # 如果失败，继续

#     # 如果所有尝试都失败，返回None并提示错误
#     print(f"无法解析数据：{data_str}")
#     return f"无法解析数据：{data_str}"

def parse_data(data_str):
    """
    从类似于 '收到数据: [x, y, z, rx, ry, rz, ...]' 的字符串中解析出数值列表。
    
    参数:
    data_str (str): 包含数值数据的字符串。
    
    返回:
    list: 解析后的数据列表。如果解析失败, 返回None。
    """
    # 正则表达式用于提取方括号中的数值部分
    pattern = r'\[([^\]]+)\]'
    match = re.search(pattern, data_str)
    
    if match:
        # 提取匹配到的数值部分，并分割成列表
        data_list = match.group(1).split(',')
        # 将字符串转换为浮点数
        return [float(i) for i in data_list]
    else:
        print(f"无法解析数据：{data_str}")
        return f"无法解析数据：{data_str}"
    


def camera_coordinate(xyxy_list,aligned_depth_frame, depth_intrin,canvas, t_start):
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
    # cv2.imshow('ObjectDetection', canvas)

    return camera_xyz_list, camera_xyz_list_mm, canvas
