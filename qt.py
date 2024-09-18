import sys
import cv2
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
from detection import *
from cobot.socket_client import *
from cobot.kinematics_solution import *


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


class MainWindow(QMainWindow):
    def __init__(self):

        self.camera_xyz = []         # 相机坐标系下的xyz 一维列表
        self.camera_xyz_mm = []      # 相机坐标系下的xyz(mm) 一维列表
        self.target_xyz = []         # 要抓取的目标坐标
        self.target_xyzRxyz = []     # 相机坐标系下的xyzRxyz 一维列表
        self.target_robot = []       # 机械臂坐标系下的目标坐标
        self.target_rotation = [-87.63, -0.71, 5.77] # 末端固定位姿 rx ry rz

        super().__init__()
        uic.loadUi("ui/main.ui", self)  # 动态加载ui文件

        # # 创建一个定时器来定期捕获摄像头的图像
        # self.timer = QTimer()
        # self.timer.timeout.connect(self.update_frame)
        # self.timer.start(30)  # 每 30 毫秒更新一次图像
        
        # 连接按钮的点击事件处理函数
        self.pushButton.clicked.connect(self.quit_app)
        self.pushButton_2.clicked.connect(lambda: self.data_handing(1))     # 发送[1], 计算关节解
        self.pushButton_3.clicked.connect(lambda: self.data_handing(2))     # 查询相机中目标坐标
        self.pushButton_4.clicked.connect(lambda: self.data_handing(3))         
        self.pushButton_5.clicked.connect(lambda: self.data_handing(4)) 
        self.pushButton_6.clicked.connect(lambda: self.send_data())
        self.pushButton_7.clicked.connect(lambda: self.start_sys())

    def start_sys(self):
        self.textBrowser.append("[INFO] 开始启动RealSense相机")
        self.realsense_camera = RealSenseCamera()
        # 创建一个定时器来定期捕获摄像头的图像
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 每 30 毫秒更新一次图像
        self.textBrowser.append("[INFO] 完成realsense相机启动")

        self.textBrowser.append("[INFO] 开始加载YoloV5模型")
        self.yolo_model = YoloV5(yolov5_yaml_path='config/strawberry_maturity.yaml')
        self.textBrowser.append("[INFO] 完成YoloV5模型加载")

        # 初始化TCP
        self.textBrowser.append("[INFO] 开始初始化TCP通信")
        self.client = TCPClient('192.168.137.238', 9000)
        self.client.connect()
        self.textBrowser.append("[INFO] 完成TCP初始化")

        self.textBrowser.append("[INFO] 开始初始化机械臂运动学解类")
        # 初始化机械臂运动学解类
        self.kinematics_solution = KinematicsSolution('cobot/dh_parameters.yaml')
        self.textBrowser.append("[INFO] 完成机械臂运动学解类初始化")


    def quit_app(self):
        # 定义退出按钮的功能
        print("退出程序")
        sys.exit()

    def send_data(self):
        text = self.textEdit.toPlainText()
        text_no_space = text.replace(' ', '')
        text_no_comma = text_no_space.replace('，', ',')
        if text_no_comma[0] == '6' and text_no_comma[-1] == '9':
            formatted_data = f"[{text_no_comma}]"
            self.textBrowser.append(f"格式化后的数据: {formatted_data}")
        else:
            self.textBrowser.append("输入不符合校验规则, 首位必须为6, 末位为9")
        self.client.send_message(str(formatted_data)) # 发送坐标信息到服务器
        self.client.receive_message()  # 接收回显数据
        

    def data_handing(self, data):
        if data == 1:
            self.textBrowser.append("发送数据到服务器，计算关节解")
            self.client.send_message(str(self.target_xyzRxyz)) # 发送坐标信息到服务器
            time.sleep(1) 

            self.client.send_message("[1]") # 发送查询指令到服务器
            recive_data = self.client.receive_message()  # 接收到的数据

            # 尝试解析接收到的数据
            par_data = parse_data(recive_data)
            if par_data is None:
                self.textBrowser.append("解析失败，无法处理数据")
                return
            if len(par_data) < 3:
                self.textBrowser.append("数据长度不足，无法提取前三个坐标值")
                return
            # 将字符串转换为列表
            # recive_data_list = ast.literal_eval(recive_data)

            # 提取前三位数据
            xyz_data_m = par_data[:3]

            # 将米转换为毫米，并保留两位小数
            xyz_data_mm = [round(coord * 1000, 2) for coord in xyz_data_m]
            self.textBrowser.append(f"xyz_data_mm为：{xyz_data_mm}")
            # 合并为新的列表
            target_list = xyz_data_mm + self.target_rotation
            self.textBrowser.append(f"转换后的坐标和旋转列表为：{target_list}")

            IS_results = self.kinematics_solution.inverse_kinematics(target_list)    # 逆运动学解
            if IS_results:
                self.textBrowser.append("有效的关节角度解:")
                for i, solution in enumerate(IS_results):
                    formatted_solution = np.round(solution, 2)        # 保留两位小数
                    self.textBrowser.append(f"解 {i+1}:", formatted_solution.tolist())  # 转换为列表并打印
        
        elif data == 2:
            self.textBrowser.append(f"相机中目标坐标(m):{self.camera_xyz}")
            self.textBrowser.append(f"相机中目标坐标(mm):{self.camera_xyz_mm}")

        elif data == 3:
            self.textBrowser.append("机械臂回到零点")
            self.client.send_message("[0, 0, 0, 0, 0, 0, 0, 9]")

        elif data == 4:
            self.textBrowser.append("机械臂到指定点")
            self.client.send_message("[0, 89.92, -59.97, -12.0, 74.3, 5.9, -0.47, 9]")

    def update_frame(self):

        intr, depth_intrin, color_image, depth_image, aligned_depth_frame = self.realsense_camera.get_aligned_images()
        if not depth_image.any() or not color_image.any():
            return

        t_start = time.time()  # 开始计时
        # 执行目标检测
        canvas, class_id_list, xyxy_list, conf_list = self.yolo_model.detect(color_image)
        t_end = time.time()  # 结束计时
        
        # 计算相机坐标系下的xyz
        self.camera_xyz, self.camera_xyz_mm, canvas = camera_coordinate(xyxy_list,aligned_depth_frame, depth_intrin,canvas, t_start)

        # 计算目标坐标，合并为要发送的相机坐标系下的目标坐标
        if self.camera_xyz:
            self.target_xyz = self.camera_xyz[0]
            self.target_xyzRxyz = self.target_xyz + self.target_rotation
        # print(f"相机中目标坐标(m):{self.camera_xyz}")


        # 将图像转换为QImage以便在Qt界面中显示
        rgb_image = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        # self.label.setPixmap(QPixmap.fromImage(qt_image))

         # 将图像转换为QPixmap，并保持比例缩放和居中显示
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        # 设置图像居中并显示在label中
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setPixmap(scaled_pixmap)
        


if __name__ == "__main__":

    # print("[INFO] 启动RealSense相机与YoloV5目标检测")

    # # 初始化相机和模型
    # realsense_camera = RealSenseCamera()
    # print("[INFO] 完成realsense相机启动")

    # yolo_model = YoloV5(yolov5_yaml_path='config/strawberry_maturity.yaml')
    # print("[INFO] 完成YoloV5模型加载")

    # # 初始化TCP
    # client = TCPClient('192.168.137.238', 9000)
    # client.connect()

    # # 初始化机械臂运动学解类
    # kinematics_solution = KinematicsSolution('cobot/dh_parameters.yaml')

    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()

