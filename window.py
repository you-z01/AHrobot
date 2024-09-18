import sys
import cv2
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap

from cobot.kinematics_solution import *
from ui.qt_utils import *
from detection import *

class WorkerThread(QThread):
    """系统启动类"""
    log_signal = pyqtSignal(str)
    initialized_signal = pyqtSignal(object, object, object, object)  # 传递初始化完成后的对象

    def run(self):
        self.log_signal.emit("[INFO] 开始启动RealSense相机")
        realsense_camera = RealSenseCamera()

        self.log_signal.emit("[INFO] 完成realsense相机启动")
        self.log_signal.emit("[INFO] 开始加载YoloV5模型")
        yolo_model = YoloV5(yolov5_yaml_path='config/strawberry_maturity.yaml')
        self.log_signal.emit("[INFO] 完成YoloV5模型加载")

        self.log_signal.emit("[INFO] 开始初始化TCP通信")
        client = TCPClient('192.168.137.238', 9000)
        client.connect()
        self.log_signal.emit("[INFO] 完成TCP初始化")

        self.log_signal.emit("[INFO] 开始初始化机械臂运动学解类")
        kinematics_solution = KinematicsSolution('cobot/dh_parameters.yaml')
        self.log_signal.emit("[INFO] 完成机械臂运动学解类初始化")

        # 将初始化的对象发送回主线程
        self.initialized_signal.emit(realsense_camera, yolo_model, client, kinematics_solution)


class MainWindow(QMainWindow):
    """"主窗口类"""
    def __init__(self):
        self.camera_xyz = []         # 相机坐标系下的xyz 一维列表
        self.camera_xyz_mm = []      # 相机坐标系下的xyz(mm) 一维列表
        self.target_xyz = []         # 要抓取的目标坐标
        self.target_xyzRxyz = []     # 相机坐标系下的xyzRxyz 一维列表
        self.target_robot = []       # 机械臂坐标系下的目标坐标
        self.target_rotation = [-87.63, -0.71, 5.77] # 末端固定位姿 rx ry rz

        super().__init__()
        uic.loadUi("ui/main.ui", self)  # 动态加载ui文件

        self.realsense_camera = None
        self.yolo_model = None
        self.client = None
        self.kinematics_solution = None

        # 连接按钮的点击事件处理函数
        self.pushButton.clicked.connect(self.quit_app)
        self.pushButton_2.clicked.connect(lambda: self.data_handing(1))     # 发送[1]查询变换, 计算关节解
        self.pushButton_3.clicked.connect(lambda: self.data_handing(2))     # 查询相机中目标坐标
        self.pushButton_4.clicked.connect(lambda: self.data_handing(3))     # 回到零点
        self.pushButton_5.clicked.connect(lambda: self.data_handing(4))     # 发送相机中目标坐标
        self.pushButton_9.clicked.connect(lambda: self.data_handing(5))     # 获取变换并计算逆运动学解

        self.pushButton_6.clicked.connect(lambda: self.send_data())     # 发送消息
        self.pushButton_7.clicked.connect(lambda: self.start_sys())     # 启动系统
        self.pushButton_8.clicked.connect(lambda: self.clear_log())     # 清除日志

    def clear_log(self):
        self.textBrowser.clear()  # 清除之前的日志信息

    def start_sys(self):
        # 启动后台线程来处理长时间操作
        self.append_log_msg("[INFO] 系统启动中...")
        self.worker_thread = WorkerThread()
        self.worker_thread.log_signal.connect(self.append_log_msg)
        self.worker_thread.initialized_signal.connect(self.initialize_components)
        self.worker_thread.start()

    def initialize_components(self, realsense_camera, yolo_model, client, kinematics_solution):
        self.realsense_camera = realsense_camera
        self.yolo_model = yolo_model
        self.client = client
        self.kinematics_solution = kinematics_solution

        # 开始捕获图像
        self.append_log_msg("[INFO] 开始捕获图像")
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)


    def quit_app(self):
        # 定义退出按钮的功能
        print("退出程序")
        sys.exit()

    def send_data(self):
        try:
            
            text = self.textEdit.toPlainText()
            text_no_space = text.replace(' ', '')
            text_no_comma = text_no_space.replace('，', ',')
            if text_no_comma[0] == '6' and text_no_comma[-1] == '9':
                formatted_data = f"[{text_no_comma}]"
                self.append_log_msg(f"[INFO] 格式化后的数据: {formatted_data}")
            else:
                self.append_log_msg("[WARN] 输入不符合校验规则, 首位必须为6, 末位为9")
            self.append_log_msg(self.client.send_message(str(formatted_data))) # 发送坐标信息到服务器
            self.append_log_msg(self.client.receive_message())  # 接收回显数据
        except Exception as e:
            self.append_log_msg(f"[ERROR] send_data()发生异常: {e}")

    def data_handing(self, data):
        try:
            if data == 1:
                self.append_log_msg("[INFO] 发送数据到服务器，计算关节解")
                self.append_log_msg(self.client.send_message(str(self.target_xyzRxyz))) # 发送坐标信息到服务器
                time.sleep(1) 

                self.append_log_msg(self.client.send_message("[1]")) # 发送查询指令到服务器
                recive_data = self.client.receive_message()  # 接收到的数据
                self.append_log_msg(f"[INFO] 接收到的数据为：{recive_data}")

                # 尝试解析接收到的数据
                par_data = parse_data(recive_data)
                if par_data is None:
                    self.append_log_msg("[WARN] 解析失败，无法处理数据")
                    return
                if len(par_data) < 3:
                    self.append_log_msg("[WARN] 数据长度不足，无法提取前三个坐标值")
                    return
                # 将字符串转换为列表
                # recive_data_list = ast.literal_eval(recive_data)
                
                # 提取前三位数据
                xyz_data_m = par_data[:3]

                # 将米转换为毫米，并保留两位小数
                xyz_data_mm = [round(coord * 1000, 2) for coord in xyz_data_m]
                self.append_log_msg(f"[INFO] xyz_data_mm为：{xyz_data_mm}")
                # 合并为新的列表
                target_list = xyz_data_mm + self.target_rotation
                self.append_log_msg(f"[INFO] 转换后的坐标和旋转列表为：{target_list}")

                IS_results = self.kinematics_solution.inverse_kinematics(target_list)    # 逆运动学解
                if IS_results:
                    self.append_log_msg("[INFO] 有效的关节角度解:")
                    for i, solution in enumerate(IS_results):
                        formatted_solution = np.round(solution, 2)        # 保留两位小数
                        self.append_log_msg(f"解 {i+1}:{formatted_solution.tolist()}" )  # 转换为列表并打印
                        # self.append_log_msg(f"解 {i+1}:", formatted_solution.tolist())  # 转换为列表并打印
            elif data == 2:
                self.append_log_msg(f"[INFO] 相机中目标坐标(m):{self.camera_xyz}")
                self.append_log_msg(f"[INFO] 相机中目标坐标(mm):{self.camera_xyz_mm}")

            elif data == 3:
                self.append_log_msg("[INFO] 机械臂回到零点")
                self.append_log_msg(self.client.send_message("[6, 20, 0, 0, 0, 0, 0, 0, 9]"))

            elif data == 4:
                self.append_log_msg("[INFO] 发送相机中目标坐标到服务器")
                self.append_log_msg(self.client.send_message(str(self.target_xyzRxyz))) # 发送坐标信息到服务器

            elif data == 5:
                self.append_log_msg("[INFO] 获取到的joint1和target的变换数据")
                self.append_log_msg(self.client.send_message("[1]")) # 发送查询指令到服务器
                recive_data = self.client.receive_message()  # 接收到的数据
                self.append_log_msg(f"[INFO] 接收到的joint1和target的变换数据为:{recive_data}")
                self.append_log_msg("[INFO] 开始解析变换数据")

                par_data = parse_data(recive_data)
                if par_data is None:
                    self.append_log_msg("[WARN] 解析失败，无法处理数据")
                    return
                if len(par_data) < 3:
                    self.append_log_msg("[WARN] 数据长度不足，无法提取前三个坐标值")
                    return

                # 提取前三位数据
                xyz_data_m = par_data[:3]

                # 将米转换为毫米，并保留两位小数
                xyz_data_mm = [round(coord * 1000, 2) for coord in xyz_data_m]
                self.append_log_msg(f"[INFO] 目标在joint1下的xyz_mm为:{xyz_data_mm}")
                # 合并为新的列表
                target_list = xyz_data_mm + self.target_rotation
                self.append_log_msg(f"[INFO] 转换后的[x,y,z,rx,ry,rz]为:{target_list}")
                self.append_log_msg("[INFO] 开始计算逆运动学解")
                IS_results = self.kinematics_solution.inverse_kinematics(target_list)    # 逆运动学解
                if IS_results:
                    self.append_log_msg("[INFO] 有效的关节角度解:")
                    for i, solution in enumerate(IS_results):
                        formatted_solution = np.round(solution, 2)        # 保留两位小数
                        self.append_log_msg(f"解 {i+1}:", formatted_solution.tolist())  # 转换为列表并打印

        except Exception as e:
            self.append_log_msg(f"[ERROR] data_handing()发生异常: {e}")


    def append_log_msg(self, message):
    # 根据消息内容设置颜色
        if message.startswith("[ERROR]"):
            color = "red"
        elif message.startswith("[WARN]"):
            color = "orange"
        elif message.startswith("[INFO]"):
            color = "black"
        else:
            color = "black"  # 默认颜色

        # 使用 HTML 标签设置颜色
        formatted_message = f'<span style="color:{color};">{message}</span>'
        self.textBrowser.append(formatted_message)

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

    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()

