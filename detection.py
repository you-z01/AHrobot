import random
import torch.backends.cudnn as cudnn
import torch
import pyrealsense2 as rs
import numpy as np
import cv2
import time
from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.torch_utils import select_device, time_sync
import yaml

class RealSenseCamera:
    def __init__(self):
        '''初始化RealSense相机'''
        self.pipeline = rs.pipeline()  # 定义pipeline
        self.config = rs.config()  # 定义配置config
        # 设置深度和彩色流的分辨率和帧率
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)  # 启动相机数据流
        self.align_to = rs.stream.color  # 设置与彩色流对齐
        self.align = rs.align(self.align_to)  # 创建对齐对象

    def get_aligned_images(self):
        '''获取对齐的彩色图像和深度图像'''
        frames = self.pipeline.wait_for_frames()  # 等待获取图像帧
        aligned_frames = self.align.process(frames)  # 获取对齐后的帧
        aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取深度帧
        color_frame = aligned_frames.get_color_frame()  # 获取彩色帧

        if not aligned_depth_frame or not color_frame:
            return None, None, None, None, None

        # 获取相机内参
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

        # 将深度图和彩色图转换为NumPy数组
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

    def stop(self):
        '''停止相机数据流'''
        self.pipeline.stop()


class YoloV5:
    def __init__(self, yolov5_yaml_path='config/yolov5s.yaml'):
        '''初始化YoloV5模型'''
        # 载入YOLOv5的配置文件
        with open(yolov5_yaml_path, 'r', encoding='utf-8') as f:
            self.yolov5 = yaml.safe_load(f.read())
        
        # 随机生成每个类别的颜色
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in range(self.yolov5['class_num'])]
        
        # 初始化模型
        self.init_model()

    @torch.no_grad()
    def init_model(self):
        '''模型初始化'''
        device = select_device(self.yolov5['device'])  # 选择计算设备
        is_half = device.type != 'cpu'  # 如果不是CPU则使用半精度
        model = attempt_load(self.yolov5['weight'], map_location=device)  # 加载模型
        input_size = check_img_size(self.yolov5['input_size'], s=model.stride.max())  # 检查输入图像大小
        
        if is_half:
            model.half()  # 使用半精度

        cudnn.benchmark = True  # 提升推理速度
        img_torch = torch.zeros((1, 3, input_size, input_size), device=device)  # 初始化图像输入
        _ = model(img_torch.half() if is_half else img_torch)  # 模型预热

        self.is_half = is_half
        self.device = device
        self.model = model

    def preprocessing(self, img):
        '''图像预处理'''
        img_resize = letterbox(img, new_shape=(self.yolov5['input_size'], self.yolov5['input_size']), auto=False)[0]
        img_arr = np.stack([img_resize], 0)  # 增加批次维度
        img_arr = img_arr[:, :, :, ::-1].transpose(0, 3, 1, 2)  # BGR转RGB并转换维度
        img_arr = np.ascontiguousarray(img_arr)  # 转换为连续数组
        return img_arr

    @torch.no_grad()
    def detect(self, img, view_img=True):
        '''执行目标检测'''
        img_resize = self.preprocessing(img)  # 图像缩放处理
        img_torch = torch.from_numpy(img_resize).to(self.device)  # 转为Torch张量
        img_torch = img_torch.half() if self.is_half else img_torch.float()  # 转为半精度或浮点数
        img_torch /= 255.0  # 归一化

        if img_torch.ndimension() == 3:
            img_torch = img_torch.unsqueeze(0)

        # 模型推理
        pred = self.model(img_torch)[0]
        pred = non_max_suppression(pred, self.yolov5['threshold']['confidence'], self.yolov5['threshold']['iou'])

        det = pred[0]
        if view_img:
            canvas = np.copy(img)  # 拷贝图像，用于绘制
        else:
            canvas = None

        xyxy_list = []
        conf_list = []
        class_id_list = []
        if det is not None and len(det):
            # 画面中存在目标对象
            # 将坐标信息恢复到原始图像的尺寸
            det[:, :4] = scale_coords(img_resize.shape[2:], det[:, :4], img.shape).round()
            for *xyxy, conf, class_id in reversed(det):
                class_id = int(class_id)
                xyxy_list.append(xyxy)
                conf_list.append(conf)
                class_id_list.append(class_id)
                if view_img:
                    # 绘制矩形框与标签
                    label = '%s %.2f' % (
                        self.yolov5['class_name'][class_id], conf)
                    self.plot_one_box(
                        xyxy, canvas, label=label, color=self.colors[class_id], line_thickness=3)
        return canvas, class_id_list, xyxy_list, conf_list
        #     for *xyxy, conf, class_id in reversed(det):
        #         label = f"{self.yolov5['class_name'][int(class_id)]} {conf:.2f}"
        #         self.plot_one_box(xyxy, canvas, label=label, color=self.colors[int(class_id)], line_thickness=3)

        # return canvas

    def plot_one_box(self, xyxy, img, color=None, label=None, line_thickness=None):
        '''绘制目标检测框及标签'''
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)

        if label:
            tf = max(tl - 1, 1)
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

