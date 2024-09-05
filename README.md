## AHrobot(Automatic harvesting robot)
**使用realsense d435i相机，基于yolov5实现多阶段检测，返回检测目标相机坐标系下的位置信息**
**由myCobot 280 Jetson Nano完成自动收获**


### 1.Environment：

1. 可以运行YOLOv5的python环境

```bash
pip install -r requirements.txt
```

2. realsense相机和pyrealsense2库

```bash
pip install pyrealsense2
```

3. mycobot库pymycobot安装
```bash
git clone https://github.com/elephantrobotics/pymycobot.git <your-path>   
#其中<your-path>填写你的安装地址，不填默认在当前路径

cd <your-path>/pymycobot    
#进入到下载包的pymycobot文件夹

#根据你的python版本运行下面其一命令    
# Install
 python2 setup.py install    
# or
 python3 setup.py install
```

**在下面两个环境中测试成功**
- **win10** python 3.8 Pytorch 1.8 CUDA 11.0  NVIDIA GeForce GTX 1060
- **ubuntu20.04**  python 3.6 Pytorch 1.7.1+GPU CUDA 10.2 


### 2.Camera config：

分辨率只能改特定的参数(可在SDK中查看)，d435i可以用 1280x720, 640x480, 848x480。

```python
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
```

### 3.code return xyz：
下方代码实现从像素坐标系到相机坐标系转换，并且标注中心点以及三维坐标信息。
```python
for i in range(len(xyxy_list)):
    ux = int((xyxy_list[i][0]+xyxy_list[i][2])/2)  # 计算像素坐标系的x
    uy = int((xyxy_list[i][1]+xyxy_list[i][3])/2)  # 计算像素坐标系的y
    dis = aligned_depth_frame.get_distance(ux, uy)  
    camera_xyz = rs.rs2_deproject_pixel_to_point(
    depth_intrin, (ux, uy), dis)  # 计算相机坐标系xyz
    camera_xyz = np.round(np.array(camera_xyz), 3)  # 转成3位小数
    camera_xyz = camera_xyz.tolist()
    cv2.circle(canvas, (ux,uy), 4, (255, 255, 255), 5)#标出中心点
    cv2.putText(canvas, str(camera_xyz), (ux+20, uy+10), 0, 1,
                                [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)#标出坐标
    camera_xyz_list.append(camera_xyz)
    #print(camera_xyz_list)
```