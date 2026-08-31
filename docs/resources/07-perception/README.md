# 07 Perception

收录机器人视觉、相机标定、深度数据、激光雷达、点云和传感器融合资料。

## Official documentation and tutorials

| 资源 | 提供方 | 主题 | 语言 | 难度 | 状态 | 最后检查 |
| --- | --- | --- | --- | --- | --- | --- |
| [OpenCV Tutorials](https://docs.opencv.org/5.0/tutorials/tutorials.html) | OpenCV project | 图像处理、特征、标定、检测和 DNN | English | Beginner–Advanced | Source Checked | 2026-08-31 |
| [Camera Calibration with OpenCV](https://docs.opencv.org/5.0/tutorials/calib3d/camera_calibration/camera_calibration.html) | OpenCV project | 相机内参、畸变和重投影误差 | English | Intermediate | Source Checked | 2026-08-31 |
| [Point Cloud Library Tutorials](https://pointclouds.org/documentation/tutorials/) | PCL project | 点云 I/O、滤波、特征、分割、配准和表面 | English | Intermediate–Advanced | Source Checked | 2026-08-31 |
| [Open3D Documentation](https://www.open3d.org/docs/latest/) | Open3D project | 点云、RGB-D、配准、重建和可视化 | English | Beginner–Advanced | Source Checked | 2026-08-31 |
| [Open3D Geometry Tutorials](https://www.open3d.org/docs/latest/tutorial/geometry/index.html) | Open3D project | 点云下采样、法线、聚类、平面和网格 | English | Beginner–Intermediate | Source Checked | 2026-08-31 |
| [ROS 2 image_pipeline](https://github.com/ros-perception/image_pipeline) | ROS Perception | 相机标定、图像校正、立体视觉和深度处理 | English | Intermediate | Source Checked | 2026-08-31 |
| [Intel RealSense SDK 2.0](https://dev.intelrealsense.com/docs) | Intel RealSense | 深度相机 SDK、示例和设备说明 | English | Beginner–Advanced | Source Checked | 2026-08-31 |
| [Isaac ROS Getting Started](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html) | NVIDIA | GPU 加速的 ROS 2 感知、VSLAM 和 DNN 管线 | English | Intermediate–Advanced | Source Checked | 2026-08-31 |

## Data-quality checklist

- 在算法调参前检查时间戳、坐标系、单位、帧率和丢帧。
- 保存原始数据与标定文件，并记录相机分辨率、曝光和传感器固件版本。
- 相机标定不只看“成功”提示，应检查每张图像覆盖范围和重投影误差。
- 点云效果异常时先确认坐标系与尺度，再判断是滤波、配准还是传感器噪声问题。

新增资源时使用 [统一模板](../RESOURCE_TEMPLATE.md)。
