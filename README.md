# 番茄视觉伺服抓取（JAKA Zu3 + RealSense + YOLO-pose）

基于 ROS1 catkin 工作空间的**视觉伺服抓取**实现：RealSense 深度相机采集彩色/深度图，YOLO 姿态模型检测番茄并估计其三维坐标，通过自定义话题把目标位姿传给 MoveIt，规划并驱动 JAKA Zu3 机械臂执行抓取。

## 数据流

```
RealSense D435(i)
   ├─ /camera/color/image_raw               ─┐
   └─ /camera/aligned_depth_to_color/image_raw ─┤
                                                ▼
                        biye_zuizhong_zuixin.py（dianyunceshi 包）
                        · YOLO-pose 推理（best.pt）
                        · 深度图反投影 → 相机系三维坐标
                        · 取最近目标，发布自定义消息
                                                │
                                                ▼  topic: best_tomato_pose  (tomato_poseGoal)
                        xx.py（moveit_exe 包）
                        · TF 查询当前末端位姿
                        · 手眼矩阵变换到基座系
                        · MoveIt 规划 + 执行（规划组 jaka_zu3）
                                                ▼
                                        JAKA Zu3 机械臂
```

## 目录结构

```
jaka_robot/
├── src/
│   ├── dianyunceshi/                     # ★ 自研：视觉检测与目标定位
│   │   ├── msg/posev.msg                 #   单目标检测结果（框 + 关键点 + 类别）
│   │   ├── msg/posevs.msg                #   目标数组
│   │   ├── action/tomato_pose.action     #   目标位姿 + 采摘方向
│   │   ├── scripts/biye_zuizhong_zuixin.py   # ★ 主程序（检测节点）
│   │   ├── scripts/segval.py             #   分割结果校验脚本
│   │   ├── scripts/ultralytics/          #   定制版 ultralytics（见下方说明）
│   │   └── scripts/zuizhong/weights/best.pt  # ★ 训练好的模型权重（43 MB）
│   ├── moveit_exe/                       # ★ 自研：机械臂执行端
│   │   └── scripts/xx.py                 # ★ MoveIt 规划与抓取执行
│   ├── jaka_robot_v2.2/src/              # JAKA 官方 ROS 包（驱动/消息/描述/MoveIt 配置）
│   └── JAKA_ROS_Demo/                    # JAKA 官方 ROS 示例说明书（PDF）
├── .gitignore
└── README.md
```

## 环境要求

| 项目 | 版本 / 说明 |
|---|---|
| ROS | Melodic 或 Noetic（原开发环境为 Ubuntu，工作空间路径 `/home/mzc/jaka_robot`） |
| Python | 3.6+ |
| 关键依赖 | `torch`、`open3d`、`scikit-learn`、`opencv-python`、`numpy`、`pyrealsense2` |
| ROS 依赖 | `cv_bridge`、`image_transport`、`moveit_commander`、`tf`、`sensor_msgs` |

## 依赖准备

本仓库只包含**自研代码与 JAKA 官方包**，以下第三方包需要自行获取后放进 `src/`：

```bash
# RealSense 驱动（也可直接 apt install ros-$ROS_DISTRO-realsense2-camera）
cd src && git clone https://github.com/IntelRealSense/realsense-ros.git

# ArUco 标记识别（标定用）
cd src && git clone https://github.com/pal-robotics/aruco_ros.git

# 手眼标定工具
cd src && git clone https://github.com/IFL-CAMP/easy_handeye.git
```

## 编译与运行

```bash
cd jaka_robot
catkin build            # 或 catkin_make
source devel/setup.bash

# 1) 启动相机（需对齐深度图到彩色图，节点订阅的是 aligned 话题）
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# 2) 启动视觉检测节点
rosrun dianyunceshi biye_zuizhong_zuixin.py

# 3) 启动 MoveIt 与执行节点
roslaunch jaka_zu3_moveit_config demo.launch
rosrun moveit_exe xx.py
```

## 需要注意的地方

1. **权重路径是绝对路径**。`biye_zuizhong_zuixin.py` 中写死为
   `/home/mzc/jaka_robot/src/dianyunceshi/scripts/zuizhong/weights/best.pt`，
   换机器请改为你自己的路径。
2. **相机内参硬编码在脚本里**。`TomatoDetectionNode.__init__` 与 `YOLOv10ROS.pixel_to_world`
   各有一组内参（其中一组已被注释掉），换了相机必须重新标定并替换。
3. **`scripts/ultralytics/` 是定制修改版，不能替换成 pip 安装的 ultralytics**。
   主程序依赖其中非标准的模块路径（如 `ultralytics.cfg.models.utils.datasets`）与
   自定义函数（`non_max_suppression_face`），因此这份副本必须与脚本一起保留在 `scripts/` 下。
4. **`dianyunceshi/package.xml` 存在冗余与疑似笔误的依赖声明**（供后续整理参考）：
   声明了 `pcl_ros`、`pcl_conversions`、`roscpp`、`ros_msgs`，但包内实际只有 Python 节点，
   并未用到 PCL/C++ 相关依赖；其中 `ros_msgs` 疑似应为 `std_msgs`。
   另外 `CMakeLists.txt` 中的 `set(SRC_LIST main.cpp)` 对应的 `main.cpp` 已不存在
   （`src/` 目录下只剩历史 vim 交换文件）。
5. **`jaka_driver` 依赖 JAKA 官方 C++ SDK 的动态库**，其接口说明见
   `src/JAKA_ROS_Demo/` 与 JAKA 官方仓库手册。

## 说明

本仓库由原工作空间整理而来：已剔除 catkin 构建产物、重复的官方仓库压缩包，
并将第三方依赖包排除在版本控制之外（保留在本地磁盘）。原上游仓库
`https://github.com/JakaCobot/jaka_robot` 仅提供 JAKA 官方包，本项目在其基础上补充了
`dianyunceshi` 与 `moveit_exe` 两个自研功能包。
