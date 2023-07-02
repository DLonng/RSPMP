# RSPMP

RSPMP: https://link.springer.com/article/10.1007/s10489-022-03283-z


## 编译
### 环境
- 环境：Ubuntu16.04 or Ubuntu18.04
- Ros: kinetic or Melodic
- 依赖：octomap_server, gridmap

### 编译过程
1. git clone 项目 到 ros 工作空间
2. catkin_make
3. 遇到相关库的依赖问题，根据报错提示安装：sudo apt-get install ros-[dist]-[xxx]


## 启动
### agilex_msgs
功能：提供相关语义点云的自定义 msg

### lidar_camera_fusion
功能；融合点云和语义图像，输出语义点云
输入：原始点云，原始语义图像，标定矩阵，语义置信度（语义分割网络 softmax 输出）
输出：语义点云

### octomap_generator
功能：根据输入的语义点云和定位信息，实时构建自车为中心的局部语义 3D 体素地图，并投影成 2D costmap 用于后续局部规划
输入：融合的语义点云，定位 tf
输出：语义 3D 体素地图，2D costmap

### sem_dwa_planner
功能：根据 2D costmap 使用 dwa 做局部规划，dwa 增加 语义 cost，规划过程最小化语义 cost
输入：2D costmap，自车状态
输出：车辆控制信号

## 仿真
该项目没有提供仿真，我直接用的实车（Agilex Scout）
