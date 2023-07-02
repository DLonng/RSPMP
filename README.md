# RSPMP

RSPMP: https://link.springer.com/article/10.1007/s10489-022-03283-z


## 编译
### 环境
- 环境：Ubuntu16.04 or Ubuntu18.04
- Ros: kinetic or Melodic
- 依赖：octomap_server, gridmap

### 过程
1. git clone 整个项目到你的 ros 工作空间
2. catkin_make
3. 项目依赖较少，如果遇到相关库的依赖问题，根据报错提示安装即可：sudo apt-get install ros-[dist]-[xxx]


## 节点介绍
### agilex_msgs
- 功能：提供相关语义点云的自定义 msg

### lidar_camera_fusion
- 功能；融合点云和语义图像，输出语义点云
- 输入：原始点云，原始语义图像，标定矩阵，语义置信度（语义分割网络 softmax 输出）
- 输出：语义点云

### octomap_generator
- 功能：根据输入的语义点云和定位信息，实时构建自车为中心的局部语义 3D 体素地图，并投影成 2D costmap 用于后续局部规划
- 输入：融合的语义点云，定位 tf
- 输出：语义 3D 体素地图，2D costmap

### sem_dwa_planner
- 功能：根据 2D costmap 使用 dwa 做局部规划，dwa 增加 语义 cost，规划过程最小化语义 cost
- 输入：2D costmap，自车状态
- 输出：车辆控制信号

### 语义分割节点
这个项目没有提供语义分割节点的模型，你可以使用任何一种实时的语义分割模型。

## 启动
### 建图
1. 如果只跑建图，可以录制数据集（可惜毕业时我的数据集忘记保存了。。。。）
2. 录制数据集：/raw_img, /raw_lidar, /tf，/semantic_img, /semantic_confidence
3. 单独启动融合节点：roslaunch lidar_camera_fusion lidar_camera_fusion_no_msg_sem.launch
4. 单独启动建图节点：roslaunch octomap_generator octomap_generator.launch
5. 融合+建图：roalaunch octomap_generator octomap_generator_mapping_scout.launch

主要就是把数据集路上，然后在 launch 里面设置好 topic 的名称，启动即可，要注意的是你的数据集中 tf 树里面要有 lidar 到 map 的变换，因为建图是在 lidar 坐标系下做的。

### 规划
- 该项目没有提供仿真，我当时直接用的实车（Agilex Scout）
-   1. roalaunch sem_dwa_planner sem_dwa_planner.launch
- 在实车上启动如下节点：
    1. 所以需要的传感器驱动节点
    2. 融合节点：https://www.bilibili.com/video/BV1Sp4y1S74w/?spm_id_from=333.999.0.0
    3. 建图节点：https://www.bilibili.com/video/BV1Q64y1D75N/?spm_id_from=333.999.0.0
    4. 规划节点：https://www.bilibili.com/video/BV1eT4y1P7bZ/?spm_id_from=333.999.0.0
- 在 rviz 上点击目标点即可开始规划
-   1. roalaunch sem_dwa_planner sem_dwa_planner_tf_rviz.launch
