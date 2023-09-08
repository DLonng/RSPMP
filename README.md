# RSPMP

RSPMP: https://link.springer.com/article/10.1007/s10489-022-03283-z

<img width="682" alt="RSPMP" src="https://github.com/DLonng/RSPMP/assets/12621722/10b0eb73-dbe7-4922-9e7a-53f01eb3baf4">

<img width="682" alt="Scout" src="https://github.com/DLonng/RSPMP/assets/12621722/87c599f1-0f30-487c-a4e0-754c556d4eab">

## Build
### environment
- os：Ubuntu16.04 or Ubuntu18.04
- Ros: kinetic or Melodic
- depend：octomap_server, gridmap

1. git clone git@github.com:DLonng/RSPMP.git
2. catkin_make
3. according error to apt install lib：sudo apt-get install ros-[dist]-[xxx]

## Node
### agilex_msgs
- custom msg for semantic pointcloud

### lidar_camera_fusion
- fun: fusion pointcloud and semantic img, output semantic pointcloud
- input: raw pointcloud, raw semantic img, calibration matrix, semantic confidence
- output: semantic pointcloud

### octomap_generator
- fun: based on the input semantic point cloud and localization information, real-time construction of a local semantic 3D voxel map centered around the vehicle, which is then projected into a 2D cost map for subsequent local path planning
- input: fusion semantic pointcloud, location tf
- output: semantic 3D voxel map, 2D costmap

### sem_dwa_planner
- fun: incorporate DWA (Dynamic Window Approach) for local planning based on the 2D costmap, with DWA introducing semantic costs. The planning process minimizes semantic costs.
- input: 2D costmap, ego status
- output: ego control signals

### semantic segmentation node
This project does not provide a model for the semantic segmentation node. You can use any real-time semantic segmentation model of your choice.

## Run
### Mapping
1. Only mapping, you can record dataset
2. dataset topics：/raw_img, /raw_lidar, /tf，/semantic_img, /semantic_confidence
3. start fusion node：roslaunch lidar_camera_fusion lidar_camera_fusion_no_msg_sem.launch
4. start mapping node：roslaunch octomap_generator octomap_generator.launch
5. fusion + mapping：roalaunch octomap_generator octomap_generator_mapping_scout.launch

### Planning
- This project does not provide a simulation. I used a vehicle directly at the time.（Agilex Scout）
-   1. roalaunch sem_dwa_planner sem_dwa_planner.launch
- start nodes in vehicle：
    1. all sensor nodes
    2. fusion node：https://www.bilibili.com/video/BV1Sp4y1S74w/?spm_id_from=333.999.0.0
    3. mapping node：https://www.bilibili.com/video/BV1Q64y1D75N/?spm_id_from=333.999.0.0
    4. planning node：https://www.bilibili.com/video/BV1eT4y1P7bZ/?spm_id_from=333.999.0.0
- set a point in rviz
-   1. roalaunch sem_dwa_planner sem_dwa_planner_tf_rviz.launch

## reference project
[evelation_mapping](https://github.com/ANYbotics/elevation_mapping)
