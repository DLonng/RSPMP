#include "octomap_generator/octomap_generator_ros.h"

#define USAGE "\nUSAGE: octomap_generator <map.[bt|ot]>\n" \
              "  map.bt: inital octomap 3D map file to read\n"

#define TEST_VEL 1

using namespace octomap_generator;
using namespace semantics_cost;

const std::string OctomapGeneratorNode::kNodeName = "OctomapGenerator";

OctomapGeneratorNode::OctomapGeneratorNode(ros::NodeHandle& nh)
    : nh_(nh)
    , project_tree_(nullptr)
    , low_risk_cost_(1)
    , mid_risk_cost_(10)
    , high_risk_cost_(254)
    , init_cost_(255)
    , only_project_height_(false)
    , ground_height_(0.2)
    , traversable_safe_height_(0.14)
    , maybe_traversable_safe_height_(0.50)
    , not_traversable_safe_height_(0.14)
    , gama_(2)
{
    nh_.getParam("/octomap/tree_type", tree_type_);

    // Initiate octree
    if (tree_type_ == SEMANTICS_OCTREE_BAYESIAN || tree_type_ == SEMANTICS_OCTREE_MAX) {
        if (tree_type_ == SEMANTICS_OCTREE_BAYESIAN) {
            ROS_INFO("[%s]: [%s] Semantic octomap generator [bayesian fusion]", kNodeName.c_str(), __FUNCTION__);
            octomap_generator_ = new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>();
            // 创建局部 Bayes 地图对象
            // Bayes 语义融合的发布频率问题还没解决
            local_octomap_generator_ = new OctomapGenerator<PCLSemanticsBayesian, LocalSemanticsOctreeBayesian>();
        } else {
            ROS_INFO("[%s]: [%s] Semantic octomap generator [max fusion]", kNodeName.c_str(), __FUNCTION__);
            octomap_generator_ = new OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>();
            // 再创建一个局部地图管理器，单独管理局部地图对象
            // 如果后期不需要很大的修改，其实可以把局部地图的逻辑放在全局地图中，这样才符合模板的使用理念
            local_octomap_generator_ = new OctomapGenerator<PCLSemanticsMax, LocalSemanticsOctreeMax>();
        }
        service_ = nh_.advertiseService("toggle_use_semantic_color", &OctomapGeneratorNode::toggleUseSemanticColor, this);
    } else {
        ROS_INFO("[%s]: [%s] Color octomap generator", kNodeName.c_str(), __FUNCTION__);
        // 因为没有为 ColorOcTree 加上局部地图的功能，为了能够使模板编译通过，所以注释掉
        //octomap_generator_ = new OctomapGenerator<PCLColor, ColorOcTree>();
    }

    reset();

    // 构建地图是 latch 设置为 false
    fullmap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/octomap_generator_full", 1, false);
    local_map_pub = nh_.advertise<octomap_msgs::Octomap>("/octomap_generator_local", 1, false);

    global_sem_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/octomap_global_sem_costmap", 1, false);
    local_sem_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/octomap_local_sem_costmap", 1, false);

    //bool latchedTopics = true;
    //grid_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("octomap_generator_projected_map", 5, latchedTopics);
    local_sem_gridmap_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/local_sem_gridmap", 1, true);

    // 订阅小车的状态，获取线速度和角速度
    std::string scout_status;
    nh_.param<std::string>("scout_status", scout_status, "/scout_status");
    sub_scout_status = nh_.subscribe(scout_status, 1, &OctomapGeneratorNode::ScoutStatusCallback, this);

    // 根据八叉树的类型选择订阅哪种类型的语义点云
    std::string pointcloud_topic;
    if (tree_type_ == SEMANTICS_OCTREE_BAYESIAN)
        pointcloud_topic = bayes_pointcloud_topic_;
    else
        pointcloud_topic = max_pointcloud_topic_;

    pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, pointcloud_topic, 5);
    tf_pointcloud_sub_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pointcloud_sub_, tf_listener_, world_frame_id_, 5);
    tf_pointcloud_sub_->registerCallback(boost::bind(&OctomapGeneratorNode::insertCloudCallback, this, _1));
}

OctomapGeneratorNode::~OctomapGeneratorNode()
{
    if (octomap_generator_)
        delete octomap_generator_;

    if (local_octomap_generator_)
        delete local_octomap_generator_;

    if (project_tree_)
        delete project_tree_;
}

void OctomapGeneratorNode::reset()
{
    nh_.getParam("/octomap/max_pointcloud_topic", max_pointcloud_topic_);
    nh_.getParam("/octomap/bayes_pointcloud_topic", bayes_pointcloud_topic_);
    nh_.getParam("/octomap/world_frame_id", world_frame_id_);
    nh_.getParam("/octomap/base_frame_id", base_frame_id_);
    nh_.getParam("/octomap/resolution", resolution_);
    nh_.getParam("/octomap/max_range", max_range_);
    nh_.getParam("/octomap/raycast_range", raycast_range_);
    nh_.getParam("/octomap/clamping_thres_min", clamping_thres_min_);
    nh_.getParam("/octomap/clamping_thres_max", clamping_thres_max_);
    nh_.getParam("/octomap/occupancy_thres", occupancy_thres_);
    nh_.getParam("/octomap/prob_hit", prob_hit_);
    nh_.getParam("/octomap/prob_miss", prob_miss_);
    nh_.getParam("/octomap/tree_type", tree_type_);

    nh_.getParam("/octomap/using_radius_filter", using_radius_filter_);
    nh_.getParam("/octomap/outrem_radius", m_outrem_radius);
    nh_.getParam("/octomap/outrem_neighbors", m_outrem_neighbors);

    nh_.getParam("/octomap/max_green_height", max_green_height_);
    nh_.getParam("/octomap/scout_height", scout_height_);

    nh_.getParam("/octomap/max_unknow_height", max_unknow_height_);
    nh_.getParam("/octomap/max_road_height", max_road_height_);
    nh_.getParam("/octomap/max_project_height", max_project_height_);
    nh_.getParam("/octomap/using_single_obstacle_filter", using_single_obstacle_filter_);
    nh_.getParam("/octomap/using_single_obstacle_inflation", using_single_obstacle_inflation_);
    nh_.getParam("/octomap/filter_num", filter_num_);

    nh_.getParam("/octomap/local_model", local_model_);

    nh_.getParam("/octomap/low_risk_cost", low_risk_cost_);
    nh_.getParam("/octomap/mid_risk_cost", mid_risk_cost_);
    nh_.getParam("/octomap/high_risk_cost", high_risk_cost_);
    nh_.getParam("/octomap/init_cost", init_cost_);
    nh_.getParam("/octomap/only_project_height", only_project_height_);
    nh_.getParam("/octomap/ground_height", ground_height_);

    nh_.getParam("/octomap/traversable_safe_height", traversable_safe_height_);
    nh_.getParam("/octomap/maybe_traversable_safe_height", maybe_traversable_safe_height_);
    nh_.getParam("/octomap/not_traversable_safe_height", not_traversable_safe_height_);

    nh_.getParam("/octomap/only_project_semantic", only_project_semantic_);
    nh_.getParam("/octomap/only_project_height", only_project_height_);
    nh_.getParam("/octomap/project_semantic_height", project_semantic_height_);

    nh_.getParam("/octomap/gama", gama_);

    octomap_generator_->setClampingThresMin(clamping_thres_min_);
    octomap_generator_->setClampingThresMax(clamping_thres_max_);
    octomap_generator_->setResolution(resolution_);
    octomap_generator_->setOccupancyThres(occupancy_thres_);
    octomap_generator_->setProbHit(prob_hit_);
    octomap_generator_->setProbMiss(prob_miss_);
    octomap_generator_->setRayCastRange(raycast_range_);
    octomap_generator_->setMaxRange(max_range_);
    // 设置地图模式：全局 or 局部
    octomap_generator_->setLocalModel(local_model_);

    // 初始化局部地图属性
    local_octomap_generator_->setClampingThresMin(clamping_thres_min_);
    local_octomap_generator_->setClampingThresMax(clamping_thres_max_);
    local_octomap_generator_->setResolution(resolution_);
    local_octomap_generator_->setOccupancyThres(occupancy_thres_);
    local_octomap_generator_->setProbHit(prob_hit_);
    local_octomap_generator_->setProbMiss(prob_miss_);
    local_octomap_generator_->setRayCastRange(raycast_range_);
    local_octomap_generator_->setMaxRange(max_range_);
    local_octomap_generator_->setLocalModel(local_model_);

}

bool OctomapGeneratorNode::toggleUseSemanticColor(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
#if 0
    octomap_generator_->setUseSemanticColor(!octomap_generator_->isUseSemanticColor());

    if (octomap_generator_->isUseSemanticColor())
        ROS_INFO("Full Octomap using semantic color");
    else
        ROS_INFO("Full Octomap using rgb color");

    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        fullmap_pub_.publish(map_msg_);
    else
        ROS_ERROR("Error serializing Full OctoMap");

    local_octomap_generator_->setUseSemanticColor(!local_octomap_generator_->isUseSemanticColor());

    if (local_octomap_generator_->isUseSemanticColor())
        ROS_INFO("Local Octomap using semantic color");
    else
        ROS_INFO("Local Octomap using rgb color");

    if (octomap_msgs::fullMapToMsg(*local_octomap_generator_->getOctree(), local_map_msg))
        local_map_pub.publish(local_map_msg);
    else
        ROS_ERROR("Error serializing Local OctoMap");
#endif

    std::string save_path;
    nh_.getParam("/octomap/save_path", save_path);
    this->save(save_path.c_str());
    ROS_INFO("OctoMap saved.");

    return true;
}

void OctomapGeneratorNode::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Voxel filter to down sample the point cloud
    // Create the filtering object
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // 对一帧融合后的点云进行半径滤波
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;

    // 设置输入点云，这里要传递指针，所以做个 share_ptr 的拷贝
    outrem.setInputCloud(cloud);

    // 设置滤波半径，launch 中配置
    outrem.setRadiusSearch(m_outrem_radius);

    // 设置近邻数量，launch 中配置
    outrem.setMinNeighborsInRadius(m_outrem_neighbors);

    // 执行半径滤波
    if (using_radius_filter_)
        outrem.filter(*cloud);

    try {
        tf_listener_.waitForTransform(world_frame_id_, cloud_msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
        tf_listener_.lookupTransform(world_frame_id_, cloud_msg->header.frame_id, ros::Time(0), rslidar_to_world_tf_);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f rslidar_to_world_matrix;
    pcl_ros::transformAsMatrix(rslidar_to_world_tf_, rslidar_to_world_matrix);

    octomap_generator_->insertPointCloud(cloud, rslidar_to_world_matrix);

    map_msg_.header.frame_id = world_frame_id_;
    map_msg_.header.stamp = cloud_msg->header.stamp;
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        fullmap_pub_.publish(map_msg_);
    else
        ROS_ERROR("[%s]: [%s] Error publish full OctoMap", kNodeName.c_str(), __FUNCTION__);

    // Like OctomapServer Style
    // fullMsgToMap -> AbstractOcTree -> ColorOcTree -> Project2D
    octomap::AbstractOcTree* abs_tree = octomap_msgs::fullMsgToMap(map_msg_);
    project_tree_ = static_cast<octomap::ColorOcTree*>(abs_tree);

    // dynamic_cast is not work! why?
    //project_tree_ = dynamic_cast<octomap::ColorOcTree*>(tmp_tree);

    if (!project_tree_) {
        ROS_ERROR("[%s]: [%s] Could not static_cast!", kNodeName.c_str(), __FUNCTION__);
        return;
    }

    // 初始化局部 2D 语义代价地图，当前只处理局部地图，全局地图分开做
    InitLocalCostmap();

    // 投影局部 2D 语义代价地图，可以启动全局地图模式，只投影 max_range 范围内的体素
    ProjectLocalCostmap();

    InflateSemCostmap(local_sem_costmap_);

    // 发布局部 2D 语义代价地图
    local_sem_costmap_pub_.publish(local_sem_costmap_);

    if (project_tree_) {
        delete project_tree_;
        project_tree_ = nullptr;
    }
}

void OctomapGeneratorNode::InitLocalCostmap()
{
    unsigned int tree_depth = project_tree_->getTreeDepth();
    max_tree_depth_ = tree_depth;
    double res = project_tree_->getResolution();
    double grid_res = project_tree_->getNodeSize(max_tree_depth_);

    /*
    double minX, minY, minZ;
    double maxX, maxY, maxZ;

    project_tree_->getMetricMin(minX, minY, minZ);
    project_tree_->getMetricMax(maxX, maxY, maxZ);

    update_bbx_min_[0] = project_tree_->coordToKey(minX);
    update_bbx_min_[1] = project_tree_->coordToKey(minY);
    update_bbx_min_[2] = project_tree_->coordToKey(minZ);

    update_bbx_max_[0] = project_tree_->coordToKey(maxX);
    update_bbx_max_[1] = project_tree_->coordToKey(maxY);
    update_bbx_max_[2] = project_tree_->coordToKey(maxZ);
*/

    multires_2d_scale_ = 1 << (tree_depth - max_tree_depth_);

    try {
        tf_listener_.waitForTransform(world_frame_id_, base_frame_id_, ros::Time(0), ros::Duration(3.0));
        tf_listener_.lookupTransform(world_frame_id_, base_frame_id_, ros::Time(0), base_to_world_tf_);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    double base_x = base_to_world_tf_.getOrigin().x();
    double base_y = base_to_world_tf_.getOrigin().y();

    local_sem_costmap_.header.frame_id = world_frame_id_;
    local_sem_costmap_.header.stamp = ros::Time::now();
    local_sem_costmap_.info.width = ((max_range_ + 1.0) * 2) / grid_res;
    local_sem_costmap_.info.height = ((max_range_ + 1.0) * 2) / grid_res;
    //local_sem_costmap_.info.width = 200 * (max_range_ / 12.0);
    //local_sem_costmap_.info.height = 200 * (max_range_ / 12.0);
    local_sem_costmap_.info.resolution = grid_res;

    // 以 m 为单位的高度 (local_sem_costmap_.info.height - 1 + 0.5) * local_sem_costmap_.info.resolution * 0.5
    //local_sem_costmap_.info.origin.position.x = base_x - (local_sem_costmap_.info.height - 1 + 0.5) * local_sem_costmap_.info.resolution * 0.5 - grid_res * 0.5;
    //local_sem_costmap_.info.origin.position.y = base_y - (local_sem_costmap_.info.width - 1 + 0.5) * local_sem_costmap_.info.resolution * 0.5 - grid_res * 0.5;
    local_sem_costmap_.info.origin.position.x = base_x - (local_sem_costmap_.info.height * 0.5) * grid_res;
    local_sem_costmap_.info.origin.position.y = base_y - (local_sem_costmap_.info.width * 0.5) * grid_res;
    
    
    //local_sem_costmap_.info.origin.position.z = base_to_world_tf_

    octomap::point3d min_pt = octomap::point3d(local_sem_costmap_.info.origin.position.x, local_sem_costmap_.info.origin.position.y, 0);

    if (!project_tree_->coordToKeyChecked(min_pt, max_tree_depth_, padded_min_key_)) {
        ROS_ERROR("[%s]: [%s] Could not create padded min OcTree key at %f %f %f", kNodeName.c_str(), __FUNCTION__, min_pt.x(), min_pt.y(), min_pt.z());
        return;
    }

    local_sem_costmap_.data.clear();
    local_sem_costmap_.data.resize(local_sem_costmap_.info.width * local_sem_costmap_.info.height, init_cost_);
}

void OctomapGeneratorNode::ProjectLocalCostmap()
{
    for (octomap::ColorOcTree::iterator it = project_tree_->begin(max_tree_depth_), end = project_tree_->end(); it != end; ++it) {
        // 比车身高的体素不投影？因为小车不会撞到比自己高的障碍物
        // 如何比较斜坡的障碍物？
        // 投影占用节点，且在局部地图范围内，且比投影高度低
        if (project_tree_->isNodeOccupied(*it) /*&& IsInBaseLinkFront(it.getCoordinate())*/ && IsInLocalMap(it.getCoordinate()) && IsLowerThanCar(it.getCoordinate())) {
            if (project_semantic_height_)
                Update2DMapWithSemanticAndHeight(it, true);
            else if (only_project_height_)
                Update2DMapOnlyHeight(it, true);
            else
                Update2DMapOnlySemantic(it, true);
        }
    }
}


bool OctomapGeneratorNode::IsInBaseLinkFront(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);
    
    // 在车辆后方的体素不投影
    if (base_point_.point.x < 0.0) {
        return false;
    } else {
        return true;
    }
}

bool OctomapGeneratorNode::IsInLocalMap(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);
    
    // 计算每个体素到机器人中心的平面距离
    float dist = sqrt((base_point_.point.x * base_point_.point.x) + (base_point_.point.y * base_point_.point.y));// + ((base_point_.point.z * base_point_.point.z)));
    
    // 大于最大范围的体素不投影
    if (dist > max_range_) {
        return false;
    } else {
        return true;
    }
}

bool OctomapGeneratorNode::IsLowerThanCar(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);

    // 大于最大投影高度的体素不投影
    if (base_point_.point.z > max_project_height_) {
        return false;
    } else {
        return true;
    }
}

void OctomapGeneratorNode::Update2DMapWithSemanticAndHeight(const octomap::ColorOcTree::iterator& it, bool occupied)
{
    octomap::ColorOcTreeNode* node = project_tree_->search(it.getKey());
    if (node == NULL) {
        ROS_ERROR("[%s]: [%s] node is nullptr", kNodeName.c_str(), __FUNCTION__);
        return;
    }

    //std::cout << node->getValue() << std::endl;

    const octomap::ColorOcTreeNode::Color node_color = node->getColor();

    if (it.getDepth() == max_tree_depth_) {

        unsigned idx = CostmapIdx(it.getKey());
        // change if else to switch to speed up!
        if (occupied) {
            if (node_color == kRoadGray) // ground
                local_sem_costmap_.data[idx] = Traversable(it.getCoordinate());
            else if (node_color == kLowGrassGreen) // grass
                local_sem_costmap_.data[idx] = MaybeTraversable(it.getCoordinate());
            else if (node_color == kHighGrassDeepGreen) // plants, stone, etc...
                local_sem_costmap_.data[idx] = MaybeTraversable(it.getCoordinate());
            else if (node_color == kSkyBlue)
                local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
            else if (node_color == kPeopleRed)
                local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
            else if (node_color == kCarBlue)
                local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
            else if (node_color == kBuildingGray)
                local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
            else
                local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
        }
    } else {
        int int_size = 1 << (max_tree_depth_ - it.getDepth());
        octomap::OcTreeKey min_key = it.getIndexKey();
        for (int dx = 0; dx < int_size; dx++) {

            int i = (min_key[0] + dx - padded_min_key_[0]) / multires_2d_scale_;

            for (int dy = 0; dy < int_size; dy++) {

                unsigned idx = CostmapIdx(i, (min_key[1] + dy - padded_min_key_[1]) / multires_2d_scale_);

                if (occupied) {
                    if (node_color == kRoadGray) // ground
                        local_sem_costmap_.data[idx] = Traversable(it.getCoordinate());
                    else if (node_color == kLowGrassGreen) // grass
                        local_sem_costmap_.data[idx] = MaybeTraversable(it.getCoordinate());
                    else if (node_color == kHighGrassDeepGreen) // plants, stone, etc...
                        local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
                    else if (node_color == kSkyBlue)
                        local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
                    else if (node_color == kPeopleRed)
                        local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
                    else if (node_color == kCarBlue)
                        local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
                    else if (node_color == kBuildingGray)
                        local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
                    else // unknown class
                        local_sem_costmap_.data[idx] = NotTraversable(it.getCoordinate());
                }
            }
        }
    }
}


int OctomapGeneratorNode::Traversable(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);

    return low_risk_cost_; 
/*
    if (base_point_.point.z <= traversable_safe_height_) {
        return low_risk_cost_; //
    } else {
        return high_risk_cost_;
    }
*/
}

int OctomapGeneratorNode::MaybeTraversable(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);

    if (base_point_.point.z <= maybe_traversable_safe_height_) {
        return low_risk_cost_ * gama_;
    } else {
        return high_risk_cost_;
    }
}

int OctomapGeneratorNode::NotTraversable(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);

    return high_risk_cost_;

/*
    if (base_point_.point.z <= not_traversable_safe_height_) {
        return mid_risk_cost_;
    } else {
        return high_risk_cost_;
    }
*/
}




void OctomapGeneratorNode::Update2DMapOnlySemantic(const octomap::ColorOcTree::iterator& it, bool occupied)
{
    octomap::ColorOcTreeNode* node = project_tree_->search(it.getKey());
    if (node == NULL) {
        ROS_ERROR("[%s]: [%s] node is nullptr", kNodeName.c_str(), __FUNCTION__);
        return;
    }

    //std::cout << node->getValue() << std::endl;

    const octomap::ColorOcTreeNode::Color node_color = node->getColor();

    if (it.getDepth() == max_tree_depth_) {

        unsigned idx = CostmapIdx(it.getKey());
        // change if else to switch to speed up!
        if (occupied) {
            if (node_color == kRoadGray)
                local_sem_costmap_.data[idx] = low_risk_cost_;
            else if (node_color == kLowGrassGreen)
                local_sem_costmap_.data[idx] = mid_risk_cost_;
            else if (node_color == kHighGrassDeepGreen)
                local_sem_costmap_.data[idx] = high_risk_cost_;
            else if (node_color == kSkyBlue)
                local_sem_costmap_.data[idx] = high_risk_cost_;
            else if (node_color == kPeopleRed)
                local_sem_costmap_.data[idx] = high_risk_cost_;
            else if (node_color == kCarBlue)
                local_sem_costmap_.data[idx] = high_risk_cost_;
            else if (node_color == kBuildingGray)
                local_sem_costmap_.data[idx] = high_risk_cost_;
            else
                local_sem_costmap_.data[idx] = high_risk_cost_;
        }
    } else {
        int int_size = 1 << (max_tree_depth_ - it.getDepth());
        octomap::OcTreeKey min_key = it.getIndexKey();
        for (int dx = 0; dx < int_size; dx++) {

            int i = (min_key[0] + dx - padded_min_key_[0]) / multires_2d_scale_;

            for (int dy = 0; dy < int_size; dy++) {

                unsigned idx = CostmapIdx(i, (min_key[1] + dy - padded_min_key_[1]) / multires_2d_scale_);

                if (occupied) {
                    if (node_color == kRoadGray)
                        local_sem_costmap_.data[idx] = low_risk_cost_;
                    else if (node_color == kLowGrassGreen)
                        local_sem_costmap_.data[idx] = mid_risk_cost_;
                    else if (node_color == kHighGrassDeepGreen)
                        local_sem_costmap_.data[idx] = high_risk_cost_;
                    else if (node_color == kSkyBlue)
                        local_sem_costmap_.data[idx] = high_risk_cost_;
                    else if (node_color == kPeopleRed)
                        local_sem_costmap_.data[idx] = high_risk_cost_;
                    else if (node_color == kCarBlue)
                        local_sem_costmap_.data[idx] = high_risk_cost_;
                    else if (node_color == kBuildingGray)
                        local_sem_costmap_.data[idx] = high_risk_cost_;
                    else
                        local_sem_costmap_.data[idx] = high_risk_cost_;
                }
            }
        }
    }
}


void OctomapGeneratorNode::Update2DMapOnlyHeight(const octomap::ColorOcTree::iterator& it, bool occupied)
{
    octomap::ColorOcTreeNode* node = project_tree_->search(it.getKey());
    if (node == NULL) {
        ROS_ERROR("[%s]: [%s] node is nullptr", kNodeName.c_str(), __FUNCTION__);
        return;
    }

    const octomap::ColorOcTreeNode::Color node_color = node->getColor();

    if (it.getDepth() == max_tree_depth_) {

        unsigned idx = CostmapIdx(it.getKey());
        // change if else to switch to speed up!
        if (occupied) {
            local_sem_costmap_.data[idx] = HeightProject(it.getCoordinate());
        }
    } else {
        int int_size = 1 << (max_tree_depth_ - it.getDepth());
        octomap::OcTreeKey min_key = it.getIndexKey();
        for (int dx = 0; dx < int_size; dx++) {

            int i = (min_key[0] + dx - padded_min_key_[0]) / multires_2d_scale_;

            for (int dy = 0; dy < int_size; dy++) {

                unsigned idx = CostmapIdx(i, (min_key[1] + dy - padded_min_key_[1]) / multires_2d_scale_);

                if (occupied) {
                    local_sem_costmap_.data[idx] = HeightProject(it.getCoordinate());
                }
            }
        }
    }
}

int OctomapGeneratorNode::HighGrassDeepGreenCost(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);

    // 大于车体高度或一半(设置为参数)的绿色草丛认为是不可通过的障碍物
    // 小于传感器支架高度的一半：332 + 176.7 + 305 / 2 = 0.66m
    if (base_point_.point.z > max_green_height_) {
        // 大于车高的绿植认为是不能通过
        return high_risk_cost_;
    } else {
        // 小于等于车高的绿植认为是可以通过，赋值为草丛的语义成本
        return mid_risk_cost_;
    }
}


int OctomapGeneratorNode::HeightProject(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);

    // 地面高度 = 0.2m
    if (base_point_.point.z > ground_height_) {
        // 大于地面高度认为是障碍物
        return high_risk_cost_;
    } else {
        // 小于等于地面高度认为是地面
        return low_risk_cost_;
    }
}

int OctomapGeneratorNode::UnknowClassCost(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);

    // 大于指定高度的未知类别认为是不可通行的障碍物
    if (base_point_.point.z > max_unknow_height_) {
        return high_risk_cost_;
    } else {
        return mid_risk_cost_;
    }
}

int OctomapGeneratorNode::RoadGrayCost(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);

    // 大于底盘高度的道路类别认为是不可通行的路延
    if (base_point_.point.z > max_road_height_) {
        return kHighRoadCost;
    } else {
        return kRoadCost;
    }
}

int OctomapGeneratorNode::CarBlueCost(const octomap::point3d& global_octomap_point)
{
    // 将体素坐标从 map 转换到 base_link，因为要在 base_link 中比较体素和小车的高度
    map_point_.header.frame_id = world_frame_id_;
    map_point_.header.stamp = ros::Time(0);
    map_point_.point.x = global_octomap_point.x();
    map_point_.point.y = global_octomap_point.y();
    map_point_.point.z = global_octomap_point.z();

    tf_listener_.transformPoint(base_frame_id_, map_point_, base_point_);

    // 大于底盘高度的汽车体素类别认为是不可通行
    if (base_point_.point.z > scout_height_) {
        return kCarCost;
    } else {
        // 小于汽车底盘的体素可能是误差导致的
        return kRoadCost;
    }
}

void OctomapGeneratorNode::InflateSemCostmap(nav_msgs::OccupancyGrid& sem_costmap)
{
    // 创建膨胀地图
    nav_msgs::OccupancyGrid inflation_sem_map;
    inflation_sem_map = sem_costmap;

    int index = 0;

    int top_x_index = 0;
    int down_x_index = 0;
    int left_y_index = 0;
    int right_y_index = 0;

    int top_index = 0;
    int down_index = 0;
    int left_index = 0;
    int right_index = 0;

    int left_top_index = 0;
    int left_down_index = 0;
    int right_top_index = 0;
    int right_down_index = 0;

    int inflation_data = 0;

    int filter_num = 0;

    // 在膨胀对象内做障碍物膨胀和对单点障碍物滤波
    for (int x_i = 0; x_i < sem_costmap.info.height; x_i++) {
        for (int y_j = 0; y_j < sem_costmap.info.width; y_j++) {

            int index = x_i * sem_costmap.info.width + y_j;

            // 膨胀一个障碍物栅格的八领域，膨胀半径为一个栅格的长度，即为分辨率 14cm
            top_x_index = (x_i + 1);
            down_x_index = (x_i - 1);

            left_y_index = (y_j + 1);
            right_y_index = (y_j - 1);

            top_index = (x_i + 1) * sem_costmap.info.width + y_j;
            down_index = (x_i - 1) * sem_costmap.info.width + y_j;

            left_index = x_i * sem_costmap.info.width + (y_j + 1);
            right_index = x_i * sem_costmap.info.width + (y_j - 1);

            left_top_index = (x_i + 1) * sem_costmap.info.width + (y_j + 1);
            left_down_index = (x_i - 1) * sem_costmap.info.width + (y_j + 1);

            right_top_index = (x_i + 1) * sem_costmap.info.width + (y_j - 1);
            right_down_index = (x_i - 1) * sem_costmap.info.width + (y_j - 1);

            filter_num = 0;

            // 判断障碍物点的 8 领域内有多少个非障碍物点
            if ((sem_costmap.data[index] == kObstacleCost) && using_single_obstacle_filter_) {
                if ((top_x_index <= inflation_sem_map.info.height) && (inflation_sem_map.data[top_index] != kObstacleCost))
                    filter_num += 1;

                if ((down_x_index >= 0) && (inflation_sem_map.data[down_index] != kObstacleCost))
                    filter_num += 1;

                if ((left_y_index <= inflation_sem_map.info.width) && (inflation_sem_map.data[left_index] != kObstacleCost))
                    filter_num += 1;

                if ((right_y_index >= 0) && (inflation_sem_map.data[right_index] != kObstacleCost))
                    filter_num += 1;

                if ((left_y_index <= inflation_sem_map.info.width) && (top_x_index <= inflation_sem_map.info.height) && (inflation_sem_map.data[left_top_index] != kObstacleCost))
                    filter_num += 1;

                if ((left_y_index <= inflation_sem_map.info.width) && (down_x_index >= 0) && (inflation_sem_map.data[left_down_index] != kObstacleCost))
                    filter_num += 1;

                if ((right_y_index >= 0) && (top_x_index <= inflation_sem_map.info.height) && (inflation_sem_map.data[right_top_index] != kObstacleCost))
                    filter_num += 1;

                if ((right_y_index >= 0) && (down_x_index >= 0) && (inflation_sem_map.data[right_down_index] != kObstacleCost))
                    filter_num += 1;
            }

            // 如果 8 领域内至少有 filter_num_ = 7 个领域都是非障碍物点就将该障碍物点设置为地面成本
            if (filter_num >= filter_num_) {
                inflation_sem_map.data[index] = kRoadCost;
                continue;
            }

            // 障碍物点向周围膨胀 1 个栅格
            if ((sem_costmap.data[index] == kObstacleCost) && using_single_obstacle_inflation_) {
                if ((top_x_index <= inflation_sem_map.info.height) && (inflation_sem_map.data[top_index] != kObstacleCost))
                    inflation_sem_map.data[top_index] = kInflationCost;

                if ((down_x_index >= 0) && (inflation_sem_map.data[down_index] != kObstacleCost))
                    inflation_sem_map.data[down_index] = kInflationCost;

                if ((left_y_index <= inflation_sem_map.info.width) && (inflation_sem_map.data[left_index] != kObstacleCost))
                    inflation_sem_map.data[left_index] = kInflationCost;

                if ((right_y_index >= 0) && (inflation_sem_map.data[right_index] != kObstacleCost))
                    inflation_sem_map.data[right_index] = kInflationCost;

                if ((left_y_index <= inflation_sem_map.info.width) && (top_x_index <= inflation_sem_map.info.height) && (inflation_sem_map.data[left_top_index] != kObstacleCost))
                    inflation_sem_map.data[left_top_index] = kInflationCost;

                if ((left_y_index <= inflation_sem_map.info.width) && (down_x_index >= 0) && (inflation_sem_map.data[left_down_index] != kObstacleCost))
                    inflation_sem_map.data[left_down_index] = kInflationCost;

                if ((right_y_index >= 0) && (top_x_index <= inflation_sem_map.info.height) && (inflation_sem_map.data[right_top_index] != kObstacleCost))
                    inflation_sem_map.data[right_top_index] = kInflationCost;

                if ((right_y_index >= 0) && (down_x_index >= 0) && (inflation_sem_map.data[right_down_index] != kObstacleCost))
                    inflation_sem_map.data[right_down_index] = kInflationCost;
            }
        }
    }

    // 拷贝膨胀数据到 local_sem_map
    sem_costmap = inflation_sem_map;
}

void OctomapGeneratorNode::ScoutStatusCallback(const scout_msgs::ScoutStatus::ConstPtr& scout_status)
{
    // 获取小车的线速度和角速度
    linear_velocity = scout_status->linear_velocity;
    angular_velocity = scout_status->angular_velocity;
}

bool OctomapGeneratorNode::save(const char* filename) const
{
    octomap_generator_->save(filename);
}

// 打开全局导航地图并发布占据栅格和语义栅格
bool OctomapGeneratorNode::OpenFile(const std::string& filename)
{
    std::cout << filename << std::endl;

    if (filename.length() <= 3)
        return false;

    std::string suffix = filename.substr(filename.length() - 3, 3);

    // 全局八叉树
    // 使用 dynamic_cast 转换类型会出问题，static_cast 则不会，暂时没解决
    //SemanticsOctreeMax* max_global_tree = dynamic_cast<SemanticsOctreeMax*>(octomap_generator_->getOctree());
    //SemanticsOctreeMax* max_global_tree = dynamic_cast<SemanticsOctreeMax*>(abstract_global_tree);
    //octomap::AbstractOcTree* max_global_tree = octomap_generator_->getOctree();

    SemanticsOctreeMax* max_global_tree = static_cast<SemanticsOctreeMax*>(octomap_generator_->getOctree());

    if (suffix == ".bt") {
        if (!max_global_tree->readBinary(filename)) {
            return false;
        }
    } else if (suffix == ".ot") {
        octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(filename);

        if (!tree) {
            ROS_ERROR("Could not read file.");
            return false;
        }

        // free bug!!!
        //if (max_global_tree) {
        //    delete max_global_tree;
        //    max_global_tree = nullptr;
        //}

        // 使用 dynamic_cast 转换类型会出问题，static_cast 则不会，暂时没解决
        max_global_tree = static_cast<SemanticsOctreeMax*>(tree);

        if (!max_global_tree) {
            ROS_ERROR("Could not static_cast convert to SemanticsOctreeMax.");
            return false;
        }

    } else {
        ROS_ERROR("Could not read other format octomap file.");
        return false;
    }

    ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), max_global_tree->size());
    std::cout << "TreeDepth  is: " << max_global_tree->getTreeDepth() << std::endl;
    std::cout << "Resolution is: " << max_global_tree->getResolution() << std::endl;

    grid_map::GridMap occ_grid_map;
    grid_map::GridMap sem_grid_map;

    // init map info
    double resolution = max_global_tree->getResolution();

    grid_map::Position3 min_bound;
    grid_map::Position3 max_bound;

    max_global_tree->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
    max_global_tree->getMetricMax(max_bound(0), max_bound(1), max_bound(2));

    grid_map::Length length = grid_map::Length(max_bound(0) - min_bound(0), max_bound(1) - min_bound(1));
    grid_map::Position position = grid_map::Position((max_bound(0) + min_bound(0)) / 2.0, (max_bound(1) + min_bound(1)) / 2.0);

    std::string static_layer = "static_layer";
    occ_grid_map.setGeometry(length, resolution, position);
    occ_grid_map.add(static_layer);
    occ_grid_map.setBasicLayers({ static_layer });

    std::string semantic_layer = "semantic_layer";
    sem_grid_map.setGeometry(length, resolution, position);
    sem_grid_map.add(semantic_layer);
    sem_grid_map.setBasicLayers({ semantic_layer });

    grid_map::Matrix& occ_data = occ_grid_map[static_layer];
    grid_map::Matrix& sem_data = sem_grid_map[semantic_layer];

    octomap::ColorOcTreeNode::Color red_semantic(255, 0, 0);
    octomap::ColorOcTreeNode::Color green_semantic(0, 255, 0);
    octomap::ColorOcTreeNode::Color blue_semantic(0, 0, 255);

    int no_semantic = 0;
    int unknow_semantic = 1;
    int road = 2;
    int low_grass = 3;
    int high_grass = 4;

    // 遍历树的所有叶节点
    for (typename SemanticsOctreeMax::leaf_iterator it = max_global_tree->begin_leafs(), end = max_global_tree->end_leafs(); it != end; ++it) {

        octomap::point3d cur_coord = it.getCoordinate();

        grid_map::Position position(cur_coord.x(), cur_coord.y());
        grid_map::Index index;
        occ_grid_map.getIndex(position, index);

        // 节点被占用，占用概率设置为 100，语义信息设置为节点的颜色
        if (max_global_tree->isNodeOccupied(*it)) {
            occ_data(index(0), index(1)) = 100;

            SemanticsOcTreeNodeMax* node = max_global_tree->search(it.getKey());

            octomap::ColorOcTreeNode::Color node_color = node->getColor();

            // 根据节点颜色设置语义类别
            if (node_color == red_semantic)
                sem_data(index(0), index(1)) = road; // red is road
            else if (node_color == green_semantic)
                sem_data(index(0), index(1)) = low_grass; // green is low grass
            else if (node_color == blue_semantic)
                sem_data(index(0), index(1)) = high_grass; // blue is high grass
            else
                sem_data(index(0), index(1)) = unknow_semantic;
        } else {
            // 节点不被占用，占用概率设置为 0，语义信息设置为 -1 表示没有语义
            occ_data(index(0), index(1)) = 0;
            sem_data(index(0), index(1)) = no_semantic;
        }
    }

    nav_msgs::OccupancyGridPtr occ_nav_map(new nav_msgs::OccupancyGrid);
    nav_msgs::OccupancyGridPtr sem_nav_map(new nav_msgs::OccupancyGrid);

    //grid_map -> OccupancyGrid
    grid_map::GridMapRosConverter occ_converter;
    occ_converter.toOccupancyGrid(occ_grid_map, static_layer, 0, 100, *occ_nav_map);
    occ_converter.toOccupancyGrid(sem_grid_map, semantic_layer, 0, 10, *sem_nav_map);

    //global_occ_nav_pub_.publish(occ_nav_map);
    //global_sem_nav_pub_.publish(sem_nav_map);

    return true;
}

void OctomapGeneratorNode::PublishSemCostMap(octomap::AbstractOcTree* abstract_tree, ros::Publisher sem_pub, std_msgs::Header header)
{
    SemanticsOctreeMax* tree = static_cast<SemanticsOctreeMax*>(abstract_tree);

    double resolution = tree->getResolution();

    grid_map::Position3 min_bound;
    grid_map::Position3 max_bound;

    tree->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
    tree->getMetricMax(max_bound(0), max_bound(1), max_bound(2));

    grid_map::Length length = grid_map::Length(max_bound(0) - min_bound(0), max_bound(1) - min_bound(1));
    grid_map::Position position = grid_map::Position((max_bound(0) + min_bound(0)) / 2.0, (max_bound(1) + min_bound(1)) / 2.0);

    grid_map::GridMap sem_grid_map;
    std::string semantic_layer = "semantic_layer";
    sem_grid_map.setFrameId(header.frame_id);
    sem_grid_map.setGeometry(length, resolution, position);
    sem_grid_map.add(semantic_layer);
    sem_grid_map.setBasicLayers({ semantic_layer });

    ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
        sem_grid_map.getLength().x(), sem_grid_map.getLength().y(),
        sem_grid_map.getSize()(0), sem_grid_map.getSize()(1),
        sem_grid_map.getPosition().x(), sem_grid_map.getPosition().y(), sem_grid_map.getFrameId().c_str());

    grid_map::Matrix& sem_data = sem_grid_map[semantic_layer];

    /*
          170, 170, 170,
          0, 255, 0,
          0, 60, 0,
          0, 120, 255,
          0, 0, 0,
    */
    octomap::ColorOcTreeNode::Color road_gray(170, 170, 170); // 灰色地面
    octomap::ColorOcTreeNode::Color low_grass_green(0, 255, 0); // 绿色小草
    octomap::ColorOcTreeNode::Color high_grass_deep_green(0, 60, 0); // 深绿色树木
    octomap::ColorOcTreeNode::Color sky_blue(0, 120, 255); // 淡蓝色天空
    octomap::ColorOcTreeNode::Color unknow_black(0, 0, 0); // 未知类别黑色

    unsigned char road_cost = 1; //costmap_2d::FREE_SPACE; // 0
    unsigned char low_grass_cost = 10; //costmap_2d::FREE_SPACE; // 0
    unsigned char high_grass_cost = 50; //costmap_2d::LETHAL_OBSTACLE; // 254
    unsigned char sky_blue_cost = 50; //costmap_2d::LETHAL_OBSTACLE; // 254
    unsigned char unknow_class_cost = 50; //costmap_2d::NO_INFORMATION; // 255
    unsigned char no_occ_cost = 50; // 网格不被占用，给最高的成本

    for (grid_map::GridMapIterator it(sem_grid_map); !it.isPastEnd(); ++it) {
        sem_grid_map.at("semantic_layer", *it) = 50;
    }

    // 遍历树的所有叶节点
    for (typename SemanticsOctreeMax::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        octomap::point3d cur_coord = it.getCoordinate();

        grid_map::Position position(cur_coord.x(), cur_coord.y());
        grid_map::Index index;

        sem_grid_map.getIndex(position, index);

        // 节点被占用，占用概率设置为 100，语义信息设置为节点的颜色
        // 只对高度小于 0.15m 的栅格进行投影
        //  && (cur_coord.z() > -0.15) && (cur_coord.z() <= 0)
        if (tree->isNodeOccupied(*it)) {
            SemanticsOcTreeNodeMax* node = tree->search(it.getKey());
            octomap::ColorOcTreeNode::Color node_color = node->getColor();

            if (node_color == road_gray)
                sem_data(index(0), index(1)) = road_cost;
            else if (node_color == low_grass_green)
                sem_data(index(0), index(1)) = low_grass_cost;
            else if (node_color == high_grass_deep_green)
                sem_data(index(0), index(1)) = high_grass_cost;
            else if (node_color == sky_blue)
                sem_data(index(0), index(1)) = sky_blue_cost;
            else
                sem_data(index(0), index(1)) = unknow_class_cost;
        }
    }

    if (tree->getNumLeafNodes() != 0) {

        nav_msgs::OccupancyGridPtr sem_nav_map(new nav_msgs::OccupancyGrid);

        //grid_map -> OccupancyGrid
        grid_map::GridMapRosConverter occ_converter;
        occ_converter.toOccupancyGrid(sem_grid_map, semantic_layer, 0, 100, *sem_nav_map);

        sem_nav_map->header.frame_id = header.frame_id;
        sem_nav_map->header.stamp = header.stamp;

        // 发布全局/局部语义栅格地图
        sem_pub.publish(sem_nav_map);
    }
}

#if 0
void OctomapGeneratorNode::PublishOccAndSemGridMap(octomap::AbstractOcTree* abstract_tree, ros::Publisher occ_pub, ros::Publisher sem_pub)
{
    SemanticsOctreeMax* tree = static_cast<SemanticsOctreeMax*>(abstract_tree);

    //grid_map::GridMap occ_grid_map;
    grid_map::GridMap sem_grid_map;

    // init map info
    double resolution = tree->getResolution();

    grid_map::Position3 min_bound;
    grid_map::Position3 max_bound;

    tree->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
    tree->getMetricMax(max_bound(0), max_bound(1), max_bound(2));
/*
    // User can provide coordinate limits to only convert a bounding box.
    octomap::point3d minBbx(min_bound(0), min_bound(1), min_bound(2));
    grid_map::Position3 minPoint(0.5, 0.5, 0.0);
    minBbx = octomap::point3d(minPoint(0), minPoint(1), minPoint(2));
    max_bound = grid_map::Position3(minBbx.x(), minBbx.y(), minBbx.z());

    octomap::point3d maxBbx(max_bound(0), max_bound(1), max_bound(2));
    grid_map::Position3 maxPoint(2.0, 2.0, 1.0);
    maxBbx = octomap::point3d(maxPoint(0), maxPoint(1), maxPoint(2));
    max_bound = grid_map::Position3(maxBbx.x(), maxBbx.y(), maxBbx.z());
*/
    grid_map::Length length = grid_map::Length(max_bound(0) - min_bound(0), max_bound(1) - min_bound(1));
    grid_map::Position position = grid_map::Position((max_bound(0) + min_bound(0)) / 2.0, (max_bound(1) + min_bound(1)) / 2.0);


/*
    octomap::OcTreeKey min_key = octomap_generator_->GetMinKey();
    octomap::OcTreeKey max_key = octomap_generator_->GetMaxKey();
    grid_map::Length length = grid_map::Length(max_key[0] - min_key[0], max_key[0] - min_key[0]);
    grid_map::Position position = grid_map::Position((max_key[0] + min_key[0]) / 2.0, (max_key[0] + min_key[0]) / 2.0);
*/
    //std::string static_layer = "static_layer";
    //occ_grid_map.setGeometry(length, resolution, position);
    //occ_grid_map.add(static_layer);
    //occ_grid_map.setBasicLayers({ static_layer });

    std::string semantic_layer = "semantic_layer";
    sem_grid_map.setGeometry(length, resolution, position);
    //sem_grid_map.setGeometry(grid_map::Length(100.0, 100.0), resolution);
    sem_grid_map.add(semantic_layer);
    sem_grid_map.setBasicLayers({ semantic_layer });

    //grid_map::Matrix& occ_data = occ_grid_map[static_layer];
    grid_map::Matrix& sem_data = sem_grid_map[semantic_layer];

    octomap::ColorOcTreeNode::Color red_semantic(255, 0, 0);
    octomap::ColorOcTreeNode::Color green_semantic(0, 255, 0);
    octomap::ColorOcTreeNode::Color blue_semantic(0, 0, 255);

    // 语义信息设置为 -1 表示未知语义，因为占据栅格中 -1 表示未知
    float unknow_semantic = -1;
    // 空闲节点没有语义
    float no_semantic = 0;
    // 当前采用的 3 种语义
    float road = 1;
    float low_grass = 2;
    float high_grass = 3;

    // 遍历树的所有叶节点
    for (typename SemanticsOctreeMax::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        // 未知的坐标不进行投影，未知的栅格数据是 -1
        octomap::point3d cur_coord = it.getCoordinate();

        grid_map::Position position(cur_coord.x(), cur_coord.y());
        grid_map::Index index;

        sem_grid_map.getIndex(position, index);

        // 节点被占用，占用概率设置为 100，语义信息设置为节点的颜色
        if (tree->isNodeOccupied(*it)) {
            //occ_data(index(0), index(1)) = 100;

            // 测试时把所有体素的语义类别都设置为道路
            sem_data(index(0), index(1)) = road; // 1
#if 0
            SemanticsOcTreeNodeMax* node = tree->search(it.getKey());
            octomap::ColorOcTreeNode::Color node_color = node->getColor();

            // 根据节点颜色设置语义类别
            // 需要确定语义类别和颜色、及对应的语义成本
            if (node_color == red_semantic)
                sem_data(index(0), index(1)) = road; // red is road
            else if (node_color == green_semantic)
                sem_data(index(0), index(1)) = low_grass; // green is low grass
            else if (node_color == blue_semantic)
                sem_data(index(0), index(1)) = high_grass; // blue is high grass
            else
                sem_data(index(0), index(1)) = unknow_semantic; // other color is unknow
#endif
        } else {
            // 节点不被占用，占用概率设置为 0，其他未知节点默认设置为 -1
            //occ_data(index(0), index(1)) = 0;
            // 空闲节点没有语义信息
            sem_data(index(0), index(1)) = no_semantic; // 0
        }
    }

    if (tree->getNumLeafNodes() != 0) {

        //nav_msgs::OccupancyGridPtr occ_nav_map(new nav_msgs::OccupancyGrid);
        nav_msgs::OccupancyGridPtr sem_nav_map(new nav_msgs::OccupancyGrid);

        //grid_map -> OccupancyGrid
        grid_map::GridMapRosConverter occ_converter;
        //occ_converter.toOccupancyGrid(occ_grid_map, static_layer, 0, 100, *occ_nav_map);
        occ_converter.toOccupancyGrid(sem_grid_map, semantic_layer, 0, 100, *sem_nav_map);

        //sem_nav_map->info.width = 10;
        //sem_nav_map->info.height = 10;
        /*
        sem_nav_map->info.origin.position.x = 5;
        sem_nav_map->info.origin.position.y = 5;
        sem_nav_map->info.origin.position.z = 0.0;

        sem_nav_map->info.origin.orientation.x = 0.0;
        sem_nav_map->info.origin.orientation.y = 0.0;
        sem_nav_map->info.origin.orientation.z = 0.0;
        sem_nav_map->info.origin.orientation.w = 1.0;
*/
        /* 
    for (int i = 0; i < occ_nav_map->info.width; i++) {
        for (int j = 0; j < occ_nav_map->info.height; j++) {
            // 把未知的导航栅格占用概率设置为 0
            if (occ_nav_map->data[i * occ_nav_map->info.width + j] == -1)
                occ_nav_map->data[i * occ_nav_map->info.width + j] = 0;

            printf("%d ", occ_nav_map->data[i * occ_nav_map->info.width + j]);
        }
    }
        */
        /*
        for (int i = 0; i < occ_nav_map->info.width * occ_nav_map->info.height; i++) {
            // 把未知的导航栅格占用概率设置为 0
            if (occ_nav_map->data[i] == -1)
                occ_nav_map->data[i] = 0;

            //printf("%d ", occ_nav_map->data[i]);
        }
*/
        //occ_nav_map->header.frame_id = "map";
        //occ_nav_map->header.stamp = ros::Time::now();

        // 发布全局/局部占据栅格地图
        //occ_pub.publish(occ_nav_map);

        sem_nav_map->header.frame_id = "map";
        sem_nav_map->header.stamp = ros::Time::now();

        // 发布全局/局部语义栅格地图
        sem_pub.publish(sem_nav_map);
    }
}

#endif

// like octomap_server_node.cpp
int main2(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_generator");
    ros::NodeHandle nh;

    if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")) {
        ROS_ERROR("%s", USAGE);
        exit(-1);
    }

    OctomapGeneratorNode octomap_generator_node(nh);
    ros::spinOnce();

    std::string map_file_name("");
    std::string map_file_name_param("");

    if (argc == 2) {
        map_file_name = std::string(argv[1]);
    }

    if (nh.getParam("/octomap/map_file", map_file_name_param)) {
        if (map_file_name != "") {
            ROS_WARN("map_file is specified by the argument '%s' and rosparam '%s'. now loads '%s'",
                map_file_name.c_str(), map_file_name_param.c_str(), map_file_name.c_str());
        } else {
            map_file_name = map_file_name_param;
        }
    }

    if (map_file_name != "") {
        if (!octomap_generator_node.OpenFile(map_file_name)) {
            ROS_ERROR("Could not open file %s", map_file_name.c_str());
            exit(1);
        }
    } else {
        std::cout << "map_file_name is null, don't read map." << std::endl;
    }

    try {
        ros::spin();
    } catch (std::runtime_error& e) {
        ROS_ERROR("octomap_generator exception: %s", e.what());
        return -1;
    }

    return 0;
}

// origin main
int main(int argc, char** argv)
{
#if 0
    // Initialize node and publisher.
    ros::init(argc, argv, "octomap_generator");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    // Create grid map.
    grid_map::GridMap map({ "elevation" });
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(1.2, 2.0), 0.03);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
        map.getLength().x(), map.getLength().y(),
        map.getSize()(0), map.getSize()(1));

    // Work with grid map in a loop.
    ros::Rate rate(30.0);
    while (nh.ok()) {

        // Add data to grid map.
        ros::Time time = ros::Time::now();
        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map.getPosition(*it, position);
            map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
        }

        // Publish grid map.
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(map, message);
        publisher.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

        // Wait for next cycle.
        rate.sleep();
    }
#endif
#if 1
    ros::init(argc, argv, "octomap_generator");
    ros::NodeHandle nh;
    OctomapGeneratorNode octomapGeneratorNode(nh);
    ros::spin();

    // 暂时不需要保存地图，这个 save 暂时还有 bug！
    //std::string save_path;
    //nh.getParam("/octomap/save_path", save_path);
    //octomapGeneratorNode.save(save_path.c_str());
    //ROS_INFO("OctoMap saved.");
#endif
    return 0;
}