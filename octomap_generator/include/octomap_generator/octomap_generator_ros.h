#ifndef OCTOMAP_GENERATOR_ROS_H
#define OCTOMAP_GENERATOR_ROS_H

#include <ros/ros.h>

#include <iostream>
#include <sstream>
#include <string>

#include <cmath>
#include <cstring>
#include <memory>

#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>

// 编译完消息后，如果没有找到头文件
// 重新 source，然后重启 code
#include <octomap_generator/octomap_generator.h>
#include <scout_msgs/ScoutStatus.h>
#include <semantics_octree/semantics_octree.h>

// sudo apt install ros-kinetic-grid-map*
//#include <grid_map_core/GridMap.hpp>
//#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <octomap/ColorOcTree.h>

#include "octomap_generator/semantis_cost.h"

#define COLOR_OCTREE 0
#define SEMANTICS_OCTREE_MAX 1
#define SEMANTICS_OCTREE_BAYESIAN 2

namespace octomap_generator {

/**
 * \brief ROS wrapper for octomap generator
 * \details Adapted from [RGBD slam](http://wiki.ros.org/rgbdslam) and [octomap server](http://wiki.ros.org/octomap_server)
 * \author Xuan Zhang
 * \data Mai-July 2018
 */
class OctomapGeneratorNode {
public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in OctomapGenerator
     */
    OctomapGeneratorNode(ros::NodeHandle& nh);

    // Desturctor
    virtual ~OctomapGeneratorNode();

    // Reset values to paramters from parameter server
    void reset();

    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

    /**
     * \brief Save octomap to a file. NOTE: Not tested
     * \param filename The output filename
     */
    bool save(const char* filename) const;

    void ScoutStatusCallback(const scout_msgs::ScoutStatus::ConstPtr& scout_status);

    bool OpenFile(const std::string& filename);

    void PublishSemCostMap(octomap::AbstractOcTree* abstract_tree, ros::Publisher sem_pub, std_msgs::Header header);

    /**
     * @brief init local semantic costmap(nav_msgs::OccupancyGrid)
     */
    void InitLocalCostmap();

    /**
     * @brief project voxel's semantic cost into locat costmap(base_link center)
     */
    void ProjectLocalCostmap();

    /**
     * @brief project single voxel's semantic cost
     * @param it the iterator of the current projection voxel
     * @param occupied is occupied for current voxel
     */
    void Update2DMapWithSemanticAndHeight(const octomap::ColorOcTree::iterator& it, bool occupied);

    void Update2DMapOnlyHeight(const octomap::ColorOcTree::iterator& it, bool occupied);
    void Update2DMapOnlySemantic(const octomap::ColorOcTree::iterator& it, bool occupied);

    /**
     * @brief calc a high grass semantic voxel cost
     * @param global_octomap_point high grass global point
     * @return kHighGrassCost if z > car's height, otherwise kLowGrassCost
     */
    int HighGrassDeepGreenCost(const octomap::point3d& global_octomap_point);

    /**
     * @brief calc a road semantic voxel cost
     * @param global_octomap_point road semantic voxel global point
     * @return  kHighRoadCost if z > Chassis's height, otherwise kRoadCost
     */
    int RoadGrayCost(const octomap::point3d& global_octomap_point);

    /**
     * @brief calc a unknow semantic voxel cost
     * @param global_octomap_point unknow semantic voxel global point
     * @return kUnknowClassCost if z > Chassis's height, otherwise kLowUnknowClassCost
     */
    int UnknowClassCost(const octomap::point3d& global_octomap_point);

    int Traversable(const octomap::point3d& global_octomap_point);
    int MaybeTraversable(const octomap::point3d& global_octomap_point);
    int NotTraversable(const octomap::point3d& global_octomap_point);

    bool IsInLocalMap(const octomap::point3d& global_octomap_point);

    bool IsInBaseLinkFront(const octomap::point3d& global_octomap_point);
    
    int CarBlueCost(const octomap::point3d& global_octomap_point);


    bool IsLowerThanCar(const octomap::point3d& global_octomap_point);

    inline unsigned CostmapIdx(int i, int j) const
    {
        return local_sem_costmap_.info.width * j + i;
    }

    inline unsigned CostmapIdx(const octomap::OcTreeKey& key) const
    {
        return CostmapIdx((key[0] - padded_min_key_[0]) / multires_2d_scale_,
            (key[1] - padded_min_key_[1]) / multires_2d_scale_);
    }

    // don't use now
    inline bool IsInUpdateBBX(const octomap::ColorOcTree::iterator& it, const int max_tree_depth) const
    {
        // 2^(tree_depth-depth) voxels wide:
        unsigned voxelWidth = (1 << (max_tree_depth - it.getDepth()));
        octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
        return (key[0] + voxelWidth >= update_bbx_min_[0]
            && key[1] + voxelWidth >= update_bbx_min_[1]
            && key[0] <= update_bbx_max_[0]
            && key[1] <= update_bbx_max_[1]);
    }

    void InflateSemCostmap(nav_msgs::OccupancyGrid& sem_costmap);
    int HeightProject(const octomap::point3d& global_octomap_point);

protected:
    OctomapGeneratorBase* octomap_generator_; ///<Octomap instance pointer
    ros::ServiceServer service_; ///<ROS service to toggle semantic color display
    bool toggleUseSemanticColor(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); ///<Function to toggle whether write semantic color or rgb color as when serializing octree
    ros::NodeHandle nh_; ///<ROS handler
    ros::Publisher fullmap_pub_; ///<ROS publisher for octomap message
    message_filters::Subscriber<sensor_msgs::PointCloud2>* pointcloud_sub_; ///<ROS subscriber for pointcloud message
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pointcloud_sub_; ///<ROS tf message filter to sychronize the tf and pointcloud messages
    tf::TransformListener tf_listener_; ///<Listener for the transform between the camera and the world coordinates
    std::string world_frame_id_; ///<Id of the world frame
    std::string base_frame_id_;
    std::string max_pointcloud_topic_; ///<Topic name for subscribed pointcloud message
    std::string bayes_pointcloud_topic_;

    float max_range_; ///<Max range for points to be inserted into octomap
    float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space
    float clamping_thres_max_; ///<Upper bound of occupancy probability for a node
    float clamping_thres_min_; ///<Lower bound of occupancy probability for a node
    float resolution_; ///<Resolution of octomap
    float occupancy_thres_; ///<Minimum occupancy probability for a node to be considered as occupied
    float prob_hit_; ///<Hit probability of sensor
    float prob_miss_; ///<Miss probability of sensor
    int tree_type_; ///<0: color octree, 1: semantic octree using bayesian fusion, 2: semantic octree using max fusion

    octomap_msgs::Octomap map_msg_; ///<ROS octomap message

    // 局部地图管理器
    OctomapGeneratorBase* local_octomap_generator_;
    ros::Publisher local_map_pub;
    octomap_msgs::Octomap local_map_msg;

    // 小车当前速度，主要是为了根据小车运动的快慢来更新地图的消失速度
    float linear_velocity;
    float angular_velocity;
    ros::Subscriber sub_scout_status;

    ros::Publisher grid_map_pub;

    // RadiusOutlierRemoval
    double m_outrem_radius;
    int m_outrem_neighbors;

    tf::StampedTransform base_to_world_tf_;
    tf::StampedTransform rslidar_to_world_tf_;

    ros::Publisher global_sem_costmap_pub_;
    ros::Publisher local_sem_costmap_pub_;
    ros::Publisher local_sem_gridmap_pub_;
    nav_msgs::OccupancyGrid local_sem_costmap_;

    octomap::OcTreeKey padded_min_key_;

    // current no use
    octomap::OcTreeKey update_bbx_min_;
    octomap::OcTreeKey update_bbx_max_;

    geometry_msgs::PointStamped map_point_;
    geometry_msgs::PointStamped base_point_;

    octomap::ColorOcTree* project_tree_;
    unsigned int multires_2d_scale_;
    unsigned int max_tree_depth_;

    double max_green_height_;
    double scout_height_;
    double max_unknow_height_;
    double max_road_height_;
    double max_project_height_;

    bool using_radius_filter_;
    bool using_single_obstacle_filter_;
    bool using_single_obstacle_inflation_;
    int filter_num_;

    bool local_model_;

    int low_risk_cost_;
    int mid_risk_cost_;
    int high_risk_cost_;
    int init_cost_;

    bool only_project_semantic_;
    bool only_project_height_;
    bool project_semantic_height_;
    double ground_height_;

    double traversable_safe_height_;
    double maybe_traversable_safe_height_;
    double not_traversable_safe_height_;
    int gama_;
private:
    static const std::string kNodeName;
};
}
#endif //OCTOMAP_GENERATOR_ROS