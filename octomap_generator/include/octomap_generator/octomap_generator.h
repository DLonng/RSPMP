#ifndef OCTOMAP_GENERATOR_H
#define OCTOMAP_GENERATOR_H

#include <octomap_generator/octomap_generator_base.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/projection_matrix.h>
#include <semantics_octree/semantics_bayesian.h>
#include <semantics_octree/semantics_max.h>
#include <semantics_octree/semantics_octree.h>
#include <semantics_point_type/semantics_point_type.h>

#include <semantics_octree/local_semantics_octree.h>

//#include <semantics_octree/MyColorOcTree.h>

typedef pcl::PointCloud<PointXYZRGBSemanticsBayesian> PCLSemanticsBayesian;
typedef pcl::PointCloud<PointXYZRGBSemanticsMax> PCLSemanticsMax;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLColor;

typedef octomap::ColorOcTree ColorOcTree;

// 全局 Max 语义八叉树节点
typedef octomap::SemanticsOcTreeNode<octomap::SemanticsMax> SemanticsOcTreeNodeMax;
// 全局 Max 语义八叉树
typedef octomap::SemanticsOcTree<octomap::SemanticsMax> SemanticsOctreeMax;

// 局部 Max 语义八叉树节点
typedef octomap::LocalSemanticsOcTreeNode<octomap::SemanticsMax> LocalSemanticsOcTreeNodeMax;
// 局部 Max 语义八叉树
typedef octomap::LocalSemanticsOcTree<octomap::SemanticsMax> LocalSemanticsOctreeMax;

// 全局 Bayes 语义八叉树节点
typedef octomap::SemanticsOcTreeNode<octomap::SemanticsBayesian> SemanticsOcTreeNodeBayesian;
// 全局 Bayes 语义八叉树
typedef octomap::SemanticsOcTree<octomap::SemanticsBayesian> SemanticsOctreeBayesian;

// 局部 Bayes 语义八叉树节点
typedef octomap::LocalSemanticsOcTreeNode<octomap::SemanticsBayesian> LocalSemanticsOcTreeNodeBayesian;
// 局部 Bayes 语义八叉树
typedef octomap::LocalSemanticsOcTree<octomap::SemanticsBayesian> LocalSemanticsOctreeBayesian;

//typedef octomap::SemanticsOcTreeNode<octomap::SemanticsBayesian> SemanticsOcTreeNodeBayesian;

/**
 * Templated octomap generator to generate a color octree or a semantic octree (with different fusion methods)
 * See base class for details
 * \author Xuan Zhang
 * \data Mai-July 2018
 */
template <class CLOUD, class OCTREE>
class OctomapGenerator : public OctomapGeneratorBase {
public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in OctomapGenerator
     */
    OctomapGenerator();

    virtual ~OctomapGenerator();

    virtual void setMaxRange(float max_range) { max_range_ = max_range; }

    virtual void setRayCastRange(float raycast_range) { raycast_range_ = raycast_range; }

    virtual void setClampingThresMin(float clamping_thres_min)
    {
        octomap_.setClampingThresMin(clamping_thres_min);
    }

    virtual void setClampingThresMax(float clamping_thres_max)
    {
        octomap_.setClampingThresMax(clamping_thres_max);
    }

    virtual void setResolution(float resolution)
    {
        octomap_.setResolution(resolution);
    }

    virtual void setOccupancyThres(float occupancy_thres)
    {
        octomap_.setOccupancyThres(occupancy_thres);
    }

    virtual void setProbHit(float prob_hit)
    {
        octomap_.setProbHit(prob_hit);
    }

    virtual void setProbMiss(float prob_miss)
    {
        octomap_.setProbMiss(prob_miss);
    }

    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld);

    virtual void setUseSemanticColor(bool use);

    virtual bool isUseSemanticColor();

    //virtual octomap::AbstractOcTree* getOctree(){return &octomap_;}
    virtual octomap::AbstractOcTree* getOctree() { return &octomap_; }

    /**
     * \brief Save octomap to a file. NOTE: Not tested
     * \param filename The output filename
     */
    virtual bool save(const char* filename) const;

    // 增加了一个用于更新局部的地图的函数
    // 最好还是把局部地图单独定义成一个类，不要在全局地图中实现局部地图的功能
    virtual void UpdateLocalMap(unsigned int time_thres);

    virtual octomap::OcTreeKey GetMinKey() { return min_key_; };
    virtual octomap::OcTreeKey GetMaxKey() { return max_key_; };

    virtual void setLocalModel(bool local_model) { local_model_ = local_model; };

protected:
    OCTREE octomap_; ///<Templated octree instance
    float max_range_; ///<Max range for points to be inserted into octomap
    float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space

    octomap::OcTreeKey min_key_;
    octomap::OcTreeKey max_key_;

    bool local_model_;
    
    void updateColorAndSemantics(CLOUD* pcl_cloud);
};
#endif //OCTOMAP_GENERATOR
