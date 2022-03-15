#include <cmath>
#include <cstring> // For std::memcpy
#include <octomap_generator/octomap_generator.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <semantics_point_type/semantics_point_type.h>
#include <sstream>

#include <omp.h>

template <class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::OctomapGenerator()
    : octomap_(0.05)
    , max_range_(1.)
    , raycast_range_(1.)
    , local_model_(true)
{
    /*    
    double maxX, maxY, maxZ;
    octomap_getMetricMin(minX, minY, minZ);
    octomap_getMetricMax(maxX, maxY, maxZ);

    m_updateBBXMin[0] = octomap_.coordToKey(minX);
    m_updateBBXMin[1] = octomap_.coordToKey(minY);
    m_updateBBXMin[2] = octomap_.coordToKey(minZ);

    m_updateBBXMax[0] = octomap_.coordToKey(maxX);
    m_updateBBXMax[1] = octomap_.coordToKey(maxY);
    m_updateBBXMax[2] = octomap_.coordToKey(maxZ);
*/
    min_key_[0] = 0;
    min_key_[1] = 0;
    min_key_[2] = 0;

    max_key_[0] = 0;
    max_key_[1] = 0;
    max_key_[2] = 0;
}

template <class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::~OctomapGenerator() { }

template <class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::setUseSemanticColor(bool use)
{
    octomap_.setUseSemanticColor(use);
}

template <>
void OctomapGenerator<PCLColor, ColorOcTree>::setUseSemanticColor(bool use) { }

template <class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::isUseSemanticColor()
{
    return octomap_.isUseSemanticColor();
}

template <>
bool OctomapGenerator<PCLColor, ColorOcTree>::isUseSemanticColor() { return false; }

/*
template <class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld)
{
    // Convert to PCL pointcloud
    CLOUD pcl_cloud;
    //pcl::fromPCLPointCloud2(*cloud_filtered, pcl_cloud);
    pcl::fromPCLPointCloud2(*cloud, pcl_cloud);

    //std::cout << "Voxel filtered cloud size: "<< pcl_cloud.size() << std::endl;
    // Transform coordinate
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensorToWorld);

    //float startTime = omp_get_wtime();
    //tf::Vector3 originTf = sensorToWorldTf.getOrigin();
    //octomap::point3d origin(originTf[0], originTf[1], originTf[2]);

    octomap::point3d origin(static_cast<float>(sensorToWorld(0, 3)), static_cast<float>(sensorToWorld(1, 3)), static_cast<float>(sensorToWorld(2, 3)));
    octomap::Pointcloud raycast_cloud; // Point cloud to be inserted with ray casting
    int endpoint_count = 0; // total number of endpoints inserted

    // 遍历每一帧语义点云
    for (typename CLOUD::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it) {
        // Check if the point is invalid
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z)) {
            octomap::point3d end(it->x, it->y, it->z);
            // 计算每个点到原点的距离
            float dist = sqrt((it->x - origin.x()) * (it->x - origin.x()) + (it->y - origin.y()) * (it->y - origin.y()) + (it->z - origin.z()) * (it->z - origin.z()));
            // Check if the point is in max_range
            // 只插入在 max_range = 20m 内的点云
            if (dist <= max_range_) {
                // Check if the point is in the ray casting range
                if (dist <= raycast_range_) // Add to a point cloud and do ray casting later all together
                {
                    raycast_cloud.push_back(it->x, it->y, it->z);
                } else // otherwise update the occupancy of node and transfer the point to the raycast range
                {
                    octomap::point3d direction = (octomap::point3d(it->x, it->y, it->z) - origin).normalized();
                    octomap::point3d new_end = origin + direction * (raycast_range_ + octomap_.getResolution() * 2);
                    raycast_cloud.push_back(new_end);
                    octomap_.updateNode(it->x, it->y, it->z, true, false); // use lazy_eval, run updateInnerOccupancy() when done
                }

                endpoint_count++;
            }
        }
    }


#if 0
    octomap::KeyRay keyRay;
    for (auto it = raycast_cloud.begin(); it != raycast_cloud.end(); it++) {
        if (octomap_.computeRayKeys(origin, *it, keyRay)) {
            //for (octomap::KeyRay::iterator it = keyRay.begin(), end = keyRay.end(); it != end; ++it) {
            //    octomap_.updateNode(*it, false, false);
            //}

            octomap_.insertRay(origin, *it, max_range_, false);
        }
    }
#endif

    // Do ray casting for points in raycast_range_
    if (raycast_cloud.size() > 0)
        octomap_.insertPointCloud(raycast_cloud, origin, raycast_range_, false, true);

    // Update colors and semantics, differs between templates
    // 这个更新是否可以移动到前面的循环中来加速？
    updateColorAndSemantics(&pcl_cloud);

    //if (endpoint_count > 0)
        //octomap_.updateInnerOccupancy();

    //float endTime = omp_get_wtime();
    //printf("插入一帧点云不用 lazy_eval + updateInnerOccupancy 的执行时间: %f\n", endTime - startTime);
}
*/

template <class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld)
{
    float start_time = omp_get_wtime();
    // Voxel filter to down sample the point cloud
    // Create the filtering object
    //pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    // Perform voxel filter
    //float voxel_flt_size = octomap_.getResolution();
    //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    //sor.setInputCloud(cloud);
    //sor.setLeafSize(voxel_flt_size, voxel_flt_size, voxel_flt_size);
    //sor.filter(*cloud_filtered);

    // Convert to PCL pointcloud
    CLOUD pcl_cloud;
    //pcl::fromPCLPointCloud2(*cloud_filtered, pcl_cloud);
    pcl::fromPCLPointCloud2(*cloud, pcl_cloud);

    //std::cout << "Voxel filtered cloud size: "<< pcl_cloud.size() << std::endl;
    // Transform coordinate
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensorToWorld);

    //float startTime = omp_get_wtime();
    //tf::Vector3 originTf = sensorToWorldTf.getOrigin();
    //octomap::point3d origin(originTf[0], originTf[1], originTf[2]);

    octomap::point3d origin(static_cast<float>(sensorToWorld(0, 3)), static_cast<float>(sensorToWorld(1, 3)), static_cast<float>(sensorToWorld(2, 3)));

    if (local_model_) {
        // rslidar_origin -> base_link
        // 0.259 0 0.370
        octomap::point3d base_origin(origin.x() - 0.259, origin.y(), origin.z() - 0.370);

        // 遍历整棵树，删除与当前帧原点距离大于 max_range 的占用节点
        for (typename OCTREE::leaf_iterator it = octomap_.begin_leafs(), end = octomap_.end_leafs(); it != end; ++it) {
            octomap::point3d point = it.getCoordinate();
            //octomap::point3d point(it->x, it->y, it->z);
            float dist = (point - base_origin).norm();
            // octomap_.isNodeOccupied(*it) &&
            if ((dist > max_range_)) {
                //if ((dist > max_range_)) {
                octomap_.deleteNode(it.getKey());
            }
        }
    }

    octomap::Pointcloud raycast_cloud; // Point cloud to be inserted with ray casting
    int endpoint_count = 0; // total number of endpoints inserted

    // 遍历每一帧语义点云
    for (typename CLOUD::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it) {
        // Check if the point is invalid
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z)) {
            // 计算每个点到原点的距离
            //float dist = sqrt((it->x - origin.x()) * (it->x - origin.x()) + (it->y - origin.y()) * (it->y - origin.y()) + (it->z - origin.z()) * (it->z - origin.z()));
            octomap::point3d point(it->x, it->y, it->z);
            float dist = (point - origin).norm();
            // Check if the point is in max_range
            // 只插入在 max_range = 20m 内的点云
            if (dist <= max_range_) {
                // Check if the point is in the ray casting range
                if (dist <= raycast_range_) // Add to a point cloud and do ray casting later all together
                {
                    raycast_cloud.push_back(it->x, it->y, it->z);
                } else // otherwise update the occupancy of node and transfer the point to the raycast range
                {
                    octomap::point3d direction = (octomap::point3d(it->x, it->y, it->z) - origin).normalized();
                    octomap::point3d new_end = origin + direction * (raycast_range_ + octomap_.getResolution() * 2);
                    raycast_cloud.push_back(new_end);
                    // 更新后是否省略内部节点的更新（默认值：false）
                    // 设置为 true 可以加快插入速度，但是完成后需要调用 updateInnerOccupancy() 比较耗时
                    octomap_.updateNode(it->x, it->y, it->z, true, false); // use lazy_eval, run updateInnerOccupancy() when done
                }

                endpoint_count++;
            }
            /*
            // 更新最大最小边界，用于投影 2D 导航地图
            octomap::OcTreeKey end_key;

            if (octomap_.coordToKeyChecked(point, end_key)) {
                for (unsigned i = 0; i < 3; ++i)
                    min_key_[i] = std::min(end_key[i], min_key_[i]);

                for (unsigned i = 0; i < 3; ++i)
                    max_key_[i] = std::max(end_key[i], max_key_[i]);
            } else {
                std::cout << "Could not generate Key for endpoint " << "(" << point.x() << ", "<< point.y() << ", " << point.z() << ")" << std::endl;
            }
*/
        }
    }

    // Do ray casting for points in raycast_range_
    if (raycast_cloud.size() > 0)
        octomap_.insertPointCloud(raycast_cloud, origin, raycast_range_, false, true); // use lazy_eval, run updateInnerOccupancy() when done, use discretize to downsample cloud

    //double start_time = ros::Time::now().toSec();
    //float start_time = omp_get_wtime();

    // Update colors and semantics, differs between templates
    updateColorAndSemantics(&pcl_cloud);

    float end_time = omp_get_wtime();
    float semantic_update_time =  end_time - start_time;
    //std::cout << "updateColorAndSemantics single time: " <<  semantic_update_time << std::endl;
    //std::cout << "single frame insert time: " <<  semantic_update_time << std::endl;
    static float all_update_time = 0;
    all_update_time += semantic_update_time;
    //std::cout << "All semantic update time: " <<  all_update_time << std::endl;

    // updates inner node occupancy and colors
    //if (endpoint_count > 0)
    //    octomap_.updateInnerOccupancy();

    //float endTime = omp_get_wtime();
    //printf("插入一帧点云不用 lazy_eval + updateInnerOccupancy 的执行时间: %f\n", endTime - startTime);
}

/*
template <class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld)
{
    // Convert to PCL pointcloud
    CLOUD pcl_cloud;
    //pcl::fromPCLPointCloud2(*cloud_filtered, pcl_cloud);
    pcl::fromPCLPointCloud2(*cloud, pcl_cloud);

    //std::cout << "Voxel filtered cloud size: "<< pcl_cloud.size() << std::endl;
    // Transform coordinate
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensorToWorld);

    octomap::KeyRay keyRay;
    octomap::point3d origin(static_cast<float>(sensorToWorld(0, 3)), static_cast<float>(sensorToWorld(1, 3)), static_cast<float>(sensorToWorld(2, 3)));

    // instead of direct scan insertion, compute update to filter ground:
    octomap::KeySet free_cells, occupied_cells;

    float startTime = omp_get_wtime();

    // all other points: free on ray, occupied on endpoint:
    for (typename CLOUD::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it) {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z)) {
            octomap::point3d point(it->x, it->y, it->z);
            
            // maxrange check
            if ((max_range_ < 0.0) || ((point - origin).norm() <= max_range_)) {
                // free cells
                if (octomap_.computeRayKeys(origin, point, keyRay)) {
                    free_cells.insert(keyRay.begin(), keyRay.end());
                }

                // occupied endpoint
                octomap::OcTreeKey key;
                if (octomap_.coordToKeyChecked(point, key)) {
                    occupied_cells.insert(key);
                }

            } else { 
                // ray longer than maxrange:;
                octomap::point3d new_end = origin + (point - origin).normalized() * max_range_;
                if (octomap_.computeRayKeys(origin, new_end, keyRay)) {
                    free_cells.insert(keyRay.begin(), keyRay.end());

                    octomap::OcTreeKey endKey;
                    if (octomap_.coordToKeyChecked(new_end, endKey)) {
                        free_cells.insert(endKey);
                    } else { 
                        std::cout << "Could not generate Key for endpoint" << std::endl;
                    }
                }
            }
        }
    }

 
    // mark free cells only if not seen occupied in this cloud
    for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
        if (occupied_cells.find(*it) == occupied_cells.end()) {
            octomap_.updateNode(*it, false);
        }
    }

    // now mark all occupied cells:
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
        octomap_.updateNode(*it, true);
    }

    float endTime = omp_get_wtime();
	printf("octomap_server for 算法的执行时间: %f\n", endTime - startTime);

    
    updateColorAndSemantics(&pcl_cloud);
}
*/

/**
  * @brief 用于对局部地图中的每个过期节点进行删除
  * @details 目前只是在每个节点内增加时间戳，删除与当前时刻间隔大于指定阈值的节点
  * @param[in] time_thres 该参数不一定使用时间命名，考虑更改，因为测试时时间戳与速度相关联
  * @return void
  * @note 
  *     局部地图的实现方法还需要优化！
  * @todo 
  *     
  * @author DLonng
  * @date 2020-07-24
  */
template <class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::UpdateLocalMap(unsigned int time_thres)
{
    octomap_.degradeOutdatedNodes(time_thres);
    //octomap_.updateInnerOccupancy();
}

// 普通 RGB 地图语义更新策略
template <>
void OctomapGenerator<PCLColor, ColorOcTree>::updateColorAndSemantics(PCLColor* pcl_cloud)
{
    for (PCLColor::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++) {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z)) {
            octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
        }
    }
    octomap::ColorOcTreeNode* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    //std::cout << "Example octree node: " << std::endl;
    //std::cout << "Color: " << node->getColor()<< std::endl;
}

// 全局 Max 地图语义更新策略
template <>
void OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>::updateColorAndSemantics(PCLSemanticsMax* pcl_cloud)
{
    for (PCLSemanticsMax::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++) {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z)) {
            octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
            // Get semantics
            octomap::SemanticsMax sem;
            uint32_t rgb;

            // 0RGB
            std::memcpy(&rgb, &it->semantic_color, sizeof(uint32_t));
            sem.semantic_color.r = (rgb >> 16) & 0x0000ff;
            sem.semantic_color.g = (rgb >> 8) & 0x0000ff;
            sem.semantic_color.b = (rgb)&0x0000ff;
            sem.confidence = it->confidence;

            octomap_.updateNodeSemantics(it->x, it->y, it->z, sem);
        }
    }

    //SemanticsOcTreeNodeMax* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    //std::cout << "Example octree node: " << std::endl;
    //std::cout << "Color: " << node->getColor()<< std::endl;
    //std::cout << "Semantics: " << node->getSemantics() << std::endl;
}

// 局部 Max 地图语义更新策略
template <>
void OctomapGenerator<PCLSemanticsMax, LocalSemanticsOctreeMax>::updateColorAndSemantics(PCLSemanticsMax* pcl_cloud)
{
    for (PCLSemanticsMax::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++) {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z)) {
            octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
            // Get semantics
            octomap::SemanticsMax sem;
            uint32_t rgb;

            // 0RGB
            std::memcpy(&rgb, &it->semantic_color, sizeof(uint32_t));
            sem.semantic_color.r = (rgb >> 16) & 0x0000ff;
            sem.semantic_color.g = (rgb >> 8) & 0x0000ff;
            sem.semantic_color.b = (rgb)&0x0000ff;
            sem.confidence = it->confidence;

            octomap_.updateNodeSemantics(it->x, it->y, it->z, sem);
        }
    }
    //LocalSemanticsOcTreeNodeMax* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    //std::cout << "Example octree node: " << std::endl;
    //std::cout << "Color: " << node->getColor()<< std::endl;
    //std::cout << "Semantics: " << node->getSemantics() << std::endl;
}

// 全局 Bayes 地图语义更新策略
template <>
void OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>::updateColorAndSemantics(PCLSemanticsBayesian* pcl_cloud)
{
    for (PCLSemanticsBayesian::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++) {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z)) {
            octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
            // Get semantics
            octomap::SemanticsBayesian sem;
            for (int i = 0; i < 3; i++) {
                uint32_t rgb;
                std::memcpy(&rgb, &it->data_sem[i], sizeof(uint32_t));
                sem.data[i].color.r = (rgb >> 16) & 0x0000ff;
                sem.data[i].color.g = (rgb >> 8) & 0x0000ff;
                sem.data[i].color.b = (rgb)&0x0000ff;
                sem.data[i].confidence = it->data_conf[i];
            }
            octomap_.updateNodeSemantics(it->x, it->y, it->z, sem);
        }
    }
    //SemanticsOcTreeNodeBayesian* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    //std::cout << "Example octree node: " << std::endl;
    //std::cout << "Color: " << node->getColor()<< std::endl;
    //std::cout << "Semantics: " << node->getSemantics() << std::endl;
}

// 局部 Bayes 地图语义更新策略
template <>
void OctomapGenerator<PCLSemanticsBayesian, LocalSemanticsOctreeBayesian>::updateColorAndSemantics(PCLSemanticsBayesian* pcl_cloud)
{
    for (PCLSemanticsBayesian::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++) {
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z)) {
            octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
            // Get semantics
            octomap::SemanticsBayesian sem;
            for (int i = 0; i < 3; i++) {
                uint32_t rgb;
                std::memcpy(&rgb, &it->data_sem[i], sizeof(uint32_t));
                sem.data[i].color.r = (rgb >> 16) & 0x0000ff;
                sem.data[i].color.g = (rgb >> 8) & 0x0000ff;
                sem.data[i].color.b = (rgb)&0x0000ff;
                sem.data[i].confidence = it->data_conf[i];
            }
            octomap_.updateNodeSemantics(it->x, it->y, it->z, sem);
        }
    }
    //LocalSemanticsOcTreeNodeBayesian* node = octomap_.search(pcl_cloud->begin()->x, pcl_cloud->begin()->y, pcl_cloud->begin()->z);
    //std::cout << "Example octree node: " << std::endl;
    //std::cout << "Color: " << node->getColor()<< std::endl;
    //std::cout << "Semantics: " << node->getSemantics() << std::endl;
}

template <class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::save(const char* filename) const
{
    //std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
    std::ofstream outfile;
    outfile.open(filename, std::ios::out | std::ios::binary);
    if (outfile.is_open()) {
        std::cout << "Writing octomap generator full [.ot] to " << filename << std::endl;
        octomap_.write(outfile);
        outfile.close();
        std::cout << "Color tree written " << filename << std::endl;
        return true;
    } else {
        std::cout << "Could not open " << filename << " for writing" << std::endl;
        return false;
    }
}

//Explicit template instantiation
//template class OctomapGenerator<PCLColor, ColorOcTree>;

template class OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>;
template class OctomapGenerator<PCLSemanticsMax, LocalSemanticsOctreeMax>;

// 新添加的类模板需要在导出模板实例
template class OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>;
template class OctomapGenerator<PCLSemanticsBayesian, LocalSemanticsOctreeBayesian>;
