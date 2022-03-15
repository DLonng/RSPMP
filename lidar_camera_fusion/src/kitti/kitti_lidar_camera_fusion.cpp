/*
 * @Description: ROS Node, Fusion img and point cloud
 * @Author: Dlonng
 * @Date: 2020-05-03 20:47:00
 * @LastEditTime: 2020-06-17 09:12:00
 */

#include "kitti_lidar_camera_fusion.h"

#define USING_TF 0
#define USING_RAW 1
#define USING_KITTI 1

// 在 scout 车上测试局部地图功能：以下 3 个都设置为 0，并且把地图类型改为 max tree，如果需要显示颜色就把树节点里面的语义关掉，或者使用服务？

// 1: 在单独的语义回调函数中融合并发布语义点云
// 0: 在点云回调中融合 2 种类型的语义点云
#define SWITCH_FUSION 1

// SWITCH_FUSION 设置为 0 才使用下面这 2 个开关
// USING_MAX_SEMANTIC 0 表示在点云回调中使用 Max 类型的点云，但是不給语义信息
#define USING_MAX_SEMANTIC 0
// USING_BAYES 1 表示在点云回调中使用 Bayes 融合，并发布 Bayes 语义点云
#define USING_BAYES 0

const std::string LidarCameraFusion::kNodeName = "lidar_camera_fusion";

// 0: max semantic, 1: bayes semantic
const int LidarCameraFusion::kMaxSemanticType = 0;
const int LidarCameraFusion::kBayesSemanticType = 1;

LidarCameraFusion::LidarCameraFusion()
    : param_handle("~")
    , camera_lidar_tf_ok(false)
    , camera_instrinsics_mat_ok(false)
    , camera_extrinsic_mat_ok(false)
    , is_kitti(false)
    , image_frame_id("")
    , semantic_type(kMaxSemanticType)
{

    InitROS();
}

/**
  * @brief 函数简要说明
  * @details 函数细节说明
  * @param[in] 参数名 参数描述
  * @param[out] 参数名 参数描述
  * @return pointer to the updated NODE
  * @note 注意事项
  * @todo 需要去实现的功能
  * @author 作者
  * @date 日期
  */
void LidarCameraFusion::InitROS()
{
    param_handle.param<int>("semantic_type", semantic_type, kMaxSemanticType);

    std::string calibration_file;
    param_handle.param<std::string>("calibration_file", calibration_file, "");

    if (calibration_file.empty()) {
        ROS_ERROR("[%s]: missing calibration file path '%s'. ", kNodeName.c_str(), calibration_file.c_str());
        ros::shutdown();
        return;
    }

    // 读取 Autoware 雷达相机外参标定文件
    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        ROS_ERROR("[%s]: cannot open file calibration file [%s]. ", kNodeName.c_str(), calibration_file.c_str());
        ros::shutdown();
        return;
    }

    // 导入雷达相机外参
    fs["CameraExtrinsicMat"] >> camera_extrinsic_mat;

    // 不用 TF 转换，需要对外参矩阵求逆，因为标定的方向是 [相机 -> 雷达]，而融合的方向是 [雷达 -> 相机]
    // 不需要对 KITTI 求逆，因为给的标定矩阵已经是 [雷达 -> 相机]
    param_handle.param<bool>("is_kitti", is_kitti, false);

    // 在 launch 中增加了一个是否使用 kitti 数据集的参数，主要是区分是否对外参矩阵求逆！
    if (is_kitti)
        camera_extrinsic_mat = camera_extrinsic_mat; // KITTI 不需要求逆
    else
        camera_extrinsic_mat = camera_extrinsic_mat.inv(); // 自己的小车如果不使用 TF 则需要求逆

    camera_extrinsic_mat_ok = true;

    ROS_INFO("[%s]: read camera_extrinsic_mat[0][0] %f", kNodeName.c_str(), camera_extrinsic_mat.at<double>(0, 0));

    // 读取相机内参，相机内参也可以考虑订阅 CameraInfo
    fs["CameraMat"] >> camera_instrinsics_mat;
    camera_instrinsics_mat_ok = true;

    // 用于直接转换到像素平面
    fx = static_cast<float>(camera_instrinsics_mat.at<double>(0, 0));
    fy = static_cast<float>(camera_instrinsics_mat.at<double>(1, 1));
    cx = static_cast<float>(camera_instrinsics_mat.at<double>(0, 2));
    cy = static_cast<float>(camera_instrinsics_mat.at<double>(1, 2));

    ROS_INFO("[%s]: read camera_instrinsics_mat fx %f fy %f cx %f cy %f", kNodeName.c_str(), fx, fy, cx, cy);

    // 图像大小
    fs["ImageSize"] >> image_size;

    // 初始化置信度矩阵
    confidences = cv::Mat::zeros(image_size.height, image_size.width, cv::DataType<float>::type);
    bayes_confidences_1 = cv::Mat::zeros(image_size.height, image_size.width, cv::DataType<float>::type);
    bayes_confidences_2 = cv::Mat::zeros(image_size.height, image_size.width, cv::DataType<float>::type);

    // 畸变矩阵
    fs["DistCoeff"] >> distortion_coefficients;

    // 畸变模型
    fs["DistModel"] >> dist_model;

    // 原始 ZED left 图像
    std::string image_raw;
    // 原始 Robosense-16 点云
    std::string cloud_raw;

    // LEDNet 分割后的语义图像
    std::string semantic_img;

    // LEDNet 语义图像对应的置信度矩阵
    std::string semantic_confidence;

    // Max Fusion 使用的语义点云类型
    std::string semantic_cloud_max;

    // Bayes Fusion 使用的语义点云类型
    std::string semantic_cloud_bayes;

    // max 概率语义消息
    std::string max_semantic;

    // bayes 语义消息
    std::string bayes_semantic;

    // 获取的参数名：image_input，获取的参数值存储在：image_input，缺省值：/camera/left/image_raw
    param_handle.param<std::string>("image_raw", image_raw, "/camera/left/image_raw");
    param_handle.param<std::string>("cloud_raw", cloud_raw, "/rslidar_points");

    param_handle.param<std::string>("semantic_img", semantic_img, "/semantic_img");
    param_handle.param<std::string>("semantic_confidence", semantic_confidence, "/semantic_array");

    param_handle.param<std::string>("semantic_cloud_max", semantic_cloud_max, "/semantic_cloud_max");
    param_handle.param<std::string>("semantic_cloud_bayes", semantic_cloud_bayes, "/semantic_cloud_bayes");

    param_handle.param<std::string>("max_semantic", max_semantic, "/max_semantic");
    param_handle.param<std::string>("bayes_semantic", bayes_semantic, "/bayes_semantic");

    // 订阅 image_input 话题
    // 第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到 1 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。
    // 当处理消息速度不够时，可以调大第二个参数！
    // 收到订阅消息后调用 ImageCallback 处理图像 msg
    int topic_buff = 5;
    sub_image_raw = topic_handle.subscribe(image_raw, topic_buff, &LidarCameraFusion::ImageRawCallback, this);
    sub_cloud_raw = topic_handle.subscribe(cloud_raw, topic_buff, &LidarCameraFusion::CloudRawCallback, this);

    // 最大概率语义图像和置信度分开订阅的逻辑
    sub_semantic_img = topic_handle.subscribe(semantic_img, topic_buff, &LidarCameraFusion::SemanticImageCallback, this);
    sub_confidence = topic_handle.subscribe(semantic_confidence, topic_buff, &LidarCameraFusion::ConfidenceCallback, this);

    sub_max_semantic = topic_handle.subscribe(max_semantic, topic_buff, &LidarCameraFusion::MaxSemanticCallback, this);
    sub_bayes_semantic = topic_handle.subscribe(bayes_semantic, topic_buff, &LidarCameraFusion::BayesSemanticCallback, this);

    // 在 fusion_topic 上发布 sensor_msgs::PointCloud2 类型的消息
    // 第二个参数为缓冲区大小，缓冲区被占满后会丢弃先前发布的消息，目前为 1，可能需要更改！
    // 发布消息：pub_fusion_cloud.publish(msg)
    pub_max_semantic_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(semantic_cloud_max, topic_buff);
    pub_bayes_semantic_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(semantic_cloud_bayes, topic_buff);
}

/**
  * @brief ZED 左相机图像订阅回调函数
  * @details 相机目前的发送频率是 20 Hz
  * @param[in]  
  * @return void
  * @note
  *     注意不同数据集的去畸变问题
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::ImageRawCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    // 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // ros_img -> cv_imag
    // image_msg: 图像指针，brg8: 编码参数
    // rgb8: CV_8UC3, color image with red-green-blue color order
    // https://blog.csdn.net/bigdog_1027/article/details/79090571
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");

    if (is_kitti) {
        // KITTI 已经去畸变
        image_frame = cv_image_ptr->image;
    } else {
        // cv_image: 原畸变 OpenCV 图像，image_frame：去畸变后的图像
        // camera_instrinsics：相机内参矩阵，distortion_coefficients：相机畸变矩阵
        // 我们自己做语义分割之前图像没有去畸变，这里调用 opencv undistort
        cv::undistort(cv_image_ptr->image, image_frame, camera_instrinsics_mat, distortion_coefficients);
    }

#if 0
    // 4. 发布去畸变的图像消息
    static image_transport::ImageTransport it(topic_handle);
    static image_transport::Publisher pub_image = it.advertise("identified_image", 1);
    static sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_frame).toImageMsg();
    pub_image.publish(msg);
#endif

    // 5. 保存当前图像帧的 ID 和大小
    image_frame_id = image_msg->header.frame_id;
    image_size.height = image_frame.rows;
    image_size.width = image_frame.cols;

    //ROS_INFO("[%s]: image_frame_id %s", kNodeName.c_str(), image_frame_id.c_str());
}

#if SWITCH_FUSION // 把 Bayes 语义点云的融合逻辑移动到语义 Bayes 回调中

/**
  * @brief  融合 image_raw, cloud_raw, semantic_img, confidences 为一帧语义点云
  * @details 雷达目前的发送频率是 10 Hz
  * @param[in] cloud_msg 订阅的一帧原始点云
  * @return void
  * @note
  *     1. 因为语义颜色的解析有 BUG，所以给 PointXYZRGBSemanticsMax 加上结构体作为 semantic_color 的 union
  *             struct {uint8_t s_b; uint8_t s_g; uint8_t s_r; uint8_t s_a;};
  * 
  *     2. 给 Bayes 结构体也加上了 RGBA 的 union struct
  * 
  *     3. 目前没有使用 TF，直接读取外参矩阵
  *     
  *     4. 目前订阅 2 组语义分割的 Bayes 语义后的发布频率为 5 Hz左右，与语义分割的发布频率几乎相同
  * @todo 
  *     image_raw, cloud_raw, semantic_img, confidences 没有进行同步处理！
  *     语义图像暂时没有加上 ID
  *     测试自定义 semantic/max_msg 消息！
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::CloudRawCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    cloud_frame = *cloud_msg;
}

/**
  * @brief 用于订阅 Bayes Fusion 的自定义语义消息
  * @details bayes_semantic 包含前 3 组最大概率的语义标签组成的图像和置信度数组
  * @param[in] bayes_semantic 订阅的 semantic/bayes_msg 自定义语义消息
  * @return void
  * @note 
  *     1. 需要考虑数据传输问题！因为一个 msg 里面包含 3 组图像和置信度！
  *     2. 语义分割产生 2 组语义信息 5Hz，该点云回调为 10 Hz，导致发布的语义点云也为 10Hz
  *        考虑融合的逻辑性，应该在频率最慢的回调中进行语义的融合操作，即应该将融合的逻辑移动到该回调中！
  * @todo 
  *     把点云回调中的融合逻辑移动到这个函数中！
  *     记得让庄明溪修改 confidence 单词！
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::BayesSemanticCallback(const semantic_msg::bayes_msg::ConstPtr& bayes_semantic)
{
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] BayesSemanticCallback: wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // Get 3 best semantic_img
    cv_bridge::CvImagePtr p_semantic_img_1 = cv_bridge::toCvCopy(bayes_semantic->semantic_img_1, "bgr8");
    cv_bridge::CvImagePtr p_semantic_img_2 = cv_bridge::toCvCopy(bayes_semantic->semantic_img_2, "bgr8");
    //cv_bridge::CvImagePtr p_semantic_img_3 = cv_bridge::toCvCopy(bayes_semantic->semantic_img_3, "bgr8");

    cv::undistort(p_semantic_img_1->image, bayes_frame_1, camera_instrinsics_mat, distortion_coefficients);
    cv::undistort(p_semantic_img_2->image, bayes_frame_2, camera_instrinsics_mat, distortion_coefficients);
    //cv::undistort(p_semantic_img_3->image, bayes_frame_3, camera_instrinsics_mat, distortion_coefficients);

    // 在融合语义点云之前进行必要的判断
    if (image_frame.empty()) {
        ROS_INFO("[%s]: image_frame is empty! Waiting for current image frame ...", kNodeName.c_str());
        return;
    }

    if (cloud_frame.height * cloud_frame.width == 0) {
        ROS_INFO("[%s]: cloud_frame is empty! Waiting for current cloud frame ...", kNodeName.c_str());
        return;
    }

    if (bayes_frame_1.empty()) {
        ROS_INFO("[%s]: bayes_frame_1 is empty! Waiting for current bayes_frame_1 ...", kNodeName.c_str());
        return;
    }

    if (bayes_frame_2.empty()) {
        ROS_INFO("[%s]: bayes_frame_2 is empty! Waiting for current bayes_frame_2 ...", kNodeName.c_str());
        return;
    }

    // 再次确保 image_frame_id 不为空，因为 TF 要用到
    if (image_frame_id == "") {
        ROS_INFO("[%s]: image_frame_id is null! Please check image topic sender!", kNodeName.c_str());
        return;
    }

#if USING_TF

    // 从 tf 树中寻找雷达和相机的坐标变换关系
    if (camera_lidar_tf_ok == false)
        camera_lidar_tf = FindTransform(image_frame_id, cloud_msg->header.frame_id);

    // 保证相机内参和坐标转换准备完成
    if (camera_instrinsics_mat_ok == false || camera_lidar_tf_ok == false) {
        ROS_INFO("[%s]: waiting for camera intrinsics and camera lidar tf ...", kNodeName.c_str());
        return;
    }

#else
    // 直接使用读取的外参矩阵，保证相机内参和坐标转换读取成功
    if (camera_instrinsics_mat_ok == false || camera_extrinsic_mat_ok == false) {
        ROS_INFO("[%s]: waiting for camera_instrinsics_mat and camera_extrinsic_mat...", kNodeName.c_str());
        return;
    }
#endif

    // ROS point cloud -> PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);

    // void pcl::fromROSMsg(const sensor_msgs::PointCloud2 &, pcl::PointCloud<T> &);
    pcl::fromROSMsg(cloud_frame, *pcl_cloud_msg);

    auto in_cloud_msg = pcl_cloud_msg;

    // 初始化 Bayes 概率语义点云
    pcl::PointCloud<PointXYZRGBSemanticsBayesian>::Ptr out_cloud(new pcl::PointCloud<PointXYZRGBSemanticsBayesian>);
    out_cloud->points.clear();

    PointXYZRGBSemanticsBayesian semantic_point_bayes;
    std::vector<PointXYZRGBSemanticsBayesian> cam_cloud(in_cloud_msg->points.size());

    int row = 0;
    int col = 0;

    cv::Vec3b rgb_pixel;

    cv::Mat raw_point(4, 1, cv::DataType<double>::type);
    cv::Mat transformed_point(4, 1, cv::DataType<double>::type);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double tmp_z = 0.0;

    cv::Vec3b semantic_pixel;

    // 遍历点云，融合语义
    for (size_t i = 0; i < in_cloud_msg->points.size(); i++) {

#if USING_TF

#else
        // 用 RT 矩阵变换
        raw_point.at<double>(0, 0) = in_cloud_msg->points[i].x;
        raw_point.at<double>(1, 0) = in_cloud_msg->points[i].y;
        raw_point.at<double>(2, 0) = in_cloud_msg->points[i].z;
        raw_point.at<double>(3, 0) = 1;

        // 4 X 1 = 4 X 4 * 4 X 1;
        transformed_point = camera_extrinsic_mat * raw_point;

        x = transformed_point.at<double>(0, 0);
        y = transformed_point.at<double>(1, 0);
        z = transformed_point.at<double>(2, 0);
        // [3][0] = 1;

        // 使用相机内参将三维空间点投影到像素平面
        col = int(x * fx / z + cx);
        row = int(y * fy / z + cy);

        tmp_z = z;
#endif
        // 只融合在像素平面内的点云, TF 没有效果是因为没改 z 的判断条件！
        if ((row >= 0) && (row < image_size.height) && (col >= 0) && (col < image_size.width) && (tmp_z > 0)) {
            // XYZ
            semantic_point_bayes.x = in_cloud_msg->points[i].x;
            semantic_point_bayes.y = in_cloud_msg->points[i].y;
            semantic_point_bayes.z = in_cloud_msg->points[i].z;

            // RGB
            rgb_pixel = image_frame.at<cv::Vec3b>(row, col);
            semantic_point_bayes.r = rgb_pixel[2];
            semantic_point_bayes.g = rgb_pixel[1];
            semantic_point_bayes.b = rgb_pixel[0];

            // semantic 1
            semantic_pixel = bayes_frame_1.at<cv::Vec3b>(row, col);
            semantic_point_bayes.s1_r = semantic_pixel[2];
            semantic_point_bayes.s1_g = semantic_pixel[1];
            semantic_point_bayes.s1_b = semantic_pixel[0];

            // semantic 2
            semantic_pixel = bayes_frame_2.at<cv::Vec3b>(row, col);
            semantic_point_bayes.s2_r = semantic_pixel[2];
            semantic_point_bayes.s2_g = semantic_pixel[1];
            semantic_point_bayes.s2_b = semantic_pixel[0];

            // semantic 3
            //semantic_pixel = bayes_frame_3.at<cv::Vec3b>(row, col);
            //semantic_point_bayes.s3_r = semantic_pixel[2];
            //semantic_point_bayes.s3_g = semantic_pixel[1];
            //semantic_point_bayes.s3_b = semantic_pixel[0];

            // confidence
            semantic_point_bayes.confidence1 = bayes_semantic->condifence_1.data[row * image_size.width + col];
            semantic_point_bayes.confidence2 = bayes_semantic->condifence_2.data[row * image_size.width + col];
            //semantic_point_bayes.confidence3 = bayes_semantic->condifence_3.data[row * image_size.width + col];

            // 这样做一次转换也不会降低太多频率，不过还是直接用 bayes_semantic 里面的数组
            //semantic_point_bayes.confidence1 = bayes_confidences_1.at<float>(row, col);
            //semantic_point_bayes.confidence2 = bayes_confidences_2.at<float>(row, col);

            out_cloud->points.push_back(semantic_point_bayes);
        }
    }

    sensor_msgs::PointCloud2 bayes_semantic_cloud;
    pcl::toROSMsg(*out_cloud, bayes_semantic_cloud);
    bayes_semantic_cloud.header = cloud_frame.header;

    ROS_INFO("[%s]: [%s] publish bayes semantic cloud.", kNodeName.c_str(), __FUNCTION__);
    pub_bayes_semantic_cloud.publish(bayes_semantic_cloud);
}

#else

/**
  * @brief  融合 image_raw, cloud_raw, semantic_img, confidences 为一帧语义点云
  * @details 雷达目前的发送频率是 10 Hz
  * @param[in] cloud_msg 订阅的一帧原始点云
  * @return void
  * @note
  *     1. 因为语义颜色的解析有 BUG，所以给 PointXYZRGBSemanticsMax 加上结构体作为 semantic_color 的 union
  *             struct {uint8_t s_b; uint8_t s_g; uint8_t s_r; uint8_t s_a;};
  * 
  *     2. 给 Bayes 结构体也加上了 RGBA 的 union struct
  * 
  *     3. 目前没有使用 TF，直接读取外参矩阵
  *     
  *     4. 目前订阅 2 组语义分割的 Bayes 语义后的发布频率为 5 Hz左右，与语义分割的发布频率几乎相同
  * @todo 
  *     image_raw, cloud_raw, semantic_img, confidences 没有进行同步处理！
  *     语义图像暂时没有加上 ID
  *     测试自定义 semantic/max_msg 消息！
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::CloudRawCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // 确保当前融合的原始图像不为空
    if (image_frame.empty()) {
        ROS_INFO("[%s]: image_frame is empty! Waiting for current image frame ...", kNodeName.c_str());
        return;
    }

#if USING_BAYES

    if (bayes_frame_1.empty()) {
        ROS_INFO("[%s]: bayes_frame_1 is empty! Waiting for current bayes_frame_1 ...", kNodeName.c_str());
        return;
    }

    if (bayes_frame_2.empty()) {
        ROS_INFO("[%s]: bayes_frame_2 is empty! Waiting for current bayes_frame_2 ...", kNodeName.c_str());
        return;
    }

    if (bayes_confidences_1.empty()) {
        ROS_INFO("[%s]: bayes_confidences_1 is empty! Waiting for current bayes_confidences_1 ...", kNodeName.c_str());
        return;
    }

    if (bayes_confidences_2.empty()) {
        ROS_INFO("[%s]: bayes_confidences_2 is empty! Waiting for current bayes_confidences_2 ...", kNodeName.c_str());
        return;
    }

#endif

// 关闭代码：把 RGB 当做语义来测试
#if 0
    // 确保当前融合的语义图像不为空
    if (max_frame.empty()) {
        ROS_INFO("[%s]: semantic_frame is empty! Waiting for current semantic frame ...", kNodeName.c_str());
        return;
    }

    // 确保当前融合的置信度矩阵不为空
    if (max_confidences.empty()) {
        ROS_INFO("[%s]: confidence is empty! Waiting for current confidence frame ...", kNodeName.c_str());
        return;
    }
#endif

// 关闭代码：把 RGB 当做语义来测试
#if USING_MAX_SEMANTIC
    // 确保当前融合的语义图像不为空
    if (semantic_frame.empty()) {
        ROS_INFO("[%s]: semantic_frame is empty! Waiting for current semantic frame ...", kNodeName.c_str());
        return;
    }

    // 确保当前融合的置信度矩阵不为空
    if (confidences.empty()) {
        ROS_INFO("[%s]: confidence is empty! Waiting for current confidence frame ...", kNodeName.c_str());
        return;
    }
#endif

    // 再次确保 image_frame_id 不为空，因为 TF 要用到
    if (image_frame_id == "") {
        ROS_INFO("[%s]: image_frame_id is null! Please check image topic sender!", kNodeName.c_str());
        return;
    }

#if USING_TF

    // 从 tf 树中寻找雷达和相机的坐标变换关系
    if (camera_lidar_tf_ok == false)
        camera_lidar_tf = FindTransform(image_frame_id, cloud_msg->header.frame_id);

    // 保证相机内参和坐标转换准备完成
    if (camera_instrinsics_mat_ok == false || camera_lidar_tf_ok == false) {
        ROS_INFO("[%s]: waiting for camera intrinsics and camera lidar tf ...", kNodeName.c_str());
        return;
    }

#else
    // 直接使用读取的外参矩阵
    // 保证相机内参和坐标转换读取成功
    if (camera_instrinsics_mat_ok == false || camera_extrinsic_mat_ok == false) {
        ROS_INFO("[%s]: waiting for camera_instrinsics_mat and camera_extrinsic_mat...", kNodeName.c_str());
        return;
    }
#endif

    // 4. ROS point cloud -> PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);

    // void pcl::fromROSMsg(const sensor_msgs::PointCloud2 &, pcl::PointCloud<T> &);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud_msg);

    auto in_cloud_msg = pcl_cloud_msg;

#if USING_BAYES
    // 初始化 Bayes 概率语义点云
    pcl::PointCloud<PointXYZRGBSemanticsBayesian>::Ptr out_cloud(new pcl::PointCloud<PointXYZRGBSemanticsBayesian>);
    out_cloud->points.clear();

    PointXYZRGBSemanticsBayesian semantic_point_bayes;
    std::vector<PointXYZRGBSemanticsBayesian> cam_cloud(in_cloud_msg->points.size());
#else

    // 初始化最大概率语义点云
    pcl::PointCloud<PointXYZRGBSemanticsMax>::Ptr out_cloud(new pcl::PointCloud<PointXYZRGBSemanticsMax>);
    out_cloud->points.clear();

    PointXYZRGBSemanticsMax semantic_point_max;
    std::vector<PointXYZRGBSemanticsMax> cam_cloud(in_cloud_msg->points.size());

#endif

    int row = 0;
    int col = 0;

    cv::Vec3b rgb_pixel;

    cv::Mat raw_point(4, 1, cv::DataType<double>::type);
    cv::Mat transformed_point(4, 1, cv::DataType<double>::type);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double tmp_z = 0.0;

    // 6. 遍历点云，融合像素
    for (size_t i = 0; i < in_cloud_msg->points.size(); i++) {
#if USING_TF

#else
        // 用 RT 矩阵变换
        raw_point.at<double>(0, 0) = in_cloud_msg->points[i].x;
        raw_point.at<double>(1, 0) = in_cloud_msg->points[i].y;
        raw_point.at<double>(2, 0) = in_cloud_msg->points[i].z;
        raw_point.at<double>(3, 0) = 1;

        // 4 X 1 = 4 X 4 * 4 X 1;
        transformed_point = camera_extrinsic_mat * raw_point;

        x = transformed_point.at<double>(0, 0);
        y = transformed_point.at<double>(1, 0);
        z = transformed_point.at<double>(2, 0);
        // [3][0] = 1;

        // 使用相机内参将三维空间点投影到像素平面
        col = int(x * fx / z + cx);
        row = int(y * fy / z + cy);

        tmp_z = z;
#endif

        // 只融合在像素平面内的点云, TF 没有效果是因为没改 z 的判断条件！
        if ((row >= 0) && (row < image_size.height) && (col >= 0) && (col < image_size.width) && (tmp_z > 0)) {

#if USING_BAYES
            // XYZ
            semantic_point_bayes.x = in_cloud_msg->points[i].x;
            semantic_point_bayes.y = in_cloud_msg->points[i].y;
            semantic_point_bayes.z = in_cloud_msg->points[i].z;

            // RGB
            rgb_pixel = image_frame.at<cv::Vec3b>(row, col);
            semantic_point_bayes.r = rgb_pixel[2];
            semantic_point_bayes.g = rgb_pixel[1];
            semantic_point_bayes.b = rgb_pixel[0];

            // semantic 1
            cv::Vec3b semantic_pixel_1 = bayes_frame_1.at<cv::Vec3b>(row, col);
            semantic_point_bayes.s1_r = semantic_pixel_1[2];
            semantic_point_bayes.s1_g = semantic_pixel_1[1];
            semantic_point_bayes.s1_b = semantic_pixel_1[0];
            // semantic 2
            cv::Vec3b semantic_pixel_2 = bayes_frame_2.at<cv::Vec3b>(row, col);
            semantic_point_bayes.s2_r = semantic_pixel_2[2];
            semantic_point_bayes.s2_g = semantic_pixel_2[1];
            semantic_point_bayes.s2_b = semantic_pixel_2[0];
            // semantic 3 no use!

            // confidence 1 and 2
            semantic_point_bayes.confidence1 = bayes_confidences_1.at<float>(row, col);
            semantic_point_bayes.confidence2 = bayes_confidences_2.at<float>(row, col);
            // confidence 3 no use!

            out_cloud->points.push_back(semantic_point_bayes);

#else
            semantic_point_max.x = in_cloud_msg->points[i].x;
            semantic_point_max.y = in_cloud_msg->points[i].y;
            semantic_point_max.z = in_cloud_msg->points[i].z;

            rgb_pixel = image_frame.at<cv::Vec3b>(row, col);
            semantic_point_max.r = rgb_pixel[2];
            semantic_point_max.g = rgb_pixel[1];
            semantic_point_max.b = rgb_pixel[0];

            // kitti 没有语义，以下逻辑会导致节点报错，使用没有语义的 bag 注释以下代码
            //cv::Vec3b semantic_pixel = semantic_frame.at<cv::Vec3b>(row, col);
#if USING_MAX_SEMANTIC
            // 测试使用：把 RGB 当做语义
            cv::Vec3b semantic_pixel = semantic_frame.at<cv::Vec3b>(row, col);
            //cv::Vec3b semantic_pixel = max_frame.at<cv::Vec3b>(row, col);
#else
            cv::Vec3b semantic_pixel = image_frame.at<cv::Vec3b>(row, col);
#endif

            semantic_point_max.s_r = semantic_pixel[2];
            semantic_point_max.s_g = semantic_pixel[1];
            semantic_point_max.s_b = semantic_pixel[0];

#if USING_MAX_SEMANTIC
            semantic_point_max.confidence = confidences.at<float>(row, col);
            //semantic_point_max.confidence = max_confidences.at<float>(row, col);
#else
            // 测试使用：把 RGB 当做语义，设置固定置信度
            semantic_point_max.confidence = 0.8;
#endif

            out_cloud->points.push_back(semantic_point_max);

#endif
        }
    }

#if USING_BAYES
    sensor_msgs::PointCloud2 bayes_semantic_cloud;
    pcl::toROSMsg(*out_cloud, bayes_semantic_cloud);
    bayes_semantic_cloud.header = cloud_msg->header;

    ROS_INFO("[%s]: [%s] publish bayes_semantic_cloud.", kNodeName.c_str(), __FUNCTION__);

    pub_bayes_semantic_cloud.publish(bayes_semantic_cloud);
#else
    sensor_msgs::PointCloud2 max_semantic_cloud;
    pcl::toROSMsg(*out_cloud, max_semantic_cloud);
    max_semantic_cloud.header = cloud_msg->header;

    ROS_INFO("[%s]: [%s] publish max_semantic_cloud.", kNodeName.c_str(), __FUNCTION__);

    pub_max_semantic_cloud.publish(max_semantic_cloud);
#endif
}

/**
  * @brief 用于订阅 Bayes Fusion 的自定义语义消息
  * @details bayes_semantic 包含前 3 组最大概率的语义标签组成的图像和置信度数组
  * @param[in] bayes_semantic 订阅的 semantic/bayes_msg 自定义语义消息
  * @return void
  * @note 
  *     1. 需要考虑数据传输问题！因为一个 msg 里面包含 3 组图像和置信度！
  *     2. 语义分割产生 2 组语义信息 5Hz，该点云回调为 10 Hz，导致发布的语义点云也为 10Hz
  *        考虑融合的逻辑性，应该在频率最慢的回调中进行语义的融合操作，即应该将融合的逻辑移动到该回调中！
  * @todo 
  *     把点云回调中的融合逻辑移动到这个函数中！
  *     记得让庄明溪修改 confidence 单词！
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::BayesSemanticCallback(const semantic_msg::bayes_msg::ConstPtr& bayes_semantic)
{
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] BayesSemanticCallback: wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // Get 3 best semantic_img
    cv_bridge::CvImagePtr p_semantic_img_1 = cv_bridge::toCvCopy(bayes_semantic->semantic_img_1, "bgr8");
    cv_bridge::CvImagePtr p_semantic_img_2 = cv_bridge::toCvCopy(bayes_semantic->semantic_img_2, "bgr8");
    //cv_bridge::CvImagePtr p_semantic_img_3 = cv_bridge::toCvCopy(bayes_semantic->semantic_img_3, "bgr8");

    if (is_kitti) {
        bayes_frame_1 = p_semantic_img_1->image;
        bayes_frame_2 = p_semantic_img_2->image;
        //bayes_frame_3 = p_semantic_img_3->image;
    } else {
        cv::undistort(p_semantic_img_1->image, bayes_frame_1, camera_instrinsics_mat, distortion_coefficients);
        cv::undistort(p_semantic_img_2->image, bayes_frame_2, camera_instrinsics_mat, distortion_coefficients);
        //cv::undistort(p_semantic_img_3->image, bayes_frame_3, camera_instrinsics_mat, distortion_coefficients);
    }

    // Get 3 best bayes_confidences
    for (int r = 0; r < image_size.height; r++) {
        for (int l = 0; l < image_size.width; l++) {
            // 暂时改为 condifence，因为录制包的时候写错了，记得让庄明溪改回来
            this->bayes_confidences_1.at<float>(r, l) = bayes_semantic->condifence_1.data[r * image_size.width + l];
            this->bayes_confidences_2.at<float>(r, l) = bayes_semantic->condifence_2.data[r * image_size.width + l];
            //this->bayes_confidences_3.at<float>(r, l) = bayes_semantic->confidence_3.data[r * image_size.width + l];
        }
    }
}

#endif

/**
  * @brief 语义分割的最大概率语义图像回调
  * @details 没有使用 semantic_msg/max_msg
  * @param[in] semantic_img 订阅的语义图像
  * @return void
  * @note 
  *     语义分割暂时没有去畸变
  *     语义图像暂时没有加上 ID
  * @todo 
  *     是否需要保存语义图像的 ID？
  *     是否需要保证语义图像与原始图像间的同步？
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::SemanticImageCallback(const sensor_msgs::Image::ConstPtr& semantic_img)
{
    // 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] SemanticImageCallback: wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // ros img -> cv img
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(semantic_img, "bgr8");

    // KITTI 数据集已经矫正过了，不需要去畸变
    if (is_kitti)
        semantic_frame = cv_image_ptr->image;
    else // 我们自己的语义分割没有去畸变，这里要处理
        cv::undistort(cv_image_ptr->image, semantic_frame, camera_instrinsics_mat, distortion_coefficients);
}

/**
  * @brief 语义分割的最大概率置信度回调
  * @details 没有使用 semantic_msg/max_msg
  * @param[in] conf 订阅的置信度矩阵
  * @return void
  * @note 
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::ConfidenceCallback(const rospy_tutorials::Floats::ConstPtr& conf)
{
    // 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] SemanticImageCallback: wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // TODO: init confidences!
    for (int r = 0; r < image_size.height; r++) {
        for (int l = 0; l < image_size.width; l++) {
            this->confidences.at<float>(r, l) = conf->data[r * image_size.width + l];
        }
    }
}

#if SWITCH_FUSION

/**
  * @brief 用于订阅 Max Fusion 的自定义语义消息
  * @details 消息包含一副最大概率的语义分割图像和对应的置信度一维数组
  * @param[in] max_semantic 订阅的 semantic_msg/max_msg 自定义消息
  * @return void
  * @note 
  *     可能需要考虑一次传输的数据量太大的问题
  * @todo 
  *     这个消息还没测试！
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::MaxSemanticCallback(const semantic_msg::max_msg::ConstPtr& max_semantic)
{
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s]: [%s] wait to read camera instrinsics mat!", kNodeName.c_str(), __FUNCTION__);
        return;
    }

    // Get semantic_img_max
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(max_semantic->semantic_img_max, "bgr8");
    cv::undistort(cv_image_ptr->image, max_frame, camera_instrinsics_mat, distortion_coefficients);

    if (image_frame.empty()) {
        ROS_INFO("[%s]: image_frame is empty! Waiting for current image frame ...", kNodeName.c_str());
        return;
    }

    if (cloud_frame.height * cloud_frame.width == 0) {
        ROS_INFO("[%s]: cloud_frame is empty! Waiting for current cloud frame ...", kNodeName.c_str());
        return;
    }

    // 这里没有对置信度进行判断，因为两者是同时发布的，基本不会为空，后期考虑加上置信度的判断
    if (max_frame.empty()) {
        ROS_INFO("[%s]: semantic_frame is empty! Waiting for current semantic frame ...", kNodeName.c_str());
        return;
    }

    // 再次确保 image_frame_id 不为空，因为 TF 要用到
    if (image_frame_id == "") {
        ROS_INFO("[%s]: image_frame_id is null! Please check image topic sender!", kNodeName.c_str());
        return;
    }

#if USING_TF

    // 从 tf 树中寻找雷达和相机的坐标变换关系
    if (camera_lidar_tf_ok == false)
        camera_lidar_tf = FindTransform(image_frame_id, cloud_msg->header.frame_id);

    // 保证相机内参和坐标转换准备完成
    if (camera_instrinsics_mat_ok == false || camera_lidar_tf_ok == false) {
        ROS_INFO("[%s]: waiting for camera intrinsics and camera lidar tf ...", kNodeName.c_str());
        return;
    }

#else
    // 直接使用读取的外参矩阵
    // 保证相机内参和坐标转换读取成功
    if (camera_instrinsics_mat_ok == false || camera_extrinsic_mat_ok == false) {
        ROS_INFO("[%s]: waiting for camera_instrinsics_mat and camera_extrinsic_mat...", kNodeName.c_str());
        return;
    }
#endif

    // ROS point cloud -> PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);

    // void pcl::fromROSMsg(const sensor_msgs::PointCloud2 &, pcl::PointCloud<T> &);
    pcl::fromROSMsg(cloud_frame, *pcl_cloud_msg);

    auto in_cloud_msg = pcl_cloud_msg;

    // 初始化最大概率语义点云
    pcl::PointCloud<PointXYZRGBSemanticsMax>::Ptr out_cloud(new pcl::PointCloud<PointXYZRGBSemanticsMax>);
    out_cloud->points.clear();

    PointXYZRGBSemanticsMax semantic_point_max;
    std::vector<PointXYZRGBSemanticsMax> cam_cloud(in_cloud_msg->points.size());

    int row = 0;
    int col = 0;

    cv::Vec3b rgb_pixel;

    cv::Mat raw_point(4, 1, cv::DataType<double>::type);
    cv::Mat transformed_point(4, 1, cv::DataType<double>::type);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double tmp_z = 0.0;

    cv::Vec3b semantic_pixel;

    // 遍历点云，融合像素
    for (size_t i = 0; i < in_cloud_msg->points.size(); i++) {
#if USING_TF

#else
        // 用 RT 矩阵变换
        raw_point.at<double>(0, 0) = in_cloud_msg->points[i].x;
        raw_point.at<double>(1, 0) = in_cloud_msg->points[i].y;
        raw_point.at<double>(2, 0) = in_cloud_msg->points[i].z;
        raw_point.at<double>(3, 0) = 1;

        // 4 X 1 = 4 X 4 * 4 X 1;
        transformed_point = camera_extrinsic_mat * raw_point;

        x = transformed_point.at<double>(0, 0);
        y = transformed_point.at<double>(1, 0);
        z = transformed_point.at<double>(2, 0);
        // [3][0] = 1;

        // 使用相机内参将三维空间点投影到像素平面
        col = int(x * fx / z + cx);
        row = int(y * fy / z + cy);

        tmp_z = z;
#endif

        // 只融合在像素平面内的点云, TF 没有效果是因为没改 z 的判断条件！
        if ((row >= 0) && (row < image_size.height) && (col >= 0) && (col < image_size.width) && (tmp_z > 0)) {
            // XYZ
            semantic_point_max.x = in_cloud_msg->points[i].x;
            semantic_point_max.y = in_cloud_msg->points[i].y;
            semantic_point_max.z = in_cloud_msg->points[i].z;

            // RGB
            rgb_pixel = image_frame.at<cv::Vec3b>(row, col);
            semantic_point_max.r = rgb_pixel[2];
            semantic_point_max.g = rgb_pixel[1];
            semantic_point_max.b = rgb_pixel[0];

            // Max semantic img
            semantic_pixel = semantic_frame.at<cv::Vec3b>(row, col);
            semantic_point_max.s_r = semantic_pixel[2];
            semantic_point_max.s_g = semantic_pixel[1];
            semantic_point_max.s_b = semantic_pixel[0];

            // Max confidence
            semantic_point_max.confidence = max_semantic->confidence_max.data[row * image_size.width + col];

            out_cloud->points.push_back(semantic_point_max);
        }
    }

    sensor_msgs::PointCloud2 max_semantic_cloud;
    pcl::toROSMsg(*out_cloud, max_semantic_cloud);
    max_semantic_cloud.header = cloud_frame.header;

    ROS_INFO("[%s]: [%s] publish max semantic cloud.", kNodeName.c_str(), __FUNCTION__);
    pub_max_semantic_cloud.publish(max_semantic_cloud);
}

#else

/**
  * @brief 用于订阅 Max Fusion 的自定义语义消息
  * @details 消息包含一副最大概率的语义分割图像和对应的置信度一维数组
  * @param[in] max_semantic 订阅的 semantic_msg/max_msg 自定义消息
  * @return void
  * @note 
  *     可能需要考虑一次传输的数据量太大的问题
  * @todo 
  *     这个消息还没测试！
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::MaxSemanticCallback(const semantic_msg::max_msg::ConstPtr& max_semantic)
{
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s]: [%s] wait to read camera instrinsics mat!", kNodeName.c_str(), __FUNCTION__);
        return;
    }

    // Get semantic_img_max
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(max_semantic->semantic_img_max, "bgr8");

    if (is_kitti)
        max_frame = cv_image_ptr->image;
    else
        cv::undistort(cv_image_ptr->image, max_frame, camera_instrinsics_mat, distortion_coefficients);

    // Get max_confidences
    for (int r = 0; r < image_size.height; r++) {
        for (int l = 0; l < image_size.width; l++) {
            this->max_confidences.at<float>(r, l) = max_semantic->confidence_max.data[r * image_size.width + l];
        }
    }
}

#endif

tf::StampedTransform LidarCameraFusion::FindTransform(const std::string& target_frame, const std::string& source_frame)
{
    tf::StampedTransform transform;

    camera_lidar_tf_ok = false;

    try {
        transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        camera_lidar_tf_ok = true;
        ROS_INFO("[%s]: camera_lidar_tf Get!", kNodeName.c_str());
    } catch (tf::TransformException e) {
        ROS_INFO("[%s]: %s", kNodeName.c_str(), e.what());
    }

    return transform;
}

pcl::PointXYZ LidarCameraFusion::TransformPoint(const pcl::PointXYZ& in_point, const tf::StampedTransform& in_transform)
{
    tf::Vector3 point(in_point.x, in_point.y, in_point.z);

    tf::Vector3 point_transform = in_transform * point;

    return pcl::PointXYZ(point_transform.x(), point_transform.y(), point_transform.z());
}

/**
 * @brief: 对融合后的点云做欧拉聚类分割
 * @param[in]: in_cloud 要聚类的点云
 * @param[in]: cluster_tolerance 聚类容忍度
 * @param[in]: min_cluster_size 聚类的最小点云数量
 * @param[in]: max_cluster_size 聚类的最大点云数量
 * @return: 聚类的结果 std::vector<pcl::PointIndices>，每行代表一个聚类簇，pcl::PointIndeices = std::vector<int>
 * @author: DLonng
 * @todo:
 *      聚类工作还未开始！
 * @date: 2020-07-24
 */
void LidarCameraFusion::EucCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
    std::vector<pcl::PointIndices>& cluster_indices,
    int cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size)
{
    // 设置使用 kdtree 搜索
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kd_tree->setInputCloud(in_cloud);

    // 创建欧拉聚类对象
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euc;

    // 设置聚类参数
    euc.setClusterTolerance(cluster_tolerance);
    euc.setMinClusterSize(min_cluster_size);
    euc.setMaxClusterSize(max_cluster_size);

    // 设置使用 kdtree 搜索最近邻居
    euc.setSearchMethod(kd_tree);

    euc.setInputCloud(in_cloud);

    // 执行欧拉聚类
    euc.extract(cluster_indices);
}
