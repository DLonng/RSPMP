#include "lidar_camera_fusion.h"

#define USING_TF 0

const std::string LidarCameraFusion::kNodeName = "lidar_camera_fusion";

LidarCameraFusion::LidarCameraFusion()
    : param_handle("~")
    , camera_lidar_tf_ok(false)
    , camera_instrinsics_mat_ok(false)
    , camera_extrinsic_mat_ok(false)
    , image_frame_id("")
{
    InitROS();
}

/**
  * @brief 初始化该节点需要用的参数和话题
  * @details 读取外惨文件、订阅图像和点云，发布 2 种类型的语义点云
  * @return void
  * @author DLonng
  * @date 2020-08-10
  */
void LidarCameraFusion::InitROS()
{
    std::string calibration_file;
    param_handle.param<std::string>("calibration_file", calibration_file, "");

    if (calibration_file.empty()) {
        ROS_ERROR("[%s]: missing calibration file path '%s'. ", kNodeName.c_str(), calibration_file.c_str());
        return;
    }

    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("[%s]: cannot open file calibration file [%s]. ", kNodeName.c_str(), calibration_file.c_str());
        return;
    }

    fs["CameraExtrinsicMat"] >> camera_extrinsic_mat;
    camera_extrinsic_mat = camera_extrinsic_mat.inv();
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

    fs["ImageSize"] >> image_size;
    fs["DistCoeff"] >> distortion_coefficients;
    fs["DistModel"] >> dist_model;

    std::string image_raw;
    std::string cloud_raw;

    param_handle.param<std::string>("image_raw", image_raw, "/camera/left/image_raw");
    param_handle.param<std::string>("cloud_raw", cloud_raw, "/rslidar_points");

    // 订阅 image_input 话题
    // 第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到 1 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。
    // 当处理消息速度不够时，可以调大第二个参数！
    // 收到订阅消息后调用 ImageCallback 处理图像 msg
    int topic_buff = 5;
    sub_image_raw = topic_handle.subscribe(image_raw, topic_buff, &LidarCameraFusion::ImageRawCallback, this);
    sub_cloud_raw = topic_handle.subscribe(cloud_raw, topic_buff, &LidarCameraFusion::CloudRawCallback, this);

    std::string max_semantic;
    std::string bayes_semantic;

    param_handle.param<std::string>("max_semantic", max_semantic, "/max_semantic");
    param_handle.param<std::string>("bayes_semantic", bayes_semantic, "/bayes_semantic");

    sub_max_semantic = topic_handle.subscribe(max_semantic, topic_buff, &LidarCameraFusion::MaxSemanticCallback, this);
    sub_bayes_semantic = topic_handle.subscribe(bayes_semantic, topic_buff, &LidarCameraFusion::BayesSemanticCallback, this);

    std::string semantic_cloud_max;
    std::string semantic_cloud_bayes;

    param_handle.param<std::string>("semantic_cloud_max", semantic_cloud_max, "/semantic_cloud_max");
    param_handle.param<std::string>("semantic_cloud_bayes", semantic_cloud_bayes, "/semantic_cloud_bayes");

    // 在 fusion_topic 上发布 sensor_msgs::PointCloud2 类型的消息
    // 第二个参数为缓冲区大小，缓冲区被占满后会丢弃先前发布的消息，目前为 1，可能需要更改！
    pub_max_semantic_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(semantic_cloud_max, topic_buff);
    pub_bayes_semantic_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(semantic_cloud_bayes, topic_buff);

    std::string semantic_img_topic;
    param_handle.param<std::string>("semantic_img_topic", semantic_img_topic, "/semantic_img_topic");

    pub_semantic_img = topic_handle.advertise<sensor_msgs::Image>(semantic_img_topic, topic_buff);
}

/**
  * @brief ZED 左相机图像订阅回调函数
  * @details 相机目前的发送频率是 20 Hz
  * @param[in] image_msg ROS Image Ptr
  * @return void
  * @note
  *     1. 注意去畸变
  *     2. 没有处理同步问题
  * @author DLonng
  * @date 2020-08-10
  */
void LidarCameraFusion::ImageRawCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    // 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // ros_img -> cv_imag
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");

 
    // cv_image: 原畸变 OpenCV 图像，image_frame：去畸变后的图像
    // 我们自己做语义分割之前图像没有去畸变，这里调用 opencv undistort
    cv::undistort(cv_image_ptr->image, image_frame, camera_instrinsics_mat, distortion_coefficients);
    

    // 发布去畸变的图像消息
    //static image_transport::ImageTransport it(topic_handle);
    //static image_transport::Publisher pub_image = it.advertise("identified_image", 1);
    //static sensor_msgs::ImagePtr msg;
    //msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_frame).toImageMsg();
    //pub_image.publish(msg);

    image_frame_id = image_msg->header.frame_id;
    image_size.height = image_frame.rows;
    image_size.width = image_frame.cols;
}

/**
  * @brief 订阅点云的回调
  * @details 雷达目前的发送频率是 10 Hz
  * @param[in] cloud_msg 订阅的一帧原始点云
  * @return void
  * @note
  *     没有处理同步问题
  * @author DLonng
  * @date 2020-08-10
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
  *     2. 目前只使用 2 组语义
  * @todo 
  *     让庄明溪修改 semantic/bayes_msg 里面的 confidence 单词！
  * @author DLonng
  * @date 2020-08-10
  */
void LidarCameraFusion::BayesSemanticCallback(const semantic_msg::bayes_msg::ConstPtr& bayes_semantic)
{
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] BayesSemanticCallback: wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    pub_semantic_img.publish(bayes_semantic->semantic_img_1);

    cv_bridge::CvImagePtr p_semantic_img_1 = cv_bridge::toCvCopy(bayes_semantic->semantic_img_1, "bgr8");
    cv_bridge::CvImagePtr p_semantic_img_2 = cv_bridge::toCvCopy(bayes_semantic->semantic_img_2, "bgr8");
    //cv_bridge::CvImagePtr p_semantic_img_3 = cv_bridge::toCvCopy(bayes_semantic->semantic_img_3, "bgr8");

    cv::undistort(p_semantic_img_1->image, bayes_frame_1, camera_instrinsics_mat, distortion_coefficients);
    cv::undistort(p_semantic_img_2->image, bayes_frame_2, camera_instrinsics_mat, distortion_coefficients);
    //cv::undistort(p_semantic_img_3->image, bayes_frame_3, camera_instrinsics_mat, distortion_coefficients);

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

    //if (bayes_frame_3.empty()) {
    //    ROS_INFO("[%s]: bayes_frame_3 is empty! Waiting for current bayes_frame_3 ...", kNodeName.c_str());
    //    return;
    //}

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
       // //ROS_INFO("[%s]: in_cloud_msg x %f y %f z %f", kNodeName.c_str(), in_cloud_msg->points[i].x, in_cloud_msg->points[i].y, in_cloud_msg->points[i].z);
       // 
       // geometry_msgs::PointStamped rslidar_point; 
       // rslidar_point.header.frame_id = "rslidar";
       // 
       // rslidar_point.point.x = in_cloud_msg->points[i].x;
       // rslidar_point.point.y = in_cloud_msg->points[i].y;
       // rslidar_point.point.z = in_cloud_msg->points[i].z;

       // geometry_msgs::PointStamped left_point; 

       // transform_listener.transformPoint(image_frame_id, rslidar_point, left_point);
       // 
       // //ROS_INFO("[%s]:    cam_cloud x %f y %f z %f", kNodeName.c_str(), cam_cloud[i].x, cam_cloud[i].y, cam_cloud[i].z);
       // //ROS_INFO("[%s]:    left_cloud x %f y %f z %f", kNodeName.c_str(), left_point.point.x, left_point.point.y, left_point.point.z);
       // 
       // col = int(left_point.point.x * fx / left_point.point.z + cx);
       // row = int(left_point.point.y * fy / left_point.point.z + cy);
       // 
       // //ROS_INFO("[%s]: col %d row %d z %f", kNodeName.c_str(), col, row, left_point.point.z);
      

        cam_cloud[i] = TransformPoint(in_cloud_msg->points[i], camera_lidar_tf);

        col = int(cam_cloud[i].x * fx / cam_cloud[i].z + cx);
        row = int(cam_cloud[i].y * fy / cam_cloud[i].z + cy);

        tmp_z = cam_cloud[i].z;
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
            semantic_point_bayes.confidence1 = bayes_semantic->confidence_1.data[row * image_size.width + col];
            semantic_point_bayes.confidence2 = bayes_semantic->confidence_2.data[row * image_size.width + col];
            //semantic_point_bayes.confidence3 = bayes_semantic->condifence_3.data[row * image_size.width + col];

            out_cloud->points.push_back(semantic_point_bayes);
        }
    }

    sensor_msgs::PointCloud2 bayes_semantic_cloud;
    pcl::toROSMsg(*out_cloud, bayes_semantic_cloud);
    bayes_semantic_cloud.header = cloud_frame.header;

    ROS_INFO("[%s]: [%s] publish bayes semantic cloud.", kNodeName.c_str(), __FUNCTION__);
    pub_bayes_semantic_cloud.publish(bayes_semantic_cloud);
}

/**
  * @brief 用于订阅 Max Fusion 的自定义语义消息
  * @details 消息包含一副最大概率的语义分割图像和对应的置信度一维数组
  * @param[in] max_semantic 订阅的 semantic_msg/max_msg 自定义消息
  * @return void
  * @note 
  *     语义分割节点需要对置信度矩阵强制类型转换导致耗时，暂时没解决
  * @todo 
  *     这个消息还没测试！
  * @author DLonng
  * @date 2020-08-10
  */
void LidarCameraFusion::MaxSemanticCallback(const semantic_msg::max_msg::ConstPtr& max_semantic)
{
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s]: [%s] wait to read camera instrinsics mat!", kNodeName.c_str(), __FUNCTION__);
        return;
    }

    pub_semantic_img.publish(max_semantic->semantic_img_max);

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
       // //ROS_INFO("[%s]: in_cloud_msg x %f y %f z %f", kNodeName.c_str(), in_cloud_msg->points[i].x, in_cloud_msg->points[i].y, in_cloud_msg->points[i].z);
       // 
       // geometry_msgs::PointStamped rslidar_point; 
       // rslidar_point.header.frame_id = "rslidar";
       // 
       // rslidar_point.point.x = in_cloud_msg->points[i].x;
       // rslidar_point.point.y = in_cloud_msg->points[i].y;
       // rslidar_point.point.z = in_cloud_msg->points[i].z;

       // geometry_msgs::PointStamped left_point; 

       // transform_listener.transformPoint(image_frame_id, rslidar_point, left_point);
       // 
       // //ROS_INFO("[%s]:    cam_cloud x %f y %f z %f", kNodeName.c_str(), cam_cloud[i].x, cam_cloud[i].y, cam_cloud[i].z);
       // //ROS_INFO("[%s]:    left_cloud x %f y %f z %f", kNodeName.c_str(), left_point.point.x, left_point.point.y, left_point.point.z);
       // 
       // col = int(left_point.point.x * fx / left_point.point.z + cx);
       // row = int(left_point.point.y * fy / left_point.point.z + cy);
       // 
       // //ROS_INFO("[%s]: col %d row %d z %f", kNodeName.c_str(), col, row, left_point.point.z);
      

        cam_cloud[i] = TransformPoint(in_cloud_msg->points[i], camera_lidar_tf);

        col = int(cam_cloud[i].x * fx / cam_cloud[i].z + cx);
        row = int(cam_cloud[i].y * fy / cam_cloud[i].z + cy);

        tmp_z = cam_cloud[i].z;
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
            semantic_pixel = max_frame.at<cv::Vec3b>(row, col);
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


/**
 * @brief: 寻找 2 个坐标系之间的转换
 * @param[in]: target_frame 目标帧
 * @param[in]: source_frame 源帧
 * @return: transform 从 TF 树中找到的变换
 * @note: 目前没有使用 TF
 * @author: DLonng
 * @date: 2020-07-24
 */
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

/**
 * @brief: 将一个点进行转换
 * @param[in]: in_point 目标点
 * @param[in]: in_transform 使用的变换
 * @return: pcl::PointXYZ 变换后的点
 * @note: 目前没有使用 TF，并且该函数可以被 transform_listener.transformPoint 代替
 * @author: DLonng
 * @date: 2020-07-24
 */
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
