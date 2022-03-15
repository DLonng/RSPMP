#include "sem_dwa_planner/sem_dwa_planner_ros.h"

using namespace sem_dwa_local_planner;

SemDWAPlannerROS::SemDWAPlannerROS()
    : local_nh_("~")
    , local_goal_subscribed_(false)
    , local_map_updated_(false)
    , odom_updated_(false)
    , sem_dwa_planner_(nullptr)
    , use_cmu_planning_(true)
    , start_time_(0)
    , path_len_(0)
    , path_len2_(0.0)
{

    local_nh_.param("local_map_frame", local_map_frame_, { "map" });
    local_nh_.param("robot_frame", robot_frame_, { "base_link" });
    local_nh_.param("HZ", hz_, { 10 });
    local_nh_.param("use_cmu_planning", use_cmu_planning_, { true });
    local_nh_.param("send_goal_model", send_goal_model_, { false });
    local_nh_.param("multi_goal_model", multi_goal_model_, { false });
    local_nh_.param("local_goal_tor", local_goal_tor_, { 2.0 });
    local_nh_.param("global_goal_tor", global_goal_tor_, { 1.0 });
    local_nh_.param("goal_z_tor", goal_z_tor_, { 0.5 });
    local_nh_.param("rviz_traj_z", rviz_traj_z_, { 0.5 });

    local_nh_.param("select_z", select_z_, { 0.5 });
    local_nh_.param("trave_z", trave_z_, { 0.5 });
    local_nh_.param("untrave_z", untrave_z_, { 0.5 });
    local_nh_.param("cmu_z", cmu_z_, { 0.5 });
    local_nh_.param("best_cmu_z", best_cmu_z_, { 0.5 });
    local_nh_.param("width_z", width_z_, { 0.5 });

    std::cout << "local_map_frame: " << local_map_frame_ << std::endl;
    std::cout << "robot_frame: " << robot_frame_ << std::endl;
    std::cout << "HZ: " << hz_ << std::endl;
    std::cout << "use_cmu_planning: " << use_cmu_planning_ << std::endl;
    std::cout << "send_goal_model: " << send_goal_model_ << std::endl;
    std::cout << "multi_goal_model: " << multi_goal_model_ << std::endl;
    std::cout << "local_goal_tor: " << local_goal_tor_ << std::endl;
    std::cout << "global_goal_tor: " << global_goal_tor_ << std::endl;
    std::cout << "goal_z_tor: " << goal_z_tor_ << std::endl;
    std::cout << "rviz_traj_z: " << rviz_traj_z_ << std::endl;

    std::cout << "select_z: " << select_z_ << std::endl;
    std::cout << "trave_z: " << trave_z_ << std::endl;
    std::cout << "untrave_z: " << untrave_z_ << std::endl;
    std::cout << "cmu_z: " << cmu_z_ << std::endl;
    std::cout << "best_cmu_z: " << best_cmu_z_ << std::endl;
    std::cout << "width_z: " << width_z_ << std::endl;

    robot_traj_.poses.clear();

    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    trave_trajectories_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("/candidate_trajectories", 1);
    cmu_trajectories_pub_  = local_nh_.advertise<visualization_msgs::MarkerArray>("/cmu_trajectories", 1);
    width_trajectories_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("/width_trajectories", 1);
    untrave_trajectories_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("/untrave_trajectories", 1);
    selected_trajectory_pub_ = local_nh_.advertise<visualization_msgs::Marker>("/selected_trajectory", 1);
    best_cmu_trajectory_pub_ = local_nh_.advertise<visualization_msgs::Marker>("/best_cmu_trajectory", 1);
    current_goal_pub_ = local_nh_.advertise<visualization_msgs::Marker>("/current_goal", 1);
    robot_traj_pub_ = nh_.advertise<nav_msgs::Path>("/robot_traj", 1);

    local_goal_sub_ = nh_.subscribe("/local_goal", 1, &SemDWAPlannerROS::LocalGoalCallback, this);
    local_map_sub_ = nh_.subscribe("/local_map", 1, &SemDWAPlannerROS::LocalMapCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &SemDWAPlannerROS::OdomCallback, this);
    rviz_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &SemDWAPlannerROS::RvizGoalCallback, this);

    sem_dwa_planner_ = new SemDWAPlanner();

    record_vel_x_.clear();

    std::cout << "SemDWAPlannerROS Init OK!" << std::endl;
    std::cout << std::endl;
}

SemDWAPlannerROS::~SemDWAPlannerROS()
{
    if (sem_dwa_planner_)
        delete sem_dwa_planner_;
}

void SemDWAPlannerROS::Process(void)
{
    ros::Rate loop_rate(hz_);
    bool input_updated = false;

    tf::StampedTransform baseToWorldTf;

    

    while (ros::ok()) {
        double start_loop = ros::Time::now().toSec();
        input_updated = false;

        if (local_map_updated_)
            input_updated = true;
        
        double cmd_v_x = 0;

        if (input_updated && local_goal_subscribed_ && odom_updated_) {
            double start_planning = ros::Time::now().toSec();
            
            // 把当前的全局目标航点转到 base_link 坐标系下
            listener_.transformPose(robot_frame_, ros::Time(0), current_global_goal_, current_global_goal_.header.frame_id, current_local_goal_);
            Eigen::Vector3d local_goal(current_local_goal_.pose.position.x, current_local_goal_.pose.position.y, tf::getYaw(current_local_goal_.pose.orientation));

            if (send_goal_model_) {
                //std::cout << "curren_global_goal: \n" << current_global_goal_ << std::endl;
                //std::cout << std::endl;
                //std::cout << "curren_local_goal: \n" << current_local_goal_ << std::endl;
                ;
            }

            trave_trajectories_.clear();
            cmu_trajectories_.clear();
            untrave_trajectories_.clear();
            width_trajectories_.clear();
            best_traj_.clear();
            best_cmu_traj_.clear();


            if (use_cmu_planning_)
                best_traj_ = sem_dwa_planner_->CMUPlanning(local_map_, local_goal, current_velocity_, trave_trajectories_, cmu_trajectories_, untrave_trajectories_, width_trajectories_, best_cmu_traj_);
            else
                best_traj_ = sem_dwa_planner_->SemDWAPlanning(local_map_, local_goal, current_velocity_, trave_trajectories_);


            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = best_traj_[0].velocity_;
            cmd_vel.angular.z = best_traj_[0].yawrate_;

            //cmd_v_x = cmd_vel.linear.x;
            path_len2_ += cmd_vel.linear.x * (1 / hz_);

            record_vel_x_.push_back(cmd_vel.linear.x);
            record_vel_z_.push_back(cmd_vel.angular.z);

            //std::cout << "cmd_vel.linear.x: " << cmd_vel.linear.x << std::endl;
            //std::cout << "cmd_vel.angular.z: " << cmd_vel.angular.z << std::endl;
            //std::cout << std::endl;

            VisualizeCurrentGoal(current_global_goal_, 1, 0, 0, current_goal_pub_);
            
            // 线段
            VisualizeTrajectories(trave_trajectories_, 0, 1, 0, visualization_msgs::Marker::LINE_STRIP, trave_trajectories_.size(), trave_z_, trave_trajectories_pub_);
            VisualizeTrajectories(untrave_trajectories_, 0, 0, 0, visualization_msgs::Marker::LINE_STRIP, untrave_trajectories_.size(), untrave_z_, untrave_trajectories_pub_);
            
            // 点
            VisualizeTrajectories(cmu_trajectories_, 1, 1, 0, visualization_msgs::Marker::LINE_STRIP, cmu_trajectories_.size(), cmu_z_, cmu_trajectories_pub_);
            // 线段
            VisualizeTrajectory(best_cmu_traj_, 0, 1, 1, visualization_msgs::Marker::LINE_STRIP, best_cmu_z_, best_cmu_trajectory_pub_);
            // 线段
            VisualizeTrajectories(width_trajectories_, 0, 0, 1, visualization_msgs::Marker::LINE_STRIP, width_trajectories_.size(), width_z_, width_trajectories_pub_);
            
            VisualizeTrajectory(best_traj_, 1, 0, 0, visualization_msgs::Marker::LINE_STRIP, select_z_, selected_trajectory_pub_);

            VisualizeRobotTrajectory();

            velocity_pub_.publish(cmd_vel);



            //ROS_INFO_STREAM("planning time: " << ros::Time::now().toSec() - start_planning << "[s]");

            local_map_updated_ = false;
            odom_updated_ = false;
        } else {
            if (!local_goal_subscribed_) {
                //std::cout << "waiting for local goal" << std::endl;
                ;
            }

            if (!local_map_updated_) {
                //std::cout << "waiting for local map" << std::endl;
                ;
            }
        }

        // 只有当前是发送目标模式并且接收到局部目标才判断是否到达目标点
        // 这个最好放在上面的循环内部？
        if (send_goal_model_ && local_goal_subscribed_) {
#if 0
            try {
                listener_.waitForTransform(current_global_goal_.header.frame_id, robot_frame_, ros::Time(0), ros::Duration(3.0));
                listener_.lookupTransform(current_global_goal_.header.frame_id, robot_frame_, ros::Time(0), baseToWorldTf);
            } catch (tf::TransformException& ex) {
                ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
                return;
            }

            Eigen::Vector3d base_origin(baseToWorldTf.getOrigin().x(), baseToWorldTf.getOrigin().y(), tf::getYaw(baseToWorldTf.getRotation()));
            // z == 0？
            Eigen::Vector3d map_goal(current_global_goal_.pose.position.x, current_global_goal_.pose.position.y, tf::getYaw(current_global_goal_.pose.orientation));


            if ((map_goal.segment(0, 2) - base_origin.segment(0, 2)).norm() < goal_x_tor_) {
                // 如果没有到达终的目标点，则切换到下一个目标点
                if (!way_point_queue_.empty()) {
                    std::cout << "Temp Goal is reach!" << std::endl;
                    current_global_goal_ = way_point_queue_.front();

                    // 到达一个目标点的范围时，将该目标点从队列头部弹出
                    way_point_queue_.pop_front();
                } else {
                    // 到达最终的目标点附近，停止即可，重置目标标志位，等待下一次规划
                    std::cout << "Final Goal is reach!!!" << std::endl;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;

                    if (std::fabs(map_goal[2]) > goal_z_tor_) {
                        // 让小车原地旋转到指定角度
                        cmd_vel.angular.z = std::min(std::max(map_goal[2], -sem_dwa_planner_->GetMaxYawrate()), sem_dwa_planner_->GetMaxYawrate());
                    } else {
                        cmd_vel.angular.z = 0;
                        // 只有旋转到指定角度才停止
                        local_goal_subscribed_ = false;
                    }

                    velocity_pub_.publish(cmd_vel);
                }
            }
#else
            // 在 base_link 坐标系下判断是否到达目标点
            // 在临时目标点多次弹出航点可能是因为切换全局目标点后，current_local_goal_ 没有及时更新到 base_link 下
            listener_.transformPose(robot_frame_, ros::Time(0), current_global_goal_, current_global_goal_.header.frame_id, current_local_goal_);
            Eigen::Vector3d local_goal(current_local_goal_.pose.position.x, current_local_goal_.pose.position.y, tf::getYaw(current_local_goal_.pose.orientation));

            if (local_goal.segment(0, 2).norm() < (way_point_queue_.empty() ? global_goal_tor_ : local_goal_tor_)) {
                // 如果没有到达终的目标点，则切换到下一个目标点，每次到达一个目标点只能切换一次，不能多次弹出
                if ((!way_point_queue_.empty())) {
                    std::cout << "Current global goal(" << current_global_goal_.pose.position.x << ", " << current_global_goal_.pose.position.y << ")" << " has reached!" << std::endl;
                    
                    // 到达一个目标点的范围时，将该目标点从队列头部弹出
                    current_global_goal_ = way_point_queue_.front();
                    way_point_queue_.pop_front();

                    std::cout << "Switch to next global goal(" << current_global_goal_.pose.position.x << ", " << current_global_goal_.pose.position.y << ")" << std::endl;
                    std::cout << "Current way_point_queue.size() = " << way_point_queue_.size() << std::endl;
                    std::cout << std::endl;
                } else {
                    // 到达最终的目标点附近，停止即可，重置目标标志位，等待下一次规划
                    std::cout << "Final global goal(" << current_global_goal_.pose.position.x << ", " << current_global_goal_.pose.position.y << ")" << " has reached!" << std::endl;
                    std::cout << std::endl;
                    double planning_time = ros::Time::now().toSec() - start_time_;
                    std::cout << "Planning time: " << planning_time << "[s]" << std::endl;
                    //std::cout << "Path len: " << path_len_ << "[m]" << std::endl;
                    std::cout << "Path len2: " << path_len2_ << "[m]" << std::endl;
                    double pathlen = path_len2_;
                    std::cout << std::endl;
                    path_len_ = 0;
                    // len2 是正确的路径长度
                    path_len2_ = 0;

                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;

                    /*
                    if (std::fabs(local_goal[2]) > goal_z_tor_) {
                        // 让小车原地旋转到指定角度
                        cmd_vel.angular.z = std::min(std::max(local_goal[2], -sem_dwa_planner_->GetMaxYawrate()), sem_dwa_planner_->GetMaxYawrate());
                        std::cout << "Adjust yaw..." << std::endl;
                    } else {
                        cmd_vel.angular.z = 0;
                        // 只有旋转到指定角度才停止
                        local_goal_subscribed_ = false;
                        std::cout << "Adjust yaw ok!" << std::endl;
                    }
                    */

                   // 论文做实验取消到达目标点的航向调整，主要是为了方便直接停车
                   cmd_vel.angular.z = 0;
                   local_goal_subscribed_ = false;

                    velocity_pub_.publish(cmd_vel);

                    // 将线速度和角速度写入文件
                    std::string time_len = std::to_string(planning_time) + "[s]_" + std::to_string(pathlen) + "[m]";

                    std::fstream data_out;
                    std::string vel_x_file_name = "/home/zmx/mapping_navigation_code/jetson_nav_ws/" + time_len + "vel_x.txt";
                    data_out.open(vel_x_file_name, std::fstream::out);
                    if (!data_out.is_open()) {
                        std::cout << "file open fail." << std::endl;
                    }

                    data_out << std::setprecision(3);
                    double t = 0;
                    for (auto& x : record_vel_x_) {
                        data_out << t << "\t" << x << '\n';
                        t += 0.01;
                    }

                    data_out.close();
                    std::cout << "Save vel x to " << vel_x_file_name << std::endl;

                    std::string vel_z_file_name = "/home/zmx/mapping_navigation_code/jetson_nav_ws/" + time_len + "vel_z.txt";
                    data_out.open(vel_z_file_name, std::fstream::out);
                    if (!data_out.is_open()) {
                        std::cout << "file open fail." << std::endl;
                    }

                    data_out << std::setprecision(3);
                    t = 0;
                    for (auto& z : record_vel_z_) {
                        data_out << t << "\t" << z << '\n';
                        t += 0.01;
                    }

                    data_out.close();
                    std::cout << "Save vel z to " << vel_z_file_name << std::endl;

                    record_vel_x_.clear();
                    record_vel_z_.clear();
                }
            }
#endif
        }

        ros::spinOnce();
        loop_rate.sleep();

        
        //std::cout << "loop path delta: " << cmd_v_x * (1 / hz_) << std::endl;
        //std::cout << "loop path_len2: " << path_len2_ << std::endl;

        //ROS_INFO_STREAM("loop time: " << ros::Time::now().toSec() - start_loop << "[s]");
    }
}

// 暂时不用这个函数，目标从 RVIZ 中发布
void SemDWAPlannerROS::LocalGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // 保存 map 下的目标，用于计算是否到达目标点附近，不需要转换到 base_link
    current_global_goal_ = *msg;

    // 用于计算目标代价函数，需要转换到 base_link
    //local_goal_ = *msg;

    try {
        listener_.transformPose(robot_frame_, ros::Time(0), current_global_goal_, current_global_goal_.header.frame_id, local_goal_);
        local_goal_subscribed_ = true;
    } catch (tf::TransformException ex) {
        std::cout << ex.what() << std::endl;
    }
}

void SemDWAPlannerROS::RvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (multi_goal_model_ == true) {
        InitWayPoints();
    } else {
        // rviz 发送的目标 frame 是 base_link
        local_goal_ = *msg;

        try {
            // local_goal_ 需要转到 map 下, 因为是用 map 下的坐标来计算是否到达目标点
            // 目标代价函数也用全局目标点的坐标来计算
            listener_.transformPose(local_map_frame_, ros::Time(0), local_goal_, local_goal_.header.frame_id, current_global_goal_);
            
            // 每次接收一个新的目标点，将之前的目标点清除
            way_point_queue_.clear();
            way_point_queue_.push_back(current_global_goal_);
        } catch (tf::TransformException ex) {
            std::cout << ex.what() << std::endl;
        }
    }

    // 拿到航点队列中的第一个目标点，测试时航点队列不为空
    current_global_goal_ = way_point_queue_.front();
    way_point_queue_.pop_front();
    
    std::cout << "First global goal (" << current_global_goal_.pose.position.x << ", " << current_global_goal_.pose.position.y << ")" << std::endl;
    std::cout << "Current way_point_queue.size() = " << way_point_queue_.size() << std::endl;
    std::cout << std::endl;

    start_time_ = ros::Time::now().toSec();
    // 清除轨迹，准备当前局部规划
    robot_traj_.poses.clear();
    // 记录规划期间的线速度
    record_vel_x_.clear();
    // 记录规划期间的角速度
    record_vel_z_.clear();
    local_goal_subscribed_ = true;
}

void SemDWAPlannerROS::LocalMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    local_map_ = *msg;
    local_map_updated_ = true;
}

void SemDWAPlannerROS::OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity_ = msg->twist.twist;
    odom_updated_ = true;
}

void SemDWAPlannerROS::VisualizeTrajectories(const std::vector<std::vector<State>>& trajectories, const double r, const double g, const double b, const int type, const int trajectories_size, const double traj_z, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();

    for (; count < size; count++) {
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = robot_frame_;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.color.r = r;
        v_trajectory.color.g = g;
        v_trajectory.color.b = b;
        v_trajectory.color.a = 0.8;
        v_trajectory.ns = pub.getTopic();
        //v_trajectory.type = visualization_msgs::Marker::POINTS;
        v_trajectory.type = type;
        v_trajectory.action = visualization_msgs::Marker::ADD;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectory.scale.x = 0.02;
        geometry_msgs::Point p;

        for (const auto& pose : trajectories[count]) {
            p.x = pose.x_;
            p.y = pose.y_;
            p.z = traj_z;
            v_trajectory.points.push_back(p);
        }

        v_trajectories.markers.push_back(v_trajectory);
    }

    // 这里为何要继续 push？
    for (; count < trajectories_size;) {
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = robot_frame_;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::DELETE;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectories.markers.push_back(v_trajectory);
        count++;
    }

    pub.publish(v_trajectories);
}

void SemDWAPlannerROS::VisualizeTrajectory(const std::vector<State>& trajectory, const double r, const double g, const double b, const int type, const double traj_z, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = robot_frame_;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    //v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
     v_trajectory.type = type;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.scale.x = 0.05;
    geometry_msgs::Point p;

    for (const auto& pose : trajectory) {
        p.x = pose.x_;
        p.y = pose.y_;
        p.z = traj_z;
        v_trajectory.points.push_back(p);
    }

    pub.publish(v_trajectory);
}

void SemDWAPlannerROS::VisualizeRobotTrajectory()
{
    tf::StampedTransform baseToWorldTf;

    try {
        listener_.waitForTransform(local_map_frame_, robot_frame_, ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform(local_map_frame_, robot_frame_, ros::Time(0), baseToWorldTf);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    robot_traj_.header.frame_id = local_map_frame_;
    robot_traj_.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped this_pose_stamped;

    this_pose_stamped.pose.position.x = baseToWorldTf.getOrigin().x();
    this_pose_stamped.pose.position.y = baseToWorldTf.getOrigin().y();
    this_pose_stamped.pose.position.z = rviz_traj_z_;

    this_pose_stamped.pose.orientation.x = 0;
    this_pose_stamped.pose.orientation.y = 0;
    this_pose_stamped.pose.orientation.z = 0;
    this_pose_stamped.pose.orientation.w = 1;

    this_pose_stamped.header.frame_id = local_map_frame_;
    this_pose_stamped.header.stamp = ros::Time::now();

    if (robot_traj_.poses.size() == 0) {
        robot_traj_.poses.push_back(this_pose_stamped);
        pre_pose_stamped_ = this_pose_stamped;
    } else {
        //if ((current_velocity_.linear.x > 0) && ((ros::Time::now() - robot_traj_.poses.back().header.stamp) > ros::Duration(0.050))) {
        // 1 / hz_ = 1 / 10 = 0.1
        if ((current_velocity_.linear.x > 0) && ((ros::Time::now() - robot_traj_.poses.back().header.stamp) > ros::Duration(1 / hz_))) {
            robot_traj_.poses.push_back(this_pose_stamped);
            // 计算每次规划的轨迹长度
            path_len_ += sqrt(pow(this_pose_stamped.pose.position.x - pre_pose_stamped_.pose.position.x, 2) + pow(this_pose_stamped.pose.position.y - pre_pose_stamped_.pose.position.y, 2));
            pre_pose_stamped_ = this_pose_stamped;
        }
    }

    robot_traj_pub_.publish(robot_traj_);
}

void SemDWAPlannerROS::VisualizeCurrentGoal(const geometry_msgs::PoseStamped& cur_goal, const double r, const double g, const double b, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_goal;
    v_goal.header.frame_id = local_map_frame_;
    v_goal.header.stamp = ros::Time::now();

    v_goal.color.r = r;
    v_goal.color.g = g;
    v_goal.color.b = b;
    v_goal.color.a = 1;

    v_goal.ns = pub.getTopic();

    v_goal.type = visualization_msgs::Marker::SPHERE;
    v_goal.action = visualization_msgs::Marker::ADD;
    //v_goal.lifetime = ros::Duration();
    //v_goal.pose.orientation.w = 1.0;

    v_goal.scale.x = 1.0;
    v_goal.scale.y = 1.0;
    v_goal.scale.z = 1.0;

    geometry_msgs::Pose pose;

    pose.position.x = cur_goal.pose.position.x;
    pose.position.y = cur_goal.pose.position.y;
    pose.position.z = rviz_traj_z_;

    pose.orientation = cur_goal.pose.orientation;

    //v_goal.text = "G";
    v_goal.pose = pose;

    pub.publish(v_goal);
}

bool SemDWAPlannerROS::IsGoalReach(const geometry_msgs::PoseStamped& goal)
{
    tf::StampedTransform baseToWorldTf;

    try {
        listener_.waitForTransform(goal.header.frame_id, robot_frame_, ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform(goal.header.frame_id, robot_frame_, ros::Time(0), baseToWorldTf);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return false;
    }

    Eigen::Vector3d base_origin(baseToWorldTf.getOrigin().x(), baseToWorldTf.getOrigin().y(), 0);
    Eigen::Vector3d current_goal(goal.pose.position.x, goal.pose.position.y, 0);

    //if ((current_goal.segment(0, 2) - base_origin.segment(0, 2)).norm() < goal_x_tor_) {
    //    ;
    //} else {
    //    ;
    //}

    return true;
}

void SemDWAPlannerROS::InitWayPoints()
{
    way_point_queue_.clear();
    
    // Init way point queue
    geometry_msgs::PoseStamped tmp_goal;
    tmp_goal.header.frame_id = local_map_frame_;

    std::string way_points_string;
    local_nh_.param("way_points", way_points_string, { "" });

    std::stringstream ss(way_points_string);
    std::string way_point;

    while (ss >> way_point) {
        ros::NodeHandle way_point_nh(local_nh_, way_point);

        tmp_goal.header.stamp = ros::Time::now();

        way_point_nh.param("x", tmp_goal.pose.position.x, { 0.0 });
        way_point_nh.param("y", tmp_goal.pose.position.y, { 0.0 });
        way_point_nh.param("z", tmp_goal.pose.position.z, { 0.0 });

        way_point_nh.param("ox", tmp_goal.pose.orientation.x, { 0.0 });
        way_point_nh.param("oy", tmp_goal.pose.orientation.y, { 0.0 });
        way_point_nh.param("oz", tmp_goal.pose.orientation.z, { 0.0 });
        way_point_nh.param("ow", tmp_goal.pose.orientation.w, { 1.0 });

        way_point_queue_.push_back(tmp_goal);
    }
}
