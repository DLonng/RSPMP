#ifndef SEM_DWA_PLANNER_ROS_H_
#define SEM_DWA_PLANNER_ROS_H_

#include <ros/ros.h>

#include <deque>
#include <iostream>
#include <fstream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "sem_dwa_planner/sem_dwa_planner.h"

namespace sem_dwa_local_planner {

class SemDWAPlannerROS {
    public:
        SemDWAPlannerROS();
        ~SemDWAPlannerROS();

        /**
         * @brief start planning cycle in 10Hz.
         */
        void Process(void);

        void LocalGoalCallback(const geometry_msgs::PoseStampedConstPtr&);
        void LocalMapCallback(const nav_msgs::OccupancyGridConstPtr&);
        void OdomCallback(const nav_msgs::OdometryConstPtr&);
        void RvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

        void VisualizeTrajectories(const std::vector<std::vector<State>>&, const double, const double, const double, const int, const int, const double, const ros::Publisher&);
        void VisualizeTrajectory(const std::vector<State>&, const double, const double, const double, const int, const double, const ros::Publisher&);
        void VisualizeRobotTrajectory();
        void VisualizeCurrentGoal(const geometry_msgs::PoseStamped& cur_goal, const double r, const double g, const double b, const ros::Publisher& pub);

        bool IsGoalReach(const geometry_msgs::PoseStamped& goal);
        void InitWayPoints();
    protected:
        std::string robot_frame_;
        std::string local_map_frame_;

        double hz_;
        bool send_goal_model_;
        bool multi_goal_model_;
        double global_goal_tor_;
        double local_goal_tor_;
        double goal_z_tor_;
        double rviz_traj_z_;

        bool local_goal_subscribed_;
        bool local_map_updated_;
        bool odom_updated_;

        bool use_cmu_planning_;

        // 当前选择的 dwa 轨迹
        double select_z_;
        // 当前所有的 dwa 采样轨迹
        double trave_z_;
        
        // 当前不可通行轨迹，禁用
        double untrave_z_;
        
        // 当前所有的语义感知轨迹
        double cmu_z_;
        // 当前代价最低的语义感知轨迹
        double best_cmu_z_;
        // 模型感知轨迹
        double width_z_;

        nav_msgs::Path robot_traj_;

        ros::NodeHandle nh_;
        ros::NodeHandle local_nh_;

        ros::Publisher velocity_pub_;
        ros::Publisher trave_trajectories_pub_;
        ros::Publisher cmu_trajectories_pub_;
        ros::Publisher width_trajectories_pub_;
        ros::Publisher untrave_trajectories_pub_;
        ros::Publisher selected_trajectory_pub_;
        ros::Publisher best_cmu_trajectory_pub_;
        ros::Publisher robot_traj_pub_;
        ros::Publisher current_goal_pub_;

        ros::Subscriber local_map_sub_;
        ros::Subscriber local_goal_sub_;
        ros::Subscriber rviz_goal_sub_;
        ros::Subscriber odom_sub_;

        tf::TransformListener listener_;

        geometry_msgs::PoseStamped local_goal_;
        
        geometry_msgs::PoseStamped current_global_goal_;
        geometry_msgs::PoseStamped current_local_goal_;

        std::deque<geometry_msgs::PoseStamped> way_point_queue_;

        nav_msgs::OccupancyGrid local_map_;
        geometry_msgs::Twist current_velocity_;

        SemDWAPlanner* sem_dwa_planner_;

        std::vector<std::vector<State>> trave_trajectories_;
        std::vector<std::vector<State>> untrave_trajectories_;
        std::vector<std::vector<State>> cmu_trajectories_;
        std::vector<std::vector<State>> width_trajectories_;
        std::vector<State> best_traj_;
        std::vector<State> best_cmu_traj_;

        double start_time_;
        double path_len_;
        double path_len2_;
        geometry_msgs::PoseStamped pre_pose_stamped_;


        std::vector<double> record_vel_x_;
        std::vector<double> record_vel_z_;
    };
}

#endif // SEM_DWA_PLANNER_ROS_H_