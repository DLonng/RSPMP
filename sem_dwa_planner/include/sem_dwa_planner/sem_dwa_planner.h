#ifndef SEM_DWA_PLANNER_H_
#define SEM_DWA_PLANNER_H_

#include <ros/ros.h>

#include <algorithm>
#include <numeric>

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/OccupancyGrid.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "sem_dwa_planner/state.h"

#include "../../octomap_generator/include/octomap_generator/semantis_cost.h"

namespace sem_dwa_local_planner {

class SemDWAPlanner {
public:
    SemDWAPlanner(void);

    class DynamicWindow {
        public:
            DynamicWindow(void);
            DynamicWindow(const double min_velocity, const double max_velocity, const double min_yawrate, const double max_yawrate);
            double min_velocity_;
            double max_velocity_;
            double min_yawrate_;
            double max_yawrate_;
    };

    /**
     * @brief DWAPlanning with static window.
     *
     * @param goal the goal need to rearch
     * @return best_traj
     */
    std::vector<State> CMUPlanning(const nav_msgs::OccupancyGrid& local_sem_map, const Eigen::Vector3d& goal, 
                                   const geometry_msgs::Twist& current_velocity, std::vector<std::vector<State>>& trave_trajectories, std::vector<std::vector<State>>& cmu_trajectories,
                                   std::vector<std::vector<State>>& untrave_trajectories, std::vector<std::vector<State>>& width_trajectories, std::vector<State>& best_cmu_traj);

    // add semantic_cost_function to DWA
    std::vector<State> SemDWAPlanning(const nav_msgs::OccupancyGrid& local_sem_map, 
                                      const Eigen::Vector3d& goal, const geometry_msgs::Twist& current_velocity, 
                                      std::vector<std::vector<State>>& trajectories);
    
    void Raycast(const nav_msgs::OccupancyGrid& local_sem_map);
    double ObstacleCostFunction(const nav_msgs::OccupancyGrid& local_sem_map, const std::vector<State>& traj);
    double SpeedCostFunction(const std::vector<State>& traj, const double target_velocity);
    double GoalCostFunction(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    double YawrateCostFunction(const std::vector<State>& traj);
    double HeadingCostFunction(const std::vector<State>& traj, const Eigen::Vector3d& goal);


    DynamicWindow CalcDynamicWindow(const geometry_msgs::Twist& current_velocity);

    void GenerateSemDWATrajectory(const DynamicWindow& dynamic_window, const geometry_msgs::Twist& current_velocity);
    void GenerateROSDWATrajectory(const DynamicWindow& dynamic_window, const geometry_msgs::Twist& current_velocity);
    void CostSemDWATrajectory(const nav_msgs::OccupancyGrid& local_sem_map, const Eigen::Vector3d& goal);
    void NormalizeSemDWATrajectory();
    void SelectSemDWATrajectory();
    void Motion(State& state, const double velocity, const double yawrate);
    void ROSDWAMotion(State& state, const double velocity, const double yawrate, double dt);
    double SemanticCostFunction(const nav_msgs::OccupancyGrid& local_sem_map, const std::vector<State>& traj, int& untrave_point_num);

    void GenerateCMUTrajectory();
    void GenerateCMUWidthTrajectory(const std::vector<State>& center_traj);
    void GenerateCMUTrajectoryWithYawrate(const std::vector<State>& center_traj, std::vector<State>& cmu_traj);
    void CostCMUTrajectory(const nav_msgs::OccupancyGrid& local_sem_map, const Eigen::Vector3d& goal);
    void NormalizeCMUTrajectory();
    void SelectCMUTrajectory();
    double CMUSemanticCostFunction(const nav_msgs::OccupancyGrid& local_sem_map, const std::vector<State>& traj, int& untrave_point_num);

    double GetMaxYawrate() { return max_yawrate_; }

    double ToDegree(double rad);

    double CalcCMUTrajLength(const double cur_velocity, const double base_length);
protected:
    std::string robot_frame_;

    double hz_;
    double dt_;
    double cmu_max_vel_;
    double cmu_max_yaw_;
    double cmu_yaw_res_;
    double cmu_traj_len_;

    double sem_gain_;
    double obs_gain_;
    double goal_gain_;
    double yawrate_gain_;
    double speed_gain_;
    double heading_gain_;
    
    double untrave_point_scale_;

    double max_velocity_;
    double min_velocity_;
    double max_yawrate_;
    double max_acceleration_;
    double max_d_yawrate_;
    double velocity_resolution_;
    double yawrate_resolution_;
    double predict_time_;
    double target_velocity_;

    double angle_resolution_;
    double max_dist_;
    double car_rad_;

    double car_width_;
    int shift_traj_num_;
    double local_map_res_;
    double local_map_range_;

    bool using_cmu_dynamic_window_;
    bool abandon_untrave_traj_;
    double untrave_point_num_;

    bool calc_cmu_traj_len_;
    double max_sem_traj_yawrate_;
    
    ros::NodeHandle local_nh_;

    tf::TransformListener listener_;

    std::vector<std::vector<State>> trajectories_;
    std::vector<std::vector<State>> cmu_trajectories_;
    std::vector<std::vector<State>> width_trajectories_;
    std::vector<std::vector<State>> trave_trajectories_;
    std::vector<std::vector<State>> untrave_trajectories_;
    std::vector<State> best_traj_;
    std::vector<State> best_cmu_traj_;
    std::vector<State> cmu_traj_;

    std::vector<double> semantic_cost_vec_;
    std::vector<double> obstacle_cost_vec_;
    std::vector<double> goal_cost_vec_;
    std::vector<double> yawrate_cost_vec_;
    std::vector<double> speed_cost_vec_;
    std::vector<double> heading_cost_vec_;

    std::vector<std::vector<double>> sem_obs_list_;
};
}
#endif // SEM_DWA_PLANNER_H_
