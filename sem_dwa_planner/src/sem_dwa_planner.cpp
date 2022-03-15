#include "sem_dwa_planner/sem_dwa_planner.h"

using namespace sem_dwa_local_planner;
using namespace semantics_cost;

#define DEBUG_INFO 0

SemDWAPlanner::SemDWAPlanner(void)
    : local_nh_("~")
{
    local_nh_.param("HZ", hz_, { 10 });
    local_nh_.param("robot_frame", robot_frame_, { "base_link" });
    local_nh_.param("cmu_max_vel", cmu_max_vel_, { 0.5 });
    local_nh_.param("cmu_max_yaw", cmu_max_yaw_, { 0.26 });
    local_nh_.param("cmu_yaw_res", cmu_yaw_res_, { 0.017 });
    local_nh_.param("cmu_traj_len", cmu_traj_len_, { 5.0 });

    local_nh_.param("sem_gain", sem_gain_, { 1.0 });
    local_nh_.param("obs_gain", obs_gain_, { 1.0 });
    local_nh_.param("goal_gain", goal_gain_, { 1.0 });
    local_nh_.param("yawrate_gain", yawrate_gain_, { 1.0 });
    local_nh_.param("speed_gain", speed_gain_, { 1.0 });
    local_nh_.param("heading_gain", heading_gain_, { 1.0 });

    local_nh_.param("untrave_point_scale", untrave_point_scale_, { 0.75 });

    local_nh_.param("max_velocity", max_velocity_, { 0.5 });
    local_nh_.param("min_velocity", min_velocity_, { 0.0 });
    local_nh_.param("max_yawrate", max_yawrate_, { 0.1 });
    local_nh_.param("max_acceleration", max_acceleration_, { 5.0 });
    local_nh_.param("max_d_yawrate", max_d_yawrate_, { 3.0 });
    local_nh_.param("velocity_resolution", velocity_resolution_, { 0.1 });
    local_nh_.param("yawrate_resolution", yawrate_resolution_, { 0.1 });
    local_nh_.param("predict_time", predict_time_, { 1.0 });
    local_nh_.param("target_velocity", target_velocity_, { 0.5 });

    local_nh_.param("angle_resolution", angle_resolution_, { 0.0087 });
    local_nh_.param("max_dist", max_dist_, { 5.0 });
    local_nh_.param("car_rad", car_rad_, { 0.465 });
    local_nh_.param("car_width", car_width_, { 0.70 });
    local_nh_.param("shift_traj_num", shift_traj_num_, { 7 });

    local_nh_.param("using_cmu_dynamic_window", using_cmu_dynamic_window_, { false });
    local_nh_.param("abandon_untrave_traj", abandon_untrave_traj_, { false });
    local_nh_.param("untrave_point_num", untrave_point_num_, { 10 });
    local_nh_.param("calc_cmu_traj_len", calc_cmu_traj_len_, { false });
    local_nh_.param("max_sem_traj_yawrate", max_sem_traj_yawrate_, { 1.0 });
    
    // dt = 1.0 / 10 = 0.1s
    dt_ = 1.0 / hz_;

    std::cout << "robot_frame: " << robot_frame_ << std::endl;
    std::cout << "DT: " << dt_ << std::endl;
    std::cout << "cmu_max_vel: " << cmu_max_vel_ << std::endl;
    std::cout << "cmu_max_yaw: " << cmu_max_yaw_ << std::endl;
    std::cout << "cmu_yaw_res: " << cmu_yaw_res_ << std::endl;
    std::cout << "cmu_traj_len: " << cmu_traj_len_ << std::endl;

    std::cout << "sem_gain: " << sem_gain_ << std::endl;
    std::cout << "obs_gain: " << obs_gain_ << std::endl;
    std::cout << "goal_gain: " << goal_gain_ << std::endl;
    std::cout << "yawrate_gain: " << yawrate_gain_ << std::endl;
    std::cout << "speed_gain: " << speed_gain_ << std::endl;
    std::cout << "heading_gain: " << heading_gain_ << std::endl;

    std::cout << "untrave_point_scale: " << untrave_point_scale_ << std::endl;

    std::cout << "max_velocity: " << max_velocity_ << std::endl;
    std::cout << "min_velocity: " << min_velocity_ << std::endl;
    std::cout << "max_yawrate: " << max_yawrate_ << std::endl;
    std::cout << "max_acceleration: " << max_acceleration_ << std::endl;
    std::cout << "max_d_yawrate: " << max_d_yawrate_ << std::endl;
    std::cout << "velocity_resolution: " << velocity_resolution_ << std::endl;
    std::cout << "yawrate_resolution: " << yawrate_resolution_ << std::endl;
    std::cout << "predict_time: " << predict_time_ << std::endl;
    std::cout << "target_velocity: " << target_velocity_ << std::endl;

    std::cout << "angle_resolution: " << angle_resolution_ << std::endl;
    std::cout << "max_dist: " << max_dist_ << std::endl;
    std::cout << "car_rad: " << car_rad_ << std::endl;
    std::cout << "car_width: " << car_width_ << std::endl;
    std::cout << "shift_traj_num: " << shift_traj_num_ << std::endl;

    std::cout << "using_cmu_dynamic_window: " << using_cmu_dynamic_window_ << std::endl;
    std::cout << "abandon_untrave_traj: " << abandon_untrave_traj_ << std::endl;
    std::cout << "untrave_point_num: " << untrave_point_num_ << std::endl;
    std::cout << "calc_cmu_traj_len: " << calc_cmu_traj_len_ << std::endl;
    std::cout << "max_sem_traj_yawrate: " << max_sem_traj_yawrate_ << std::endl;
    
    std::cout << "SemDWAPlanner Init OK!" << std::endl;
}

SemDWAPlanner::DynamicWindow::DynamicWindow(void)
    : min_velocity_(0.0)
    , max_velocity_(0.0)
    , min_yawrate_(0.0)
    , max_yawrate_(0.0)
{
}

SemDWAPlanner::DynamicWindow::DynamicWindow(const double min_velocity, const double max_velocity, const double min_yawrate, const double max_yawrate)
    : min_velocity_(min_velocity)
    , max_velocity_(max_velocity)
    , min_yawrate_(min_yawrate)
    , max_yawrate_(max_yawrate)
{
}

#if 0
// 弃用
std::vector<State> SemDWAPlanner::CMUPlanning(const nav_msgs::OccupancyGrid& local_sem_map,
    const Eigen::Vector3d& goal,
    std::vector<std::vector<State>>& trajectories)
{
    double min_cost = 1e6;

    double semantic_cost = min_cost;
    double goal_cost = min_cost;
    double yawrate_cost = min_cost;

    double min_semantic_cost = min_cost;
    double min_goal_cost = min_cost;
    double min_yawrate_cost = min_cost;

    double final_cost = min_cost;

    int untrave_point_num = 0;

    trajectories_.clear();
    best_traj_.clear();
    semantic_cost_vec_.clear();
    goal_cost_vec_.clear();
    yawrate_cost_vec_.clear();

    double vel_car = 0.5; // m/s
    double yaw_car = yaw_car_; // 15 度 = 0.26 弧度

    double cur_vel = 0.0;
    double cur_yaw = 0.0;

    double traj_len = 0.0;

    double yaw_inc = 0.0;
    double x_inc = 0.0;
    double y_inc = 0.0;

    // 20 条相同长度的轨迹
    for (int i = 0; i < traj_size_; i++) {
        // 每条轨迹的线速度保持不变
        cur_vel = vel_car;
        // 每条轨迹的角速度依次减少 1 度 = 0.017 弧度
        cur_yaw = yaw_car - (i * 0.017);

        State state(0.0, 0.0, 0.0, cur_vel, cur_yaw);

        // 当前采样的一条轨迹
        std::vector<State> traj;
        traj_len = 0.0;

        while (traj_len <= traj_len_) {
            // 计算角度，x，y 的增量
            yaw_inc = cur_yaw * dt_;
            x_inc = cur_vel * std::cos(state.yaw_) * dt_;
            y_inc = cur_vel * std::sin(state.yaw_) * dt_;

            // 累加当前 dt_ 时间段内的增量到轨迹点中
            state.yaw_ += yaw_inc;
            state.x_ += x_inc;
            state.y_ += y_inc;
            state.velocity_ = cur_vel;
            state.yawrate_ = cur_yaw;

            // 曲线积分：DT 时间段内，曲线上 2 点之间长度可用两点的欧式距离近视代替
            traj_len += sqrt((x_inc * x_inc) + (y_inc * y_inc));

            // 把当前采样的轨迹点 push 到当前采样的轨迹中
            traj.push_back(state);
        }

        // 保证轨迹点为 80 个方便后续计算丢弃轨迹，不 pop_back 则会采样 81 的轨迹点
        traj.pop_back();

        trajectories_.push_back(traj);

        semantic_cost = SemanticCostFunction(local_sem_map, traj, untrave_point_num);
        semantic_cost_vec_.push_back(semantic_cost);

        goal_cost = GoalCostFunction(traj, goal);
        goal_cost_vec_.push_back(goal_cost);

        yawrate_cost = YawrateCostFunction(traj);
        yawrate_cost_vec_.push_back(yawrate_cost);
    }

    // 将每组轨迹代价进行归一化，即每一项除以该项的总和
    double semantic_cost_sum = std::accumulate(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), 0.0);
    double goal_cost_sum = std::accumulate(goal_cost_vec_.begin(), goal_cost_vec_.end(), 0.0);
    double yawrate_cost_sum = std::accumulate(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), 0.0);

    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    std::cout << std::endl;

    std::cout << "semantic_cost_sum: " << semantic_cost_sum << std::endl;
    std::cout << "goal_cost_sum: " << goal_cost_sum << std::endl;
    std::cout << "yawrate_cost_sum: " << yawrate_cost_sum << std::endl;
    std::cout << std::endl;

    std::for_each(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), [semantic_cost_sum](double& sem_cost) { sem_cost /= semantic_cost_sum; });
    std::for_each(goal_cost_vec_.begin(), goal_cost_vec_.end(), [goal_cost_sum](double& goal_cost) { goal_cost /= goal_cost_sum; });
    std::for_each(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), [yawrate_cost_sum](double& yawrate_cost) { yawrate_cost /= yawrate_cost_sum; });

    std::cout << "After Normalization..." << std::endl;
    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    std::cout << std::endl;

    trajectories = trajectories_;

    std::cout << "min semantic cost: " << min_semantic_cost << std::endl;
    std::cout << "min goal cost: " << min_goal_cost << std::endl;
    std::cout << "min yawrate cost: " << min_yawrate_cost << std::endl;
    std::cout << "min cost: " << min_cost << std::endl;
    std::cout << std::endl;

    std::cout << "trajectories size: " << trajectories_.size() << std::endl;
    std::cout << "trajectorie[0] size: " << trajectories_[0].size() << std::endl;
    std::cout << "trajectorie[1] size: " << trajectories_[1].size() << std::endl;
    std::cout << std::endl;

    if (min_cost == 1e6) {
        std::cout << "min_cost to big, pub cmd_vel 0:" << min_cost << std::endl;
        std::vector<State> traj;
        State state(0.0, 0.0, 0.0, 0.0, 0.0);
        traj.push_back(state);
        best_traj_ = traj;
    }

    return best_traj_;
}

#else

// 使用 cmu 的采样轨迹方法 + semantic_cost_function + 车身宽度附加轨迹 进行避障
// 也可以换成 dwa 轨迹采样方法测试
std::vector<State> SemDWAPlanner::CMUPlanning(const nav_msgs::OccupancyGrid& local_sem_map,
    const Eigen::Vector3d& goal,
    const geometry_msgs::Twist& current_velocity,
    std::vector<std::vector<State>>& trave_trajectories,
    std::vector<std::vector<State>>& cmu_trajectories,
    std::vector<std::vector<State>>& untrave_trajectories,
    std::vector<std::vector<State>>& width_trajectories,
    std::vector<State>& best_cmu_traj)
{
    // 0.14
    local_map_res_ = local_sem_map.info.resolution;
    local_map_range_ = (local_sem_map.info.width / 2) * local_map_res_;

    if (using_cmu_dynamic_window_) {
        // CMU 语义轨迹 + DWA 轨迹
        GenerateSemDWATrajectory(CalcDynamicWindow(current_velocity), current_velocity);

        CostSemDWATrajectory(local_sem_map, goal);

        NormalizeSemDWATrajectory();

        SelectSemDWATrajectory();

        // 返回可通行轨迹
        trave_trajectories = trave_trajectories_;

        cmu_trajectories = cmu_trajectories_;

        // 返回不可通行轨迹
        untrave_trajectories = untrave_trajectories_;

        // 产生当前最优轨迹的附加轨迹，并返回用于 RVIZ 显示
        GenerateCMUWidthTrajectory(best_cmu_traj_);
        width_trajectories = width_trajectories_;

        best_cmu_traj = best_cmu_traj_;

        return best_traj_;
    } else {
        // CMU 语义轨迹
        GenerateCMUTrajectory();

        CostCMUTrajectory(local_sem_map, goal);

        NormalizeCMUTrajectory();

        SelectCMUTrajectory();

        // 返回可通行轨迹
        trave_trajectories = trave_trajectories_;

        cmu_trajectories = cmu_trajectories_;

        // 返回不可通行轨迹
        untrave_trajectories = untrave_trajectories_;

        // 产生最优轨迹的宽度轨迹
        GenerateCMUWidthTrajectory(best_traj_);
        width_trajectories = width_trajectories_;

        // 这个返回空
        best_cmu_traj = best_cmu_traj_;

        return best_traj_;
    }
}

#endif

#if 0
std::vector<State> SemDWAPlanner::SemDWAPlanning(const nav_msgs::OccupancyGrid& local_sem_map,
    const Eigen::Vector3d& goal,
    const geometry_msgs::Twist& current_velocity,
    std::vector<std::vector<State>>& trajectories)
{
    double min_cost = 1e6;

    double semantic_cost = min_cost;
    double goal_cost = min_cost;
    double yawrate_cost = min_cost;

    double min_semantic_cost = min_cost;
    double min_goal_cost = min_cost;
    double min_yawrate_cost = min_cost;

    double final_cost = min_cost;

    int untrave_point_num = 0;

    trajectories_.clear();
    best_traj_.clear();
    semantic_cost_vec_.clear();
    goal_cost_vec_.clear();
    yawrate_cost_vec_.clear();

    // 根据当前速度计算动态窗口
    DynamicWindow dynamic_window = CalcDynamicWindow(current_velocity);

    // 在动态窗口内采样轨迹，并计算每个轨迹的代价
    for (double v = dynamic_window.min_velocity_; v <= dynamic_window.max_velocity_; v += velocity_resolution_) {
        for (double y = dynamic_window.min_yawrate_; y <= dynamic_window.max_yawrate_; y += yawrate_resolution_) {

            State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);

            std::vector<State> traj;

            for (double t = 0; t < predict_time_; t += dt_) {
                Motion(state, v, y);

                traj.push_back(state);

                //t += dt_;
            }

            // predict_time_ = 5s, for 循环应该采样 50 点，但是会多采样一个，这里 pop 丢掉
            traj.pop_back();

            trajectories_.push_back(traj);

            semantic_cost = SemanticCostFunction(local_sem_map, traj, untrave_point_num);
            semantic_cost_vec_.push_back(semantic_cost);

            goal_cost = GoalCostFunction(traj, goal);
            goal_cost_vec_.push_back(goal_cost);

            yawrate_cost = YawrateCostFunction(traj);
            yawrate_cost_vec_.push_back(yawrate_cost);
        }
    }

    // 将每组轨迹代价进行归一化，即每一项除以该项的总和
    double semantic_cost_sum = std::accumulate(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), 0.0);
    double goal_cost_sum = std::accumulate(goal_cost_vec_.begin(), goal_cost_vec_.end(), 0.0);
    double yawrate_cost_sum = std::accumulate(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), 0.0);

    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    std::cout << std::endl;

    std::cout << "semantic_cost_sum: " << semantic_cost_sum << std::endl;
    std::cout << "goal_cost_sum: " << goal_cost_sum << std::endl;
    std::cout << "yawrate_cost_sum: " << yawrate_cost_sum << std::endl;
    std::cout << std::endl;

    std::for_each(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), [semantic_cost_sum](double& sem_cost) { sem_cost /= semantic_cost_sum; });
    std::for_each(goal_cost_vec_.begin(), goal_cost_vec_.end(), [goal_cost_sum](double& goal_cost) { goal_cost /= goal_cost_sum; });
    std::for_each(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), [yawrate_cost_sum](double& yawrate_cost) { yawrate_cost /= yawrate_cost_sum; });

    /*
    for (auto& sem_cost : semantic_cost_vec_)
        sem_cost /= semantic_cost_sum;

    for (auto& goal_cost : goal_cost_vec_)
        goal_cost /= goal_cost_sum;
	
    for (auto& yawrate_cost : yawrate_cost_vec_)
        yawrate_cost /= yawrate_cost_sum;
    */

    std::cout << "After Normalization..." << std::endl;
    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    std::cout << std::endl;

    // 选择代价最小的轨迹
    for (int i = 0; i < trajectories_.size(); i++) {

        final_cost = sem_gain_ * semantic_cost_vec_[i] + goal_gain_ * goal_cost_vec_[i] + yawrate_gain_ * yawrate_cost_vec_[i];

        if (final_cost <= min_cost) {
            min_semantic_cost = semantic_cost_vec_[i];
            min_goal_cost = goal_cost_vec_[i];
            min_yawrate_cost = yawrate_cost_vec_[i];

            min_cost = final_cost;

            best_traj_ = trajectories_[i];
        }
    }

    // 返回所有的轨迹，用于 RVIZ 显示
    trajectories = trajectories_;

    std::cout << "min semantic cost: " << min_semantic_cost << std::endl;
    std::cout << "min goal cost: " << min_goal_cost << std::endl;
    std::cout << "min yawrate cost: " << min_yawrate_cost << std::endl;
    std::cout << "min cost: " << min_cost << std::endl;
    std::cout << std::endl;

    std::cout << "trajectories size: " << trajectories_.size() << std::endl;
    std::cout << "trajectorie[0] size: " << trajectories_[0].size() << std::endl;
    std::cout << "trajectorie[1] size: " << trajectories_[1].size() << std::endl;
    std::cout << std::endl;

    if (min_cost == 1e6) {
        std::cout << "min_cost to big, pub cmd_vel 0:" << min_cost << std::endl;
        std::vector<State> traj;
        State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);
        traj.push_back(state);
        best_traj_ = traj;
    }

    return best_traj_;
}

#else

// 使用 dwa 采样轨迹 + raycast + obs_cost_function 进行避障
std::vector<State> SemDWAPlanner::SemDWAPlanning(const nav_msgs::OccupancyGrid& local_sem_map,
    const Eigen::Vector3d& goal,
    const geometry_msgs::Twist& current_velocity,
    std::vector<std::vector<State>>& trajectories)
{
    DynamicWindow dynamic_window = CalcDynamicWindow(current_velocity);

    GenerateSemDWATrajectory(dynamic_window, current_velocity);
    //GenerateROSDWATrajectory(dynamic_window, current_velocity);

    CostSemDWATrajectory(local_sem_map, goal);

    NormalizeSemDWATrajectory();

    SelectSemDWATrajectory();

    // 返回所有的轨迹，用于 RVIZ 显示
    trajectories = trajectories_;

    return best_traj_;
}

#endif

void SemDWAPlanner::Motion(State& state, const double velocity, const double yawrate)
{
    // 当前模拟周期内应该先计算角度，再计算 x y 增量
    state.x_ += velocity * std::cos(state.yaw_) * dt_;
    state.y_ += velocity * std::sin(state.yaw_) * dt_;

    state.yaw_ += yawrate * dt_;

    state.velocity_ = velocity;
    state.yawrate_ = yawrate;
}

void SemDWAPlanner::ROSDWAMotion(State& state, const double velocity, const double yawrate, double dt)
{
    state.x_ += velocity * std::cos(state.yaw_) * dt;
    state.y_ += velocity * std::sin(state.yaw_) * dt;

    // using ros dwa
    state.yaw_ += yawrate * dt;

    state.velocity_ = velocity;
    state.yawrate_ = yawrate;
}

SemDWAPlanner::DynamicWindow SemDWAPlanner::CalcDynamicWindow(const geometry_msgs::Twist& current_velocity)
{
    DynamicWindow window(min_velocity_, max_velocity_, -max_yawrate_, max_yawrate_);

    window.min_velocity_ = std::max((current_velocity.linear.x - max_acceleration_ * dt_), min_velocity_);
    window.max_velocity_ = std::min((current_velocity.linear.x + max_acceleration_ * dt_), max_velocity_);

    window.min_yawrate_ = std::max((current_velocity.angular.z - max_d_yawrate_ * dt_), -max_yawrate_);
    window.max_yawrate_ = std::min((current_velocity.angular.z + max_d_yawrate_ * dt_), max_yawrate_);

    return window;
}

double SemDWAPlanner::SemanticCostFunction(const nav_msgs::OccupancyGrid& local_sem_map, const std::vector<State>& traj, int& untrave_point_num)
{
    // 设置为 1.0 而不是 0 是为了防止一条轨迹的语义代价为 0，归一化分母为 0
    // 如果一条轨迹上的点都是未初始化的语义代价，并且不允许访问未知空间，那么该条轨迹的语义成本就是 0
    double cost = 0.0; // 0.0

    // 归一化替代缩放
    double cost_scale = 100.0;

    unsigned char single_cost = 0;

    int untrave_size = 0;

    geometry_msgs::PointStamped map_point;
    geometry_msgs::PointStamped base_point;

    for (const auto& state : traj) {
        // traj 点是以 base_link 为起点的世界坐标，需要先转换到 map 世界坐标系中
        base_point.header.frame_id = robot_frame_;
        base_point.header.stamp = ros::Time(0);
        base_point.point.x = state.x_;
        base_point.point.y = state.y_;

        listener_.transformPoint(local_sem_map.header.frame_id, base_point, map_point);

        // 通过轨迹点在 map 坐标系中的位置和局部地图的原点，计算一个点与局部地图原点的 x y 偏移
        double x_offset = map_point.point.x - local_sem_map.info.origin.position.x;
        double y_offset = map_point.point.y - local_sem_map.info.origin.position.y;

        if ((x_offset < 0 || x_offset >= local_sem_map.info.width) || (y_offset < 0 || y_offset >= local_sem_map.info.height)) {
            std::cout << "当前轨迹点的世界坐标不在局部地图范围内，x 和 y 方向的偏移距离为：x_offset = "
                      << x_offset << ", y_offset = " << y_offset << std::endl;
            // 只要有一个点超过当前局部地图的范围，其他点就不再计算了
            break;
        }

        // 通过偏移计算一个轨迹点在局部地图的像素坐标系中的栅格坐标 (row, col)
        // row 代表 x 方向的坐标，col 代表 y 方向的坐标
        // x 轴表示地图的宽度(列数)，y 轴表示地图的高度（行数）
        int x_col = floor(x_offset / local_sem_map.info.resolution + 0.5);
        int y_row = floor(y_offset / local_sem_map.info.resolution + 0.5);

        // 通过 row col 坐标系获取对应栅格存储的语义成本
        single_cost = local_sem_map.data[y_row * local_sem_map.info.width + x_col];

        // kNoSemCost = 70，初始无语义栅格不累加代价
        // 为了防止归一化分母为 0，累加无语义栅格，想当与允许小车访问未建图的未知区域
        // 目前只是刚启动的时候，如果采样时间太短会导致轨迹上的点都是初始未知成本代价
        // 激光点云不能打到离小车非常近的位置
        // 解决方法 1: 打开注释，提高模拟时间，使得每条轨迹上有其他的语义成本
        // 解决方法 2???: 设置每条轨迹的初始成本为 1.0，而不是 0

        // 对于未初始化的语义栅格，要么累加到语义代价中，即注释掉下面 2 行
        //if (single_cost == kNoSemCost)
        //continue; // 都为 70 的未初始化轨迹，cost = 0.0

        cost += (double)single_cost;

        // 统计不可通行轨迹点的数量，用于计算是否丢弃轨迹，目前不使用
        // 与障碍物相关的逻辑放在障碍物代价函数中处理
        if ((single_cost == kHighGrassCost) || (single_cost == kPeopleCost)
            || (single_cost == kCarCost) || (single_cost == kSkyCost)
            || (single_cost == kUnknowClassCost))
            untrave_size++;
    }

    // 返回不可通行轨迹点的数量，用于判断是否丢弃轨迹，目前不使用
    untrave_point_num = untrave_size;

    // 对语义成本进行归一化
    // 对每个代价函数的结果都归一化后，是否还需要单独处理这里？
    // (x - min) / (max - min)
    //cost = (cost - traj.size() * kMinCost) / (traj.size() * kMaxCost - traj.size() * kMinCost);
    //ROS_INFO("SemDWAPlanner [%s] Cur traj semantic cost = %f(Normalization)", __FUNCTION__, cost);

    // 要么单独处理全部都是未初始化的语义轨迹，赋值给高的代价
    /*
    if (std::fabs(cost) < 0.000001) {
        //cost = ((double)(kNoSemCost * traj.size()) / cost_scale);
        cost = 10000.0;
        ROS_INFO("SemDWAPlanner [%s] Cur traj(size = %u) semantic no init, so cost = %f", __FUNCTION__, traj.size(), cost);
    } else {
        cost /= cost_scale;
        ROS_INFO("SemDWAPlanner [%s] Cur traj(size = %u) semantic cost = %f(X100)", __FUNCTION__, traj.size(), cost);
    }
    */

    cost /= cost_scale;

    if (cost == (kNoSemantic * traj.size() / cost_scale))
        ROS_INFO("SemDWAPlanner [%s] Cur traj(size = %zu) semantic no init, so cost = %f(X100)", __FUNCTION__, traj.size(), cost);
    else
        ROS_INFO("SemDWAPlanner [%s] Cur traj(size = %zu) semantic cost = %f(X100)", __FUNCTION__, traj.size(), cost);

    return cost;
}

double SemDWAPlanner::CMUSemanticCostFunction(const nav_msgs::OccupancyGrid& local_sem_map, const std::vector<State>& traj, int& untrave_point_num)
{
    // 设置为 1.0 而不是 0 是为了防止一条轨迹的语义代价为 0，归一化分母为 0
    // 如果一条轨迹上的点都是未初始化的语义代价，并且不允许访问未知空间，那么该条轨迹的语义成本就是 0
    double cost = 0.0; // 0.0

    // 归一化替代缩放
    double cost_scale = 100.0;

    unsigned char single_cost = 0;

    int untrave_size = 0;

    geometry_msgs::PointStamped map_point;
    geometry_msgs::PointStamped base_point;

    for (const auto& state : traj) {
        // traj 点是以 base_link 为起点的世界坐标，需要先转换到 map 世界坐标系中
        base_point.header.frame_id = robot_frame_;
        base_point.header.stamp = ros::Time(0);
        base_point.point.x = state.x_;
        base_point.point.y = state.y_;

        listener_.transformPoint(local_sem_map.header.frame_id, base_point, map_point);

        // 通过轨迹点在 map 坐标系中的位置和局部地图的原点，计算一个点与局部地图原点的 x y 偏移
        double x_offset = map_point.point.x - local_sem_map.info.origin.position.x;
        double y_offset = map_point.point.y - local_sem_map.info.origin.position.y;

        if ((x_offset < 0 || x_offset >= local_sem_map.info.width) || (y_offset < 0 || y_offset >= local_sem_map.info.height)) {
            std::cout << "当前轨迹点的世界坐标不在局部地图范围内，x 和 y 方向的偏移距离为：x_offset = "
                      << x_offset << ", y_offset = " << y_offset << std::endl;
            // 只要有一个点超过当前局部地图的范围，其他点就不再计算了
            break;
        }

        // 通过偏移计算一个轨迹点在局部地图的像素坐标系中的栅格坐标 (row, col)
        // row 代表 x 方向的坐标，col 代表 y 方向的坐标
        // x 轴表示地图的宽度(列数)，y 轴表示地图的高度（行数）
        int x_col = floor(x_offset / local_sem_map.info.resolution + 0.5);
        int y_row = floor(y_offset / local_sem_map.info.resolution + 0.5);

        // 通过 row col 坐标系获取对应栅格存储的语义成本
        single_cost = local_sem_map.data[y_row * local_sem_map.info.width + x_col];

        // kNoSemCost = 70，初始无语义栅格不累加代价
        // 为了防止归一化分母为 0，累加无语义栅格，想当与允许小车访问未建图的未知区域
        // 目前只是刚启动的时候，如果采样时间太短会导致轨迹上的点都是初始未知成本代价
        // 激光点云不能打到离小车非常近的位置
        // 解决方法 1: 打开注释，提高模拟时间，使得每条轨迹上有其他的语义成本
        // 解决方法 2???: 设置每条轨迹的初始成本为 1.0，而不是 0

        // 因为雷达线束少，所以前方轨迹末端会出现地图空缺，进而轨迹的代价为初始的无语义值
        // 目前的方法：把投影到代价地图上的无语义轨迹点的成本设置为道路成本，即把空洞当做道路看待
        //if (single_cost == kNoSemantic) {
            //continue; // 都为 70 的未初始化轨迹，cost = 0.0
           // single_cost = kRoadCost;
        //}

        cost += (double)single_cost;

        // 统计不可通行轨迹点的数量，用于计算是否丢弃轨迹，目前使用
        // 与障碍物相关的逻辑放在障碍物代价函数中处理
        // 是否需要把膨胀栅格当做不可通行点？为了更好的避障，需要
        if ((single_cost == kHighGrassCost) || (single_cost == kPeopleCost)
            || (single_cost == kCarCost) || (single_cost == kSkyCost) || (single_cost == kBuildingCost)
            || (single_cost == kUnknowClassCost)) //|| (single_cost == kInflationCost))
            untrave_size++;
    }

    // 返回不可通行轨迹点的数量，用于判断是否丢弃轨迹，目前使用
    untrave_point_num = untrave_size;

    // 对语义成本进行归一化
    // 对每个代价函数的结果都归一化后，是否还需要单独处理这里？
    // (x - min) / (max - min)
    //cost = (cost - traj.size() * kMinCost) / (traj.size() * kMaxCost - traj.size() * kMinCost);
    //ROS_INFO("SemDWAPlanner [%s] Cur traj semantic cost = %f(Normalization)", __FUNCTION__, cost);

    // 要么单独处理全部都是未初始化的语义轨迹，赋值给高的代价
    /*
    if (std::fabs(cost) < 0.000001) {
        //cost = ((double)(kNoSemCost * traj.size()) / cost_scale);
        cost = 10000.0;
        ROS_INFO("SemDWAPlanner [%s] Cur traj(size = %u) semantic no init, so cost = %f", __FUNCTION__, traj.size(), cost);
    } else {
        cost /= cost_scale;
        ROS_INFO("SemDWAPlanner [%s] Cur traj(size = %u) semantic cost = %f(X100)", __FUNCTION__, traj.size(), cost);
    }
    */

    cost /= cost_scale;

    return cost;
}

void SemDWAPlanner::Raycast(const nav_msgs::OccupancyGrid& local_sem_map)
{
    sem_obs_list_.clear();

    unsigned char single_cost = 0;

    double x = 0.0;
    double y = 0.0;

    int i = 0;
    int j = 0;

    // M_PI / 2 只寻找车辆前方的障碍物 有待测试
    // M_PI 寻找车辆周围所有的障碍物
    //for (double angle = -M_PI; angle <= M_PI; angle += angle_resolution_) {
    for (double angle = -M_PI_2; angle <= M_PI_2; angle += angle_resolution_) {
        for (double dist = 0.0; dist <= max_dist_; dist += local_sem_map.info.resolution) {
            x = dist * cos(angle);
            y = dist * sin(angle);

            i = floor(x / local_sem_map.info.resolution + 0.5) + local_sem_map.info.width * 0.5;
            j = floor(y / local_sem_map.info.resolution + 0.5) + local_sem_map.info.height * 0.5;

            if ((i < 0 || i >= local_sem_map.info.width) || (j < 0 || j >= local_sem_map.info.height)) {
                break;
            }

            single_cost = local_sem_map.data[j * local_sem_map.info.width + i];

            // 判断语义类别，保存一条射线上最近的不可通行障碍物坐标到障碍物向量中
            // 把未初始化的语义类别暂时当成障碍物
            // 未初始化的无语义成本不认为是障碍物
            if ((single_cost != 1)) //|| (single_cost == kNoSemantic)
            {

                // std::vector<double> obs_state = { x, y, single_cost};
                std::vector<double> obs_state = { x, y };

                sem_obs_list_.push_back(obs_state);

                // 只保存最近的障碍物
                break;
            }
        }
    }
}

double SemDWAPlanner::ObstacleCostFunction(const nav_msgs::OccupancyGrid& local_sem_map, const std::vector<State>& traj)
{
    double cost = 0.0;
    double dist = 0.0;
    double min_dist = 1e3;

    // 计算一条轨迹到障碍物集合的最小代价
    for (const auto& state : traj) {
        // 初始都是无语义成本的栅格，sem_obs_list_ 为空，返回 1 / 1000
        for (const auto& sem_obs : sem_obs_list_) {

            // 计算每个轨迹点与最近障碍物的距离
            dist = sqrt((state.x_ - sem_obs[0]) * (state.x_ - sem_obs[0]) + (state.y_ - sem_obs[1]) * (state.y_ - sem_obs[1]));

            // 不把小车当成质点，把 scout 当成以车体为中心，长度为直径的圆
            // 如果一个轨迹点与障碍物的距离小于 scout 车的外接圆半径？
            // 说明该条轨迹可能会发生碰撞，给高代价，否则，距离越远，代价越低

            // 轨迹采样太少，或者道路太窄会导致所有轨迹点到障碍物的最小距离都小于车的半径
            // 进而导致无法避障，相当于障碍物代价函数失效了

            //if (dist <= local_sem_map.info.resolution) {
            //if (dist <= car_rad_) {
            // 1e6 -> 1e3
            //    cost = 1e3;
            //min_dist =
            //    return cost;
            //}

            // 保存一条轨迹中所有轨迹点离障碍物的最小距离
            min_dist = std::min(min_dist, dist);
        }
    }

    // 距离越短，代价越高
    cost = 1.0 / min_dist;

    return cost;
}

double SemDWAPlanner::SpeedCostFunction(const std::vector<State>& traj, const double target_velocity)
{
    // 速度与 target_velocity 越接近，代价越低
    // 为了防止每项都为 0，增加一个额外的增量
    return std::fabs(target_velocity - traj[traj.size() - 1].velocity_) + 1.0;
}

double SemDWAPlanner::GoalCostFunction(const std::vector<State>& traj, const Eigen::Vector3d& goal)
{
#if 0
    // 传递的是全局目标点
    // 把轨迹末端点转到 map 下计算与目标点的距离
    geometry_msgs::PointStamped map_point;
    geometry_msgs::PointStamped base_point;

    // traj 点是以 base_link 为起点的世界坐标，需要先转换到 map 世界坐标系中
    base_point.header.frame_id = robot_frame_;
    base_point.header.stamp = ros::Time(0);
    base_point.point.x = traj.back().x_;
    base_point.point.y = traj.back().y_;
    base_point.point.z = traj.back().yaw_;

    listener_.transformPoint("map", base_point, map_point);

    Eigen::Vector3d traj_back(map_point.point.x, map_point.point.y, map_point.point.z);

    tf::StampedTransform baseToWorldTf;

    listener_.waitForTransform("map", robot_frame_, ros::Time(0), ros::Duration(3.0));
    listener_.lookupTransform("map", robot_frame_, ros::Time(0), baseToWorldTf);

    Eigen::Vector3d robot_base(baseToWorldTf.getOrigin().x(), baseToWorldTf.getOrigin().y(), tf::getYaw(baseToWorldTf.getRotation()));
#else
    // 传递的是局部目标点
    // 计算轨迹末端点离目标点的距离，距离越短，代价越低
    Eigen::Vector3d traj_back(traj.back().x_, traj.back().y_, traj.back().yaw_);
    return (traj_back.segment(0, 2) - goal.segment(0, 2)).norm();
#endif
}

double SemDWAPlanner::HeadingCostFunction(const std::vector<State>& traj, const Eigen::Vector3d& goal)
{
#if 0
    // 传递的是全局目标点
    //double goal_magnitude = std::sqrt(pow(goal_[0] - x_[0], 2) + pow(goal_[1] - x_[1], 2));
    double goal_magnitude = (goal.segment(0, 2) - robot_base.segment(0, 2)).norm();

    //double traj_magnitude = std::sqrt(std::pow(traj.back()[0] - x_[0], 2) + std::pow(traj.back()[1] - x_[1], 2));
    double traj_magnitude = (traj_back.segment(0, 2) - robot_base.segment(0, 2)).norm();

    //double dot_product = ((goal_[0] - x_[0]) * (traj.back()[0] - x_[0])) + ((goal_[1] - x_[1]) * (traj.back()[1] - x_[1]));
    double dot_product = ((goal[0] - robot_base[0]) * (traj_back[0] - robot_base[0])) + ((goal[1] - robot_base[1]) * (traj_back[1] - robot_base[1]));

    double error = dot_product / (goal_magnitude * traj_magnitude);

    double error_angle = std::acos(error);

    return error_angle;
#endif

#if 0
    // 这种方法计算的 cmu 轨迹与目标点的航向代价偏差较大
    // 传递的是局部目标点
    // 计算模拟轨迹的末端点与目标点的角度差，离目标点角度越小，代价越低
    // 有点问题
    Eigen::Vector3d traj_back(traj.back().x_, traj.back().y_, traj.back().yaw_);

    double traj_back_theta = ToDegree(traj_back[2]);

    double goal_theta = ToDegree(std::atan2(goal[1] - traj_back[1], goal[0] - traj_back[0]));

    double target_theta = (goal_theta > traj_back_theta) ? (goal_theta - traj_back_theta) : (traj_back_theta - goal_theta);

    return target_theta;
#endif

#if 1
    // 这种方法计算的 cmu 轨迹与目标点的航向代价偏差较小
    // 传递的是局部目标点
    Eigen::Vector3d traj_back(traj.back().x_, traj.back().y_, traj.back().yaw_);

    double goal_magnitude = goal.segment(0, 2).norm();

    // DWA 采样轨迹过短会导致 traj_magnitude = 0
    double traj_magnitude = traj_back.segment(0, 2).norm();

    double dot_product = (goal[0] * traj_back[0]) + (goal[1] * traj_back[1]);

    double error = dot_product / (goal_magnitude * traj_magnitude);

    double error_angle = std::acos(error);

    return error_angle;
#endif
}

double SemDWAPlanner::YawrateCostFunction(const std::vector<State>& traj)
{
    return std::fabs(traj[0].yawrate_);
}

// 如果轨迹长度相同，轨迹点数量相同 -> 轨迹点间隔相同 -> dt 时间段内 vel 相同
void SemDWAPlanner::GenerateSemDWATrajectory(const SemDWAPlanner::DynamicWindow& dynamic_window, const geometry_msgs::Twist& current_velocity)
{
    trajectories_.clear();

    // 在动态窗口内采样轨迹，并计算每个轨迹的代价
    // Bug: 当 min_velocity_ 参数设置为 0.1 时，max_velocity_ 计算为 0.0997，会导致轨迹采样数量为 0
    for (double v = dynamic_window.min_velocity_; v <= dynamic_window.max_velocity_; v += velocity_resolution_) {
        for (double y = dynamic_window.min_yawrate_; y <= dynamic_window.max_yawrate_; y += yawrate_resolution_) {

            State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);

            std::vector<State> traj;

            // 每条轨迹的线速度和角速度不同，虽然采样相同时间，轨迹点数相同，但是轨迹间隔（分辨率）不同
            for (double t = 0; t <= predict_time_; t += dt_) {
                Motion(state, v, y);

                traj.push_back(state);
                // predict_time = 20s, 实际上采样 10s
                t += dt_;
            }

            // predict_time_ = 5s, for 循环应该采样 50 点，但是会多采样一个，这里 pop 丢掉
            //traj.pop_back();

            trajectories_.push_back(traj);
        }
    }
}

void SemDWAPlanner::GenerateROSDWATrajectory(const DynamicWindow& dynamic_window, const geometry_msgs::Twist& current_velocity)
{
    double sim_time = 5.0;

    // 查看 dwa_local_planner 的默认参数
    double sim_granularity = 0.025;
    double angular_sim_granularity = 0.025;

    trajectories_.clear();

    // 在动态窗口内采样轨迹，并计算每个轨迹的代价
    for (double v = dynamic_window.min_velocity_; v <= dynamic_window.max_velocity_; v += velocity_resolution_) {
        for (double y = dynamic_window.min_yawrate_; y <= dynamic_window.max_yawrate_; y += yawrate_resolution_) {

            double sim_time_distance = current_velocity.linear.x * sim_time;
            double sim_time_angle = std::fabs(current_velocity.angular.z) * sim_time;

            // 根据仿真粒度计算仿真步数，即样本点数
            int num_steps = ceil(std::max(sim_time_distance / sim_granularity, sim_time_angle / angular_sim_granularity));

            // 计算仿真（采样）步长
            double dt = sim_time / num_steps;

            State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z);

            std::vector<State> traj;

            for (int i = 0; i < num_steps; ++i) {
                // 每计算一个点就加到轨迹中
                traj.push_back(state);

                // 计算每个轨迹点新的位置
                ROSDWAMotion(state, v, y, dt);
            }

            trajectories_.push_back(traj);
        }
    }
}

#if 0
// 弃用
void SemDWAPlanner::CostSemDWATrajectory(const nav_msgs::OccupancyGrid& local_sem_map, const Eigen::Vector3d& goal)
{
    double semantic_cost = 0.0;
    double obstacle_cost = 0.0;
    double goal_cost = 0.0;
    double yawrate_cost = 0.0;

    // 不可通行轨迹点的判断目前不使用
    int untrave_point_num = 0;

    semantic_cost_vec_.clear();
    obstacle_cost_vec_.clear();
    speed_cost_vec_.clear();
    goal_cost_vec_.clear();
    yawrate_cost_vec_.clear();

    // 初始化语义障碍物集合，用于计算障碍物代价函数
    Raycast(local_sem_map);

    for (const auto& traj : trajectories_) {
        semantic_cost = SemanticCostFunction(local_sem_map, traj, untrave_point_num);
        semantic_cost_vec_.push_back(semantic_cost);

        obstacle_cost = ObstacleCostFunction(local_sem_map, traj);
        obstacle_cost_vec_.push_back(obstacle_cost);

        goal_cost = GoalCostFunction(traj, goal);
        goal_cost_vec_.push_back(goal_cost);

        yawrate_cost = YawrateCostFunction(traj);
        yawrate_cost_vec_.push_back(yawrate_cost);
    }
}
#endif



void SemDWAPlanner::NormalizeSemDWATrajectory()
{
    if (trave_trajectories_.size() == 0) {
        ROS_INFO("SemDWAPlanner [%s] Currently not traveable trajectories.", __FUNCTION__);
        return;
    }

    // 将每组轨迹代价进行归一化，即每一项除以该项的总和
    double semantic_cost_sum = std::accumulate(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), 0.0);
    double goal_cost_sum = std::accumulate(goal_cost_vec_.begin(), goal_cost_vec_.end(), 0.0);
    double yawrate_cost_sum = std::accumulate(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), 0.0);
    //double obstacle_cost_sum = std::accumulate(obstacle_cost_vec_.begin(), obstacle_cost_vec_.end(), 0.0);
    double speed_cost_sum = std::accumulate(speed_cost_vec_.begin(), speed_cost_vec_.end(), 0.0);
    double heading_cost_sum = std::accumulate(heading_cost_vec_.begin(), heading_cost_vec_.end(), 0.0);

#if DEBUG_INFO
    // min_velocity 设置为 0.1 可能导致 trajectories_ 为空，进而导致 semantic_cost_vec_ 为空
    std::cout << std::endl;
    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    //std::cout << "obstacle_cost_vec_[0]: " << obstacle_cost_vec_[0] << std::endl;
    std::cout << "speed_cost_vec_[0]: " << speed_cost_vec_[0] << std::endl;
    std::cout << "heading_cost_vec_[0]: " << heading_cost_vec_[0] << std::endl;
    std::cout << std::endl;

    std::cout << "semantic_cost_sum: " << semantic_cost_sum << std::endl;
    std::cout << "goal_cost_sum: " << goal_cost_sum << std::endl;
    std::cout << "yawrate_cost_sum: " << yawrate_cost_sum << std::endl;
    //std::cout << "obstacle_cost_sum: " << obstacle_cost_sum << std::endl;
    std::cout << "speed_cost_sum: " << speed_cost_sum << std::endl;
    std::cout << "heading_cost_sum: " << heading_cost_sum << std::endl;
    std::cout << std::endl;
#endif

    // 只有总和大于 0，才进行归一化，或者允许类别初始无语义成本 70
    //if (std::fabs(semantic_cost_sum) > 0.000001)
    std::for_each(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), [semantic_cost_sum](double& sem_cost) { sem_cost /= semantic_cost_sum; });
    std::for_each(goal_cost_vec_.begin(), goal_cost_vec_.end(), [goal_cost_sum](double& goal_cost) { goal_cost /= goal_cost_sum; });
    std::for_each(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), [yawrate_cost_sum](double& yawrate_cost) { yawrate_cost /= yawrate_cost_sum; });
    //std::for_each(obstacle_cost_vec_.begin(), obstacle_cost_vec_.end(), [obstacle_cost_sum](double& obs_cost) { obs_cost /= obstacle_cost_sum; });
    std::for_each(speed_cost_vec_.begin(), speed_cost_vec_.end(), [speed_cost_sum](double& speed_cost) { speed_cost /= speed_cost_sum; });
    std::for_each(heading_cost_vec_.begin(), heading_cost_vec_.end(), [heading_cost_sum](double& heading_cost) { heading_cost /= heading_cost_sum; });

#if DEBUG_INFO
    std::cout << "After Normalization..." << std::endl;
    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    //std::cout << "obstacle_cost_vec_[0]: " << obstacle_cost_vec_[0] << std::endl;
    std::cout << "speed_cost_vec_[0]: " << speed_cost_vec_[0] << std::endl;
    std::cout << "heading_cost_vec_[0]: " << heading_cost_vec_[0] << std::endl;
    std::cout << std::endl;
#endif
}

#if 0
// 弃用
void SemDWAPlanner::NormalizeSemDWATrajectory()
{
    // 将每组轨迹代价进行归一化，即每一项除以该项的总和
    double semantic_cost_sum = std::accumulate(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), 0.0);
    double obstacle_cost_sum = std::accumulate(obstacle_cost_vec_.begin(), obstacle_cost_vec_.end(), 0.0);
    double goal_cost_sum = std::accumulate(goal_cost_vec_.begin(), goal_cost_vec_.end(), 0.0);
    double yawrate_cost_sum = std::accumulate(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), 0.0);

    // min_velocity 设置为 0.1 可能导致 trajectories_ 为空，进而导致 semantic_cost_vec_ 为空
    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "obstacle_cost_vec_[0]: " << obstacle_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    std::cout << std::endl;

    std::cout << "semantic_cost_sum: " << semantic_cost_sum << std::endl;
    std::cout << "obstacle_cost_sum: " << obstacle_cost_sum << std::endl;
    std::cout << "goal_cost_sum: " << goal_cost_sum << std::endl;
    std::cout << "yawrate_cost_sum: " << yawrate_cost_sum << std::endl;
    std::cout << std::endl;

    // 只有总和大于 0，才进行归一化，或者允许类别初始无语义成本 70
    //if (std::fabs(semantic_cost_sum) > 0.000001)
    std::for_each(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), [semantic_cost_sum](double& sem_cost) { sem_cost /= semantic_cost_sum; });

    std::for_each(obstacle_cost_vec_.begin(), obstacle_cost_vec_.end(), [obstacle_cost_sum](double& obs_cost) { obs_cost /= obstacle_cost_sum; });
    std::for_each(goal_cost_vec_.begin(), goal_cost_vec_.end(), [goal_cost_sum](double& goal_cost) { goal_cost /= goal_cost_sum; });
    std::for_each(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), [yawrate_cost_sum](double& yawrate_cost) { yawrate_cost /= yawrate_cost_sum; });

    std::cout << "After Normalization..." << std::endl;
    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "obstacle_cost_vec_[0]: " << obstacle_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    std::cout << std::endl;
}
#endif

#if 0
// 弃用
void SemDWAPlanner::SelectSemDWATrajectory()
{
    double min_cost = 1e6;

    double min_semantic_cost = min_cost;
    double min_obstacle_cost = min_cost;
    double min_goal_cost = min_cost;
    double min_yawrate_cost = min_cost;

    double final_cost = min_cost;

    best_traj_.clear();

    for (int i = 0; i < trajectories_.size(); i++) {

        // minimize this funciton!
        // 如果不想使用某个代价函数的约束，将增益参数配置为 0 即可
        final_cost = sem_gain_ * semantic_cost_vec_[i] + obs_gain_ * obstacle_cost_vec_[i] + goal_gain_ * goal_cost_vec_[i] + yawrate_gain_ * yawrate_cost_vec_[i];

        if (final_cost <= min_cost) {
            min_semantic_cost = semantic_cost_vec_[i];
            min_obstacle_cost = obstacle_cost_vec_[i];
            min_goal_cost = goal_cost_vec_[i];
            min_yawrate_cost = yawrate_cost_vec_[i];

            min_cost = final_cost;

            best_traj_ = trajectories_[i];
        }
    }

    std::cout << "min semantic cost: " << min_semantic_cost << std::endl;
    std::cout << "min obstacle cost: " << min_obstacle_cost << std::endl;
    std::cout << "min goal cost: " << min_goal_cost << std::endl;
    std::cout << "min yawrate cost: " << min_yawrate_cost << std::endl;
    std::cout << "min cost: " << min_cost << std::endl;
    std::cout << std::endl;

    std::cout << "trajectories size: " << trajectories_.size() << std::endl;
    std::cout << "trajectorie[0] size: " << trajectories_[0].size() << std::endl;
    std::cout << "trajectorie[1] size: " << trajectories_[1].size() << std::endl;
    std::cout << std::endl;

    if (min_cost == 1e6) {
        std::cout << "min_cost to big, pub cmd_vel 0:" << min_cost << std::endl;
        std::vector<State> traj;

        State state(0.0, 0.0, 0.0, 0.0, 0.0);
        traj.push_back(state);

        best_traj_ = traj;
    }
}
#endif

void SemDWAPlanner::GenerateCMUTrajectory()
{
    double cur_vel = 0.0;
    double cur_yaw = 0.0;

    double traj_len = 0.0;

    double yaw_inc = 0.0;
    double x_inc = 0.0;
    double y_inc = 0.0;

    int traj_size = (cmu_max_yaw_ / cmu_yaw_res_) * 2;

    trajectories_.clear();

    for (int i = 0; i < traj_size; i++) {
        // 每条轨迹的线速度保持不变
        cur_vel = cmu_max_vel_;
        // 每条轨迹的角速度依次减少 1 度 = 0.017 弧度
        cur_yaw = cmu_max_yaw_ - (i * cmu_yaw_res_);

        State state(0.0, 0.0, 0.0, cur_vel, cur_yaw);

        // 当前采样的一条轨迹
        std::vector<State> traj;
        traj_len = 0.0;

        while (traj_len <= cmu_traj_len_) {
            // 计算角度，x，y 的增量
            yaw_inc = cur_yaw * dt_;
            x_inc = cur_vel * std::cos(state.yaw_) * dt_;
            y_inc = cur_vel * std::sin(state.yaw_) * dt_;

            // 累加当前 dt_ 时间段内的增量到轨迹点中
            state.yaw_ += yaw_inc;
            state.x_ += x_inc;
            state.y_ += y_inc;
            state.velocity_ = cur_vel;
            state.yawrate_ = cur_yaw;

            // 曲线积分：DT 时间段内，曲线上 2 点之间长度可用两点的欧式距离近视代替
            traj_len += sqrt((x_inc * x_inc) + (y_inc * y_inc));

            // 把当前采样的轨迹点 push 到当前采样的轨迹中
            traj.push_back(state);
        }

        // 保证轨迹点为 80 个方便后续计算丢弃轨迹，不 pop_back 则会采样 81 的轨迹点
        traj.pop_back();

        trajectories_.push_back(traj);
    }
}

// 附加轨迹的末端靠的的太密了，不知为何
void SemDWAPlanner::GenerateCMUWidthTrajectory(const std::vector<State>& center_traj)
{
    width_trajectories_.clear();

    //int width_traj_size = scout_width_ / local_map_res_;
    double shift_traj_interval = car_width_ / (shift_traj_num_ - 1);

    // 采样 y 轴负方向的 3 条轨迹
    for (int i = 1; i <= shift_traj_num_ / 2; i++) {
        std::vector<State> shift_traj;

        State shift_state(0.0, 0.0, 0.0, 0.0, 0.0);

        for (auto& point : center_traj) {
            shift_state.x_ = point.x_;
            shift_state.y_ = point.y_ - i * shift_traj_interval;
            shift_state.yaw_ = point.yaw_;

            shift_state.velocity_ = point.velocity_;
            shift_state.yawrate_ = point.yawrate_;

            shift_traj.push_back(shift_state);
        }

        width_trajectories_.push_back(shift_traj);
    }

    // 把中心轨迹放入 width 轨迹向量
    width_trajectories_.push_back(center_traj);

    // 采样 y 轴正方向的 3 条轨迹
    for (int i = 1; i <= shift_traj_num_ / 2; i++) {
        std::vector<State> shift_traj;

        State shift_state(0.0, 0.0, 0.0, 0.0, 0.0);

        for (auto& point : center_traj) {
            shift_state.x_ = point.x_;
            shift_state.y_ = point.y_ + i * shift_traj_interval;
            shift_state.yaw_ = point.yaw_;

            shift_state.velocity_ = point.velocity_;
            shift_state.yawrate_ = point.yawrate_;

            shift_traj.push_back(shift_state);
        }

        width_trajectories_.push_back(shift_traj);
    }
}

// 用 DWA 采样的轨迹的第一个位姿点来重新采样一条长度为 X 的 CMU 轨迹，线速度 0.5m/s，角速度跟输出轨迹相同
void SemDWAPlanner::GenerateCMUTrajectoryWithYawrate(const std::vector<State>& center_traj, std::vector<State>& cmu_traj)
{
    // 采样一条 CMU 轨迹，线速度 0.5m/s，角速度与参数 center_traj 相同
    double cur_vel = 0.0;
    double cur_yaw = 0.0;

    double traj_len = 0.0;

    double yaw_inc = 0.0;
    double x_inc = 0.0;
    double y_inc = 0.0;

    // 只采样 1 条 CMU 轨迹
    int traj_size = 1;
    cmu_traj.clear();

    for (int i = 0; i < traj_size; i++) {
        // 轨迹的线速度保持不变 0.14 / 0.1 = 1.4 m/s
        cur_vel = local_map_res_ / dt_;
        // 轨迹的角速度为 dwa 的采样速度
        //cur_yaw = (cur_vel / max_velocity_) * center_traj[0].yawrate_;

        // 可以
        if (center_traj[0].velocity_ < 0.000001)
            cur_yaw = center_traj[0].yawrate_; // 在计算语义代价前丢弃线速度为 0 的轨迹？或者增加角速度代价函数，限制原地旋转的轨迹？
        else
            cur_yaw = cur_vel * (center_traj[0].yawrate_ / center_traj[0].velocity_);
       
        if (std::fabs(cur_yaw) >= max_sem_traj_yawrate_)
            cur_yaw = (cur_yaw > 0) ? max_sem_traj_yawrate_ : -max_sem_traj_yawrate_;

        State state(0.0, 0.0, 0.0, cur_vel, cur_yaw);

        traj_len = 0.0;

        while (traj_len <= CalcCMUTrajLength(center_traj[0].velocity_, cmu_traj_len_)) {
            // 计算角度，x，y 的增量
            yaw_inc = cur_yaw * dt_;
            x_inc = cur_vel * std::cos(state.yaw_) * dt_;
            y_inc = cur_vel * std::sin(state.yaw_) * dt_;

            // 累加当前 dt_ 时间段内的增量到轨迹点中
            state.yaw_ += yaw_inc;
            state.x_ += x_inc;
            state.y_ += y_inc;
            state.velocity_ = cur_vel;
            state.yawrate_ = cur_yaw;

            // 曲线积分：DT 时间段内，曲线上 2 点之间长度可用两点的欧式距离近视代替
            traj_len += sqrt((x_inc * x_inc) + (y_inc * y_inc));

            // 0.14 m = res
            //std::cout << "delta length = " << sqrt((x_inc * x_inc) + (y_inc * y_inc)) << std::endl;

            // 把当前采样的轨迹点 push 到当前采样的轨迹中
            cmu_traj.push_back(state);
        }

        // 保证轨迹点为 80 个方便后续计算丢弃轨迹，不 pop_back 则会采样 81 的轨迹点
        cmu_traj.pop_back();
    }

    // 每个 cmu 轨迹点数都相同，分辨率也相同
    //std::cout << "cmu_traj.size() = " << cmu_traj.size() << std::endl;
}

double SemDWAPlanner::CalcCMUTrajLength(const double cur_velocity, const double base_length)
{
    if (calc_cmu_traj_len_ == false)
        return base_length;

    // max_range = 8.0m
    double max_range = local_map_range_ - 1.0;
    // 最大速度到达最大范围的时间 作为采样时间
    double max_delta_sec = (max_range - base_length) / max_velocity_;
    // 计算以当前 dwa 轨迹的线速度采样 6s 走过的距离
    double delta_length = cur_velocity * dt_ * 10 * max_delta_sec;

    // 如果距离超过了最大范围则返回最大范围，否则返回增加的采样长度
    if (base_length + delta_length > max_range)
        return max_range;
    else
        return base_length + delta_length;
}

void SemDWAPlanner::CostSemDWATrajectory(const nav_msgs::OccupancyGrid& local_sem_map, const Eigen::Vector3d& goal)
{
    double width_semantic_cost = 0.0;
    double single_semantic_cost = 0.0;
    double goal_cost = 0.0;
    double yawrate_cost = 0.0;
    double obstacle_cost = 0.0;
    double speed_cost = 0.0;
    double heading_cost = 0.0;

    int untrave_point_num = 0;
    untrave_trajectories_.clear();
    trave_trajectories_.clear();
    cmu_trajectories_.clear();

    semantic_cost_vec_.clear();
    goal_cost_vec_.clear();
    yawrate_cost_vec_.clear();
    obstacle_cost_vec_.clear();
    speed_cost_vec_.clear();
    heading_cost_vec_.clear();

    // 初始化语义障碍物集合，用于计算障碍物代价函数
    Raycast(local_sem_map);

    int traj_index = 0;

    for (const auto& traj : trajectories_) {
        // 可通行语义代价
        traj_index++;

        // 用 dwa 采样的轨迹角速度来采样一条 cmu 固定长度的轨迹
        // 用于计算当前 dwa 轨迹方向的可通行代价
        GenerateCMUTrajectoryWithYawrate(traj, cmu_traj_);

        // 计算 cmu 固定长度轨迹的宽度轨迹
        GenerateCMUWidthTrajectory(cmu_traj_);

        // 每条轨迹按照车身宽度多采样 4 条轨迹，计算 5 条轨迹的语义代价和作为当前轨迹的代价
        for (const auto& width_traj : width_trajectories_) {
            single_semantic_cost = CMUSemanticCostFunction(local_sem_map, width_traj, untrave_point_num);

            if ((untrave_point_num > untrave_point_num_) && abandon_untrave_traj_) {
                //ROS_INFO("SemDWAPlanner [%s] Cur traj[%d]'s width_traj(size = %zu) at least has %d untrave point, so throw away it.", __FUNCTION__, traj_index, traj.size(), untrave_point_num);
                break;
            }

            if (single_semantic_cost == (kNoSemantic * width_traj.size() / 100.0)) {
                //ROS_INFO("SemDWAPlanner [%s] Cur traj[%d](size = %zu) semantic no init, cost = %f(X100).", __FUNCTION__, traj_index, width_traj.size(), single_semantic_cost);
                // 如果一条轨迹上都是未初始化的语义代价为 0 的轨迹，把代价升高，让规划器选择其他轨迹
                //ROS_INFO("SemDWAPlanner [%s] Cur traj[%d](size = %zu) semantic no init, cost = %f(X100), so set cost = %f(kObstacleCost * %zu / 100.0)", __FUNCTION__, traj_index, width_traj.size(), single_semantic_cost, kObstacleCost * width_traj.size() / 100.0, width_traj.size());
                //single_semantic_cost = kObstacleCost * width_traj.size() / 100.0;
            } else {
                ; //ROS_INFO("SemDWAPlanner [%s] Cur traj(size = %zu) semantic cost = %f(X100)", __FUNCTION__, width_traj.size(), single_semantic_cost);
            }

            width_semantic_cost += single_semantic_cost;
        }

        // 或者不可通行点大于一定数量才认为是不可通行轨迹
        // 可以配置是否丢弃轨迹，默认不丢弃
        if ((untrave_point_num > untrave_point_num_) && abandon_untrave_traj_) {
            // 把当前轨迹加入丢弃向量中用于返回显示
            untrave_trajectories_.push_back(traj);

            // 丢弃当前轨迹
            continue;
        }

        // 可以通行的轨迹单独放在可通行轨迹向量中
        trave_trajectories_.push_back(traj);

        // cmu 轨迹单独放在一个向量中用于 RVIZ 显示
        cmu_trajectories_.push_back(cmu_traj_);

        //ROS_INFO("SemDWAPlanner [%s] Cur traj[%d](size = %zu) width semantic cost = %f(X100)", __FUNCTION__, traj_index, traj.size(), width_semantic_cost);

        semantic_cost_vec_.push_back(width_semantic_cost);
        width_semantic_cost = 0.0;

        // 目标距离代价，dwa 末端距离计算导致到达局部目标点减速
        goal_cost = GoalCostFunction(traj, goal);
        goal_cost_vec_.push_back(goal_cost);

        // 角速度代价
        yawrate_cost = YawrateCostFunction(traj);
        yawrate_cost_vec_.push_back(yawrate_cost);

        // 障碍物代价
        obstacle_cost = ObstacleCostFunction(local_sem_map, traj);
        obstacle_cost_vec_.push_back(obstacle_cost);

        // 期望速度代价
        speed_cost = SpeedCostFunction(traj, target_velocity_);
        speed_cost_vec_.push_back(speed_cost);

        // 目标航向偏差代价
        // 使用 cmu 轨迹计算目标航向，防止到达目标点减速
        heading_cost = HeadingCostFunction(cmu_traj_, goal);
        heading_cost_vec_.push_back(heading_cost);
    }
}

void SemDWAPlanner::CostCMUTrajectory(const nav_msgs::OccupancyGrid& local_sem_map, const Eigen::Vector3d& goal)
{
    double width_semantic_cost = 0.0;
    double single_semantic_cost = 0.0;
    double goal_cost = 0.0;
    double yawrate_cost = 0.0;
    double obstacle_cost = 0.0;
    double speed_cost = 0.0;
    double heading_cost = 0.0;

    int untrave_point_num = 0;
    untrave_trajectories_.clear();
    trave_trajectories_.clear();
    cmu_trajectories_.clear();

    semantic_cost_vec_.clear();
    goal_cost_vec_.clear();
    yawrate_cost_vec_.clear();
    obstacle_cost_vec_.clear();
    speed_cost_vec_.clear();
    heading_cost_vec_.clear();

    // 初始化语义障碍物集合，用于计算障碍物代价函数
    //Raycast(local_sem_map);

    int traj_index = 0;

    for (const auto& traj : trajectories_) {
        // 可通行语义代价
        traj_index++;

        // 用 dwa 采样的轨迹角速度来采样一条 cmu 固定长度的轨迹
        // 用于计算当前 dwa 轨迹方向的可通行代价
        //GenerateCMUTrajectoryWithYawrate(traj, cmu_traj_);

        // 计算 cmu 固定长度轨迹的宽度轨迹
        GenerateCMUWidthTrajectory(traj);

        // 每条轨迹按照车身宽度多采样 4 条轨迹，计算 5 条轨迹的语义代价和作为当前轨迹的代价
        for (const auto& width_traj : width_trajectories_) {
            single_semantic_cost = CMUSemanticCostFunction(local_sem_map, width_traj, untrave_point_num);

            if ((untrave_point_num > untrave_point_num_) && abandon_untrave_traj_) {
                //ROS_INFO("SemDWAPlanner [%s] Cur traj[%d]'s width_traj(size = %zu) at least has %d untrave point, so throw away it.", __FUNCTION__, traj_index, traj.size(), untrave_point_num);
                break;
            }

            if (single_semantic_cost == (kNoSemantic * width_traj.size() / 100.0)) {
                //ROS_INFO("SemDWAPlanner [%s] Cur traj[%d](size = %zu) semantic no init, cost = %f(X100).", __FUNCTION__, traj_index, width_traj.size(), single_semantic_cost);
                // 如果一条轨迹上都是未初始化的语义代价为 0 的轨迹，把代价升高，让规划器选择其他轨迹
                //ROS_INFO("SemDWAPlanner [%s] Cur traj[%d](size = %zu) semantic no init, cost = %f(X100), so set cost = %f(kObstacleCost * %zu / 100.0)", __FUNCTION__, traj_index, width_traj.size(), single_semantic_cost, kObstacleCost * width_traj.size() / 100.0, width_traj.size());
                //single_semantic_cost = kObstacleCost * width_traj.size() / 100.0;
            } else {
                ; //ROS_INFO("SemDWAPlanner [%s] Cur traj(size = %zu) semantic cost = %f(X100)", __FUNCTION__, width_traj.size(), single_semantic_cost);
            }

            width_semantic_cost += single_semantic_cost;
        }

        // 或者不可通行点大于一定数量才认为是不可通行轨迹
        // 可以配置是否丢弃轨迹，默认不丢弃
        if ((untrave_point_num > untrave_point_num_) && abandon_untrave_traj_) {
            // 把当前轨迹加入丢弃向量中用于返回显示
            untrave_trajectories_.push_back(traj);

            // 丢弃当前轨迹
            continue;
        }

        // 可以通行的轨迹单独放在可通行轨迹向量中
        trave_trajectories_.push_back(traj);

        // cmu 轨迹单独放在一个向量中用于 RVIZ 显示
        cmu_trajectories_.push_back(cmu_traj_);

        //ROS_INFO("SemDWAPlanner [%s] Cur traj[%d](size = %zu) width semantic cost = %f(X100)", __FUNCTION__, traj_index, traj.size(), width_semantic_cost);

        semantic_cost_vec_.push_back(width_semantic_cost);
        width_semantic_cost = 0.0;

        // 目标距离代价
        goal_cost = GoalCostFunction(traj, goal);
        goal_cost_vec_.push_back(goal_cost);

        // 角速度代价
        yawrate_cost = YawrateCostFunction(traj);
        yawrate_cost_vec_.push_back(yawrate_cost);

        // 障碍物代价
        //obstacle_cost = ObstacleCostFunction(local_sem_map, traj);
        //obstacle_cost_vec_.push_back(obstacle_cost);

        // 期望速度代价
        speed_cost = SpeedCostFunction(traj, target_velocity_);
        speed_cost_vec_.push_back(speed_cost);

        // 目标航向偏差代价
        // 使用 cmu 轨迹计算目标航向，防止到达目标点减速
        heading_cost = HeadingCostFunction(traj, goal);
        heading_cost_vec_.push_back(heading_cost);
    }
}

void SemDWAPlanner::NormalizeCMUTrajectory()
{
    if (trave_trajectories_.size() == 0) {
        ROS_INFO("SemDWAPlanner [%s] Currently not traveable trajectories.", __FUNCTION__);
        return;
    }

    // 将每组轨迹代价进行归一化，即每一项除以该项的总和
    double semantic_cost_sum = std::accumulate(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), 0.0);
    double goal_cost_sum = std::accumulate(goal_cost_vec_.begin(), goal_cost_vec_.end(), 0.0);
    double yawrate_cost_sum = std::accumulate(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), 0.0);
    //double obstacle_cost_sum = std::accumulate(obstacle_cost_vec_.begin(), obstacle_cost_vec_.end(), 0.0);
    double speed_cost_sum = std::accumulate(speed_cost_vec_.begin(), speed_cost_vec_.end(), 0.0);
    double heading_cost_sum = std::accumulate(heading_cost_vec_.begin(), heading_cost_vec_.end(), 0.0);

#if DEBUG_INFO
    // min_velocity 设置为 0.1 可能导致 trajectories_ 为空，进而导致 semantic_cost_vec_ 为空
    std::cout << std::endl;
    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    //std::cout << "obstacle_cost_vec_[0]: " << obstacle_cost_vec_[0] << std::endl;
    std::cout << "speed_cost_vec_[0]: " << speed_cost_vec_[0] << std::endl;
    std::cout << "heading_cost_vec_[0]: " << heading_cost_vec_[0] << std::endl;
    std::cout << std::endl;

    std::cout << "semantic_cost_sum: " << semantic_cost_sum << std::endl;
    std::cout << "goal_cost_sum: " << goal_cost_sum << std::endl;
    std::cout << "yawrate_cost_sum: " << yawrate_cost_sum << std::endl;
    //std::cout << "obstacle_cost_sum: " << obstacle_cost_sum << std::endl;
    std::cout << "speed_cost_sum: " << speed_cost_sum << std::endl;
    std::cout << "heading_cost_sum: " << heading_cost_sum << std::endl;
    std::cout << std::endl;
#endif

    // 只有总和大于 0，才进行归一化，或者允许类别初始无语义成本 70
    //if (std::fabs(semantic_cost_sum) > 0.000001)
    std::for_each(semantic_cost_vec_.begin(), semantic_cost_vec_.end(), [semantic_cost_sum](double& sem_cost) { sem_cost /= semantic_cost_sum; });
    std::for_each(goal_cost_vec_.begin(), goal_cost_vec_.end(), [goal_cost_sum](double& goal_cost) { goal_cost /= goal_cost_sum; });
    std::for_each(yawrate_cost_vec_.begin(), yawrate_cost_vec_.end(), [yawrate_cost_sum](double& yawrate_cost) { yawrate_cost /= yawrate_cost_sum; });
    //std::for_each(obstacle_cost_vec_.begin(), obstacle_cost_vec_.end(), [obstacle_cost_sum](double& obs_cost) { obs_cost /= obstacle_cost_sum; });
    std::for_each(speed_cost_vec_.begin(), speed_cost_vec_.end(), [speed_cost_sum](double& speed_cost) { speed_cost /= speed_cost_sum; });
    std::for_each(heading_cost_vec_.begin(), heading_cost_vec_.end(), [heading_cost_sum](double& heading_cost) { heading_cost /= heading_cost_sum; });

#if DEBUG_INFO
    std::cout << "After Normalization..." << std::endl;
    std::cout << "semantic_cost_vec_[0]: " << semantic_cost_vec_[0] << std::endl;
    std::cout << "goal_cost_vec_[0]: " << goal_cost_vec_[0] << std::endl;
    std::cout << "yawrate_cost_vec_[0]: " << yawrate_cost_vec_[0] << std::endl;
    //std::cout << "obstacle_cost_vec_[0]: " << obstacle_cost_vec_[0] << std::endl;
    std::cout << "speed_cost_vec_[0]: " << speed_cost_vec_[0] << std::endl;
    std::cout << "heading_cost_vec_[0]: " << heading_cost_vec_[0] << std::endl;
    std::cout << std::endl;
#endif
}

void SemDWAPlanner::SelectCMUTrajectory()
{
    double min_cost = 1e6;

    double min_semantic_cost = min_cost;
    double min_goal_cost = min_cost;
    double min_yawrate_cost = min_cost;
    double min_obstacle_cost = min_cost;
    double min_speed_cost = min_cost;
    double min_heading_cost = min_cost;

    double final_cost = min_cost;

    best_traj_.clear();
    best_cmu_traj_.clear();

    for (int i = 0; i < trave_trajectories_.size(); i++) {

        // minimize this funciton!
        // 如果不想使用某个代价函数的约束，将增益参数配置为 0 即可
        final_cost = sem_gain_ * semantic_cost_vec_[i]
            + goal_gain_ * goal_cost_vec_[i]
            + yawrate_gain_ * yawrate_cost_vec_[i]
            + speed_gain_ * speed_cost_vec_[i]
            + heading_gain_ * heading_cost_vec_[i];
        //+ obs_gain_ * obstacle_cost_vec_[i]

        if (final_cost <= min_cost) {
            min_semantic_cost = semantic_cost_vec_[i];
            //min_obstacle_cost = obstacle_cost_vec_[i];
            min_goal_cost = goal_cost_vec_[i];
            min_yawrate_cost = yawrate_cost_vec_[i];
            min_speed_cost = speed_cost_vec_[i];
            min_heading_cost = heading_cost_vec_[i];

            min_cost = final_cost;

            best_traj_ = trave_trajectories_[i];
            best_cmu_traj_ = cmu_trajectories_[i];
        }
    }

#if DEBUG_INFO
    std::cout << std::endl;
    std::cout << "min semantic cost: " << min_semantic_cost << std::endl;
    std::cout << "min goal cost: " << min_goal_cost << std::endl;
    std::cout << "min yawrate cost: " << min_yawrate_cost << std::endl;
    //std::cout << "min obstacle cost: " << min_obstacle_cost << std::endl;
    std::cout << "min speed cost: " << min_speed_cost << std::endl;
    std::cout << "min heading cost: " << min_heading_cost << std::endl;

    std::cout << "min cost: " << min_cost << std::endl;
    std::cout << std::endl;

    std::cout << "trave trajectories size: " << trave_trajectories_.size() << std::endl;
    std::cout << "cmu trajectories size: " << cmu_trajectories_.size() << std::endl;

    if (trave_trajectories_.size() != 0) {
        std::cout << "trave trajectorie[0] size: " << trave_trajectories_[0].size() << std::endl;
        std::cout << "trave trajectorie[1] size: " << trave_trajectories_[1].size() << std::endl;

        std::cout << "cmu trajectorie[0] size: " << cmu_trajectories_[0].size() << std::endl;
        std::cout << "cmu trajectorie[1] size: " << cmu_trajectories_[1].size() << std::endl;
    }

    std::cout << std::endl;
#endif

    if (min_cost == 1e6) {
        //std::cout << "min_cost to big: " << min_cost << ", pub cmd_vel 0." << std::endl;
        //std::cout << std::endl;

        std::vector<State> traj;

        State state(0.0, 0.0, 0.0, 0.0, 0.0);
        traj.push_back(state);

        best_traj_ = traj;
        best_cmu_traj_ = traj;
    }
}



void SemDWAPlanner::SelectSemDWATrajectory()
{
    double min_cost = 1e6;

    double min_semantic_cost = min_cost;
    double min_goal_cost = min_cost;
    double min_yawrate_cost = min_cost;
    double min_obstacle_cost = min_cost;
    double min_speed_cost = min_cost;
    double min_heading_cost = min_cost;

    double final_cost = min_cost;

    best_traj_.clear();
    best_cmu_traj_.clear();

    for (int i = 0; i < trave_trajectories_.size(); i++) {

        // minimize this funciton!
        // 如果不想使用某个代价函数的约束，将增益参数配置为 0 即可
        final_cost = sem_gain_ * semantic_cost_vec_[i]
            + goal_gain_ * goal_cost_vec_[i]
            + yawrate_gain_ * yawrate_cost_vec_[i]
            + speed_gain_ * speed_cost_vec_[i]
            + heading_gain_ * heading_cost_vec_[i];
        //+ obs_gain_ * obstacle_cost_vec_[i]

        if (final_cost <= min_cost) {
            min_semantic_cost = semantic_cost_vec_[i];
            //min_obstacle_cost = obstacle_cost_vec_[i];
            min_goal_cost = goal_cost_vec_[i];
            min_yawrate_cost = yawrate_cost_vec_[i];
            min_speed_cost = speed_cost_vec_[i];
            min_heading_cost = heading_cost_vec_[i];

            min_cost = final_cost;

            best_traj_ = trave_trajectories_[i];
            best_cmu_traj_ = cmu_trajectories_[i];
        }
    }

#if DEBUG_INFO
    std::cout << std::endl;
    std::cout << "min semantic cost: " << min_semantic_cost << std::endl;
    std::cout << "min goal cost: " << min_goal_cost << std::endl;
    std::cout << "min yawrate cost: " << min_yawrate_cost << std::endl;
    //std::cout << "min obstacle cost: " << min_obstacle_cost << std::endl;
    std::cout << "min speed cost: " << min_speed_cost << std::endl;
    std::cout << "min heading cost: " << min_heading_cost << std::endl;

    std::cout << "min cost: " << min_cost << std::endl;
    std::cout << std::endl;

    std::cout << "trave trajectories size: " << trave_trajectories_.size() << std::endl;
    std::cout << "cmu trajectories size: " << cmu_trajectories_.size() << std::endl;

    if (trave_trajectories_.size() != 0) {
        std::cout << "trave trajectorie[0] size: " << trave_trajectories_[0].size() << std::endl;
        std::cout << "trave trajectorie[1] size: " << trave_trajectories_[1].size() << std::endl;

        std::cout << "cmu trajectorie[0] size: " << cmu_trajectories_[0].size() << std::endl;
        std::cout << "cmu trajectorie[1] size: " << cmu_trajectories_[1].size() << std::endl;
    }

    std::cout << std::endl;
#endif

    if (min_cost == 1e6) {
        //std::cout << "min_cost to big: " << min_cost << ", pub cmd_vel 0." << std::endl;
        //std::cout << std::endl;

        std::vector<State> traj;

        State state(0.0, 0.0, 0.0, 0.0, 0.0);
        traj.push_back(state);

        best_traj_ = traj;
        best_cmu_traj_ = traj;
    }
}

double SemDWAPlanner::ToDegree(double rad)
{
    return rad / M_PI * 180;
}