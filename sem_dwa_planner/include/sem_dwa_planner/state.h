#ifndef STATE_H_
#define STATE_H_

namespace sem_dwa_local_planner {
    class State {
        public:
            State(double x, double y, double yaw, double velocity, double yawrate);
            ~State();

            double x_; // robot position x
            double y_; // robot posiiton y
            double yaw_; // robot orientation yaw
            double velocity_; // robot linear velocity
            double yawrate_; // robot angular velocity
    };
}
#endif