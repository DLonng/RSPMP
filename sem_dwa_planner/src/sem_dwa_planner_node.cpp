#include "sem_dwa_planner/sem_dwa_planner_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dwa_planner");
    sem_dwa_local_planner::SemDWAPlannerROS sem_planner_ros;
    sem_planner_ros.Process();
    return 0;
}