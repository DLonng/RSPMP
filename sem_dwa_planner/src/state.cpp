#include "sem_dwa_planner/state.h"

using namespace sem_dwa_local_planner;

State::State(double x, double y, double yaw, double velocity, double yawrate)
    : x_(x)
    , y_(y)
    , yaw_(yaw)
    , velocity_(velocity)
    , yawrate_(yawrate)
{

}

State::~State()
{
    
}
