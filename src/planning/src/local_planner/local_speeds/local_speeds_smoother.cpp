#include "local_speeds_smoother.h"

namespace Planning
{

LocalSpeedsSmoother::LocalSpeedsSmoother()
{
    RCLCPP_INFO(rclcpp::get_logger("local_speed"), "local_speeds_smoother created");
}

}