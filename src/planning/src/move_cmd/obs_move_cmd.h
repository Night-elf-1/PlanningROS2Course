#ifndef OBS_MOVE_CMD_H_
#define OBS_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "obs_car_info.h"

namespace Planning
{

class ObsMoveCmd : public rclcpp::Node      // 障碍物运动指令 需要编译成一个节点
{
public:
    ObsMoveCmd();
private:
};
}   // namespace Planning

#endif  // OBS_MOVE_CMD_H_