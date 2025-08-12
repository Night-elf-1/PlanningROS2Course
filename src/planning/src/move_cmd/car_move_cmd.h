#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "main_car_info.h"

namespace Planning
{

class CarMoveCmd : public rclcpp::Node      // 主车运动指令 需要编译成一个节点
{
public:
    CarMoveCmd();
private:
};
}   // namespace Planning
#endif  // CAR_MOVE_CMD_H_