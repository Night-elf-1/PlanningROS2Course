#include "pnc_map_creator_straight.h"

namespace Planning
{

    PNCMapCreatorStraight::PNCMapCreatorStraight()
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_creator created");
    }

    PNCMap PNCMapCreatorStraight::creat_pnc_map() // 生成地图
    {
        return pnc_map_;
    }

} // namespace Planning