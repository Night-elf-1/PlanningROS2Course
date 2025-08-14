#include "global_planner_normal.h"

namespace Planning
{

GlobalPlannerNormal::GlobalPlannerNormal()
{
    RCLCPP_INFO_ONCE(rclcpp::get_logger("Global_path"), "global_planner_normal created");
}

Path GlobalPlannerNormal::search_global_path(const PNCMap &pnc_map)
{
    return global_path_;
}

}