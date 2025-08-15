#ifndef GLOBAL_PLANNER_BASE_H_
#define GLOBAL_PLANNER_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace Planning
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    // 强枚举类型 全局规划类型
    enum class GlobalPlannerType
    {
        NORMAL
    };

    class GlobalPlannerBase // 全局路径规划的基类
    {

    public:
        virtual Path search_global_path(const PNCMap &pnc_map) = 0; // 寻找全局路径
        inline Path get_global_path() { return global_path_; }      // 获取全局路径
        virtual ~GlobalPlannerBase() {}                             // 虚析构函数这里一定要实现
    protected:
        std::unique_ptr<ConfigReader> global_planner_config_; // 配置
        int global_planner_type_ = 0;                         // 规划算法类型
        Path global_path_;                                    // 全局路径
    };

} // namespace Planning

#endif // !GLOBAL_PLANNER_BASE_H_