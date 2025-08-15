#include "global_planner_normal.h"

namespace Planning
{

    GlobalPlannerNormal::GlobalPlannerNormal()
    {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("Global_path"), "global_planner_normal created");

        global_planner_config_ = std::make_unique<ConfigReader>();
        global_planner_config_->read_global_path_config();                  // 读取全局路径参数
        global_planner_type_ = static_cast<int>(GlobalPlannerType::NORMAL); // 将强枚举类型转换成int类型 对应0
    }

    Path GlobalPlannerNormal::search_global_path(const PNCMap &pnc_map)
    {
        RCLCPP_INFO(rclcpp::get_logger("global_path"), "using normal global_planner");

        global_path_.header.frame_id = pnc_map.header.frame_id;
        global_path_.header.stamp = rclcpp::Clock().now(); // 获取当前时间戳
        global_path_.poses.clear();                        // 清空

        // 创建一个临时点的类型
        PoseStamped p_tmp;
        p_tmp.header = global_path_.header;
        p_tmp.pose.orientation.x = 0.0;
        p_tmp.pose.orientation.y = 0.0;
        p_tmp.pose.orientation.z = 0.0;
        p_tmp.pose.orientation.w = 1.0;

        const int midline_size = pnc_map.midline.points.size(); // 先获取中心线点的数量 这里的pncmap是之前创建的
        for (int i = 0; i < midline_size; i++)
        {
            /* code */
            // 以每个中心线的点为基准去计算全局路径 (用中心线点和左右边点取中间)
            p_tmp.pose.position.x = (pnc_map.midline.points[i].x + pnc_map.right_boundary.points[i].x) / 2.0;
            p_tmp.pose.position.y = (pnc_map.midline.points[i].y + pnc_map.right_boundary.points[i].y) / 2.0;
            // 填充
            global_path_.poses.emplace_back(p_tmp);
        }
        RCLCPP_INFO(rclcpp::get_logger("global_path"), "global_path created, points size: %ld", global_path_.poses.size());

        return global_path_;
    }

}