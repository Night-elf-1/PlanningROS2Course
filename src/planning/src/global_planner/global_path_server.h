#ifndef GLOBAL_PATH_SERVER_H_
#define GLOBAL_PATH_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "global_planner_normal.h"
#include "base_msgs/srv/global_path_service.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace Planning
{
    using base_msgs::srv::GlobalPathService;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::Marker;

    using std::placeholders::_1;
    using std::placeholders::_2;

    class GlobalPathServer : public rclcpp::Node
    {
    public:
        GlobalPathServer(); // 构造函数

    private: // 成员函数
        // 全局路径回调函数
        void response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                           const std::shared_ptr<GlobalPathService::Response> response);
        // 将生成path类型的路径转换成marker类型，供给rviz显示用
        Marker path2marker(const Path &path);

    private:                                                               // 成员变量
        std::shared_ptr<GlobalPlannerBase> global_planner_base_;           // 全局路径创建器
        rclcpp::Publisher<Path>::SharedPtr global_path_pub_;               // 全局路径发布器
        rclcpp::Publisher<Marker>::SharedPtr global_path_rviz_pub_;        // 给rviz的全局路径发布器
        rclcpp::Service<GlobalPathService>::SharedPtr global_path_server_; // 全局路径服务器
    };

} // namespace Planning

#endif // GLOBAL_PATH_SERVER_H_