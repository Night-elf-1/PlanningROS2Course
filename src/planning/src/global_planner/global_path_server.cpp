#include "global_path_server.h"

namespace Planning
{

    GlobalPathServer::GlobalPathServer() : Node("global_path_server_node") // 全局路径服务器
    {
        RCLCPP_INFO(this->get_logger(), "global path server node created");
        // 完善构造函数
        // 创建两个全局路径发布器
        global_path_pub_ = this->create_publisher<Path>("global_path", 10);
        global_path_rviz_pub_ = this->create_publisher<Marker>("global_path_rviz", 10);
        // 创建全局路径服务器
        global_path_server_ = this->create_service<GlobalPathService>("global_path_server",
                                                                      std::bind(&GlobalPathServer::response_global_path_callback, this, _1, _2));
    }
    // 全局路径回调函数 GlobalPathService为消息类型
    void GlobalPathServer::response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                                         const std::shared_ptr<GlobalPathService::Response> response)
    {
        // 实现回调函数
        // 接受请求 用多态实现
        switch (request->global_planner_type) // request->global_planner_type去自定义的消息类型中查看
        {
        case static_cast<int>(GlobalPlannerType::NORMAL):
            global_planner_base_ = std::make_shared<GlobalPlannerNormal>(); // 父类指针，指向之类的对象
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid global planner type!");
            return;
        }
        // 判断请求是否为空值
        if (request->pnc_map.midline.points.empty())
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "pnc_map empty, global_path cannot be created!");
            return;
        }

        // 搜索并且相应全局路径
        const auto global_path = global_planner_base_->search_global_path(request->pnc_map);
        response->global_path = global_path;

        // 发布全局路径 给局部规划使用
        // 这里path 类型的只发布一次，而且path没有frame_locked，所以无法在rviz中固定住 需要path2marker函数进行转换
        global_path_pub_->publish(global_path); // 发布路径
        RCLCPP_INFO(this->get_logger(), "global_path published!");

        // 发布用于rviz显示的全局路径
        const auto global_path_rviz = path2marker(global_path);
        global_path_rviz_pub_->publish(global_path_rviz);
        RCLCPP_INFO(this->get_logger(), "global_path for rviz published!");
    }

    // 将生成path类型的路径转换成marker类型，供给rviz显示用
    Marker GlobalPathServer::path2marker(const Path &path)
    {
        Marker path_rviz_; // 用于返回给rviz的变量
        path_rviz_.header = path.header;
        path_rviz_.ns = "global_path"; // 命名空间
        path_rviz_.id = 0;
        path_rviz_.action = Marker::ADD;
        path_rviz_.type = Marker::LINE_STRIP; // 连续实现
        path_rviz_.scale.x = 0.05;            // 线段宽度
        path_rviz_.color.a = 1.0;             // 线段不透明度
        path_rviz_.color.r = 0.8;
        path_rviz_.color.g = 0.0;
        path_rviz_.color.b = 0.0;
        path_rviz_.lifetime = rclcpp::Duration::max(); // 生命周期
        path_rviz_.frame_locked = true;                // 锁定坐标系

        Point p_tmp;                        // 临时点
        for (const auto &pose : path.poses) // 循环path下的每一个pose
        {
            /* code */
            p_tmp.x = pose.pose.position.x; // pose.pose.position.x 这里是全局路径传进来的
            p_tmp.y = pose.pose.position.y;
            path_rviz_.points.emplace_back(p_tmp);
        }

        return path_rviz_;
    }

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::GlobalPathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
