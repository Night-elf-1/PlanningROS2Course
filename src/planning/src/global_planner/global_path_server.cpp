#include "global_path_server.h"

namespace Planning
{

    GlobalPathServer::GlobalPathServer():Node("global_path_server_node")        // 全局路径服务器
    {
        RCLCPP_INFO(this->get_logger(), "global path server node created");
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
