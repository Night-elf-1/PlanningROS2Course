#include "pnc_map_server.h"

namespace Planning
{

PNCMapServer::PNCMapServer() : Node("pnc_map_server_node")
{
    RCLCPP_INFO(this->get_logger(), "pnc_map_server_node created");
}

}   // namespace Planning

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
