#include "pnc_map_server.h"

namespace Planning
{

    PNCMapServer::PNCMapServer() : Node("pnc_map_server_node")
    {
        RCLCPP_INFO(this->get_logger(), "pnc_map_server_node created");
        // 地图发布器
        map_pub_ = this->create_publisher<PNCMap>("pnc_map", 10); // 两个参数，第一个是发布的话题名称，第二个是消息队列缓冲长度
        map_rviz_pub_ = this->create_publisher<MarkerArray>("pnc_map_markerarray", 10);
        map_server_ = this->create_service<PNCMapService>("pnc_map_server", std::bind(&PNCMapServer::response_pnc_map_callback, this, _1, _2)); // 地图服务创建
    }

    void PNCMapServer::response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request,
                                                 const std::shared_ptr<PNCMapService::Response> response)
    {
        // 获取地图的类型 接受请求（多态）
        switch (request->map_type)
        {
        case static_cast<int>(PNCMapType::STRAIGHT):                  // static_cast<int>(PNCMapType::STRAIGHT) --> 0
            map_creator_ = std::make_shared<PNCMapCreatorStraight>(); // 创建直道
            break;
        case static_cast<int>(PNCMapType::STURN):                  // static_cast<int>(PNCMapType::STURN) --> 1
            map_creator_ = std::make_shared<PNCMapCreatorSTrun>(); // 创建弯道
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid map type!");
            return;
        }

        // 创建并相应地图
        const auto pnc_map = map_creator_->creat_pnc_map();
        response->pnc_map = pnc_map; // response->pnc_map为自定义数据类型下的格式

        // 发布地图 给planning_node用
        map_pub_->publish(pnc_map);
        RCLCPP_INFO(this->get_logger(), "pnc_map published");

        // 发布用于rviz显示的地图
        const auto pnc_map_markerarray = map_creator_->pnc_map_markerarray();
        map_rviz_pub_->publish(pnc_map_markerarray);
        RCLCPP_INFO(this->get_logger(), "pnc_map for rviz published");
    }

} // namespace Planning

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
