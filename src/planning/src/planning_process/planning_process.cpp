#include "planning_process.h"

namespace Planning
{
    PlanningProcess::PlanningProcess() : Node("planning_process") // 规划总流程
    {
        RCLCPP_INFO(this->get_logger(), "planning_process_created");
        // 配置文件读取测试
        process_config_ = std::make_unique<ConfigReader>(); // 创建一个新的 ConfigReader 对象，并立即让 process_config_ 这个智能指针接管它的所有权
        process_config_->read_planning_process_config();
        obs_dis_ = process_config_->process().obs_dis_;
        // 创建地图服务器和全局路径客户端
        map_client_ = this->create_client<PNCMapService>("pnc_map_server");
        global_path_client_ = this->create_client<GlobalPathService>("global_path_server");
    }

    bool PlanningProcess::process() // 规划总流程
    {
        // 阻塞1s 等待rviz和xacro启动
        rclcpp::Rate rate(1.0);
        rate.sleep();

        // planning init 初始化
        if (!planning_init())
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "planning init failed!");
            return false;
        }

        // 进入规划主流程

        return true;
    }

    bool PlanningProcess::planning_init()
    {
        // 生成车辆

        // 连接地图服务器
        if (!connect_server(map_client_))
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "Map Server connect failed!");
            return false;
        }

        // 获取地图
        if (!map_request())
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "Map Request and response failed!");
            return false;
        }

        // 连接全局路径服务器
        if (!connect_server(global_path_client_))
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "Global Path Server connect failed!");
            return false;
        }

        // 获取全局路径
        if (!global_path_request())
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "Global Path Request and response failed!");
            return false;
        }

        return true;
    }

    template <typename T>
    bool PlanningProcess::connect_server(const T &client)
    {
        return true;
    }

    bool PlanningProcess::map_request()
    {
        return false;
    }

    bool PlanningProcess::global_path_request()
    {
        return false;
    }

}