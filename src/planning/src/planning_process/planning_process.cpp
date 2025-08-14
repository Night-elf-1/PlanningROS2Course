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
        // 判断客户端的类型
        std::string server_name;
        if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>) // 此用法为C++17时期引入的
        {
            /* code */
            server_name = "pnc_map";
        }
        else if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
        {
            server_name = "global_path";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "wrong client type!");
            return false;
        }

        // 等待服务器
        while (!client->wait_for_service(1s))
        {
            /* code */
            if (!rclcpp::ok()) // 使用ctrl+c操作，防止进入死循环
            {
                /* code */
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server", server_name.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "%s server not avaliable, waiting again...", server_name.c_str());
        }

        return true;
    }

    bool PlanningProcess::map_request() // 发送地图请求
    {
        RCLCPP_INFO(this->get_logger(), "Sending map request/");

        // 生成请求
        auto request = std::make_shared<PNCMapService::Request>(); // 指针指向 这里要与响应函数中写的参数一致
        request->map_type = process_config_->pnc_map().type_;      // 从配置文件中读取，写的是什么地图
        // 获取响应
        auto result_future = map_client_->async_send_request(request); // map_client_->async_send_request(request)发送请求
        // 判断响应（发送）是否成功
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            /* code */
            RCLCPP_INFO(this->get_logger(), "Map response success!");
            pnc_map_ = result_future.get()->pnc_map; // 获取响应中的地图
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Map response failed!");
            return false;
        }

        return false;
    }

    bool PlanningProcess::global_path_request()
    {
        RCLCPP_INFO(this->get_logger(), "Sending global path request/");

        // 生成请求
        auto request = std::make_shared<GlobalPathService::Request>();       // 指针指向 这里要与响应函数中写的参数一致
        request->pnc_map = pnc_map_;                                         // 这里的两个pnc_map是不一样的，= 后面的是map request中相应回来的地图
        request->global_planner_type = process_config_->global_path().type_; // 这里有两个request，对应msg自定义文件中的两个request

        // 获取响应
        auto result_future = global_path_client_->async_send_request(request);
        // 判断响应（发送）是否成功
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            /* code */
            RCLCPP_INFO(this->get_logger(), "Global Path response success!");
            global_path_ = result_future.get()->global_path; // 获取响应中的全局路径
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Global Path response failed!");
            return false;
        }

        return false;
    }

}