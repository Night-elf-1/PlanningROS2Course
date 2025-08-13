#include "planning_process.h"

namespace Planning
{
    PlanningProcess::PlanningProcess():Node("planning_process")    // 规划总流程
    {
        RCLCPP_INFO(this->get_logger(), "planning_process_created");
        // 配置文件读取测试
        process_config_ = std::make_unique<ConfigReader>(); // 创建一个新的 ConfigReader 对象，并立即让 process_config_ 这个智能指针接管它的所有权
        process_config_->read_planning_process_config();
        auto obs_dis = process_config_->process().obs_dis_;
        RCLCPP_INFO(this->get_logger(), "obs_dis = %.2f", obs_dis);
    }

    bool PlanningProcess::process() // 规划总流程
    {
        return true;
    }
}