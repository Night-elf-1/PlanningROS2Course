#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"

#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;

namespace Planning
{
class PlanningProcess : public rclcpp::Node{  // 规划总流程
public:
  PlanningProcess();    // 构造函数
  // 创建一个bool函数用来判断任务的进程
  bool process();       // 规划总流程
private:
  // 在当前类中声明了一个名为 process_config_ 的成员变量，它是一个 独占所有权的智能指针，指向一个 ConfigReader 对象，用于读取或管理配置
  std::unique_ptr<ConfigReader> process_config_;
};
} // namespace Planing
#endif // PLANNING_PROCESS_H_