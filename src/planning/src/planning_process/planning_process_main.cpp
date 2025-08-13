#include "planning_process.h"

int main(int argc, char const *argv[])
{
  /* code */
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("planning_process_main"), "planning start");

  auto node = make_shared<Planning::PlanningProcess>();
  // 进行判断 判断节点是否启动成功
  if (!node->process())
  {
    RCLCPP_ERROR(rclcpp::get_logger("planning_process_main"), "planning failed");
    rclcpp::shutdown();
    return 1;
  }
  

  rclcpp::spin(node);   // 阻塞节点
  rclcpp::shutdown();
  return 0;
}
