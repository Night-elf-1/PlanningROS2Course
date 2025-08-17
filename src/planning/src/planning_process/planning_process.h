#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp" // 这里的自定义消息格式可以去文件中查看
#include "base_msgs/srv/global_path_service.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h" // 静态转换广播
#include "tf2_ros/buffer.h"                       // 广播队列
#include "tf2_ros/transform_listener.h"           // 监听车辆坐标变换的位置
#include "tf2/LinearMath/Quaternion.h"

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
  // 使用using来简化代码
  using base_msgs::msg::PNCMap;
  using base_msgs::srv::GlobalPathService;
  using base_msgs::srv::PNCMapService;
  using geometry_msgs::msg::PoseStamped;
  using nav_msgs::msg::Path;
  using tf2_ros::Buffer;
  using tf2_ros::StaticTransformBroadcaster;
  using tf2_ros::TransformListener;
  using namespace std::chrono_literals;

  class PlanningProcess : public rclcpp::Node
  { // 规划总流程
  public:
    PlanningProcess(); // 构造函数
    // 创建一个bool函数用来判断任务的进程
    bool process(); // 规划总流程

  private:                // 私有成员函数
    bool planning_init(); // 在初始化中 请求地图和全局路径

    void vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle); // 生成车辆位置
    void get_location(const std::shared_ptr<VehicleBase> &vehicle);  // 监听定位点

    template <typename T>                 // 使用模板， T为选择的客户端类型
    bool connect_server(const T &client); // 连接服务器
    bool map_request();                   // 发送地图请求
    bool global_path_request();           // 发送全局路径请求
    void planning_callback();             // 总流程回调函数

  public:
    inline PNCMap get_pnc_map() const { return pnc_map_; }
    inline Path get_global_path() const { return global_path_; }

  private: // 成员变量 写在private里面的变量，外部要获取的话要在public中写一个获取函数，保护数据安全
    // 在当前类中声明了一个名为 process_config_ 的成员变量，它是一个 独占所有权的智能指针，指向一个 ConfigReader 对象，用于读取或管理配置
    std::unique_ptr<ConfigReader> process_config_;
    std::shared_ptr<VehicleBase> car_; // 主车

    // 坐标变换
    std::shared_ptr<StaticTransformBroadcaster> tf_broadcaster_; // 坐标广播器
    std::unique_ptr<Buffer> buffer_;                             // 缓存对象
    std::shared_ptr<TransformListener> tf_listener_;             // 监听坐标位置

    // 参考线
    std::shared_ptr<ReferenceLineCreator> refer_line_creator_; // 参考线创建器
    rclcpp::Publisher<Path>::SharedPtr refer_line_pub_;        // 参考线发布器
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_; // 用于控制规划模块的运行频率

    // 地图 获取作用
    PNCMap pnc_map_;
    // 全局路径 获取作用
    Path global_path_;
    // 地图请求客户端
    rclcpp::Client<PNCMapService>::SharedPtr map_client_;
    // 全局路径请求客户端
    rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_;

    double obs_dis_ = 0.0; // 考虑障碍物的距离
  };
} // namespace Planing
#endif // PLANNING_PROCESS_H_