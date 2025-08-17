#ifndef VEHICLE_INFO_BASE_H_
#define VEHICLE_INFO_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.hpp" // 四元数计算

#include "config_reader.h"
#include "curve.h"

namespace Planning
{

    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using geometry_msgs::msg::TransformStamped;
    using nav_msgs::msg::Path;

    class VehicleBase // 车辆基类
    {
    public:
        // 更新参数
        inline void update_location(const PoseStamped &loc) { loc_point_ = loc; }
        // 更新笛卡尔参数

        // 笛卡尔转frenet

        // 获取成员变量返回值
        // 基本属性
        inline std::string get_child_frame() const { return child_frame_; }
        inline double get_length() const { return length_; }
        inline double get_width() const { return width_; }
        inline int get_id() const { return id_; }

        // 笛卡尔参数
        inline PoseStamped get_loc_point() const { return loc_point_; }
        inline double get_theta() const { return theta_; }
        inline double get_kappa() const { return kappa_; }
        inline double get_dkappa() const { return dkappa_; }
        inline double get_speed() const { return speed_; }
        inline double get_acceleration() const { return acceleration_; }
        inline double get_deacceleration() const { return deacceleration_; }

        // 向参考线投影的frenet参数

        // 向路径投影的frenet参数

        // 时间参数

        // 虚析构函数
        virtual ~VehicleBase() {}

    protected: // protected下面可以让子类中使用，但是子类的实例却无法调用
        // 基本属性
        std::unique_ptr<ConfigReader> vehicle_config_; // 车辆配置指针
        std::string child_frame_;                      // 坐标名称
        double length_ = 0.0;                          // 车长
        double width_ = 0.0;                           // 车宽
        int id_ = 0;                                   // 车辆序号

        // 笛卡尔参数
        PoseStamped loc_point_;       // 车辆定位坐标
        double theta_ = 0.0;          // 航向角
        double kappa_ = 0.0;          // 曲率
        double dkappa_ = 0.0;         // 曲率变化率
        double speed_ = 0.0;          // 速度
        double acceleration_ = 0.0;   // 加速度
        double deacceleration_ = 0.0; // 加加速度

        // 向参考线投影的frenet参数

        // 向路径投影的frenet参数

        // 时间参数
    };

} // namespace Planning

#endif // VEHICLE_INFO_BASE_H_