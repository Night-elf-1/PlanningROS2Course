#ifndef CURVE_H_
#define CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/local_path.hpp"
#include <cmath>

namespace Planning
{

    using nav_msgs::msg::Path;
    using geometry_msgs::msg::PoseStamped;
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::Referline;
    constexpr double kMathEpsilon = 1.0e-6;         // 用极小数来代表0

    class Curve
    {
    public:
        Curve() = default; // 使用默认构造函数
        // 找到匹配点下标   static的作用是不用实例化对象curve类，可以直接用curve:: 的作用域来调用
        static int find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point); // target_point为车辆当前定位点
    
        // 计算投影点参数   对参考线上的每个点进行计算
        static void cal_projection_param(Referline &refer_line);

    };

} // namespace Planning

#endif // !CURVE_H_