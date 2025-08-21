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

    using base_msgs::msg::LocalPath;
    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;
    constexpr double kMathEpsilon = 1.0e-6; // 用极小数来代表0
    constexpr double delta_s_min = 1.0;

    class Curve
    {
    public:
        Curve() = default; // 使用默认构造函数

        // 角度约束
        static double NormalizeAngle(const double &angle); // 将角度约束到 [-pi, pi)

        // 笛卡尔转frenet
        static void cartesian_to_frenet(const double &x, const double &y, const double &theta,
                                        const double &speed, const double &a, const double &kappa,
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa,
                                        double &s, double &ds_dt, double &dds_dt,
                                        double &l, double &dl_ds, double &dl_dt,
                                        double &ddl_ds, double &ddl_dt);

        // frenet 转 笛卡尔
        static void frenet_to_cartesian(const double &s, const double &ds_dt, const double &dds_dt,
                                        const double &l, const double &dl_ds, const double &ddl_ds, // 输入1 frenet下的参数
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2 目标点在参考线的投影
                                        double &x, double &y, double &theta,
                                        double &speed, double &a, double &kappa); // 输出

        // 找到匹配点下标   static的作用是不用实例化对象curve类，可以直接用curve:: 的作用域来调用
        static int find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point); // target_point为车辆当前定位点  利用上一帧
        static int find_match_point(const Referline &path, const PoseStamped &target_point);

        // 找到投影点
        static void find_projection_point(const Referline &refer_line, const PoseStamped &target_point,   // 输入
                                          double &rs, double &rx, double &ry,
                                          double &rtheta, double &rkappa, double &rdkappa);     // 输出

        // 计算投影点参数   对参考线上的每个点进行计算
        static void cal_projection_param(Referline &refer_line);
    };

} // namespace Planning

#endif // !CURVE_H_