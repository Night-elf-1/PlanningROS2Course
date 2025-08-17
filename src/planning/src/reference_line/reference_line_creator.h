#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline_point.hpp"
#include "base_msgs/msg/referline.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

#include "config_reader.h"
#include "curve.h"
#include "reference_line_smoother.h"

namespace Planning
{
    using base_msgs::msg::Referline;
    using base_msgs::msg::ReferlinePoint;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    class ReferenceLineCreator // 创建参考线
    {
    public:
        ReferenceLineCreator();
        // 创建参考线
        Referline creat_reference_line(const Path &global_path, const PoseStamped &target_point);   // target_point车辆定位点、

        Path referline_to_rviz();

    public: // 获取到成员变量的函数
        inline Referline get_reference_line() const {return refer_line_;}       // 通过public的函数获取到private的成员变量
        inline Path get_reference_line_rviz() const {return refer_line_rviz_;}
        inline int get_match_point_index() const {return match_point_index_;}
        inline int get_front_index() const {return front_index_;}
        inline int get_back_index() const {return back_index_;}

    private:                                                  // private成员变量
        std::unique_ptr<ConfigReader> reference_line_config_; // 创建一个参考线的配置指针变量
        Referline refer_line_;
        Path refer_line_rviz_;                                       // 给rviz提供的
        std::shared_ptr<ReferenceLineSmoother> refer_line_smoother_; // 创建参考线平滑指针

        int last_match_point_index_ = -1;
        int match_point_index_ = -1;
        int front_index_ = -1; // 截取到的参考线点的最前面的点
        int back_index_ = -1;  // 最后面的点
    };

} // namespace Planning

#endif // REFERENCE_LINE_CREATOR_H_