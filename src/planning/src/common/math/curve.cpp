#include "curve.h"

namespace Planning
{
    int Curve::find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point)
    {
        const int path_size = path.poses.size(); // 获取全局路径的点个数
        if (path_size <= 1)
        {
            /* code */
            return path_size - 1; // 返回当前点的-1
        }

        // 设置一个最小距离值
        double min_dis = std::numeric_limits<double>::max(); // 先取到一个极大值
        int closet_index = -1;                               // 初始化一个最近点的索引
        for (int i = 0; i < path_size; i++)
        {
            /* code */
            double dis = std::hypot(path.poses[i].pose.position.x - target_point.pose.position.x,
                                    path.poses[i].pose.position.y - target_point.pose.position.y);
            if (dis < min_dis)
            {
                // 做匹配点的重叠判断
                if (std::abs(last_match_point_index - i) > 100)
                {
                    continue;
                }

                min_dis = dis;
                closet_index = i;
            }
        }
        return closet_index;
    }

    void Curve::cal_projection_param(Referline &refer_line)
    {
        // 判断参考点的个数是否够
        const int path_size = refer_line.refer_line.size();
        if (path_size < 3)
        {
            RCLCPP_ERROR(rclcpp::get_logger("math"), "refer_line too short");
            return;
        }

        // 计算rs
        double rs = 0.0;
        for (int i = 0; i < path_size; i++)
        {
            if (i == 0)
            {
                rs = 0.0;
            }
            else
            {
                rs += std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                 refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
            }
            // 对参考线上每个点的rs进行赋值
            refer_line.refer_line[i].rs = rs;
        }

        // 投影点的航向角
        for (int i = 0; i < path_size; i++)
        {
            // 进行判断是否还没有到最后一个点
            if (i < path_size - 1)
            {
                refer_line.refer_line[i].rtheta = std::atan2(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                                             refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
            }
            else
            {
                refer_line.refer_line[i].rtheta = std::atan2(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                                             refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
            }
        }

        // 投影点的曲率
        for (int i = 0; i < path_size; i++)
        {
            if (i < path_size - 1) // 判断是否到达最后一个点
            {
                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                              refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                if (dis <= kMathEpsilon)
                {
                    refer_line.refer_line[i].rkappa = 0.0; // 当距离极小时，曲率默认等于0
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i + 1].rtheta - refer_line.refer_line[i].rtheta) / dis; // 曲率计算公式 error(rtheta) / dis
                }
            }
            else
            {
                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                if (dis <= kMathEpsilon)
                {
                    refer_line.refer_line[i].rkappa = 0.0; // 当距离极小时，曲率默认等于0
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i].rtheta - refer_line.refer_line[i - 1].rtheta) / dis; // 曲率计算公式 error(rtheta) / dis
                }
            }
        }

        // 投影点的曲率变化率
        for (int i = 0; i < path_size; i++)
        {
            if (i < path_size - 1) // 判断是否到达最后一个点
            {
                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                              refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                if (dis <= kMathEpsilon)
                {
                    refer_line.refer_line[i].rdkappa = 0.0; // 当距离极小时，曲率默认等于0
                }
                else
                {
                    refer_line.refer_line[i].rdkappa = (refer_line.refer_line[i + 1].rkappa - refer_line.refer_line[i].rkappa) / dis; // 曲率计算公式 error(rtheta) / dis
                }
            }
            else
            {
                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                if (dis <= kMathEpsilon)
                {
                    refer_line.refer_line[i].rdkappa = 0.0; // 当距离极小时，曲率默认等于0
                }
                else
                {
                    refer_line.refer_line[i].rdkappa = (refer_line.refer_line[i].rkappa - refer_line.refer_line[i - 1].rkappa) / dis; // 曲率计算公式 error(rtheta) / dis
                }
            }
        }
    }

}