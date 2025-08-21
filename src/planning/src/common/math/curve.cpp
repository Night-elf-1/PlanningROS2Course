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

    double Curve::NormalizeAngle(const double &angle)
    {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0)
        {
            a += (2.0 * M_PI);
        }

        return a - M_PI;
    }

    // 笛卡尔转frenet
    void Curve::cartesian_to_frenet(const double &x, const double &y, const double &theta,
                                    const double &speed, const double &a, const double &kappa,
                                    const double &rs, const double &rx, const double &ry,
                                    const double &rtheta, const double &rkappa, const double &rdkappa,
                                    double &s, double &ds_dt, double &dds_dt,
                                    double &l, double &dl_ds, double &dl_dt,
                                    double &ddl_ds, double &ddl_dt)
    {
        // 计算s
        s = rs;

        // 计算l
        const double dx = x - rx;
        const double dy = y - ry;

        const double cos_theta_r = std::cos(rtheta);
        const double sin_theta_r = std::sin(rtheta);

        const double cross_r_x = cos_theta_r * dy - sin_theta_r * dx; // ed
        l = std::copysign(std::hypot(dx, dy), cross_r_x);

        // 计算l'
        const double delta_theta = theta - rtheta;
        const double tan_delta_theta = std::tan(delta_theta);
        const double cos_delta_theta = std::cos(delta_theta);
        const double sin_delta_theta = std::sin(delta_theta);
        const double one_minus_kappa_l = 1 - (rkappa * l);
        dl_ds = one_minus_kappa_l * tan_delta_theta;

        // 计算l''
        const double kappa_l_prime = rdkappa * l + rkappa * dl_ds;
        const double delta_theta_prime = (one_minus_kappa_l / cos_delta_theta) * kappa - rkappa;
        ddl_ds = -kappa_l_prime * tan_delta_theta + (one_minus_kappa_l / (cos_delta_theta * cos_delta_theta)) * delta_theta_prime;

        // 计算ds/dt
        ds_dt = (speed * cos_delta_theta) / one_minus_kappa_l;

        // 计算d(ds)/dt
        dds_dt = ((a * cos_delta_theta) - (ds_dt * ds_dt) * (dl_ds * delta_theta_prime - kappa_l_prime)) / one_minus_kappa_l;

        // 计算dl_dt
        dl_dt = speed * sin_delta_theta;

        // 计算ddl_dt
        ddl_dt = a * sin_delta_theta;
    }

    // frenet 转 笛卡尔
    void Curve::frenet_to_cartesian(const double &s, const double &ds_dt, const double &dds_dt,
                                    const double &l, const double &dl_ds, const double &ddl_ds, // 输入1 frenet下的参数
                                    const double &rs, const double &rx, const double &ry,
                                    const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2 目标点在参考线的投影
                                    double &x, double &y, double &theta,
                                    double &speed, double &a, double &kappa)
    {
        // 判断 s 和 rs 是否足够近
        if (std::fabs(rs - s) > delta_s_min)
        {
            RCLCPP_INFO(rclcpp::get_logger("math"), "reference point s and projection rs don't match! rs = %.2f, s = %.2f", rs, s);
            return;
        }

        // 计算 x y
        const double cos_theta_r = std::cos(rtheta);
        const double sin_theta_r = std::sin(rtheta);
        x = rx - l * sin_theta_r;
        y = ry + l * cos_theta_r;

        // theta
        const double one_minus_kappa_l = 1 - (rkappa * l);
        const double tan_delta_theta = dl_ds / one_minus_kappa_l;
        const double delta_theta = std::atan2(dl_ds, one_minus_kappa_l); // delta_theta = theta_x - theta_r <--> theta_x = arctan(d'/(1-k_r*d)) + theta_r
        const double cos_delta_theta = std::cos(delta_theta);
        theta = NormalizeAngle(delta_theta + rtheta);

        // kappa
        const double kappa_l_prime = rdkappa * l + rkappa * dl_ds;
        kappa = ((ddl_ds + kappa_l_prime * tan_delta_theta) * ((cos_delta_theta * cos_delta_theta) / one_minus_kappa_l) + rkappa) * (cos_delta_theta / one_minus_kappa_l);

        // speed
        speed = std::hypot(ds_dt * one_minus_kappa_l, ds_dt * dl_ds);

        // a
        const double delta_theta_prime = one_minus_kappa_l / cos_delta_theta * kappa - rkappa;
        a = dds_dt * one_minus_kappa_l / cos_delta_theta + (ds_dt * ds_dt) / cos_delta_theta * (dl_ds * delta_theta_prime - kappa_l_prime);
    }

    // 找到投影点
    void Curve::find_projection_point(const Referline &refer_line, const PoseStamped &target_point, // 输入
                                      double &rs, double &rx, double &ry,
                                      double &rtheta, double &rkappa, double &rdkappa)
    {
        // 简化 用匹配点近似替代投影点  要求参考线足够密集
        const int match_index = find_match_point(refer_line, target_point);
        if (match_index < 0)
        {
            return;
        }
        rx = refer_line.refer_line[match_index].pose.pose.position.x;
        ry = refer_line.refer_line[match_index].pose.pose.position.y;
        rs = refer_line.refer_line[match_index].rs;
        rtheta = refer_line.refer_line[match_index].rtheta;
        rkappa = refer_line.refer_line[match_index].rkappa;
        rdkappa = refer_line.refer_line[match_index].rdkappa;
    }

    int Curve::find_match_point(const Referline &refer_line, const PoseStamped &target_point)
    {
        const int path_size = refer_line.refer_line.size(); // 判断路径是否为空
        if (path_size <= 1)
        {
            return path_size - 1;
        }

        double min_dis = std::numeric_limits<double>::max(); // 创建一个极大数
        int closest_index = -1;
        for (int i = 0; i < path_size; i++)
        {
            double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.x - target_point.pose.position.x,
                                    refer_line.refer_line[i].pose.pose.position.y - target_point.pose.position.y);
            if (dis < min_dis)
            {
                min_dis = dis;
                closest_index = i;
            }
        }
        return closest_index;
    }

} // namespace Planning