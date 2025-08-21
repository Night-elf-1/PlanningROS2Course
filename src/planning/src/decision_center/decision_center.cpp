#include "decision_center.h"

namespace Planning
{
    DecisionCenter::DecisionCenter()
    {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "decision center created");

        decision_config_ = std::make_unique<ConfigReader>();
        decision_config_->read_decision_config();
    }

    void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleBase> &car, const std::vector<std::shared_ptr<VehicleBase>> &obses)
    {
        // 判断传入的障碍物是否为空
        if (obses.empty())
        {
            return;
        }

        // 初始化
        sl_points_.clear();                                                                          // 清空
        const double left_bound_l = decision_config_->pnc_map().road_half_width_ * 1.5;              // 道路左边界
        const double right_bound_l = decision_config_->pnc_map().road_half_width_ / 2.0;             // 道路右边界
        const double dis_time = static_cast<double>(decision_config_->local_path().path_size_ - 50); // 开始考虑障碍物的范围
        const double least_length = std::max(car->get_ds_dt() * dis_time, 30.0);                     // 最小变道距离
        const double referline_end_length = decision_config_->refer_line().front_size_ *
                                            decision_config_->pnc_map().segment_len_; // 参考线前段的长度的最大值
        SLPoint p;                                                                    // 实例化一个结构体变量

        // 针对每个障碍物生成变道点位
        for (const auto &obs : obses)
        {
            const double obs_dis_s = obs->get_s() - car->get_s();
            if (obs_dis_s > referline_end_length || obs_dis_s < -referline_end_length) // 如果障碍物在参考线末端的前面 即使接近地图的终点，参考线变短了，也要考虑这个最长距离，防止撞上
            {
                continue; // 足够远就忽略这个障碍物
            }
            if (obs->get_l() > right_bound_l && obs->get_l() < left_bound_l &&                   // 判断障碍物是否在车道中间
                fabs(obs->get_ds_dt()) < min_speed && obs->get_ds_dt() < car->get_ds_dt() / 2.0) // 判断侧向速度是否为0 纵向速度慢
            {
                p.s_ = obs->get_s() + obs->get_ds_dt() * obs_dis_s / (car->get_ds_dt() - obs->get_ds_dt()); // p为绕障的点位
                const double obs_left_bound_l = obs->get_l() + obs->get_width() / 2.0;                      // 障碍物左边界
                const double obs_right_bound_l = obs->get_l() - obs->get_width() / 2.0;                     // 障碍物右边界
                const double left_width = left_bound_l - obs_left_bound_l;                                  // 左边宽度
                const double right_width = obs_right_bound_l - right_bound_l;                               // 右边宽度

                if (left_width > car->get_width() + decision_config_->decision().safe_dis_l_ * 2.0) // 判断左边宽度是否够通过
                {
                    p.l_ = (left_bound_l + obs_left_bound_l) / 2.0;
                    p.type_ = static_cast<int>(SLPointType::LEFT_PASS);
                    sl_points_.emplace_back(p);
                }
                else // 如果左边宽度不够
                {
                    // 如果右边宽度够
                    if (right_width > car->get_width() + decision_config_->decision().safe_dis_l_ * 2.0)
                    {
                        p.l_ = (right_bound_l + obs_right_bound_l) / 2.0;
                        p.type_ = static_cast<int>(SLPointType::RIGHT_PASS);
                        sl_points_.emplace_back(p);
                    }
                    else // 如果两边宽度都不够
                    {
                        p.l_ = 0.0;
                        p.s_ = obs->get_s() - decision_config_->decision().safe_dis_s_; // 停到障碍物前方一定距离
                        p.type_ = static_cast<int>(SLPointType::STOP);
                        sl_points_.emplace_back(p);
                        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "-----------------stop obs, p:(s=%.2f, l=%.2f)", p.s_, p.l_);
                        break; // 更前方的障碍物无需考虑了
                    }
                }
            }
        }

        if (sl_points_.empty())
        {
            return;
        }

        // 头尾处理
        SLPoint p_start;                              // 整个过程的起点
        p_start.s_ = sl_points_[0].s_ - least_length; // sl_points_[0]离你最近的障碍物
        p_start.l_ = 0.0;
        p_start.type_ = static_cast<int>(SLPointType::START);
        sl_points_.emplace(sl_points_.begin(), p_start); // 从容器的头部插入

        if (sl_points_.back().type_ != static_cast<int>(SLPointType::STOP))
        {
            SLPoint p_end; // 整个过程的终点
            p_end.s_ = sl_points_.back().s_ + least_length;
            p_end.l_ = 0.0; // 到终点后横向距离为0，代表是沿着参考线走了
            p_end.type_ = static_cast<int>(SLPointType::END);
            sl_points_.emplace_back(p_end); // 尾差
        }
        
        //
    }

} // namespace Planning