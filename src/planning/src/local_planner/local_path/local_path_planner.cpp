#include "local_path_planner.h"

namespace Planning
{

    LocalPathPlanner::LocalPathPlanner() // 局部路径规划器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local_path_planner created");
        // 读取配置文件
        local_path_config_ = std::make_unique<ConfigReader>(); // 初始化指针
        local_path_config_->read_local_path_config();
        local_path_smoother_ = std::make_shared<LocalPathSmoother>(); // 初始化平滑器指针
    }

    void LocalPathPlanner::init_local_path(){
        local_path_.header.frame_id = local_path_config_->pnc_map().frame_;          // 使用地图的坐标
        local_path_.header.stamp = rclcpp::Clock().now();
        local_path_.local_path.clear();
    }

    LocalPath LocalPathPlanner::creat_local_path(const Referline &reference_line,
                                                 const std::shared_ptr<VehicleBase> &car,
                                                 const std::shared_ptr<DecisionCenter> &decision)
    {
        // 初始化localpath
        init_local_path();

        // 计算路径点的sl值
        double point_s = car->get_s();      // 临时点的s，记录为车辆当前的s
        LocalPathPoint point_tmp;
        for (int i = 0; i < local_path_config_->local_path().path_size_; i++)
        {
            // 规划的起点
            point_s += car->get_ds_dt();        // 规划的起点
            if (point_s > reference_line.refer_line.back().rs)
            {
                break;
            }
            
            // 给point_tmp赋值
            point_tmp.s = point_s;
            point_tmp.ds_dt = car->get_ds_dt();
            point_tmp.dds_dt = car->get_dds_dt();
            point_tmp.l = 0.0;
            point_tmp.dl_ds = 0.0;
            point_tmp.ddl_ds = 0.0;

            // 计算point_tmp的l和dl_ds
            const int sl_point_size = decision->get_sl_point().size();
            for (int j = 0; j < sl_point_size - 1; j++)
            {
                // 确定每个分段起始状态和末状态
                const double start_s = decision->get_sl_point()[j].s_;
                const double start_l = decision->get_sl_point()[j].l_;
                const double start_dl_ds = 0.0;
                const double start_ddl_ds = 0.0;
                const double end_s = decision->get_sl_point()[j + 1].s_;
                const double end_l = decision->get_sl_point()[j + 1].l_;
                const double end_dl_ds = 0.0;
                const double end_ddl_ds = 0.0;

                // 如果临时点的s在分段范围内
                if (point_s >= start_s && point_s <= end_s)
                {
                    
                }
                
            }
            
        }
        

        // sl坐标下的平滑

        // 转笛卡尔

        // 计算投影点参数


        return local_path_;
    }

    Path LocalPathPlanner::path_to_rviz()
    {
        local_path_rviz_.header = local_path_.header;
        local_path_rviz_.poses.clear();

        PoseStamped point_tmp;
        point_tmp.header = local_path_rviz_.header;
        for (const auto &point : local_path_.local_path)        // 这里local_path_可以像遍历容器一样遍历 是因为在创建消息格式的时候，把local_path创建成了数组形式
        {
            point_tmp.pose = point.pose.pose;
            local_path_rviz_.poses.emplace_back(point_tmp);
        }
        
        return local_path_rviz_;
    }

} // namespace Planning