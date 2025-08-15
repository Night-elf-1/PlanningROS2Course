#include "pnc_map_creator_straight.h"

namespace Planning
{

    PNCMapCreatorStraight::PNCMapCreatorStraight()
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_creator created");
        pnc_map_config_ = std::make_unique<ConfigReader>();         // 创建配置文件，这里指向了配置文件类，然后在配置文件的构造函数里面会自动读取值
        // 让读取器pnc_map_config_（指针）去调用对应模块的读取函数
        pnc_map_config_->read_pnc_map_config();
        // 让基类中的成员变量进行赋值
        map_type_ = static_cast<int>(PNCMapType::STRAIGHT);
        // map_type_ = pnc_map_config_->pnc_map().type_;   // 这种方法我觉得也可行

        // 地图起点坐标 这里的坐标是以车辆为原点算出来的
        P_mid_.x = -3.0;    // 中点起点X坐标
        P_mid_.y = (pnc_map_config_->pnc_map().road_half_width_) / 2.0;

        // 长度步长
        len_step_ = pnc_map_config_->pnc_map().segment_len_;

        // 调用初始化init函数
        init_pnc_map();
    }

    PNCMap PNCMapCreatorStraight::creat_pnc_map() // 生成地图
    {
        draw_straight_x(pnc_map_.road_length, 1.0);
        // 还需要保证中心点的个数为偶数，不然rviz不能显示
        if (pnc_map_.midline.points.size() % 2 == 1)
        {
            /* code */
            pnc_map_.midline.points.pop_back(); // 弹出最后一个点
        }
        // 把所有marker放入pnc_map_markerarray中，因为在基类中定义了传给rviz的返回值数据类型
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.midline);    // 在pnc_map_markerarray_一共只有3个元素
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.left_boundary);
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.right_boundary);
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map created, midline points: %ld", pnc_map_.midline.points.size());

        return pnc_map_;
    }

    void PNCMapCreatorStraight::init_pnc_map()
    {
        // 读取地图坐标系名称
        pnc_map_.header.frame_id = pnc_map_config_->pnc_map().frame_;
        pnc_map_.header.stamp = rclcpp::Clock().now();  // 获取当前时间戳
        pnc_map_.road_length = pnc_map_config_->pnc_map().road_length_;
        pnc_map_.road_half_width = pnc_map_config_->pnc_map().road_half_width_;

        // 定义中心线格式
        pnc_map_.midline.header = pnc_map_.header;
        pnc_map_.midline.ns = "pnc_map";    // 获取命名空间
        pnc_map_.midline.id = 0;
        pnc_map_.midline.action = Marker::ADD;  // Marker::ADD意思为这条线的动作为新增 或 更新 到rviz中
        pnc_map_.midline.type = Marker::LINE_LIST;  // Marker::LINE_LIST为分段线条 虚线
        pnc_map_.midline.scale.x = 0.05;    // 线段宽度
        pnc_map_.midline.color.a = 1.0;     // 不透明度
        pnc_map_.midline.color.r = 0.7;
        pnc_map_.midline.color.g = 0.8;
        pnc_map_.midline.color.b = 0.0;
        pnc_map_.midline.lifetime = rclcpp::Duration::max();    // 生命周期
        pnc_map_.midline.frame_locked = true;

        // 左边界格式
        pnc_map_.left_boundary = pnc_map_.midline;  // 先将中心线的格式复制给左边界 然后再对左边界的一些参数进行修改
        pnc_map_.left_boundary.id = 1;
        pnc_map_.left_boundary.type = Marker::LINE_STRIP;   // 实线
        pnc_map_.left_boundary.color.r = 1.0;
        pnc_map_.left_boundary.color.g = 1.0;
        pnc_map_.left_boundary.color.b = 1.0;

        // 右边界格式
        pnc_map_.right_boundary = pnc_map_.left_boundary;
        pnc_map_.right_boundary.id = 2;
    }

    void PNCMapCreatorStraight::draw_straight_x(const double &length, const double &plus_flag,
                                                const double &ratio)
    {
        double len_tmp = 0.0;   // 临时长度
        while (len_tmp < length)
        {
            /* code */
            pl_.x = P_mid_.x;
            pl_.y = P_mid_.y + pnc_map_config_->pnc_map().road_half_width_;

            pr_.x = P_mid_.x;
            pr_.y = P_mid_.y - pnc_map_config_->pnc_map().road_half_width_;

            // 存储起来点集合
            pnc_map_.midline.points.emplace_back(P_mid_);
            pnc_map_.left_boundary.points.emplace_back(pl_);
            pnc_map_.right_boundary.points.emplace_back(pr_);

            len_tmp += len_step_ * ratio;       // 累加长度s
            // 迭代 让中心点前进
            P_mid_.x += len_step_ * ratio * plus_flag;
        }
        
    }

} // namespace Planning