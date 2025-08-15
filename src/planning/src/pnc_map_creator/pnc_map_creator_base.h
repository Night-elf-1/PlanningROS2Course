#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "base_msgs/msg/pnc_map.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace Planning
{

    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    enum class PNCMapType // 地图类型
    {
        STRAIGHT,
        STURN,
    };

    class PNCMapCreatorBase // pnc_map 创建器基类
    {
    public:
        virtual PNCMap creat_pnc_map() = 0;                                             // 生成地图
        inline PNCMap pnc_map() const { return pnc_map_; }                              // 获取地图
        inline MarkerArray pnc_map_markerarray() const { return pnc_map_markerarray_; } // 获取rviz地图
        virtual ~PNCMapCreatorBase() {}

    protected:                                         // 保护类型 子类中可以继承并且访问 但对象中不可访问
        std::unique_ptr<ConfigReader> pnc_map_config_; // ConfigReader地图配置文件类
        int map_type_ = 0;
        PNCMap pnc_map_;                  // 返回值 地图
        MarkerArray pnc_map_markerarray_; // 返回值 发给rviz的变量

        Point P_mid_, pl_, pr_;   // 车道的点 中心点 左点 右点
        double theta_current_;    // 车当前的航向角
        double len_step_ = 0.0;   // 步长
        double theta_step_ = 0.0; // 角度步长 分辨率
    };

} // namespace Planning

#endif // PNC_MAP_CREATOR_BASE_H_