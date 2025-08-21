#include "main_car_info.h"

namespace Planning
{
    MainCar::MainCar()
    {
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "main_car created");

        // 读取配置
        vehicle_config_ = std::make_unique<ConfigReader>(); // 初始化
        vehicle_config_->read_vehicles_config();

        // 更新基本属性
        child_frame_ = vehicle_config_->main_car().frame_;
        length_ = vehicle_config_->main_car().length_;
        width_ = vehicle_config_->main_car().width_;
        theta_ = vehicle_config_->main_car().pose_theta_;
        speed_ = vehicle_config_->main_car().speed_ori_; // // 初速度
        id_ = 0;

        // 初始化定位点
        tf2::Quaternion qtn;
        qtn.setRPY(0.0, 0.0, theta_); // 这里是四元数中的xyz轴旋转的角度，z轴传入读取到的车辆theta角
        loc_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
        loc_point_.header.stamp = rclcpp::Clock().now();
        loc_point_.pose.position.x = vehicle_config_->main_car().pose_x_;
        loc_point_.pose.position.y = vehicle_config_->main_car().pose_y_;
        loc_point_.pose.position.z = 0.0;
        loc_point_.pose.orientation.x = qtn.getX();
        loc_point_.pose.orientation.y = qtn.getY();
        loc_point_.pose.orientation.z = qtn.getZ();
        loc_point_.pose.orientation.w = qtn.getW();
    }

    void MainCar::vehicle_cartesian_to_frenet(const Referline &refer_line)
    {
        double rs, rx, ry, rtheta, rkappa, rdkappa;
        // 找到定位点在参考线上的投影点
        Curve::find_projection_point(refer_line, loc_point_, rs, rx, ry, rtheta, rkappa, rdkappa);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "main car projection_point: rs = %.2f, rx = %.2f, ry = %.2f, rtheta = %.2f, rkappa = %.2f, rdkappa = %.2f",
                    rs, rx, ry, rtheta, rkappa, rdkappa);
        // 计算定位点在frenet坐标下的参数
        Curve::cartesian_to_frenet(loc_point_.pose.position.x, loc_point_.pose.position.y,
                                   theta_, speed_, acceleration_, dkappa_,
                                   rs, rx, ry, rtheta, rkappa, rdkappa,
                                   s_, ds_dt_, dds_dt_, l_, dl_ds_, dl_dt_, ddl_ds_, ddl_dt_);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "main car cartesian_to_frenet: s = %.2f, ds_dt = %.2f, dds_dt = %.2f, l = %.2f, dl_ds = %.2f, dl_dt = %.2f, ddl_ds = %.2f, ddl_dt = %.2f",
                    s_, ds_dt_, dds_dt_, l_, dl_ds_, dl_dt_, ddl_ds_, ddl_dt_);
    }

} // namespace Planning