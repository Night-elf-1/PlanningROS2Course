#include "reference_line_creator.h"

namespace Planning
{
    ReferenceLineCreator::ReferenceLineCreator()
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_creator created");
        // 在构造函数中读取关于 参考线 配置文件
        reference_line_config_ = std::make_unique<ConfigReader>(); // 指针初始化
        reference_line_config_->read_reference_line_config();
        // 平滑器初始化
        refer_line_smoother_ = std::make_shared<ReferenceLineSmoother>(); // 指针初始化
    }

    Referline ReferenceLineCreator::creat_reference_line(const Path &global_path, const PoseStamped &target_point)
    {
        // 判断传入的全局路径是否为空
        if (global_path.poses.empty())
        {
            return refer_line_; // 如果为空，返回一个空的参考线
        }

        // 找到匹配点
        match_point_index_ = Curve::find_match_point(global_path, last_match_point_index_, target_point);
        last_match_point_index_ = match_point_index_; // 更新上一帧
        if (match_point_index_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "find match_point_index failed!");
            return refer_line_;
        }

        // 计算最前点和最后点在全局路径下的索引
        const int global_path_size = global_path.poses.size(); // 记录全局路径的点个数
        front_index_ = (global_path_size - 1 >= match_point_index_ + reference_line_config_->refer_line().front_size_) ? (match_point_index_ + reference_line_config_->refer_line().front_size_) : (global_path_size - 1);
        back_index_ = (0 <= match_point_index_ - reference_line_config_->refer_line().back_size_) ? (match_point_index_ - reference_line_config_->refer_line().back_size_) : (0);

        // 填充参考线容器
        refer_line_.header.frame_id = reference_line_config_->pnc_map().frame_; // 初始化坐标名称
        refer_line_.header.stamp = rclcpp::Clock().now();
        refer_line_.refer_line.clear(); // 这里每一帧都要进行清理，因为每一帧都要填充新的参考线
        ReferlinePoint point_tmp;       // 创建一个临时的point用于填充参考线容器
        for (int i = back_index_; i < front_index_; i++)
        {
            point_tmp.pose = global_path.poses[i];
            refer_line_.refer_line.emplace_back(point_tmp);
        }

        // 平滑整条参考线

        // 计算投影点参数
        Curve::cal_projection_param(refer_line_);
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference line created, mathc_point_index = %d, front_index = %d, back_index = %d, size = %ld",
                    match_point_index_, front_index_, back_index_, refer_line_.refer_line.size());

        return refer_line_;
    }

    Path ReferenceLineCreator::referline_to_rviz()
    {
        refer_line_rviz_.header = refer_line_.header; // 头部直接赋值
        refer_line_rviz_.poses.clear();

        PoseStamped point_tmp;
        for (const auto &point : refer_line_.refer_line)
        {
            point_tmp.header = refer_line_rviz_.header;
            point_tmp.pose = point.pose.pose;
            refer_line_rviz_.poses.emplace_back(point_tmp);
        }

        return refer_line_rviz_;
    }

} // namespace Planning