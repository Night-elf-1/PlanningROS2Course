#ifndef NEWTON_H_
#define NEWTON_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
    class NewtonMethod : public rclcpp::Node // 继承ros2的节点
    {
    public:
        NewtonMethod(); // 构造
        void solve_problem();
    private:
        double initial_iteration_point_ = 1;
        double eps_ = 1e-6;
        int maxIt_ = 500;
        int count_ = 0;
        double last_x_;
        double x_;
    };
}

#endif // !NEWTON_H_