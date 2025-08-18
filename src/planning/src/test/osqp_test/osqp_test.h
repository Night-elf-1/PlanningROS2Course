#ifndef TEST_H_
#define TEST_H_

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

namespace Planning
{

    class OsqpTest : public rclcpp::Node // 继承ros2的节点
    {
    public:
        OsqpTest(); // 构造
        void test_problem();
    };

} // namespace planning

#endif // !TEST_H_