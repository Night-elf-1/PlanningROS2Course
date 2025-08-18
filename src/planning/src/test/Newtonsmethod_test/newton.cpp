#include "newton.h"

namespace Planning
{
    NewtonMethod::NewtonMethod() : rclcpp::Node("newton_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "osqp_test_node created");
        // 调用test函数
        solve_problem();
    }

    void NewtonMethod::solve_problem()
    {
        double x_next;
        x_ = 8;
        while (count_ <= maxIt_)
        {
            x_next = x_ - ((x_*x_*x_ - 216) / (3*x_*x_));

            if (abs(x_next - x_) <= eps_)
            {
                std::cout << "根的值为:" << x_next << std::endl;
                break;
            }else
            {
                count_ += 1;
                x_ = x_next;
                std::cout << "x_next = " << x_next << std::endl;
            }
        }
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::NewtonMethod>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
