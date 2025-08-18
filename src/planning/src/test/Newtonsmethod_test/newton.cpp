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
        std::cout << "请选择1:求根 or 2:求极值" << std::endl;
        int choice;
        double x_next;
        std::cin >> choice;
        std::cout << "请选择初始位置x_:" << std::endl;
        std::cin >> x_;

        if (choice == 1)
        {
            // x_ = 8;
            while (count_ <= maxIt_)
            {
                x_next = x_ - ((x_ * x_ * x_ - 216) / (3 * x_ * x_));

                if (abs(x_next - x_) <= eps_)
                {
                    std::cout << "根的值为:" << x_next << std::endl;
                    break;
                }
                else
                {
                    count_ += 1;
                    x_ = x_next;
                    std::cout << "x_next = " << x_next << std::endl;
                }
            }
        }
        else
        {
            // x_ = 2;
            while (count_ <= maxIt_)
            {
                x_next = x_ - ((4 * x_ * x_ * x_ + 2) / (12 * x_ * x_));
                if (abs(x_next - x_) <= eps_)
                {
                    std::cout << "极值为:" << (x_next * x_next * x_next * x_next + 2 * x_next) << std::endl;
                    std::cout << "根为:" << x_next << std::endl;
                    break;
                }
                else
                {
                    count_ += 1;
                    x_ = x_next;
                }
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
