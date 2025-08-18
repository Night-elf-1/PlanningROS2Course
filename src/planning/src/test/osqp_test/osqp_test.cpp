#include "osqp_test.h"

namespace Planning
{
    OsqpTest::OsqpTest() : Node("osqp_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "osqp_test_node created");
        // 调用test函数
        test_problem();
    }

    void OsqpTest::test_problem()
    {
        // 构建矩阵
        Eigen::SparseMatrix<double> P(2, 2); // 创建一个2*2的方阵
        Eigen::VectorXd Q(2);                // 创建一个列向量   动态矩阵
        Eigen::SparseMatrix<double> A(2, 2); // 单位矩阵
        Eigen::VectorXd lowerBound(2);       // 下边界 两行的列向量
        Eigen::VectorXd upperBound(2);       // 上边界

        P.insert(0, 0) = 2.0; // 0，0代表左上角的点
        P.insert(1, 1) = 2.0; // 右下角
        std::cout << "P = \n"
                  << P << std::endl;

        Q << -2, -2;
        std::cout << "Q = \n"
                  << Q << std::endl;

        A.insert(0, 0) = 1.0;
        A.insert(1, 1) = 1.0;
        std::cout << "A = \n"
                  << A << std::endl;

        lowerBound << 0.0, 0.0;
        upperBound << 1.5, 1.5;

        OsqpEigen::Solver solver;

        solver.settings()->setVerbosity(false); // 设置求解器，提升求解速度
        solver.settings()->setWarmStart(true);

        solver.data()->setNumberOfVariables(2);   // 设置变量个数
        solver.data()->setNumberOfConstraints(2); // 设置约束数量
        if (!solver.data()->setHessianMatrix(P))
        {
            /* code */
            return;
        }
        if (!solver.data()->setGradient(Q))
        {
            /* code */
            return;
        }
        if (!solver.data()->setLinearConstraintsMatrix(A))
        {
            /* code */
            return;
        }
        if (!solver.data()->setLowerBound(lowerBound))
        {
            /* code */
            return;
        }
        if (!solver.data()->setUpperBound(upperBound))
        {
            /* code */
            return;
        }

        if (!solver.initSolver())
        {
            return;
        }

        // 创建接受变量
        Eigen::VectorXd QPSolution;

        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            return;
        }

        QPSolution = solver.getSolution(); // 获取结果
        std::cout << "QPSolution = \n" << QPSolution << std::endl;
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::OsqpTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
