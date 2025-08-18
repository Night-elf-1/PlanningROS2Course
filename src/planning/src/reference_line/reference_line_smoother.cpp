#include "reference_line_smoother.h"

namespace Planning
{
    ReferenceLineSmoother::ReferenceLineSmoother()
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_smoother created");

        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config();
    }

    void ReferenceLineSmoother::smooth_reference_line(Referline &refer_line)
    {
        // 判断原始参考线
        const int n = refer_line.refer_line.size();
        if (n < 3)
        {
            return;
        }

        // 构造p矩阵
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity(); // 2*2 单位矩阵
        Eigen::Matrix2d W1 = 2.0 * w1 * I;
        Eigen::Matrix2d W2 = 2.0 * w2 * I;
        Eigen::Matrix2d W3 = 2.0 * w3 * I;

        Eigen::Matrix2d block1 = W1 + W2 + W3;
        Eigen::Matrix2d block2 = -2.0 * W1 - W2;
        Eigen::Matrix2d block3 = -4.0 * W1 - W2;
        Eigen::Matrix2d block4 = 5.0 * W1 + 2.0 * W2 + W3;
        Eigen::Matrix2d block5 = 6.0 * W1 + 2.0 * W2 + W3;

        Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(2 * n, 2 * n); // 构建p矩阵的维度 初始化为0

        // 判断n等于几
        if (n == 3)
        {
            // 只填充上三角部分
            P_tmp.block<2, 2>(0, 0) = block1; // 将block1填充到0，0位置，大小为2*2
            P_tmp.block<2, 2>(0, 2) = block2;
            P_tmp.block<2, 2>(0, 4) = W1;
            P_tmp.block<2, 2>(2, 2) = 4.0 * W1 + 2.0 * W2 + W3;
            P_tmp.block<2, 2>(2, 4) = block2;
            P_tmp.block<2, 2>(4, 4) = block1;
        }
        else
        {
            // 只填充上三角部分
            for (int i = 0; i < n; i++)
            {
                if (i == 0)
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block1;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
                    P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
                }
                else if (i == 1)
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block4;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
                    P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
                }
                else if (i == n - 2) // 倒数第二行
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block4;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
                }
                else if (i == n - 1)
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block1;
                }
                else // 中间有规律的其他行
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block5;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
                    P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
                }
            }
        }

        P_tmp = P_tmp.selfadjointView<Eigen::Upper>();     // 通过上三角矩阵构造对称矩阵
        Eigen::SparseMatrix<double> P = P_tmp.sparseView(); // 转换成稀疏矩阵

        Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Identity(2*n, 2*n);
        Eigen::SparseMatrix<double> A = A_tmp.sparseView(); // 转化成稀疏矩阵

        // 原始点的坐标
        Eigen::VectorXd X(2*n);
        for (int i = 0; i < n; i++)
        {
            X(i*2) = refer_line.refer_line[i].pose.pose.position.x;
            X(i*2 + 1) = refer_line.refer_line[i].pose.pose.position.y;
        }
        Eigen::VectorXd Q = -2.0 * X;
        Eigen::VectorXd buff = Eigen::VectorXd::Constant(2*n, 0.2); // 偏差范围， 动态列向量 2*n行 值全部为0.2
        buff(0) = buff(1) = buff(2*n - 2) = buff(2*n - 1) = 0.0;
        Eigen::VectorXd lowerBound = X - buff;              // 不等式约束的下边界
        Eigen::VectorXd upperBound = X + buff;              // 不等式约束的上边界

        OsqpEigen::Solver solver;

        solver.settings()->setVerbosity(false); // 设置求解器，提升求解速度
        solver.settings()->setWarmStart(true);

        solver.data()->setNumberOfVariables(2*n);   // 设置变量个数
        solver.data()->setNumberOfConstraints(2*n); // 设置约束数量
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
        
        // 把计算出的结果坐标替换
        for (int i = 0; i < n; i++)
        {
            refer_line.refer_line[i].pose.pose.position.x = QPSolution(i*2);
            refer_line.refer_line[i].pose.pose.position.y = QPSolution(i*2 + 1);
        }
        
    }
} // namespace Planning