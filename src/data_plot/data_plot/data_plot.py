import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

# 创建一个画图类
class PlotData(Node):
    def __init__(self):
        super().__init__("data_plot_node")
        self.get_logger().info("data_plot_node created")

def main():
    print('Hi from data_plot.')
    rclpy.init() # 对节点的初始化
    plot_node = PlotData() # 创建类变量，初始化类

    # 创建try语句，阻塞画图节点或者中断不成功
    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        rclpy.shutdown()    # 防止按ctrl+c无法中断程序

        
if __name__ == '__main__':
    main()
