#include "planning_process.h"

namespace Planning
{
    PlanningProcess::PlanningProcess() : Node("planning_process") // 规划总流程
    {
        RCLCPP_INFO(this->get_logger(), "planning_process_created");
        // 配置文件读取测试
        process_config_ = std::make_unique<ConfigReader>(); // 创建一个新的 ConfigReader 对象，并立即让 process_config_ 这个智能指针接管它的所有权
        process_config_->read_planning_process_config();
        obs_dis_ = process_config_->process().obs_dis_;

        // 创建车辆 和 障碍物
        car_ = std::make_shared<MainCar>(); // 父类对象调用子类指针 会调用主车文件中的构造函数
        for (int i = 0; i < 3; i++)         // 用循环来生成障碍物
        {
            auto obs_car = std::make_shared<ObsCar>(i+1); // 创建一个障碍物的对象 指针
            obses_spawn_.emplace_back(obs_car);        // 父类的指针， 指向了子类的对象
        }

        // 坐标广播器
        tf_broadcaster_ = std::make_shared<StaticTransformBroadcaster>(this);

        // 创建监听器，绑定主车缓存对象
        buffer_ = std::make_unique<Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<TransformListener>(*buffer_, this);

        // 创建地图和全局路径客户端
        map_client_ = this->create_client<PNCMapService>("pnc_map_server");
        global_path_client_ = this->create_client<GlobalPathService>("global_path_server");

        // 创建参考线和参考线的发布器
        refer_line_creator_ = std::make_shared<ReferenceLineCreator>();
        refer_line_pub_ = this->create_publisher<Path>("reference_line", 10);
    }

    bool PlanningProcess::process() // 规划总流程
    {
        // 阻塞1s 等待rviz和xacro启动
        rclcpp::Rate rate(1.0);
        rate.sleep();

        // planning init 初始化
        if (!planning_init())
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "planning init failed!");
            return false;
        }

        // 进入规划主流程
        timer_ = this->create_wall_timer(0.1s, std::bind(&PlanningProcess::planning_callback, this));

        return true;
    }

    bool PlanningProcess::planning_init()
    {
        // 生成车辆
        vehicle_spawn(car_);
        
        // 生成障碍物
        for (const auto &obs : obses_spawn_)
        {
            vehicle_spawn(obs);
        }
        
        // 连接地图服务器
        if (!connect_server(map_client_))
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "Map Server connect failed!");
            return false;
        }

        // 获取地图
        if (!map_request())
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "Map Request and response failed!");
            return false;
        }

        // 连接全局路径服务器
        if (!connect_server(global_path_client_))
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "Global Path Server connect failed!");
            return false;
        }

        // 获取全局路径
        if (!global_path_request())
        {
            /* code */
            RCLCPP_ERROR(this->get_logger(), "Global Path Request and response failed!");
            return false;
        }

        return true;
    }

    template <typename T>
    bool PlanningProcess::connect_server(const T &client)
    {
        // 判断客户端的类型
        std::string server_name;
        if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>) // 此用法为C++17时期引入的
        {
            /* code */
            server_name = "pnc_map";
        }
        else if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
        {
            server_name = "global_path";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "wrong client type!");
            return false;
        }

        // 等待服务器
        while (!client->wait_for_service(1s))
        {
            /* code */
            if (!rclcpp::ok()) // 使用ctrl+c操作，防止进入死循环
            {
                /* code */
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server", server_name.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "%s server not avaliable, waiting again...", server_name.c_str());
        }

        return true;
    }

    bool PlanningProcess::map_request() // 发送地图请求
    {
        RCLCPP_INFO(this->get_logger(), "Sending map request/");

        // 生成请求
        auto request = std::make_shared<PNCMapService::Request>(); // 指针指向 这里要与响应函数中写的参数一致
        request->map_type = process_config_->pnc_map().type_;      // 从配置文件中读取，写的是什么地图
        // 获取响应
        auto result_future = map_client_->async_send_request(request); // map_client_->async_send_request(request)发送请求
        // 判断响应（发送）是否成功
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            /* code */
            RCLCPP_INFO(this->get_logger(), "Map response success!");
            pnc_map_ = result_future.get()->pnc_map; // 获取响应中的地图
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Map response failed!");
            return false;
        }

        return false;
    }

    bool PlanningProcess::global_path_request()
    {
        RCLCPP_INFO(this->get_logger(), "Sending global path request/");

        // 生成请求
        auto request = std::make_shared<GlobalPathService::Request>();       // 指针指向 这里要与响应函数中写的参数一致
        request->pnc_map = pnc_map_;                                         // 这里的两个pnc_map是不一样的，= 后面的是map request中相应回来的地图
        request->global_planner_type = process_config_->global_path().type_; // 这里有两个request，对应msg自定义文件中的两个request

        // 获取响应
        auto result_future = global_path_client_->async_send_request(request);
        // 判断响应（发送）是否成功
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            /* code */
            RCLCPP_INFO(this->get_logger(), "Global Path response success!");
            global_path_ = result_future.get()->global_path; // 获取响应中的全局路径
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Global Path response failed!");
            return false;
        }

        return false;
    }

    void PlanningProcess::vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle)
    {
        TransformStamped spawn;                                    // 坐标变换对象
        spawn.header.stamp = this->now();                          // 现在因为在一个ros的节点里面，所以可以直接用this指针来调用now 继承了rclcppNode
        spawn.header.frame_id = process_config_->pnc_map().frame_; // pnc地图坐标
        spawn.child_frame_id = vehicle->get_child_frame();         // 子坐标为车辆坐标

        spawn.transform.translation.x = vehicle->get_loc_point().pose.position.x;
        spawn.transform.translation.y = vehicle->get_loc_point().pose.position.y;
        spawn.transform.translation.z = vehicle->get_loc_point().pose.position.z;
        spawn.transform.rotation.x = vehicle->get_loc_point().pose.orientation.x;
        spawn.transform.rotation.y = vehicle->get_loc_point().pose.orientation.y;
        spawn.transform.rotation.z = vehicle->get_loc_point().pose.orientation.z;
        spawn.transform.rotation.w = vehicle->get_loc_point().pose.orientation.w;

        RCLCPP_INFO(this->get_logger(), "vehicle %s spawned, x = %.2f, y = %.2f",
                    spawn.child_frame_id.c_str(), vehicle->get_loc_point().pose.position.x, vehicle->get_loc_point().pose.position.y);
        tf_broadcaster_->sendTransform(spawn);
    }

    void PlanningProcess::get_location(const std::shared_ptr<VehicleBase> &vehicle)
    {
        try
        {
            // 定义一个位姿点
            PoseStamped point;
            auto ts = buffer_->lookupTransform(process_config_->pnc_map().frame_, vehicle->get_child_frame(), tf2::TimePointZero); // 父坐标 子坐标 当前时间
            point.header = ts.header;
            point.pose.position.x = ts.transform.translation.x;
            point.pose.position.y = ts.transform.translation.y;
            point.pose.position.z = ts.transform.translation.z;
            point.pose.orientation.x = ts.transform.rotation.x;
            point.pose.orientation.y = ts.transform.rotation.y;
            point.pose.orientation.z = ts.transform.rotation.z;
            point.pose.orientation.w = ts.transform.rotation.w;
            // 调用更新函数
            vehicle->update_location(point);
        }
        catch (const tf2::LookupException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Lookup exception:%s", e.what());
        }
    }

    void PlanningProcess::planning_callback() // 总流程回调
    {
        // 获取规划开始的时候时间戳
        const auto start_time = this->get_clock()->now();
        // 监听车辆定位
        get_location(car_); // 监听主车定位

        // 参考线
        const auto refer_line = refer_line_creator_->creat_reference_line(global_path_, car_->get_loc_point());
        if (refer_line.refer_line.empty()) // 判断参考线是否为空
        {
            RCLCPP_ERROR(this->get_logger(), "reference line is empty!");
            return;
        }
        const auto refer_line_rviz = refer_line_creator_->referline_to_rviz(); // 生成rviz用的参考线 markerarray
        refer_line_pub_->publish(refer_line_rviz);                             // 发布rviz用的参考线

        // 主车和障碍物向参考线投影

        // 障碍物按s值排序

        // 路径决策

        // 路径规划

        // 障碍物向路径投影

        // 速度决策

        // 速度规划

        // 合成轨迹

        // 更新绘图信息

        // 更新车辆信息

        RCLCPP_INFO(this->get_logger(), "----------car state: location: (%.2f, %.2f), speed: %.2f, a: %.2f, theta: %.2f, kappa: %.2f",
                    car_->get_loc_point().pose.position.x, car_->get_loc_point().pose.position.y,
                    car_->get_speed(), car_->get_acceleration(),
                    car_->get_theta(), car_->get_kappa());

        const auto end_time = this->get_clock()->now();
        const double planning_total_time = end_time.seconds() - start_time.seconds();
        RCLCPP_INFO(this->get_logger(), "planning total time: %f ms\n", planning_total_time * 1000);

        // 防止系统卡死
        if (planning_total_time > 1.0)
        {
            RCLCPP_ERROR(this->get_logger(), "planning time too long!");
            rclcpp::shutdown();
        }
    }

}