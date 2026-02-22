#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// ==========================================
// 1. 业务节点：模拟从系统查询订单信息并写入黑板
// ==========================================
class GetOrderData : public BT::SyncActionNode {
public:
    GetOrderData(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::string>("order_id"),
            BT::OutputPort<double>("target_x"),
            BT::OutputPort<double>("target_y"),
            BT::OutputPort<double>("target_yaw"),
            BT::OutputPort<std::string>("target_qr")
        };
    }

    BT::NodeStatus tick() override {
        std::string order_id;
        if (!getInput("order_id", order_id)) {
            throw BT::RuntimeError("缺少订单号输入");
        }

        std::cout << "\n[DATABASE] 正在解析订单: " << order_id << " ..." << std::endl;

        // 根据 warehouse.sdf 的布局映射目标点
        if (order_id == "Order_01") {
            setOutput("target_x", 2.5);
            setOutput("target_y", 0.9);
            setOutput("target_yaw", 1.5707);
            setOutput("target_qr", "Item_01");
        } else if (order_id == "Order_03") {
            setOutput("target_x", 2.5);
            setOutput("target_y", -0.9);
            setOutput("target_yaw", -1.5707);
            setOutput("target_qr", "Item_03");
        } else {
            std::cerr << "[DATABASE] 找不到该订单信息！" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[DATABASE] 订单解析成功！数据已写入黑板。" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// ==========================================
// 2. 视觉伺服对接节点：闭环 PID 控制
// ==========================================
class VisualServoDocking : public BT::StatefulActionNode {
public:
    VisualServoDocking(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), ros_node_(node) {
        
        cmd_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        offset_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
            "/perception/qr_offset", 10,
            [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
                last_offset_x_ = msg->x;
                last_width_ = msg->z;
                last_msg_time_ = ros_node_->now();
                data_received_ = true;
            });
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("target_width"),
                 BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override {
        std::cout << "[SERVOING] 接管底盘，开始视觉伺服对接..." << std::endl;
        start_time_ = ros_node_->now();
        data_received_ = false;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        double timeout, target_width;
        getInput("timeout", timeout);
        getInput("target_width", target_width);

        if ((ros_node_->now() - start_time_).seconds() > timeout) {
            std::cerr << "[SERVOING] 伺服对接超时！" << std::endl;
            stopRobot();
            return BT::NodeStatus::FAILURE;
        }

        // 防丢失保护：超1.5秒未见二维码则刹车等待
        if (!data_received_ || (ros_node_->now() - last_msg_time_).seconds() > 1.5) {
            stopRobot();
            return BT::NodeStatus::RUNNING; 
        }

        auto twist_msg = geometry_msgs::msg::Twist();
        
        // P 控制器算法
        double kp_yaw = 0.003; 
        twist_msg.angular.z = -kp_yaw * last_offset_x_; 

        double kp_linear = 0.005;
        double error_dist = target_width - last_width_;
        
        // 当偏航角对准（误差<40像素）时，才允许前进
        if (std::abs(last_offset_x_) < 40.0 && error_dist > 0) {
            twist_msg.linear.x = std::min(kp_linear * error_dist, 0.15); 
        } else {
            twist_msg.linear.x = 0.0;
        }

        cmd_pub_->publish(twist_msg);

        // 成功条件：误差<5像素 且 偏航对准<20像素
        if (error_dist <= 5.0 && std::abs(last_offset_x_) < 20.0) {
            std::cout << "[SERVOING] 对接完美贴合！正在模拟抓取货物..." << std::endl;
            stopRobot();
            rclcpp::sleep_for(std::chrono::seconds(2)); // 模拟机械臂动作
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[SERVOING] 伺服被紧急中止" << std::endl;
        stopRobot();
    }

private:
    void stopRobot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        cmd_pub_->publish(twist_msg);
    }

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr offset_sub_;
    
    rclcpp::Time start_time_;
    rclcpp::Time last_msg_time_;
    double last_offset_x_ = 0.0;
    double last_width_ = 0.0;
    bool data_received_ = false;
};

// ==========================================
// 3. 扫码验证节点 (异步)
// ==========================================
class WaitForQR : public BT::StatefulActionNode {
public:
    WaitForQR(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), ros_node_(node) {
        qr_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
            "/perception/qr_result", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                last_qr_data_ = msg->data;
            });
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("expected_qr"),
                 BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override {
        last_qr_data_.clear();
        start_time_ = ros_node_->now();
        std::cout << "[SCANNING] 到达货架盲区边缘！开启视觉二次验证..." << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        std::string expected_qr; double timeout;
        getInput("expected_qr", expected_qr); getInput("timeout", timeout);

        if ((ros_node_->now() - start_time_).seconds() > timeout) {
            std::cerr << "[SCANNING] 扫描超时！未找到目标货物。" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (last_qr_data_.find(expected_qr) != std::string::npos) {
            std::cout << "[SCANNING] 二次验证成功: " << last_qr_data_ << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { std::cout << "[SCANNING] 任务中断" << std::endl; }

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
    std::string last_qr_data_;
    rclcpp::Time start_time_;
};

// ==========================================
// 4. Nav2 导航节点 (异步 Action Client)
// ==========================================
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Nav2Pose : public BT::StatefulActionNode {
public:
    Nav2Pose(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), ros_node_(node) {
        action_client_ = rclcpp_action::create_client<NavigateToPose>(ros_node_, "navigate_to_pose");
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("goal_x"), BT::InputPort<double>("goal_y"), BT::InputPort<double>("goal_yaw") };
    }

    BT::NodeStatus onStart() override {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Nav2 Action Server 未上线！");
            return BT::NodeStatus::FAILURE;
        }

        double x = 0.0, y = 0.0, yaw = 0.0;
        getInput("goal_x", x); getInput("goal_y", y); getInput("goal_yaw", yaw);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = ros_node_->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;

        tf2::Quaternion q; q.setRPY(0, 0, yaw);
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        RCLCPP_INFO(ros_node_->get_logger(), "发送全局导航目标: x=%.2f, y=%.2f", x, y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNav2::WrappedResult& result) {
            this->nav_result_ = result.code;
            this->done_flag_ = true;
        };

        done_flag_ = false; nav_result_ = rclcpp_action::ResultCode::UNKNOWN;
        action_client_->async_send_goal(goal_msg, send_goal_options);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!done_flag_) return BT::NodeStatus::RUNNING;
        if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(ros_node_->get_logger(), "已到达盲区边缘，准备移交控制权！");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(ros_node_->get_logger(), "导航失败！");
        return BT::NodeStatus::FAILURE;
    }

    void onHalted() override {
        action_client_->async_cancel_all_goals();
        done_flag_ = false;
    }

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    bool done_flag_ = false;
    rclcpp_action::ResultCode nav_result_ = rclcpp_action::ResultCode::UNKNOWN;
};

// ==========================================
// 主函数与工厂注册
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_dispatcher_node");

    BT::BehaviorTreeFactory factory;

    // 注册无需 ROS node 指针的节点
    factory.registerNodeType<GetOrderData>("GetOrderData");
    
    // 注册需要 ROS node 指针的节点
    factory.registerBuilder<VisualServoDocking>("VisualServoDocking",
        [node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<VisualServoDocking>(name, config, node);
        });
        
    factory.registerBuilder<WaitForQR>("WaitForQR",
        [node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<WaitForQR>(name, config, node);
        });

    factory.registerBuilder<Nav2Pose>("Nav2Pose",
        [node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<Nav2Pose>(name, config, node);
        });

    node->declare_parameter("bt_xml_file", "");
    std::string xml_path = node->get_parameter("bt_xml_file").as_string();
    if (xml_path.empty()) {
        RCLCPP_ERROR(node->get_logger(), "必须提供 bt_xml_file 参数！");
        return -1;
    }

    auto tree = factory.createTreeFromFile(xml_path);
    RCLCPP_INFO(node->get_logger(), "--- 智能仓储任务调度系统 (支持黑板变量 & 视觉伺服) 已启动 ---");

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        BT::NodeStatus status = tree.tickOnce();
        
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "所有订单处理完毕！任务圆满结束。");
            break;
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(node->get_logger(), "任务流执行异常中断！");
            break;
        }
        
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}