#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class QrDetectorNode : public rclcpp::Node {
public:
    QrDetectorNode() : Node("qr_detector_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/perception/qr_result", 10);
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&QrDetectorNode::image_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "C++ QR 感知节点已启动！正在处理 /camera/image 图像流...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            
            cv::QRCodeDetector qr_decoder;
            std::string data = qr_decoder.detectAndDecode(frame);
            
            if (!data.empty()) {
                auto result_msg = std_msgs::msg::String();
                result_msg.data = data;
                publisher_->publish(result_msg);
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换异常: %s", e.what());
        }
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QrDetectorNode>());
    rclcpp::shutdown();
    return 0;
}