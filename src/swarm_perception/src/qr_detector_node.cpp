#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

class QrDetectorNode : public rclcpp::Node {
public:
    QrDetectorNode() : Node("qr_detector_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/perception/qr_result", 10);
        
        offset_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/perception/qr_offset", 10);
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&QrDetectorNode::image_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "视觉伺服 QR 节点已启动！");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            
            cv::QRCodeDetector qr_decoder;
            std::vector<cv::Point2f> corners;
            std::string data = qr_decoder.detectAndDecode(frame, corners);
            
            if (!data.empty() && corners.size() == 4) {
                auto result_msg = std_msgs::msg::String();
                result_msg.data = data;
                publisher_->publish(result_msg);

                float cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
                float width = cv::norm(corners[0] - corners[1]);
                
                auto offset_msg = geometry_msgs::msg::Vector3();
                offset_msg.x = cx - 320.0; 
                offset_msg.y = 0.0;
                offset_msg.z = width;      
                offset_publisher_->publish(offset_msg);
                

                std::vector<cv::Point> int_corners;
                for (size_t i = 0; i < corners.size(); i++) {
                    int_corners.push_back(cv::Point(std::round(corners[i].x), std::round(corners[i].y)));
                }
                cv::polylines(frame, int_corners, true, cv::Scalar(0, 255, 0), 2);
            }
            

            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换异常: %s", e.what());
        }
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr offset_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QrDetectorNode>());
    rclcpp::shutdown();
    return 0;
}