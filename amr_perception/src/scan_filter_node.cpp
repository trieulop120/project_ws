#include <memory>
#include <vector>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFilterNode : public rclcpp::Node
{
public:
    ScanFilterNode() : Node("scan_filter_node")
    {
        // Khai báo các tham số kích thước khung xe (Đơn vị: Mét)
        // Mốc gốc (0,0) tính từ TÂM CẢM BIẾN LIDAR
        this->declare_parameter<double>("front_limit", 0.52); // Phía trước (+X từ Lidar ra càn nâng): ~52 cm
        this->declare_parameter<double>("rear_limit",  0.26); // Phía sau   (-X từ Lidar ra đuôi xe):  ~26 cm
        this->declare_parameter<double>("left_limit",  0.29); // Bên trái   (+Y bao trọn bánh trái):   ~29 cm
        this->declare_parameter<double>("right_limit", 0.29); // Bên phải  (-Y bao trọn bánh phải):   ~29 cm

        // Lấy giá trị tham số
        front_limit_ = this->get_parameter("front_limit").as_double();
        rear_limit_  = this->get_parameter("rear_limit").as_double();
        left_limit_  = this->get_parameter("left_limit").as_double();
        right_limit_ = this->get_parameter("right_limit").as_double();

        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ScanFilterNode::scanCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);

        RCLCPP_INFO(this->get_logger(),
            "Lidar Filter đã khởi tạo với kích thước mới! "
            "Trước=+%.2fm | Sau=-%.2fm | Trái=+%.2fm | Phải=-%.2fm",
            front_limit_, rear_limit_, left_limit_, right_limit_);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_msg = *msg;

        for (size_t i = 0; i < filtered_msg.ranges.size(); ++i) {
            float r = filtered_msg.ranges[i];

            if (std::isnan(r) || std::isinf(r) || r < filtered_msg.range_min || r > filtered_msg.range_max) {
                continue;
            }

            float angle = filtered_msg.angle_min + i * filtered_msg.angle_increment;

            float x = r * std::cos(angle);
            float y = r * std::sin(angle);

            // Kiểm tra điểm (x, y) nằm trong khung che chắn của thân xe
            if (x >= -rear_limit_ && x <= front_limit_ &&
                y >= -right_limit_ && y <= left_limit_)
            {
                filtered_msg.ranges[i] = std::numeric_limits<float>::infinity();

                if (!filtered_msg.intensities.empty()) {
                    filtered_msg.intensities[i] = 0.0f;
                }
            }
        }

        pub_->publish(filtered_msg);
    }

    double front_limit_;
    double rear_limit_;
    double left_limit_;
    double right_limit_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFilterNode>());
    rclcpp::shutdown();
    return 0;
}