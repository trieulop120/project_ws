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
        // Lidar đặt tại mốc gốc (0,0)
        this->declare_parameter<double>("front_limit", 0.45); // Phía trước  (+X): 45 cm
        this->declare_parameter<double>("rear_limit", 0.25);  // Phía sau    (-X): 25 cm
        this->declare_parameter<double>("left_limit", 0.25);  // Bên trái    (+Y): 25 cm
        this->declare_parameter<double>("right_limit", 0.25); // Bên phải   (-Y): 25 cm

        // Lấy giá trị tham số
        front_limit_ = this->get_parameter("front_limit").as_double();
        rear_limit_  = this->get_parameter("rear_limit").as_double();
        left_limit_  = this->get_parameter("left_limit").as_double();
        right_limit_ = this->get_parameter("right_limit").as_double();

        // =========================================================================
        // [ĐẦU VÀO - INPUT]: Lắng nghe dữ liệu thô Lidar từ Topic /scan
        // =========================================================================
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ScanFilterNode::scanCallback, this, std::placeholders::_1));

        // =========================================================================
        // [ĐẦU RA - OUTPUT]: Xuất dữ liệu Lidar đã lọc nhiễu ra Topic /scan_filtered
        // =========================================================================
        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);

        RCLCPP_INFO(this->get_logger(),
            "Lidar Filter đã khởi tạo! Khung lọc chữ nhật [Dài x Rộng = 65cm x 50cm]: "
            "Trước=+%.2fm | Sau=-%.2fm | Trái=+%.2fm | Phải=-%.2fm",
            front_limit_, rear_limit_, left_limit_, right_limit_);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_msg = *msg; // Tạo bản sao tin nhắn Lidar gốc

        for (size_t i = 0; i < filtered_msg.ranges.size(); ++i) {
            float r = filtered_msg.ranges[i];

            // Bỏ qua các điểm lỗi khoảng cách ban đầu
            if (std::isnan(r) || std::isinf(r) || r < filtered_msg.range_min || r > filtered_msg.range_max) {
                continue;
            }

            // 1. Tính góc quay theta của tia laze thứ i (theo Radian)
            float angle = filtered_msg.angle_min + i * filtered_msg.angle_increment;

            // 2. Chuyển từ hệ tọa độ Cực (r, angle) sang Tọa độ Đề-các (x, y)
            // Chuẩn ROS 2: +X là tiến về phía trước, +Y là rẽ sang trái
            float x = r * std::cos(angle);
            float y = r * std::sin(angle);

            // 3. Kiểm tra xem điểm (x, y) có nằm TRONG khung chữ nhật của thân xe hay không
            if (x >= -rear_limit_ && x <= front_limit_ &&
                y >= -right_limit_ && y <= left_limit_)
            {
                // Nếu nằm trong khung xe -> Xóa điểm đó (gán bằng Vô cực - inf)
                filtered_msg.ranges[i] = std::numeric_limits<float>::infinity();

                // Reset độ phản xạ (nếu có)
                if (!filtered_msg.intensities.empty()) {
                    filtered_msg.intensities[i] = 0.0f;
                }
            }
        }

        // =========================================================================
        // [ĐẦU RA - OUTPUT]: Bắn dữ liệu sạch đã lọc qua Topic /scan_filtered
        // =========================================================================
        pub_->publish(filtered_msg);
    }

    double front_limit_;
    double rear_limit_;
    double left_limit_;
    double right_limit_;

    // Khai báo Subscriber (Nhận /scan) và Publisher (Bắn /scan_filtered)
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