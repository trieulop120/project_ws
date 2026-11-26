/**
 * PointCloud Downsampler Node
 *
 * Uses PCL VoxelGrid to downsample PointCloud from camera
 * for high-frequency 3D perception (10-15 Hz)
 *
 * Input:  /points       (~57,000 points @ ~4 Hz)
 * Output: /points_filtered (~2,000-3,000 points @ 10-15 Hz)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>

class PointCloudDownsampler : public rclcpp::Node
{
public:
  PointCloudDownsampler()
  : Node("pointcloud_downsampler")
  {
    // Declare parameters (use_sim_time is auto-declared by rclcpp)
    this->declare_parameter("height_min", 0.05);
    this->declare_parameter("height_max", 1.20);
    this->declare_parameter("voxel_leaf_size", 0.05);
    height_min_ = this->get_parameter("height_min").as_double();
    height_max_ = this->get_parameter("height_max").as_double();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    RCLCPP_INFO(get_logger(),
      "PointCloud: height=[%.2f, %.2f]m, voxel=%.3fm",
      height_min_, height_max_, voxel_leaf_size_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points",
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloudDownsampler::cloudCallback, this, std::placeholders::_1));

    // Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/points_filtered",
      rclcpp::SensorDataQoS());

    // Statistics timer
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&PointCloudDownsampler::printStats, this));
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dropping point cloud with empty frame_id");
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        "base_footprint", msg->header.frame_id, msg->header.stamp, tf2::Duration::zero());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dropping cloud: no transform %s -> base_footprint at cloud stamp: %s",
        msg->header.frame_id.c_str(), ex.what());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *source);
    const auto & t = transform.transform.translation;
    const auto & q = transform.transform.rotation;
    const Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
    const Eigen::Vector3f translation(t.x, t.y, t.z);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filtered->points.reserve(source->points.size());
    for (const auto & point : source->points) {
      const Eigen::Vector3f p = rotation * Eigen::Vector3f(point.x, point.y, point.z) + translation;
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()) ||
        p.z() < height_min_ || p.z() > height_max_) {
        continue;
      }
      filtered->points.emplace_back(p.x(), p.y(), p.z());
    }
    filtered->width = static_cast<std::uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(filtered);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = msg->header;
    output.header.frame_id = "base_footprint";
    publisher_->publish(output);

    // Stats
    frame_count_++;
    total_points_in_ += source->points.size();
    total_points_out_ += cloud_filtered->points.size();
  }

  void printStats()
  {
    if (frame_count_ > 0) {
      double avg_in = total_points_in_ / frame_count_;
      double avg_out = total_points_out_ / frame_count_;
      double rate = frame_count_ / 5.0;
      RCLCPP_INFO(this->get_logger(),
        "Stats: %.1f Hz | In: %.0f pts | Out: %.0f pts | Reduction: %.1f%%",
        rate, avg_in, avg_out, (1.0 - avg_out/avg_in) * 100.0);
    }
    frame_count_ = 0;
    total_points_in_ = 0;
    total_points_out_ = 0;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  double height_min_ = 0.05;
  double height_max_ = 1.20;
  double voxel_leaf_size_ = 0.05;

  int frame_count_ = 0;
  size_t total_points_in_ = 0;
  size_t total_points_out_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudDownsampler>());
  rclcpp::shutdown();
  return 0;
}
