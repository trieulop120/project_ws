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

class PointCloudDownsampler : public rclcpp::Node
{
public:
  PointCloudDownsampler()
  : Node("pointcloud_downsampler")
  {
    // Declare parameters (use_sim_time is auto-declared by rclcpp)
    this->declare_parameter("leaf_size", 0.05);
    this->declare_parameter("z_min", 0.15);
    this->declare_parameter("z_max", 1.20);

    leaf_size_ = this->get_parameter("leaf_size").as_double();
    z_min_ = this->get_parameter("z_min").as_double();
    z_max_ = this->get_parameter("z_max").as_double();

    RCLCPP_INFO(this->get_logger(),
      "PointCloud Downsampler: leaf=%.3fm, z_range=[%.2f, %.2f]",
      leaf_size_, z_min_, z_max_);

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
    // Convert to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Filter Z-axis first (PassThrough)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_);
    pass_z.filter(*cloud_z);

    // VoxelGrid downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_z);
    voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel.filter(*cloud_filtered);

    // Publish filtered cloud
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = msg->header;
    publisher_->publish(output);

    // Stats
    frame_count_++;
    total_points_in_ += cloud->points.size();
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
  rclcpp::TimerBase::SharedPtr timer_;

  double leaf_size_;
  double z_min_;
  double z_max_;

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
