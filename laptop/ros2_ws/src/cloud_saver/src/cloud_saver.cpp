#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class CloudSaver : public rclcpp::Node
{
public:
    CloudSaver() : Node("cloud_saver")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/map_points", 10,
            std::bind(&CloudSaver::callback, this, std::placeholders::_1));
        target_time_sec_ = 1758464640.341941764;
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert builtin_interfaces/Time to seconds
        double msg_time = rclcpp::Time(msg->header.stamp).seconds();

        if (std::abs(msg_time - target_time_sec_) < 1e-3) { // within 1 ms
            RCLCPP_INFO(this->get_logger(), "Matched timestamp: %.9f", msg_time);

            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*msg, cloud);

            pcl::io::savePCDFileBinary("output.pcd", cloud);
            RCLCPP_INFO(this->get_logger(), "Saved pointcloud to output.pcd");

            rclcpp::shutdown(); // stop after saving
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    double target_time_sec_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CloudSaver>());
    rclcpp::shutdown();
    return 0;
}