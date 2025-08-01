#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

//eigen
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class MonocularSlamNode : public rclcpp::Node {
    public:
        MonocularSlamNode(ORB_SLAM3::System* pSLAM);
        ~MonocularSlamNode();

    private:
        using ImageMsg = sensor_msgs::msg::Image;
        void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
        ORB_SLAM3::System* m_SLAM;
        cv_bridge::CvImagePtr m_cvImPtr;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

        ORB_SLAM3::Atlas* mpAtlas;
        ORB_SLAM3::Tracking* mpTracker;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
        void PublishPointcloud();
};

#endif
