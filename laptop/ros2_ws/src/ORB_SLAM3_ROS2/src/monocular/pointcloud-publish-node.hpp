#ifndef __POINTCLOUD_PUBLISH_NODE_HPP__
#define __POINTCLOUD_PUBLISH_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

//eigen
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class PointcloudPubNode : public rclcpp::Node
{
    public:
        PointcloudPubNode(ORB_SLAM3::System* pSLAM);
        // ~PointcloudPubNode();

    private:
        ORB_SLAM3::System* m_SLAM;
        ORB_SLAM3::Atlas* mpAtlas;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        void timerCallback();
};

#endif