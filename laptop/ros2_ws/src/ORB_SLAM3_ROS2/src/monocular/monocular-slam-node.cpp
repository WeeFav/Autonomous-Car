#include "monocular-slam-node.hpp"
#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM) : Node("ORB_SLAM3_ROS2") {
    m_SLAM = pSLAM;

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera1/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1)
    );

    mpAtlas = m_SLAM->GetAtlas();
    pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),  // 1 Hz
        std::bind(&MonocularSlamNode::PublishPointcloud, this)
    );

    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode() {
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // this->SavePointCloud("./orbslam_map.pcd");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg) {
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}

void MonocularSlamNode::PublishPointcloud() {
    std::cout << "Number of maps is: " << mpAtlas->CountMaps() << std::endl;

    ORB_SLAM3::Map *pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap) {
        std::cout << "There is no active map (pActiveMap is null)" << std::endl;
        return;
    }  

    // Vector of pointers for Map Points -- vpMPs
    // Vector of pointers for Reference Map Points -- vpRefMPs
    const std::vector<ORB_SLAM3::MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const std::vector<ORB_SLAM3::MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
    if(vpMPs.empty()) {
        std::cout << "Vector of map points vpMPs is empty!" << std::endl;
        return;
    }

    // set<ORB_SLAM3::MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());



    Eigen::Matrix3f rot_x_90;
    rot_x_90 = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX());

    // Filter points with rotated z > 0
    std::vector<Eigen::Vector3f> filtered_points;
    for (size_t i = 0; i < vpMPs.size(); i++) {
        ORB_SLAM3::MapPoint* pMP = vpMPs[i];
        if (!pMP || pMP->isBad()) continue;

        Eigen::Vector3f pos = pMP->GetWorldPos();
        Eigen::Vector3f rotated_pos = rot_x_90 * pos;

        if (rotated_pos.z() > 0) {
            filtered_points.push_back(rotated_pos);
        }
    }

    size_t num_points = filtered_points.size();
    std::cout << "Filtered number of points is: " << num_points << std::endl;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = "map";
    cloud_msg.height = 1; // Unordered point cloud
    cloud_msg.width = num_points;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;
    cloud_msg.point_step = 12; // 3 * 4 bytes (float32 x, y, z)
    cloud_msg.row_step = cloud_msg.point_step * num_points; // Total size of one row

    // Define point fields (x, y, z as float32)
    cloud_msg.fields.resize(3); // We have 3 fields: x, y, z

    // X field
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0; // Offset in bytes from the start of a point
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    // Y field
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4; // After x (4 bytes for float32)
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    // Z field
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8; // After y (4 bytes for float32)
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.data.resize(cloud_msg.row_step); // Allocate memory for all points

    // Copy data into cloud_msg.data
    for (size_t i = 0; i < num_points; i++) {
        const Eigen::Vector3f &pos = filtered_points[i];

        uint8_t* ptr = &cloud_msg.data[i * cloud_msg.point_step];

        std::memcpy(ptr + 0, &pos.x(), sizeof(float));
        std::memcpy(ptr + 4, &pos.y(), sizeof(float));
        std::memcpy(ptr + 8, &pos.z(), sizeof(float));
    }

    pointcloud_publisher->publish(cloud_msg);    

}