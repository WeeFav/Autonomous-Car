#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera1/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // this->SavePointCloud("./orbslam_map.pcd");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
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

// void MonocularSlamNode::SavePointCloud(const string &filename) {
//     mpAtlas = m_SLAM->GetAtlas();
//     std::cout << "Saving point cloud to " << filename << " ..." << std::endl;
//     std::cout << "Number of maps is: " << mpAtlas->CountMaps() << std::endl;

//     ORB_SLAM3::Map *pActiveMap = mpAtlas->GetCurrentMap();
//     if(!pActiveMap) {
//         std::cout << "There is no active map (pActiveMap is null)" << std::endl;
//         return;
//     }  

//     // Vector of pointers for Map Points -- vpMPs
//     // Vector of pointers for Reference Map Points -- vpRefMPs
//     const std::vector<ORB_SLAM3::MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
//     const std::vector<ORB_SLAM3::MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
//     if(vpMPs.empty()) {
//         std::cout << "Vector of map points vpMPs is empty!" << std::endl;
//         // return;
//     }

//     set<ORB_SLAM3::MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

//     // PointCloudXYZ::Ptr p_global_cloud(new PointCloudXYZ);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PCLPointCloud2::Ptr p_global_cloud(new pcl::PCLPointCloud2());

//     std::cout << "here123" << std::endl;

//     // for (size_t i = 0; i < vpMPs.size(); i++) {
//     //     const auto lm = vpMPs.at(i);
//     //     if (lm->isBad()) continue;

//     //     Eigen::Vector3f pos_w = lm->GetWorldPos();
//     //     pcl::PointXYZ point(pos_w.x(), pos_w.y(), pos_w.z());
//     //     p_global_cloud->push_back(point);
//     // }

//     // if (p_global_cloud->empty()) {
//     //     std::cout << "No points to save." << std::endl;
//     // } 
//     // else {
//     //     pcl::io::savePCDFileBinary(filename, *p_global_cloud);
//     //     std::cout << "Saved point cloud to " << filename << " with " << p_global_cloud->size() << " points." << std::endl;
//     // }
// }