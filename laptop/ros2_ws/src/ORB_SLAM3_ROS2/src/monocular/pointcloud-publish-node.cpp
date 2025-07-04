#include "pointcloud-publish-node.hpp"

using std::placeholders::_1;

PointcloudPubNode::PointcloudPubNode(ORB_SLAM3::System* pSLAM) : Node("PointcloudPubNode") {
    m_SLAM = pSLAM;
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz
        std::bind(&PointcloudPubNode::timerCallback, this)
    );
    std::cout << "123" << std::endl;
}

void PointcloudPubNode::timerCallback() {

    mpAtlas = m_SLAM->GetAtlas();
    ORB_SLAM3::Map *pActiveMap = mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;

    const std::vector<ORB_SLAM3::MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
    const std::vector<ORB_SLAM3::MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
    if (vpMPs.empty()) return;
    set<ORB_SLAM3::MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    pcl::PointCloud<pcl::PointXYZ> cloud_;
    // auto p_global_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr p_global_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    // for (size_t i = 0; i < vpMPs.size(); i++) {
    //     const auto lm = vpMPs.at(i);
    //     if (lm->isBad()) continue;

    //     Eigen::Vector3f pos_w;
    //     pos_w = lm->GetWorldPos();

    //     pcl::PointXYZ point(pos_w.x(), pos_w.y(), pos_w.z());
    //     p_global_cloud.push_back(point);
    // }    
    
    // sensor_msgs::msg::PointCloud2 cloud_msg;
    // pcl::toROSMsg(*p_global_cloud, cloud_msg);
    // cloud_msg.header.frame_id = "map";
    // cloud_msg.header.stamp = this->get_clock()->now();

    // publisher_->publish(cloud_msg);    
}