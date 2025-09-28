#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Dense>
#include <vector>
#include <pcl/common/centroid.h>
#include <opencv2/opencv.hpp>
#include <algorithm>

int main(int argc, char** argv)
{
    std::string filename = "../output.pcd";
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", filename.c_str());
        return -1;
    }

    std::cout << "Loaded " << cloud->points.size() << " points from " << filename << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCD Viewer"));

    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    // Center cloud
    for (auto& point : cloud->points) {
        point.x -= centroid[0];
        point.y -= centroid[1];
        point.z -= centroid[2];
    }

    // SOR
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);             // Number of neighbors to analyze
    sor.setStddevMulThresh(0.3);  // Threshold: higher = less strict

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered);

    std::cout << "Filtered cloud has " << cloud_filtered->points.size() << " points." << std::endl;

    // 3. PCA
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud_filtered);
    Eigen::Matrix3f eig_vecs = pca.getEigenVectors(); // columns = PC1, PC2, PC3

    // 4. Rotation matrix: align pc1 -> X, pc2 -> Y, pc3 -> Z
    Eigen::Matrix3f R = eig_vecs.transpose(); // transpose because we want R * pc = aligned
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_aligned->resize(cloud_filtered->size());

    for (size_t i=0; i<cloud_filtered->size(); ++i)
    {
        Eigen::Vector3f p(cloud_filtered->points[i].x,
                          cloud_filtered->points[i].y,
                          cloud_filtered->points[i].z);
        Eigen::Vector3f p_aligned = R * p;
        cloud_aligned->points[i].x = p_aligned.x();
        cloud_aligned->points[i].y = p_aligned.y();
        cloud_aligned->points[i].z = p_aligned.z();
    }






    // 1. Find XY bounds
    float x_min = FLT_MAX, x_max = -FLT_MAX;
    float y_min = FLT_MAX, y_max = -FLT_MAX;

    for(auto& pt : cloud_aligned->points)
    {
        if(pt.x < x_min) x_min = pt.x;
        if(pt.x > x_max) x_max = pt.x;
        if(pt.y < y_min) y_min = pt.y;
        if(pt.y > y_max) y_max = pt.y;
    }

    // 2. Define grid resolution
    float resolution = 0.05f; // meters per pixel
    int width = static_cast<int>((x_max - x_min)/resolution) + 1;
    int height = static_cast<int>((y_max - y_min)/resolution) + 1;

    std::cout << "Grid size: " << width << " x " << height << std::endl;

    // 3. Create grid and count points per cell
    cv::Mat grid = cv::Mat::zeros(height, width, CV_32SC1);

    for(auto& pt : cloud_aligned->points)
    {
        int ix = static_cast<int>((pt.x - x_min)/resolution);
        int iy = static_cast<int>((pt.y - y_min)/resolution);
        // OpenCV images have row = y, col = x, origin at top-left
        // Optional: flip y to have origin bottom-left
        int iy_flip = height - 1 - iy;
        grid.at<int>(iy, ix) = 255;
    }

    // 4. Normalize to 0-255
    double min_val, max_val;
    cv::minMaxLoc(grid, &min_val, &max_val);
    cv::Mat grid_normalized;
    grid.convertTo(grid_normalized, CV_8UC1, 255.0/max_val);


    // 6. Show
    cv::Mat grid_resized;
    cv::resize(grid_normalized, grid_resized, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST);
    cv::imshow("Occupancy Grid", grid_resized);
    cv::waitKey(0);

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_aligned, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem(0.5);   
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spin();
    }

    return 0;
}
