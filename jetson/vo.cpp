#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iterator>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>


#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

class VisualOdom {
public:
    VisualOdom() {
        orb = cv::ORB::create();
        cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2);
        cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);        
        flann = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);

        double fx = 651.3584644989045;
        double fy = 870.2541054486668;
        double cx = 328.3739555076033;
        double cy = 242.0301802544854;

        K = (cv::Mat_<double>(3,3) <<
                    fx, 0.0, cx,
                    0.0, fy, cy,
                    0.0, 0.0, 1.0);
    }

    cv::Point2d run(const cv::Mat &img, const int frame_idx) 
    {
        cv::Mat cur_pose;

        if (frame_idx == 0) 
        {
            orb->detectAndCompute(img, cv::noArray(), kp1, des1);
            cur_pose = cv::Mat::eye(4, 4, CV_64F);
        } 
        else 
        {
            std::vector<cv::Point2f> pts1, pts2;
            get_matches(img, pts1, pts2);

            cv::Mat R, t;

            get_pose(pts1, pts2, R, t);

            double scale = get_scale(R, t, pts1, pts2);
            
            // construct unscaled relative transform betwen frames
            cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
            t.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));
            T(cv::Range(0, 3), cv::Range(3, 4)) *= scale;

            cur_pose = prev_pose * T.inv();

            // Shift the cache: current becomes previous
            kp1 = kp2;
            des1 = des2.clone();
            prev_points_3d = points_3d;
        }

        prev_pose = cur_pose.clone();

        return cv::Point2d(cur_pose.at<double>(0, 3), cur_pose.at<double>(2, 3));          
    }


private:
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::FlannBasedMatcher> flann;
    cv::Mat K;
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;
    std::vector<cv::Point3f> prev_points_3d;
    std::vector<cv::Point3f> points_3d;
    cv::Mat prev_pose;

    void get_matches(const cv::Mat &img, std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2) {
        // Find the keypoints and descriptors
        orb->detectAndCompute(img, cv::noArray(), kp2, des2);

        // Match frame 1-2
        std::vector<std::vector<cv::DMatch>> matches;
        flann->knnMatch(des1, des2, matches, 2);

        pts1.clear();
        pts2.clear();

        // Extract corresponding keypoints
        for (size_t i = 0; i < matches.size(); i++) {
            if (matches[i].size() < 2) continue;
            cv::DMatch m = matches[i][0]; // first match
            cv::DMatch n = matches[i][1]; // second match
            if (m.distance < 0.8 * n.distance) {
                pts1.push_back(kp1[m.queryIdx].pt);
                pts2.push_back(kp2[m.trainIdx].pt);
            }
        }
    }

    void get_pose(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, cv::Mat &R, cv::Mat &t) {
        // find essential matrix using RANSAC 5-point algorithm
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);

        // // keep inliers from RANSAC
        // std::vector<cv::Point2f> inliers1, inliers2;
        // for (size_t i = 0; i < mask.rows; i++) {
        //     if (mask.at<uchar>(i)) {
        //         inliers1.push_back(pts1[i]);
        //         inliers2.push_back(pts2[i]);
        //     }
        // }

        // decompose essential matrix to get best R and best unscaled t
        cv::recoverPose(E, pts1, pts2, K, R, t, mask);
    }

    double get_scale(
        const cv::Mat &R, 
        const cv::Mat &t,
        const std::vector<cv::Point2f> &pts1, 
        const std::vector<cv::Point2f> &pts2) 
    {
        // triangulation to get 3D points
        cv::Mat P1 = cv::Mat::eye(3, 4, CV_64F); // first camera as origin
        P1 = K * P1;

        cv::Mat Rt;
        cv::hconcat(R, t, Rt);
        cv::Mat P2 = K * Rt; // second camera relative to first

        // Convert points to cv::Mat (2xN)
        cv::Mat pts1_T(2, pts1.size(), CV_64F);
        cv::Mat pts2_T(2, pts2.size(), CV_64F);
        for (size_t i = 0; i < pts1.size(); i++) {
            pts1_T.at<double>(0, i) = pts1[i].x;
            pts1_T.at<double>(1, i) = pts1[i].y;
            pts2_T.at<double>(0, i) = pts2[i].x;
            pts2_T.at<double>(1, i) = pts2[i].y;
        }

        cv::Mat points_4d_h;
        cv::triangulatePoints(P1, P2, pts1_T, pts2_T, points_4d_h); // 4xN

        // Convert from homogeneous to 3D
        points_3d.clear();
        for (int i = 0; i < points_4d_h.cols; ++i) {
            double w = points_4d_h.at<double>(3, i);
            points_3d.push_back(cv::Point3f(
                points_4d_h.at<double>(0, i) / w,
                points_4d_h.at<double>(1, i) / w,
                points_4d_h.at<double>(2, i) / w
            ));
        }

        // Estimate scale between previous and current 3D points
        if (prev_points_3d.empty() || points_3d.empty())
            return 1.0;

        size_t min_idx = std::min(prev_points_3d.size(), points_3d.size());

        std::vector<cv::Point3f> prev_pts(prev_points_3d.begin(), prev_points_3d.begin() + min_idx);
        std::vector<cv::Point3f> cur_pts(points_3d.begin(), points_3d.begin() + min_idx);

        std::vector<double> prev_dist, cur_dist;

        for (size_t i = 1; i < min_idx; ++i) {
            cv::Point3f dp = prev_pts[i] - prev_pts[i - 1];
            prev_dist.push_back(std::sqrt(dp.x * dp.x + dp.y * dp.y + dp.z * dp.z));

            cv::Point3f dc = cur_pts[i] - cur_pts[i - 1];
            cur_dist.push_back(std::sqrt(dc.x * dc.x + dc.y * dc.y + dc.z * dc.z));
        }

        std::vector<double> ratios;
        for (size_t i = 0; i < prev_dist.size(); ++i)
            ratios.push_back(prev_dist[i] / (cur_dist[i] + 1e-6));

        std::nth_element(ratios.begin(), ratios.begin() + ratios.size()/2, ratios.end());
        double scale = ratios[ratios.size()/2];  // median

        scale = std::max(0.1, std::min(5.0, scale));
        return scale;                
    }

};

bool send_all(int sock, const void *data, size_t size) {
    const char *ptr = (const char*)data;
    while (size > 0) {
        ssize_t sent = send(sock, ptr, size, 0);
        if (sent <= 0) return false; // Connection lost or error
        ptr += sent;
        size -= sent;
    }
    return true;
}

int main() {
    // ---------- TCP SERVER ----------
    int listen_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_socket == -1) {
        std::cerr << "Socket creation failed.\n";
        return 1;
    }
    
    // bind listening socket to server ip and port
    int opt = 1;
    setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(8000);
    if (bind(listen_socket, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        std::cerr << "Bind failed.\n";
        close(listen_socket);
        return 1;
    }
    
    // tell the socket is for listening
    listen(listen_socket, SOMAXCONN);
    std::cout << "Waiting for connection..." << std::endl;

    // define client ip and port structure
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    // wait for connection
    int sock = accept(listen_socket, (sockaddr*)&client_addr, &client_len);
    std::cout << "Client connected" << std::endl;
    if (sock == -1) {
        std::cerr << "Accept failed.\n";
        close(listen_socket);
        return 1;
    }
    
    // close listening socket (don't recieve other clients)
    close(listen_socket);

    // ---------- VO ----------
    VisualOdom vo;
    std::vector<cv::Point2d> est_path;

    // GStreamer pipeline string for the Raspberry Pi camera on Jetson Nano
    std::string pipeline = "nvarguscamerasrc sensor-id=0 ! "
                           "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                           "nvvidconv flip-method=2 ! "
                           "video/x-raw, format=BGRx ! "
                           "videoconvert ! "
                           "video/x-raw, format=BGR ! appsink";

    // OpenCV video capture object
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    int frame_idx = 0;

    std::cout << "Starting..." << std::endl;
    
    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed." << std::endl;
            break;
        }

        std::cout << "Frame " << std::to_string(frame_idx) << std::endl;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Point2d pose;
        pose = vo.run(gray, frame_idx);
        est_path.push_back(pose);

        // Encode image as JPEG
        std::vector<uchar> buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70}; // adjust quality
        cv::imencode(".jpg", gray, buf, params);

        uint32_t img_size = buf.size();
        uint32_t img_size_net = htonl(img_size);
        if (!send_all(sock, &img_size_net, sizeof(img_size_net))) break;
        if (!send_all(sock, buf.data(), img_size)) break;
        if (!send_all(sock, &pose, sizeof(pose))) break;
        
        frame_idx++;
    }
    cap.release();
    close(sock);
    return 0;
}
