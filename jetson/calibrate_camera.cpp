#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

int main() {
    int board_width = 10;  // number of inner corners per a chessboard row
    int board_height = 7; // number of inner corners per a chessboard column
    float square_size = 0.025f; // size of a square in mm
    cv::Size board_size(board_width, board_height);

    std::vector<cv::Point3f> objp; // world coordinates of 3D points of each corner
    for (int i=0; i<board_height; i++) {
        for (int j=0; j<board_width; j++) {
            objp.push_back(cv::Point3f(j*square_size, i*square_size, 0));
        }
    }

    // Arrays to store object points and image points from all images
    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;

    std::vector<std::string> images;
    std::string folder = "/home/car/Autonomous-Car/jetson/calib_imgs";
    for (const auto& fname : fs::directory_iterator(folder)) {
        auto path = fname.path();
        if (fs::is_regular_file(path)) {
            images.push_back(path.string());
        }
    }

    for (const auto& fname : images) {
        cv::Mat img = cv::imread(fname);
        if (img.empty()) {
            std::cerr << "Cannot read image: " << fname << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, board_size, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        
        if(found) {
            cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            imgpoints.push_back(corners);
            objpoints.push_back(objp);

            cv::drawChessboardCorners(img, board_size, corners, found);

            fs::path full_path(fname);
            cv::imwrite("/home/car/Autonomous-Car/jetson/calib/" + full_path.filename().string(), img);
            cv::waitKey(100);
        } 
        else {
            std::cout << "Chessboard not found in " << fname << std::endl;
        }
    }

    // Calibration
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    float error = cv::calibrateCamera(objpoints, imgpoints, cv::imread(images[0]).size(), 
                                      cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << "RMS error reported by calibrateCamera: " << error << std::endl;
    std::cout << "Camera matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Distortion coefficients:\n" << distCoeffs << std::endl;

    // Save results
    cv::FileStorage fs("/home/car/Autonomous-Car/jetson/calib/calibration.yml", cv::FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs.release();

    return 0;
}