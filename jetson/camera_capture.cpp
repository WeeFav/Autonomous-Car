#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // GStreamer pipeline string for the Raspberry Pi camera on Jetson Nano
    std::string pipeline = "nvarguscamerasrc sensor-id=0 ! "
                           "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
                           "nvvidconv flip-method=0 ! "
                           "video/x-raw, format=BGRx ! "
                           "videoconvert ! "
                           "video/x-raw, format=BGR ! appsink";

    // OpenCV video capture object
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    cv::Mat frame;
    int counter = 0;
    while (true) {
        cap >> frame;

        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed." << std::endl;
            break;
        }

        // Show the frame
        std::string filename = std::to_string(counter) + ".png";
        cv::imwrite(filename, frame);
        counter++;

        // Exit on ESC key
        if (cv::waitKey(1) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
