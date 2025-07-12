#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    // Open a video file
    cv::VideoCapture cap("/home/marvin/Autonomous-Car/laptop/tcp/output.avi");
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open video file" << std::endl;
        return -1;
    }

    cv::Mat frame;
    int num = 1;

    while (true) {
        cap >> frame; // read next frame
        if (frame.empty()) {
            std::cout << "Empty frame" << std::endl;
            break;
        }

        cv::imshow("Video Replay", frame);

        // Wait 30 ms between frames, exit on ESC
        int key_press = cv::waitKey(125);
        if (key_press == 27) {
            break;
        }
        else if (key_press == 'q') {
            std::string fname = "calib" + std::to_string(num) + ".jpg";
            cv::imwrite(fname, frame);
            num++;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}