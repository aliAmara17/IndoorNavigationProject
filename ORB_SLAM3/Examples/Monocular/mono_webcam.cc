#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp> // For VideoCapture
#include <System.h>

using namespace std;

int main(int argc, char **argv)
{
    // Check command line arguments
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_webcam path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // 1. Create ORB-SLAM3 System
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    // 2. Initialize with a test video file
    cv::VideoCapture cap("test_video.mp4"); // Use a video file instead

    // Check if the webcam opened successfully
    if(!cap.isOpened())
    {
        cerr << "ERROR: Could not open webcam!" << endl;
        return -1;
    }

    // Set camera resolution (optional, adjust based on your webcam)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cout << "Webcam opened successfully. Start grabbing frames..." << endl;

    // Main loop
    cv::Mat frame;
    while(true)
    {
        // Grab a new frame from the webcam
        cap >> frame;

        // Check if the frame is empty
        if(frame.empty())
        {
            cerr << "ERROR: Blank frame grabbed!" << endl;
            break;
        }

        // Pass the frame to ORB-SLAM3 for processing
        // TODO: We need to handle the timestamp properly.
        // For now, we'll use a simple counter.
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

        SLAM.TrackMonocular(frame, timestamp);

        // Display the frame (optional, for debugging)
        cv::imshow("ORB-SLAM3: Webcam Feed", frame);

        // Exit if ESC key is pressed
        char c = (char)cv::waitKey(1);
        if(c == 27) // ASCII code for ESC
            break;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory_Webcam.txt");

    // Close the webcam
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
