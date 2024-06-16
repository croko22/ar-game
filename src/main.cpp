#include <opencv2/opencv.hpp>
#include <iostream>

int main(int, char **)
{
    // open the first webcam plugged in the computer
    cv::VideoCapture camera(0); // in linux check $ ls /dev/video0
    if (!camera.isOpened())
    {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam", cv::WINDOW_AUTOSIZE);

    // array to hold image
    cv::Mat frame;

    // display the frame until you press a key
    while (1)
    {
        // capture the next frame from the webcam
        camera >> frame;

        //* AIM MARK
        // calculate the center of the frame
        cv::Point center(frame.cols / 2, frame.rows / 2);
        // define the length of the aim mark lines
        int lineLength = 50;
        // draw vertical line
        cv::line(frame, cv::Point(center.x, center.y - lineLength / 2), cv::Point(center.x, center.y + lineLength / 2), cv::Scalar(0, 0, 255), 2);
        // draw horizontal line
        cv::line(frame, cv::Point(center.x - lineLength / 2, center.y), cv::Point(center.x + lineLength / 2, center.y), cv::Scalar(0, 0, 255), 2);

        // show the image on the window
        cv::imshow("Webcam", frame);
        // wait (10ms) for esc key to be pressed to stop
        if (cv::waitKey(10) == 27)
            break;
    }
    return 0;
}