#include <opencv2/opencv.hpp>
#include <iostream>

void drawCube(cv::Mat &_image, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
              cv::InputArray _rvec, cv::InputArray _tvec, float length)
{
    std::vector<cv::Point3f> cubePoints;
    cubePoints.push_back(cv::Point3f(0, 0, 0));
    cubePoints.push_back(cv::Point3f(length, 0, 0));
    cubePoints.push_back(cv::Point3f(length, length, 0));
    cubePoints.push_back(cv::Point3f(0, length, 0));
    cubePoints.push_back(cv::Point3f(0, 0, -length));
    cubePoints.push_back(cv::Point3f(length, 0, -length));
    cubePoints.push_back(cv::Point3f(length, length, -length));
    cubePoints.push_back(cv::Point3f(0, length, -length));
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(cubePoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

    cv::line(_image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
    cv::line(_image, imagePoints[1], imagePoints[2], cv::Scalar(0, 0, 255), 3);
    cv::line(_image, imagePoints[2], imagePoints[3], cv::Scalar(0, 0, 255), 3);
    cv::line(_image, imagePoints[3], imagePoints[0], cv::Scalar(0, 0, 255), 3);

    cv::line(_image, imagePoints[4], imagePoints[5], cv::Scalar(0, 0, 255), 3);
    cv::line(_image, imagePoints[5], imagePoints[6], cv::Scalar(0, 0, 255), 3);
    cv::line(_image, imagePoints[6], imagePoints[7], cv::Scalar(0, 0, 255), 3);
    cv::line(_image, imagePoints[7], imagePoints[4], cv::Scalar(0, 0, 255), 3);

    cv ::line(_image, imagePoints[0], imagePoints[4], cv::Scalar(0, 0, 255), 3);
    cv ::line(_image, imagePoints[1], imagePoints[5], cv::Scalar(0, 0, 255), 3);
    cv ::line(_image, imagePoints[2], imagePoints[6], cv::Scalar(0, 0, 255), 3);
    cv ::line(_image, imagePoints[3], imagePoints[7], cv::Scalar(0, 0, 255), 3);
}

void drawAxis(cv::Mat &_image, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
              cv::InputArray _rvec, cv::InputArray _tvec, float length)
{
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));
    axisPoints.push_back(cv::Point3f(length, 0, 0));
    axisPoints.push_back(cv::Point3f(0, length, 0));
    axisPoints.push_back(cv::Point3f(0, 0, length));
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

    cv::line(_image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
    cv::line(_image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
    cv::line(_image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
}

int main()
{
    int board_width = 9, board_height = 6, num_imgs = 50;
    double square_size = 0.036;
    std::string out_file = "calibration.yaml";

    cv::Size boardSize(board_width, board_height);
    cv::Mat matImg, src_gray;

    // Specify backend
    cv::VideoCapture capture(0, cv::CAP_V4L2);
    capture.open(0);
    // cv::VideoCapture capture(0);
    // cv::VideoCapture capture("http://192.168.1.14:4747/video");

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;

    while (true)
    {
        capture >> matImg;
        if (matImg.empty())
            break;

        std::vector<cv::Point2f> corners;

        bool found = cv::findChessboardCorners(matImg, boardSize, corners, cv::CALIB_CB_FAST_CHECK);

        if (found)
        {
            cv::Size winSize = cv::Size(5, 5);
            cv::Size zeroZone = cv::Size(-1, -1);
            cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);

            cvtColor(matImg, src_gray, cv::COLOR_BGR2GRAY);

            cv::cornerSubPix(src_gray, corners, winSize, zeroZone, criteria);

            cv::drawChessboardCorners(matImg, boardSize, cv::Mat(corners), found);

            std::vector<cv::Point3f> obj;
            for (int i = 0; i < board_height; i++)
                for (int j = 0; j < board_width; j++)
                    obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));

            std::cout << "Found corners!" << std::endl;
            image_points.push_back(corners);
            object_points.push_back(obj);
        }
        cv::imshow("Camera", matImg);
        cv::waitKey(1);

        if (image_points.size() == num_imgs)
            break;
    }

    printf("Starting Calibration\n");
    cv::Mat K;
    cv::Mat D;
    std::vector<cv::Mat> rvecs, tvecs;

    int flag = 0;
    flag |= cv::CALIB_FIX_K4;
    flag |= cv::CALIB_FIX_K5;

    cv::calibrateCamera(object_points, image_points, matImg.size(), K, D, rvecs, tvecs, flag);

    while (true)
    {
        capture >> matImg;
        if (matImg.empty())
            break;

        std::vector<cv::Point2f> imagePoints;
        bool found = cv::findChessboardCorners(matImg, boardSize, imagePoints, cv::CALIB_CB_FAST_CHECK);

        if (found)
        {
            cv::drawChessboardCorners(matImg, boardSize, cv::Mat(imagePoints), found);

            std::vector<cv::Point3f> obj;
            for (int i = 0; i < board_height; i++)
                for (int j = 0; j < board_width; j++)
                    obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));

            cv::Mat rvec, tvec;
            cv::solvePnP(obj, imagePoints, K, D, rvec, tvec);

            drawAxis(matImg, K, D, rvec, tvec, square_size);
            //* Draw cube
            drawCube(matImg, K, D, rvec, tvec, square_size);

            std::cout << "Distance to chessboard: " << cv::norm(tvec) << std::endl;
        }

        cv::imshow("Camera", matImg);
        // Check for 'ESC' key press
        auto c = cv::waitKey(30);
        // If 'ESC' key is pressed, break the loop
        if (c == 27)
            break;
    }

    cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
    fs << "K" << K;
    fs << "D" << D;
    fs << "board_width" << board_width;
    fs << "board_height" << board_height;
    fs << "square_size" << square_size;
    printf("Done Calibration\n");

    return 0;
}