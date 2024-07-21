#include <opencv2/opencv.hpp>
#include <iostream>

void drawAxis(cv::Mat &_image, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
              cv::InputArray _rvec, cv::InputArray _tvec, float length)
{
    // project axis points
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));
    axisPoints.push_back(cv::Point3f(length, 0, 0));
    axisPoints.push_back(cv::Point3f(0, length, 0));
    axisPoints.push_back(cv::Point3f(0, 0, length));
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

    // draw axis lines
    cv::line(_image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
    cv::line(_image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
    cv::line(_image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
}

int main()
{
    int board_width = 9, board_height = 6, num_imgs = 50;
    double square_size = 0.036;
    std::string out_file = "calibration.txt";

    cv::Size boardSize(board_width, board_height);
    cv::Mat matImg, src_gray;

    //* CAMARA
    // cv::VideoCapture capture("http://192.168.1.13:4747/video");
    cv::VideoCapture capture(0);

    // Checkerboard corner coordinates in the image
    std::vector<std::vector<cv::Point2f>> image_points;

    // Object points are the actual 3D coordinate of checkerboard points
    std::vector<std::vector<cv::Point3f>> object_points;

    //---------------------------------------------------------
    // Loop through video stream until we capture enough images (num_imgs) of checkerboard
    while (true)
    {
        capture >> matImg;
        if (matImg.empty())
        {
            break;
        }

        // Сheckerboard corner coordinates in the image
        std::vector<cv::Point2f> corners;

        //--- Here we find all the corner points of each image and their corresponding 3D world points ---
        //--- and prepare the corresponding vectors

        // Find all the checkerboard corners
        bool found = cv::findChessboardCorners(matImg, boardSize, corners, cv::CALIB_CB_FAST_CHECK);

        if (found)
        {

            cv::Size winSize = cv::Size(5, 5);
            cv::Size zeroZone = cv::Size(-1, -1);
            cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);

            // Convert image to grayscale images
            cvtColor(matImg, src_gray, cv::COLOR_BGR2GRAY);

            // Find more exact corner positions (more exact than integer pixels)
            cv::cornerSubPix(src_gray, corners, winSize, zeroZone, criteria);

            // This function helps to visualize the checkerboard corners found (optional)
            cv::drawChessboardCorners(matImg, boardSize, cv::Mat(corners), found);

            // Prepare the object_points and image_points vectors
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

        // Exit if we have enough images of checkerboard
        if (image_points.size() == num_imgs)
            break;
    }

    //---------------------------------------------------------
    //--- At this point we get all the necessary user input ---

    printf("Starting Calibration\n");
    // K contains the intrinsics
    cv::Mat K;

    // D contains the distortion coefficients
    cv::Mat D;

    // The rotation and translation vectors
    std::vector<cv::Mat> rvecs, tvecs;

    // Set flag to ignore higher order distortion coefficients k4 and k5.
    int flag = 0;
    flag |= cv::CALIB_FIX_K4;
    flag |= cv::CALIB_FIX_K5;

    cv::calibrateCamera(object_points, image_points, matImg.size(), K, D, rvecs, tvecs, flag);

    while (true)
    {
        capture >> matImg;
        if (matImg.empty())
        {
            break;
        }

        // Found chessboard corners
        std::vector<cv::Point2f> imagePoints;
        bool found = cv::findChessboardCorners(matImg, boardSize, imagePoints, cv::CALIB_CB_FAST_CHECK);

        if (found)
        {
            cv::drawChessboardCorners(matImg, boardSize, cv::Mat(imagePoints), found);

            std::vector<cv::Point3f> obj;
            for (int i = 0; i < board_height; i++)
                for (int j = 0; j < board_width; j++)
                    obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));

            // SolvePnP
            cv::Mat rvec, tvec;
            cv::solvePnP(obj, imagePoints, K, D, rvec, tvec);

            drawAxis(matImg, K, D, rvec, tvec, square_size);
            std::cout << "Distance to chessboard: " << cv::norm(tvec) << std::endl;
        }

        cv::imshow("Camera", matImg);
        auto c = cv::waitKey(30);

        // 27 == ESC
        // !! The window with the image displayed in it must have focus (i.e. be selected) when you press the key
        if (c == 27)
        {
            break;
        }
    }

    //--------------------------------------------------
    // Writing the data in a YAML file
    cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
    fs << "K" << K;
    fs << "D" << D;
    fs << "board_width" << board_width;
    fs << "board_height" << board_height;
    fs << "square_size" << square_size;
    printf("Done Calibration\n");

    return 0;
}