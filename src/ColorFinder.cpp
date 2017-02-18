#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <stdlib.h>

int low_r=128, low_g=65, low_b=26;
int high_r=204, high_g=192, high_b=150;

void on_low_r_thresh_trackbar(int, void *)
{
    low_r = std::min(high_r-1, low_r);
    cv::setTrackbarPos("Low R","Object Detection", low_r);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void on_high_r_thresh_trackbar(int, void *)
{
    high_r = std::max(high_r, low_r+1);
    cv::setTrackbarPos("High R", "Object Detection", high_r);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void on_low_g_thresh_trackbar(int, void *)
{
    low_g = std::min(high_g-1, low_g);
    cv::setTrackbarPos("Low G","Object Detection", low_g);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void on_high_g_thresh_trackbar(int, void *)
{
    high_g = std::max(high_g, low_g+1);
    cv::setTrackbarPos("High G", "Object Detection", high_g);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void on_low_b_thresh_trackbar(int, void *)
{
    low_b= std::min(high_b-1, low_b);
    cv::setTrackbarPos("Low B","Object Detection", low_b);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void on_high_b_thresh_trackbar(int, void *)
{
    high_b = std::max(high_b, low_b+1);
    cv::setTrackbarPos("High B", "Object Detection", high_b);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
int main()
{
    cv::Mat frame, frame_threshold;
    cv::VideoCapture cap;
    cap.open("/home/dloman/HackerspaceVideoData/Table.mkv");
    cv::namedWindow("Video Capture", cv::WINDOW_NORMAL);
    cv::namedWindow("Object Detection", cv::WINDOW_NORMAL);
    //-- Trackbars to set thresholds for RGB values
    cv::createTrackbar("Low R","Object Detection", &low_r, 255, on_low_r_thresh_trackbar);
    cv::createTrackbar("High R","Object Detection", &high_r, 255, on_high_r_thresh_trackbar);
    cv::createTrackbar("Low G","Object Detection", &low_g, 255, on_low_g_thresh_trackbar);
    cv::createTrackbar("High G","Object Detection", &high_g, 255, on_high_g_thresh_trackbar);
    cv::createTrackbar("Low B","Object Detection", &low_b, 255, on_low_b_thresh_trackbar);
    cv::createTrackbar("High B","Object Detection", &high_b, 255, on_high_b_thresh_trackbar);
    while((char)cv::waitKey(10)!='q'){
        cap>>frame;
        //-- Detect the object based on RGB Range Values
        cv::inRange(frame,cv::Scalar(low_b,low_g,low_r), cv::Scalar(high_b,high_g,high_r),frame_threshold);
        if(frame.empty() || frame_threshold.empty())
        {
          cap.open("/home/dloman/HackerspaceVideoData/Table.mkv");
          continue;
        }

        //-- Show the frames
        cv::imshow("Video Capture",frame);
        cv::imshow("Object Detection",frame_threshold);
    }
    return 0;
}
