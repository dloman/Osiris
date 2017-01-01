#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <atomic>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>

std::atomic<bool> gIsRunning(true);

using namespace std::literals;

//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
  std::string OutputDirectory("");

  if (argc > 1)
  {
    OutputDirectory = argv[1];
  }
  cv::VideoCapture VideoSource;

  VideoSource.open(1);

  if(!VideoSource.isOpened())
  {
    std::cerr << "ERROR: unable to open camera";

    return 1;
  }

  cv::Mat VideoFrame;


  while(gIsRunning)
  {
    VideoSource >> VideoFrame;

    cv::Mat GreyFrame;
    cv::cvtColor(VideoFrame, GreyFrame, cv::COLOR_BGR2GRAY);

    cv::Mat CannyFrame;

    cv::Canny(GreyFrame, CannyFrame, 100, 200, 3);

    cv::imshow("canny", CannyFrame);

    std::vector<std::vector<cv::Point>> Contours;

    cv::findContours(
      CannyFrame,
      Contours,
      cv::RETR_TREE,
      cv::CHAIN_APPROX_SIMPLE);

    cv::Mat HoughFrame;
    VideoFrame.copyTo(HoughFrame);

    for (auto i = 0u; i < Contours.size(); ++i)
    {
      cv::drawContours(VideoFrame, Contours, i, {255, 60, 60}, 2);
    }

    cv::imshow("fuck", VideoFrame);

    std::vector<cv::Vec4i> Lines;

    cv::HoughLinesP(CannyFrame, Lines, 1, CV_PI / 180, 50, 50, 10);

    for (size_t i = 0; i < Lines.size(); i++)
    {
      cv::Vec4i l = Lines[i];
      cv::line(
        HoughFrame,
        cv::Point(l[0], l[1]),
        cv::Point(l[2], l[3]),
        cv::Scalar(0, 0, 255),
        3,
        2);

    }

    cv::imshow("hough", HoughFrame);
    cv::waitKey(3);
    //std::this_thread::sleep_for(1s);

  }

  return 0;
}
