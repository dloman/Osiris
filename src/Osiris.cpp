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

  VideoSource.open("/home/dloman/HackerspaceVideoData/Table.mkv");

  if(!VideoSource.isOpened())
  {
    std::cerr << "ERROR: unable to open camera";

    return 1;
  }

  cv::Mat VideoFrame;


  while(gIsRunning)
  {
    VideoSource >> VideoFrame;

    if (VideoFrame.rows > 0 && VideoFrame.cols > 0)
    {
      cv::Rect RegionOfInterest(0, 0, 640, 450);

      VideoFrame = VideoFrame(RegionOfInterest);

      cv::Mat ColorFiltered;

      int RedMinimum = 128, GreenMinimum = 65, BlueMinimum = 26;

      int RedMaximum = 204, GreenMaximum = 192, BlueMaximum = 150;

      cv::Mat HsvFrame;

      cv::cvtColor(VideoFrame, HsvFrame, cv::COLOR_BGR2HSV);

      cv::inRange(
        VideoFrame,
        cv::Scalar(BlueMinimum, GreenMinimum, RedMinimum),
        cv::Scalar(BlueMaximum, GreenMaximum, RedMaximum),
        ColorFiltered);

      cv::Mat MaskedFrame;

      cv::bitwise_and(VideoFrame, VideoFrame, MaskedFrame, ColorFiltered);

      cv::imshow("masked frame", MaskedFrame);

      cv::Mat GreyFrame;

      cv::cvtColor(MaskedFrame, GreyFrame, cv::COLOR_BGR2GRAY);

      cv::Mat CannyFrame;

      cv::Canny(GreyFrame, CannyFrame, 250, 500, 3);

      cv::imshow("filtered", ColorFiltered);

      cv::imshow("canny", CannyFrame);

      std::vector<cv::Vec4i> Lines;

      cv::HoughLinesP(CannyFrame, Lines, 1, CV_PI / 180, 50, 100, 10);

      cv::Mat HoughFrame;
      VideoFrame.copyTo(HoughFrame);

      for (size_t i = 0; i < Lines.size(); i++)
      {
        cv::Vec4i Line = Lines[i];
        cv::line(
          HoughFrame,
          cv::Point(Line[0], Line[1]),
          cv::Point(Line[2], Line[3]),
          cv::Scalar(0, 0, 255),
          3,
          2);

      }

      std::vector<std::vector<cv::Point>> Contours;

      std::vector<cv::Point> AllContourPoints;

      cv::findContours(
        CannyFrame,
        Contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_TC89_L1);

      for (auto i = 0u; i < Contours.size(); ++i)
      {
        if (Contours[i].size() > 50)
        {
          std::copy(
            Contours[i].begin(),
            Contours[i].end(),
            std::back_inserter(AllContourPoints));
        }
        cv::drawContours(VideoFrame, Contours, i, {255, 60, 60}, 2);
      }

      auto BoundingBox = cv::minAreaRect(AllContourPoints);

      std::array<cv::Point2f, 4> BoundingPoints;

      BoundingBox.points(BoundingPoints.data());
      for(int i = 0; i < 4; ++i)
      {
        cv::line(
          VideoFrame,
          BoundingPoints[i],
          BoundingPoints[(i+1)%4],
          cv::Scalar(60,255,60),
          2);
      }

      cv::imshow("contour", VideoFrame);
      cv::imshow("hough", HoughFrame);
      cv::waitKey(3);

    }
    else
    {
      VideoSource.open("/home/dloman/HackerspaceVideoData/Table.mkv");
    }
  }

  return 0;
}
