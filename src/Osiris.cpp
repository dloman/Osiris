#include "FilteredCircularBuffer.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <iostream>
#include <iomanip>
#include <thread>

using namespace std::literals;

std::atomic<bool> gIsRunning(true);

//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
auto Distance (const cv::Point& LineBegin, const cv::Point& LineEnd)
{
  return sqrt(
    pow(LineEnd.x - LineBegin.x, 2) +
    pow(LineEnd.y - LineBegin.y, 2));
}

//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
auto LineDistance(
  const cv::Point& LineBegin,
  const cv::Point& LineEnd,
  const cv::Point& Point)
{
  return std::abs(
    ((LineEnd.y - LineBegin.y) * Point.x) -
    ((LineEnd.x - LineBegin.x) * Point.y) +
    (LineEnd.x * LineBegin.y) -
    (LineEnd.y * LineBegin.x)) / Distance(LineBegin, LineEnd);
}

//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
void DrawRoundedRectangle(
  const cv::RotatedRect& Rectangle,
  const cv::Mat& Image,
  const cv::Scalar Color)
{
  std::array<cv::Point2f, 4> BoundingPoints;

  Rectangle.points(BoundingPoints.data());

  for(int i = 0; i < 4; ++i)
  {
    cv::line(
      Image,
      BoundingPoints[i],
      BoundingPoints[(i+1)%4],
      Color,
      2);
  }
}

//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
int main(int argc, const char** argv)
{
  std::deque<cv::Mat> Frames;

  FilteredCircularBuffer<10> BoundingBoxBuffer;

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

      Frames.push_back(MaskedFrame);

      if (Frames.size() > 30)
      {
        Frames.pop_front();
      }

      cv::imshow("masked frame", MaskedFrame);

      cv::Mat AveragedFrame = cv::Mat::zeros(MaskedFrame.rows, MaskedFrame.cols, CV_32FC3);

      for (auto& Frame : Frames)
      {
        cv::accumulate(Frame, AveragedFrame);
      }

      AveragedFrame = AveragedFrame / Frames.size();

      AveragedFrame.convertTo(AveragedFrame, CV_8U);

      cv::imshow("averaged frame", AveragedFrame);

      cv::Mat Gaussed;

      cv::Mat GreyFrame;

      cv::cvtColor(AveragedFrame, GreyFrame, cv::COLOR_BGR2GRAY);

      cv::Mat CannyFrame;

      cv::Canny(GreyFrame, CannyFrame, 250, 500, 3);

      cv::imshow("filtered", ColorFiltered);

      cv::imshow("canny", CannyFrame);

      std::vector<cv::Vec4i> Lines;
      std::vector<cv::Point> LinePoints;

      cv::HoughLinesP(CannyFrame, Lines, 1, CV_PI / 180, 10, 50, 10);

      cv::Mat HoughFrame;
      VideoFrame.copyTo(HoughFrame);

      cv::Mat FilteredFrame;
      VideoFrame.copyTo(FilteredFrame);

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

        LinePoints.emplace_back(Line[0], Line[1]);
        LinePoints.emplace_back(Line[2], Line[3]);
      }

       auto Temp = cv::minAreaRect(LinePoints);
       auto DidItWork = BoundingBoxBuffer.push_back(Temp);

       if (DidItWork)
       {
         DrawRoundedRectangle(Temp, HoughFrame, cv::Scalar(60, 255, 60));
       }
       else
       {
         DrawRoundedRectangle(Temp, HoughFrame, cv::Scalar(255, 255, 60));
       }


       DrawRoundedRectangle(BoundingBoxBuffer.GetValue(), FilteredFrame, cv::Scalar(255, 60, 60));

       std::vector<std::vector<cv::Point>> Contours;

       std::vector<cv::Point> AllContourPoints;

       std::vector<cv::Point> LineMatchedPoints;

       cv::findContours(
         CannyFrame,
        Contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_TC89_L1);

      auto LineMatchedContoursFrame = VideoFrame.clone();

      for (auto i = 0u; i < Contours.size(); ++i)
      {
        if (Contours[i].size() > 50)
        {
          cv::drawContours(VideoFrame, Contours, i, {255, 60, 60}, 2);

          if (std::any_of(
              Lines.begin(),
              Lines.end(),
              [Point = Contours[i][0]] (const cv::Vec4i& Line)
              {
                return LineDistance(
                  cv::Point(Line[0], Line[1]),
                  cv::Point(Line[2], Line[3]),
                  Point) < 1;
              }))
          {
            std::copy(
              Contours[i].begin(),
              Contours[i].end(),
              std::back_inserter(LineMatchedPoints));
            cv::drawContours(LineMatchedContoursFrame, Contours, i, {60, 255, 60}, 2);
          }
        }
      }

      cv::imshow("contour", VideoFrame);
      cv::imshow("Line matched contour", LineMatchedContoursFrame);
      cv::imshow("hough", HoughFrame);
      cv::imshow("filtered Frame", FilteredFrame);
      cv::waitKey(1);

    }
    else
    {
      VideoSource.open("/home/dloman/HackerspaceVideoData/Table.mkv");
    }
  }

  return 0;
}
