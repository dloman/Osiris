#include "FilteredCircularBuffer.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <deque>
#include <iostream>
#include <iomanip>
#include <thread>

using namespace std::literals;

std::atomic<bool> gIsRunning(true);

std::array<std::vector<double>, 3> gCleanTable{{
  {.077694, .089892, .303642, .305637, .144795, .056087, .015366, .006887},
  {.071506, .048397, .057974, .283089, .421860, .097151, .016917, .003106},
  {.077833, .042354, .053092, .052188, .239345, .499132, .035480, .000575}}};

//-----------------------------------------------------------------------------
//------------------------------------------------------------------------------
template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
  return v < lo ? lo : hi < v ? hi : v;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
double ComputeScore(std::array<std::vector<double>, 3> Input)
{
  double Score = 0.0;

  for (auto i = 0; i < 3; ++i)
  {
    for (auto j = 0; j < 8; ++j)
    {
      Score += std::abs(Input[i][j] -gCleanTable[i][j]);
    }
  }
  return 33 * Score;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
auto Distance(const cv::Point& LineBegin, const cv::Point& LineEnd)
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
//-----------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------
std::array<std::vector<double>, 3> ComputeColorHistogram(
  const cv::Mat& Frame,
  unsigned NumberOfBins = 8)
{
  std::array<std::vector<double>, 3> ColorHistogram;

  for (auto& Histogram : ColorHistogram)
  {
    Histogram.assign(NumberOfBins, 0u);
  }

  for (auto i = 0; i < Frame.rows; ++i)
  {
    for (auto j = 0; j < Frame.cols; ++j)
    {
      const auto& Pixel = Frame.at<cv::Vec3b>(i, j);

      ColorHistogram[0][Pixel[0] / 256.0 * NumberOfBins]++;
      ColorHistogram[1][Pixel[1] / 256.0 * NumberOfBins]++;
      ColorHistogram[2][Pixel[2] / 256.0 * NumberOfBins]++;
    }
  }

  const double NumberOfPixels = Frame.rows * Frame.cols;

  for (auto i = 0u; i < NumberOfBins; ++i)
  {
    ColorHistogram[0][i] /= NumberOfPixels;
    ColorHistogram[1][i] /= NumberOfPixels;
    ColorHistogram[2][i] /= NumberOfPixels;
  }
  return ColorHistogram;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
cv::Point2i GetPoint(const cv::Point2i& Input, const cv::Size& Size)
{
  cv::Point2i Output;

  Output.x = clamp(Input.x, 0, Size.width);

  Output.y = clamp(Input.y, 0, Size.height);

  return Output;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
cv::Mat GetImageChip(const cv::Mat Image, cv::RotatedRect BoundingBox)
{
  std::array<cv::Point2f, 4> BoundingPoints;

  BoundingBox.points(BoundingPoints.data());

  auto TopLeft = GetPoint(BoundingPoints[0], Image.size());

  auto BottomRight = GetPoint(BoundingPoints[2], Image.size());

  return cv::Mat(Image, cv::Rect(TopLeft, BottomRight));
}

//------------------------------------------------------------------------------
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

      cv::Mat AveragedFrame = cv::Mat::zeros(MaskedFrame.rows, MaskedFrame.cols, CV_32FC3);

      for (auto& Frame : Frames)
      {
        cv::accumulate(Frame, AveragedFrame);
      }

      AveragedFrame = AveragedFrame / Frames.size();

      AveragedFrame.convertTo(AveragedFrame, CV_8U);

      cv::Mat GreyFrame;

      cv::cvtColor(AveragedFrame, GreyFrame, cv::COLOR_BGR2GRAY);

      cv::Mat CannyFrame;

      cv::Canny(GreyFrame, CannyFrame, 250, 500, 3);

      std::vector<cv::Vec4i> Lines;
      std::vector<cv::Point> LinePoints;

      cv::HoughLinesP(CannyFrame, Lines, 1, CV_PI / 180, 10, 50, 10);

      cv::Mat HoughFrame;
      VideoFrame.copyTo(HoughFrame);

      cv::Mat FilteredFrame;
      VideoFrame.copyTo(FilteredFrame);

      cv::Mat LabFrame;
      VideoFrame.copyTo(LabFrame);
      cv::cvtColor(AveragedFrame, LabFrame, cv::COLOR_BGR2Luv);

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

      if (LinePoints.size())
      {
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
      }

      auto CurrentTablePosition = BoundingBoxBuffer.GetValue();

      DrawRoundedRectangle(CurrentTablePosition, FilteredFrame, cv::Scalar(255, 60, 60));

      auto ColorHistogram = ComputeColorHistogram(
        GetImageChip(VideoFrame, CurrentTablePosition));

      std::cout << ComputeScore(ColorHistogram) << '\n';


      //cv::imshow("averaged", AveragedFrame);
      //cv::imshow("hough", HoughFrame);
      cv::imshow("filtered Frame", FilteredFrame);
      cv::waitKey(1);

    }
    else
    {
      return 0;
      VideoSource.open("/home/dloman/HackerspaceVideoData/Table.mkv");
    }
  }

  return 0;
}
