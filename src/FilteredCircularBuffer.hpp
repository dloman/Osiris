#pragma once

#include <opencv2/core/types.hpp>
#include <deque>
#include <numeric>
#include <stdlib.h>
#include <tuple>

#include <iostream>

template <size_t Size>
class FilteredCircularBuffer
{
  public:

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    FilteredCircularBuffer() = default;

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    auto begin()
    {
      return mDeque.begin();
    }

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    auto end()
    {
      return mDeque.end();
    }

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    size_t size()
    {
      return mDeque.size();
    }

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    cv::RotatedRect GetValue()
    {
      auto [AverageAngle, AverageCenter, AverageSize] = GetAverages();

      return cv::RotatedRect(AverageCenter, AverageSize, AverageAngle);
    }

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    bool push_back(const cv::RotatedRect Rectangle)
    {
      bool Return = false;
      if (mDeque.size() == Size)
      {

        auto [AverageAngle, AverageCenter, AverageSize] = GetAverages();

        auto [StdDevAngle, StdDevCenter, StdDevSize] =
          GetStdDevs(AverageAngle, AverageCenter, AverageSize);

        auto CenterDeviation = AverageCenter - Rectangle.center;
        CenterDeviation.x = std::abs(CenterDeviation.x);
        CenterDeviation.y = std::abs(CenterDeviation.y);

        cv::Point2f SizeDeviation(
          std::abs(AverageSize.width - Rectangle.size.width),
          std::abs(AverageSize.height - Rectangle.size.height));

        if (
          std::abs(Rectangle.angle - AverageAngle) < 4 * StdDevAngle &&
          CenterDeviation.x < 4 * StdDevCenter.x &&
          CenterDeviation.y < 4 * StdDevCenter.y &&
          SizeDeviation.x <  4 * StdDevSize.width &&
          SizeDeviation.y < 4 * StdDevSize.height)
        {
          mDeque.push_back(Rectangle);
          Return = true;
        }

        //std::cout
        //<< "angle dev = " << std::abs(Rectangle.angle - AverageAngle)
        //<< " std angle =" <<3*StdDevAngle << '\n'
        //<< " Center x = " << CenterDeviation.x << " stddev center x = " << 4*StdDevCenter.x << '\n'
        //<< " dev center y = " << CenterDeviation.y << " stddev center y=" << 4*StdDevCenter.y << '\n'
        //<< " size width = " << SizeDeviation.x <<  " stdeb size width = " << 4*StdDevSize.width << '\n'
        //<< " size height = " << SizeDeviation.y << " stddev height = " << 4*StdDevSize.height << std::endl;
        //auto i = 5;

      }
      else
      {
        Return = true;
        mDeque.push_back(Rectangle);
      }

      if (mDeque.size() > Size)
      {
        mDeque.pop_front();
      }
      return Return;
    }

  private:

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    std::tuple<float, cv::Point2f, cv::Size2f> GetAverages()
    {
      float AverageAngle = 0.0f;

      cv::Point2f AverageCenter(0.0f, 0.0f);

      cv::Point2f AverageSize(0.0f, 0.0f);

      for (const auto& Rectangle : mDeque)
      {
        AverageAngle += Rectangle.angle;

        AverageCenter += Rectangle.center;

        AverageSize += cv::Point2f(Rectangle.size.width, Rectangle.size.height);
      }

      const auto& CurrentSize = mDeque.size();

      return std::make_tuple(
        AverageAngle/CurrentSize,
        AverageCenter/static_cast<float>(CurrentSize),
        AverageSize/static_cast<float>(CurrentSize));
    }

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    std::tuple<float, cv::Point2f, cv::Size2f> GetStdDevs(
      const float AverageAngle,
      const cv::Point2f AverageCenter,
      const cv::Size2f AverageSize)
    {
      cv::Point2f TempSize(AverageSize.width, AverageSize.height);

      float StdDevAngle = 0.0f;

      cv::Point2f StdDevCenter(0.0f, 0.0f);

      cv::Point2f StdDevSize(0.0f, 0.0f);

      for (const auto& Rectangle : mDeque)
      {
        StdDevAngle += std::abs(AverageAngle - Rectangle.angle);

        auto TempPoint = (AverageCenter - Rectangle.center);
        TempPoint.x = std::abs(TempPoint.x);
        TempPoint.y = std::abs(TempPoint.y);
        StdDevCenter += TempPoint;

        TempPoint = TempSize - cv::Point2f(Rectangle.size.width, Rectangle.size.height);
        TempPoint.x = std::abs(TempPoint.x);
        TempPoint.y = std::abs(TempPoint.y);
        StdDevSize += TempPoint;
      }

      const auto& CurrentSize = mDeque.size();

      return std::make_tuple(
        StdDevAngle/CurrentSize,
        StdDevCenter/static_cast<float>(CurrentSize),
        StdDevSize/static_cast<float>(CurrentSize));
    }

    std::deque<cv::RotatedRect> mDeque;
};
