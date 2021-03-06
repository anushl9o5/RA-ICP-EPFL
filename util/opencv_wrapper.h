#pragma once
#ifdef WITH_OPENCV

#define NOMINMAX
#include <algorithm>
#include <limits>

#include "opencv2/core/core.hpp"       ///< cv::Mat
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp" ///< cv::imShow
#include "opencv2/imgproc/imgproc.hpp" ///< cvtConvert, cv::Circle, cv::Line
#include "opencv2/imgproc/types_c.h"   ///< CV_BGRA2RGBA
#endif

namespace cv{
    /// Gives a string representation for the cv::Mat type
    /// @example LOG(INFO) << cv::type2str(boundary_image.type());
    inline std::string type2str(int type) {
      std::string r;

      uchar depth = type & CV_MAT_DEPTH_MASK;
      uchar chans = 1 + (type >> CV_CN_SHIFT);

      switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
      }

      r += "C";
      r += (chans+'0');

      return r;
    }
} ///< cv::
