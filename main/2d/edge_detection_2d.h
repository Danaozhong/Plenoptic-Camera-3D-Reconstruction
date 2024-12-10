#ifndef _EDGE_DETECTION_2D_H_
#define _EDGE_DETECTION_2D_H_

#include <cassert>
#include <ostream>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/common/distances.h>

#include "base/geometrics/line_3d.h"


using namespace sgBase;

namespace sgMain
{
	namespace EdgeDetection2D
	{
		cv::Mat CannyEdgeDetector(const cv::Mat &sourceImage, int lowerTreshold);
	};
}

#endif