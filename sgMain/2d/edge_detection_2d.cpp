#include <cassert>
#include <ostream>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/common/distances.h>

#include "sgBase/geometrics/line_3d.h"
#include "sgMain/2d/edge_detection_2d.h"

using namespace sgBase;

namespace sgMain
{
	cv::Mat EdgeDetection2D::CannyEdgeDetector(const cv::Mat &sourceImage, int lowerTreshold)
	{
		cv::Mat sourceImageGray;
		cv::Mat detectedEdges;
		int upperTreshold = lowerTreshold * 3;
		int cannyKernelSize = 3;

		/// Convert the image to grayscale
		cv::cvtColor(sourceImage, sourceImageGray, CV_BGR2GRAY);

		/// Reduce noise with a kernel 3x3
		cv::blur(sourceImageGray, detectedEdges, cv::Size(4, 4));

		/// Canny detector
		cv::Canny(detectedEdges, detectedEdges, lowerTreshold, upperTreshold, cannyKernelSize);

		cv::Mat output;
		output.create(sourceImage.size(), sourceImage.type());
		output = cv::Scalar::all(0);
		sourceImage.copyTo(output, detectedEdges);
		return output;
	}
}