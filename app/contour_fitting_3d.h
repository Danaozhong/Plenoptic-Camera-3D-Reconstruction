#ifndef _CONTOUR_FITTING_3D_H_
#define _CONTOUR_FITTING_3D_H_

#include <ostream>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/common/distances.h>

#include "base/geometrics/line_3d.h"

#include "main/2d/contour_2d.h"
#include "main/3d/contour_3d.h"

using namespace sgBase;
using namespace sgMain;

namespace sgExecution
{
	class ContourFitting3D
	{
	private:
		std::vector<Contour2D> contours;
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloudInCameraSpace;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloudInWorldSpace;
		const cv::Mat matrixOfIntrinsicParameters;

	public:
		ContourFitting3D(const std::vector<Contour2D> &contours2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsics);

		std::vector<Contour3D> FitContours(Segmentation3DDepthFittingMethod fittingMethod, double radius);
	};
}

#endif