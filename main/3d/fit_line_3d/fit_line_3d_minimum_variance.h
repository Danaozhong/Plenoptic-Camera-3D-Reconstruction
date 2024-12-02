#ifndef _FIT_LINE_3D_MINIMUM_VARIANCE_H_
#define _FIT_LINE_3D_MINIMUM_VARIANCE_H_

#include "base/geometrics/line_3d.h"
#include "main/3d/fit_line_3d.h"

using namespace sgBase;

namespace sgMain
{
	class FitLine3DMinimumVariance : public FitLine3D
	{
	protected:
		int FitMinimumVariance();
	public:
		FitLine3DMinimumVariance(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double radius);
		virtual ~FitLine3DMinimumVariance();
		virtual int FitLine();

#if 0
		virtual void Render(Visualization3D &viewport3D, int contourIndex, int lineIndex) const;
#endif
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> averagePointClouds;
		std::vector<unsigned int> averagePointDensities;
		std::vector<Eigen::Vector3f> averageLinePoints;
		

		double stepSize;

	};
}

#endif