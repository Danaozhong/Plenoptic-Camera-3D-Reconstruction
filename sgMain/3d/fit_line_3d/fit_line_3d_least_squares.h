#ifndef _FIT_LINE_3D_LEAST_SQUARES_H_
#define _FIT_LINE_3D_LEAST_SQUARES_H_

#include "sgBase/geometrics/line_3d.h"
#include "sgMain/3d/fit_line_3d.h"

using namespace sgBase;

namespace sgMain
{
	class FitLine3DLeastSquares : public FitLine3D
	{
	protected:
	public:
		FitLine3DLeastSquares(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double radius);
		virtual ~FitLine3DLeastSquares();
		virtual int FitLine();
	};
}

#endif