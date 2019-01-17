#ifndef _FIT_LINE_3D_BEGIN_END_POINT_AVERAGING_H_
#define _FIT_LINE_3D_BEGIN_END_POINT_AVERAGING_H_

#include "sgBase/geometrics/line_3d.h"
#include "sgMain/3d/fit_line_3d.h"

using namespace sgBase;

namespace sgMain
{
	class FitLine3DBeginEndPointAveraging : public FitLine3D
	{
	public:
		FitLine3DBeginEndPointAveraging(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double radius);
		virtual ~FitLine3DBeginEndPointAveraging();
		virtual int FitLine();
	};
}

#endif



