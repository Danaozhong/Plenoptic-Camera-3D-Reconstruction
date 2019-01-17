
#include "sgMain/3d/transformation_3d.h"
#include "sgMain/3d/fit_line_3d/fit_line_3d_begin_end_point_averaging.h"

namespace sgMain
{
	FitLine3DBeginEndPointAveraging::FitLine3DBeginEndPointAveraging(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double radius)
		: FitLine3D(line2D, depthCloud, intrinsicParameters, radius)
	{}
	
	FitLine3DBeginEndPointAveraging::~FitLine3DBeginEndPointAveraging()
	{}

	int FitLine3DBeginEndPointAveraging::FitLine()
	{

		Line3D line2DShortend(this->line2D.p1 + 0.0 * this->line2D.Length() * this->line2D.Direction(), this->line2D.p1 + 1.0 * this->line2D.Length() * this->line2D.Direction());
		
		// create simple fitting
		if (0 != FitLine3DHelper::FitLineIntoPointCloudByAveragingBeginAndEndPoints(line2DShortend, this->depthCloud, this->intrinsics, radius, this->fittedLine))
		{
			return -1;
		}
		this->isFitted = true;
		return 0;
	}


}