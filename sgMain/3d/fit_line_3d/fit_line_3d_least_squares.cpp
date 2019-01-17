
#include "sgMain/3d/transformation_3d.h"
#include "sgMain/3d/fit_line_3d/fit_line_3d_least_squares.h"

namespace sgMain
{
	FitLine3DLeastSquares::FitLine3DLeastSquares(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double radius)
		: FitLine3D(line2D, depthCloud, intrinsicParameters, radius)
	{}

	FitLine3DLeastSquares::~FitLine3DLeastSquares()
	{}

	int FitLine3DLeastSquares::FitLine()
	{
		this->isFitted = false;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloudInWorldCoordinates = Transformation3D::TransformFromCameraToWorldCoordinates(this->lineCloud, this->intrinsics);
		if (0 != FitLine3DHelper::FitLine2DIntoPointCloud3DLeastSquares(this->line2D, this->fittedLine, this->intrinsics, lineCloudInWorldCoordinates))
		{
			return -1;
		}
		this->isFitted = true;
		return 0;
	}


}