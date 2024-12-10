
#include "main/3d/transformation_3d.h"
#include "main/3d/fit_line_3d/fit_line_3d_minimum_variance.h"

namespace sgMain
{
	FitLine3DMinimumVariance::FitLine3DMinimumVariance(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double radius)
		: FitLine3D(line2D, depthCloud, intrinsicParameters, radius)
	{}

	FitLine3DMinimumVariance::~FitLine3DMinimumVariance()
	{}

	int FitLine3DMinimumVariance::FitLine()
	{
		// first, perform an initial fit
		if (0 == FitMinimumVariance())
		{
			this->isFitted = true;
			return 0;
		}
		this->isFitted = false;
		return -1;
	}

	int FitLine3DMinimumVariance::FitMinimumVariance()
	{
		// calculate depth average at different line positions
		unsigned int numberOfDivisions = (unsigned int)(this->line2D.Length() * 100.0);
		this->stepSize = this->line2D.Length() / (double)numberOfDivisions;

		this->averageLinePoints.clear();
		this->averagePointDensities.clear();
		
		for (unsigned int i = 0; i != numberOfDivisions + 1; i++)
		{
			double d = (double)i * stepSize;
			pcl::PointXYZRGB currentPoint = PointCloudHelper::PointXYZRGB(this->line2D.p1 + d * this->line2D.Direction());

			// perform radius search
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPointCloud = PointCloudHelper::NearestNeighbourSearch2D(this->lineCloud, currentPoint, this->radius);
			//...and project back into 3D 
			if (0 == currentPointCloud->size())
			{
				continue;
			}
			currentPointCloud = Transformation3D::TransformFromCameraToWorldCoordinates(currentPointCloud, this->intrinsics);
			currentPoint.z = PointCloudHelper::AverageZValue(currentPointCloud);
			this->averagePointClouds.push_back(currentPointCloud);

			currentPoint = Transformation3D::TransformFromCameraToWorldCoordinates(currentPoint, this->intrinsics);
			this->averageLinePoints.push_back(PointCloudHelper::Vector3f(currentPoint));
		}

		for (unsigned int i = 0; i != this->averageLinePoints.size(); i++)
		{
			this->averagePointDensities.push_back(this->averagePointClouds[i]->size());
		}

		// fit line into point cloud
		if (0 != FitLine3DHelper::FitLine2DIntoPointCloud3DMinimumVariance(line2D, this->fittedLine, this->intrinsics, this->averageLinePoints, this->averagePointDensities))
		{
			return -1;
		}

		return 0;
	}

#if 0
	void FitLine3DMinimumVariance::Render(Visualization3D &viewport3D, int contourIndex, int lineIndex) const
	{
		// render usual lines
		FitLine3D::Render(viewport3D, contourIndex, lineIndex);

		// and add the average point data
		Color currentColor = ColorHelper::GetColorFromIndex(contourIndex);
		viewport3D.AddPointCloudToViewPort(1, 0, PointCloudHelper::PointCloud(this->averageLinePoints), "average_points_" + std::to_string((long long)contourIndex) + "_" + std::to_string((long long)lineIndex), currentColor);
	}
#endif
}
