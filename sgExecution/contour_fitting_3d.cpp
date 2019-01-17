
#include <boost/thread.hpp>
#include "sgBase/visualization_3d.h"
#include "sgMain/3d/fit_line_3d.h"
#include "sgMain/3d/fit_line_3d/fit_line_3d_begin_end_point_averaging.h"
#include "sgMain/3d/fit_line_3d/fit_line_3d_least_squares.h"
#include "sgMain/3d/fit_line_3d/fit_line_3d_minimum_variance.h"

#include "sgMain/3d/transformation_3d.h"
#include "sgExecution/contour_fitting_3d.h"

namespace sgExecution
{
	ContourFitting3D::ContourFitting3D(const std::vector<Contour2D> &contours2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsics)
		:contours(contours2D), depthCloudInCameraSpace(depthCloud), matrixOfIntrinsicParameters(intrinsics)
	{
		this->depthCloudInWorldSpace = Transformation3D::TransformFromCameraToWorldCoordinates(this->depthCloudInCameraSpace, this->matrixOfIntrinsicParameters);
	}

	std::vector<Contour3D> ContourFitting3D::FitContours(Segmentation3DDepthFittingMethod fittingMethod, double radius)
	{
		std::vector<Contour3D> fittedContours;

		if (FITTING_NONE == fittingMethod)
		{
			return fittedContours;
		}

		for (size_t i = 0; i != contours.size(); i++)
		{
			// perform data analysis for first line
			Contour3D currentContour3D;
			for (size_t k = 0; k != contours[i].contourLines.size(); k++)
			{
				switch (fittingMethod)
				{
				case FITTING_BEGIN_AND_END_POINT_AVERAGING:
					{
						std::shared_ptr<FitLine3DBeginEndPointAveraging> lineFit(new FitLine3DBeginEndPointAveraging(contours[i].contourLines[k], this->depthCloudInCameraSpace, this->matrixOfIntrinsicParameters, radius));
						lineFit->FitLine();
						currentContour3D.contourLines.push_back(lineFit);
						break;
					}
				case FITTING_LEAST_SQUARES:
					{
						std::shared_ptr<FitLine3DLeastSquares> lineFit(new FitLine3DLeastSquares(contours[i].contourLines[k], this->depthCloudInCameraSpace, this->matrixOfIntrinsicParameters, radius));
						lineFit->FitLine();
						currentContour3D.contourLines.push_back(lineFit);
						break;
					}
				case FITTING_MINIMUM_VARIANCE:
					{
						std::shared_ptr<FitLine3DMinimumVariance> lineFit(new FitLine3DMinimumVariance(contours[i].contourLines[k], this->depthCloudInCameraSpace, this->matrixOfIntrinsicParameters, radius));
						lineFit->FitLine();
						currentContour3D.contourLines.push_back(lineFit);
						break;
					}
				default:
					{
						   // do nothing
						   break;
					}
				}
			}
			fittedContours.push_back(currentContour3D);
		}
		return fittedContours;		
	}
}