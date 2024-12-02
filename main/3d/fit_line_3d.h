#ifndef _FIT_LINE_3D_H_
#define _FIT_LINE_3D_H_

#include <ostream>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/common/distances.h>

#include "base/misc/histogram.h"
#include "base/geometrics/line_3d.h"


using namespace sgBase;

namespace sgMain
{

	enum Segmentation3DDepthFittingMethod
	{
		FITTING_MINIMUM_VARIANCE,
		FITTING_LEAST_SQUARES,
		FITTING_BEGIN_AND_END_POINT_AVERAGING,
		FITTING_NONE
	};

	class FitLine3D
	{
	protected:
		Segmentation3DDepthFittingMethod fittingMethod;
		bool isFitted;
		
		cv::Mat intrinsics;
		Line3D line2D;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloud;
		double radius;
		virtual void print(std::ostream& os) const;
	public:
		
		FitLine3D(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double radius);

		virtual ~FitLine3D() = 0;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloud;
		Line3D fittedLine;
		
		virtual int FitLine() = 0;
		
		friend std::ostream& operator<< (std::ostream& os, const FitLine3D& line);

		virtual double GetVariance() const;
		virtual Histogram GetHistogram(unsigned int numberOfBins) const;
		virtual bool IsFitted() const;

#if 0
		virtual void Render(Visualization3D &viewport3D, int contourIndex, int lineIndex) const;
#endif
	};

	std::ostream& operator<< (std::ostream& os, const FitLine3D& line);

	namespace FitLine3DHelper
	{
		int FitLineIntoPointCloudByAveragingBeginAndEndPoints(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double maxDistancePointToLine, Line3D &result);
		
		/**
		Fits a line given in 2D coordinates into a 3D point cloud using least-squares fitting.
		\param line2D The 2D line.
		\param output The calculated 3D line.
		\param intrinsic The matrix of intrinsic parameters of the camera, necessary to project the 2D line into 3D space
		\param depthCloud The point cloud to fit the line into
		\return 0 if successful, otherwise nonzero
		*/
		int FitLine2DIntoPointCloud3DLeastSquares(const Line3D &line2D, Line3D &output, const cv::Mat &intrinsic, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud);
		
		int FitLine2DIntoPointCloud3DMinimumVariance(const Line3D &line2D, Line3D &output, const cv::Mat &intrinsic, const std::vector<Eigen::Vector3f> &points, const std::vector<unsigned int> &pointImportance);


		int FitLine2DIntoPoints2DLeastSquares(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, Eigen::Vector2f &linePoint, Eigen::Vector2f &lineDirection);
		int FitLine2DIntoPoints2DLeastSquares(const std::vector<Eigen::Vector3f> &points, Eigen::Vector2f &linePoint, Eigen::Vector2f &lineDirection);
		int FitLine2DIntoPoints2DMinimumVariance(const std::vector<Eigen::Vector3f> &points, const std::vector<unsigned int> &pointImportance, Eigen::Vector2f &linePoint, Eigen::Vector2f &lineDirection);
	}
}

#endif