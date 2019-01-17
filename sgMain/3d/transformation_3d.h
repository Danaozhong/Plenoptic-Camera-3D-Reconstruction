#ifndef _TRANSFORMATION_3D_H_
#define _TRANSFORMATION_3D_H_

#include <cassert>
#include <math.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/distances.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>


#include "sgBase/camera_3d.h"
#include "sgBase/visualization_3d.h"

using namespace Eigen;

namespace sgMain
{
	namespace Transformation3D
	{
		pcl::PointXYZRGB TransformFromCameraToWorldCoordinates(const pcl::PointXYZRGB &point, const cv::Mat &intrinsic);
		/**
		* Transform a point cloud from camera space to world space.
		*
		*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TransformFromCameraToWorldCoordinates(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sourceCloud, const cv::Mat &intrinsic);

		pcl::PointXYZRGB TransformFromWordCoordinatesToCameraCoordinates(const pcl::PointXYZRGB &point, const cv::Mat &intrinsic);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TransformFromWordCoordinatesToCameraCoordinates(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sourceCloud, const cv::Mat &intrinsic);

		double DistancePointPlane(const Eigen::Vector3f &point, const Eigen::Vector3f &planePoint, const Eigen::Vector3f &planeNormal);

		Eigen::Vector3f ProjectPointOnPlane(const Eigen::Vector3f &point, const Eigen::Vector3f &planePoint, const Eigen::Vector3f &planeNormal);

		double sqrDistancePointLine(const Vector2f &point, const Vector2f &linePoint, const Vector2f &lineDir);

		int FindLineStartAndEndPoint(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &linePointCloud,
			const Eigen::Vector3f &linePoint, const Eigen::Vector3f &lineDirection,
			Eigen::Vector3f &lineBegin, Eigen::Vector3f &lineEnd);
	}

	namespace PlenopticCameraHelper
	{
		double CalculateRealDepthValue(double virtualDepth);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CalculateRealDepthValue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sourceCloud);
	}
}

#endif