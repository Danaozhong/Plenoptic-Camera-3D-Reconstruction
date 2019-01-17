#include <cassert>
#include <math.h>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <pcl/common/distances.h>

#include "sgBase/geometrics/point_cloud.h"
#include "sgMain/3d/transformation_3d.h"

using namespace sgBase;
using namespace Eigen;

namespace sgMain
{
	pcl::PointXYZRGB Transformation3D::TransformFromCameraToWorldCoordinates(const pcl::PointXYZRGB &point, const cv::Mat &intrinsic)
	{
		// calculate camera space coordinates by undoing the camera perspective projection (multiply x' and y' values with z)
		float depth = point.z;
		float x = (point.x - intrinsic.at<double>(0, 2)) / intrinsic.at<double>(0, 0) * depth;
		float y = (point.y - intrinsic.at<double>(1, 2)) / intrinsic.at<double>(1, 1) * depth;

		pcl::PointXYZRGB currentPoint(point);
		currentPoint.x = x;
		currentPoint.y = y;
		currentPoint.z = depth;
		return currentPoint;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Transformation3D::TransformFromCameraToWorldCoordinates(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sourceCloud, const cv::Mat &intrinsic)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFinal(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (pcl::PointCloud<pcl::PointXYZRGB>::iterator itr = sourceCloud->begin(); itr != sourceCloud->end(); itr++)
		{
			cloudFinal->push_back(TransformFromCameraToWorldCoordinates(*itr, intrinsic));
		}
		return cloudFinal;
	}


	pcl::PointXYZRGB Transformation3D::TransformFromWordCoordinatesToCameraCoordinates(const pcl::PointXYZRGB &point, const cv::Mat &intrinsic)
	{
		// calculate camera space coordinates by multiplying with the projection matrix and dividing by z 
		float depth = point.z;
		float u = point.x * intrinsic.at<double>(0, 0) / depth + intrinsic.at<double>(0, 2);
		float v = point.y * intrinsic.at<double>(1, 1) / depth + intrinsic.at<double>(1, 2);

		pcl::PointXYZRGB currentPoint(point);
		currentPoint.x = u;
		currentPoint.y = v;
		currentPoint.z = depth;
		return currentPoint;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Transformation3D::TransformFromWordCoordinatesToCameraCoordinates(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sourceCloud, const cv::Mat &intrinsic)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFinal(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (pcl::PointCloud<pcl::PointXYZRGB>::iterator itr = sourceCloud->begin(); itr != sourceCloud->end(); itr++)
		{
			cloudFinal->push_back(TransformFromWordCoordinatesToCameraCoordinates(*itr, intrinsic));
		}
		return cloudFinal;
	}

	double Transformation3D::DistancePointPlane(const Eigen::Vector3f &point, const Eigen::Vector3f &planePoint, const Eigen::Vector3f &planeNormal)
	{
		return planeNormal.dot(planePoint - point);
	}

	Eigen::Vector3f Transformation3D::ProjectPointOnPlane(const Eigen::Vector3f &point, const Eigen::Vector3f &planePoint, const Eigen::Vector3f &planeNormal)
	{
		double distance = Transformation3D::DistancePointPlane(point, planePoint, planeNormal);
		return point + distance * planeNormal;
	}

	double Transformation3D::sqrDistancePointLine(const Vector2f &point, const Vector2f &linePoint, const Vector2f &lineDir)
	{
		return pcl::sqrPointToLineDistance(Vector4f(point.x(), point.y(), 0.0f, 0.0f), Vector4f(linePoint.x(), linePoint.y(), 0.0f, 0.0f), Vector4f(lineDir.x(), lineDir.y(), 0.0f, 0.0f));
	}

	int Transformation3D::FindLineStartAndEndPoint(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &linePointCloud,
		const Eigen::Vector3f &linePoint, const Eigen::Vector3f &lineDirection,
		Eigen::Vector3f &lineBegin, Eigen::Vector3f &lineEnd)
	{
		if (linePointCloud->size() < 10)
		{
			return -2;
		}
		float maxPos = std::numeric_limits<float>::min();
		float minPos = std::numeric_limits<float>::max();

		for (auto itr = linePointCloud->begin(); itr != linePointCloud->end(); itr++)
		{
			Eigen::Vector3f currentPointVector = Eigen::Vector3f(itr->x, itr->y, itr->z) - linePoint;

			float currentPos = currentPointVector.dot(lineDirection) / lineDirection.norm();
			if (maxPos < currentPos)
			{
				maxPos = currentPos;
			}
			if (minPos > currentPos)
			{
				minPos = currentPos;
			}
		}

		// calculate final outer line points
		lineBegin = linePoint + minPos * lineDirection;
		lineEnd = linePoint + maxPos * lineDirection;
		return 0;
	}

	double PlenopticCameraHelper::CalculateRealDepthValue(double virtualDepth)
	{
		double c2 = -1598.2;
		double c1 = -36.389;
		double c0 = 0.54795;
		return (virtualDepth * c1 + c2) / (1.0 - virtualDepth * c0) + 2200.0;
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlenopticCameraHelper::CalculateRealDepthValue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &sourceCloud)
	{

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFinal(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (pcl::PointCloud<pcl::PointXYZRGB>::iterator itr = sourceCloud->begin(); itr != sourceCloud->end(); itr++)
		{
			pcl::PointXYZRGB newPoint(*itr);
			newPoint.z = PlenopticCameraHelper::CalculateRealDepthValue(newPoint.z);
			cloudFinal->push_back(newPoint);
		}
		return cloudFinal;
	}

	

}
