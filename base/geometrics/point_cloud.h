/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* point_cloud.h
* Contains fucntions to handle, convert and analyze point clouds.
*/


#ifndef _POINT__CLOUD_H_
#define _POINT__CLOUD_H_

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "base/image.h"

namespace sgBase
{
	class PointCloudSizeInformation
	{
	private:
		int GetPointCloudSize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud, Eigen::Vector3f &min, Eigen::Vector3f &max);
		PointCloudSizeInformation();
	public:
		Eigen::Vector3f min;
		Eigen::Vector3f max;

		PointCloudSizeInformation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud);
	};

	namespace PointCloudHelper
	{
		std::vector<Eigen::Vector2f> PointCloudToMap2D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr NearestNeighbourSearch2D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const pcl::PointXYZRGB &centerPoint, double sqrDistanceTreshold);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map2DToPointCloud(const std::vector<Eigen::Vector2f> &map2d);

		int PointCloudFromPCDFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ> &cloud);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredPointCloudFromImage(const cv::Mat &colorImage);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredPointCloudFromImages(const cv::Mat &colorImage, const cv::Mat &depthImage);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudFromOpenCVList(const std::vector<cv::Point> &cvPointList, const cv::Mat &colorImage);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ScalePointCloudInZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double factor);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr FlatPointCloudInZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

		double AverageZValue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr StatisticalOutlierFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int mean, double threshold);

		inline pcl::PointXYZRGB PointXYZRGB(const Eigen::Vector3f &vector)
		{
			pcl::PointXYZRGB point;
			point.x = vector.x();
			point.y = vector.y();
			point.z = vector.z();
			point.r = 255;
			point.g = 255;
			point.b = 255;
			return point;
		}

		inline Eigen::Vector3f Vector3f(const pcl::PointXYZRGB &point)
		{
			return Eigen::Vector3f(point.x, point.y, point.z);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud(const std::vector<Eigen::Vector3f> &vectorList);
	}
	std::ostream& operator<<(std::ostream& os, const sgBase::PointCloudSizeInformation &pointCloud);
}
#endif