//==============================================================================
//
// Title:       point_cloud.cpp
// Purpose:     See header.
//
// Created on:  04/21/2014 at 9:45:01 PM by Clemens Zangl.
// Copyright:   University of Applied Sciences, Karlsruhe. All Rights Reserved.
//
//==============================================================================

#include <cassert>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "sgBase/geometrics/point_cloud.h" 


namespace sgBase
{
	int PointCloudSizeInformation::GetPointCloudSize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud, Eigen::Vector3f &min, Eigen::Vector3f &max)
	{
		auto minX = std::numeric_limits<float>::max();
		auto maxX = std::numeric_limits<float>::min();
		auto minY = std::numeric_limits<float>::max();
		auto maxY = std::numeric_limits<float>::min();
		auto minZ = std::numeric_limits<float>::max();
		auto maxZ = std::numeric_limits<float>::min();

		for (auto itr = pointCloud->begin(); itr != pointCloud->end(); itr++)
		{
			if (itr->x < minX) minX = itr->x;
			if (itr->x > maxX) maxX = itr->x;
			if (itr->y < minY) minY = itr->y;
			if (itr->y > maxY) maxY = itr->y;
			if (itr->z < minZ) minZ = itr->z;
			if (itr->z > maxZ) maxZ = itr->z;
		}
		min[0] = minX;
		min[1] = minY;
		min[2] = minZ;
		max[0] = maxX;
		max[1] = maxY;
		max[2] = maxZ;
		return 0;
	}

	PointCloudSizeInformation::PointCloudSizeInformation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud)
	{
		GetPointCloudSize(pointCloud, min, max);
	}

	std::ostream& sgBase::operator<<(std::ostream& os, const PointCloudSizeInformation &pointCloud)
	{
		os << "Min: " << std::endl << pointCloud.min << std::endl << "Max: " << pointCloud.max << std::endl;
		return os;
	}

	std::vector<Eigen::Vector2f> PointCloudHelper::PointCloudToMap2D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
	{
		std::vector<Eigen::Vector2f> map2D;
		for (auto itr = cloud->begin(); itr != cloud->end(); itr++)
		{
			Eigen::Vector2f point2f = Eigen::Vector2f((float)itr->x, (float)itr->y);
			map2D.push_back(point2f);
		}
		return map2D;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::NearestNeighbourSearch2D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const pcl::PointXYZRGB &centerPoint, double DistanceThreshold)
	{
		double sqrDistanceThreshold = DistanceThreshold * DistanceThreshold;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (auto itr = cloud->begin(); itr != cloud->end(); itr++)
		{
			double sqrDistance = (itr->x - centerPoint.x) * (itr->x - centerPoint.x) + (itr->y - centerPoint.y) * (itr->y - centerPoint.y);
			if (sqrDistance <= sqrDistanceThreshold)
			{
				pcl::PointXYZRGB currenPoint(*itr);
				resultCloud->push_back(currenPoint);
			}
		}
		return resultCloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::Map2DToPointCloud(const std::vector<Eigen::Vector2f> &map2D)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (auto itr = map2D.begin(); itr != map2D.end(); itr++)
		{
			pcl::PointXYZRGB point = pcl::PointXYZRGB(100, 100, 100);

			point.x = itr->x();
			point.y = itr->y();
			point.z = 0.0f;
			cloud->push_back(point);
		}
		return cloud;
	}

	int PointCloudHelper::PointCloudFromPCDFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>& cloud)
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, cloud) == -1)
		{
			return 0;
		}
		return -1;
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::ColoredPointCloudFromImage(const cv::Mat &inputImage)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		cv::Mat colorImage;
		colorImage.create(inputImage.size(), CV_8UC4);
		int numofChannels = inputImage.channels();
		if (1 == inputImage.channels())
		{
			// convert to four channel image
			cv::cvtColor(inputImage, colorImage, CV_GRAY2BGRA);
		}
		else
		{
			inputImage.copyTo(colorImage);
		}
		int numofChannels2 = colorImage.channels();
		assert(colorImage.channels() == 4);

		int bytesPerChannel = colorImage.channels();

		for (size_t i = 0; i < colorImage.size().width * colorImage.size().height; i++)
		{
			unsigned char r = colorImage.data[i * bytesPerChannel + 0];
			unsigned char g = colorImage.data[i * bytesPerChannel + 1];
			unsigned char b = colorImage.data[i * bytesPerChannel + 2];

			if (0 != r && 0 != g && 0 != b)
			{
				float u = (float)(i % colorImage.size().width) / (float)colorImage.size().width;
				float v = (float)(i / colorImage.size().width) / (float)colorImage.size().height;

				pcl::PointXYZRGB point(colorImage.data[i * bytesPerChannel + 0], colorImage.data[i * bytesPerChannel + 1], colorImage.data[i * bytesPerChannel + 2]);
				point.x = u;
				point.y = v;
				point.z = 0.0f;
				cloud->push_back(point);
			}
		}

		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::ColoredPointCloudFromImages(const cv::Mat &colorImage, const cv::Mat &depthImage)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		// depth image must be greyscale with two bytes per pixel
		int numOfChannels = depthImage.channels();

		assert(numOfChannels == 1);

		for (size_t i = 0; i < depthImage.size().width * depthImage.size().height; i++)
		{
			unsigned char currentPixel[2];
			currentPixel[1] = depthImage.data[2 * i + 0];
			currentPixel[0] = depthImage.data[2 * i + 1];

			if (0 == currentPixel[0] && 0 == currentPixel[1])
			{
				continue;
			}
			float positionX = (float)(i % depthImage.size().width);
			float positionY = (float)(i / depthImage.size().width);

			unsigned short max = std::numeric_limits<unsigned short>::max();
			unsigned short colorValue = (((unsigned short)currentPixel[0]) << 8) | (((unsigned short)currentPixel[1]) << 0);
			double normalizedColorValue = (double)colorValue / (double)max;


			if (normalizedColorValue < 0.5 || normalizedColorValue > 1.0)
			{
				throw;
			}
			double depthValue = 1.0 / (1.0 - normalizedColorValue);

			float u = positionX / (float)depthImage.size().width;
			float v = positionY / (float)depthImage.size().height;

			// get color from image
			Color regionColor = sgBase::ImageHelper::GetAverageColorValueAtPosition(colorImage, Eigen::Vector2i((int)positionX, (int)positionY), 2);

			pcl::PointXYZRGB point(regionColor.r(), regionColor.g(), regionColor.b());
			point.x = u;
			point.y = v;
			point.z = (float)depthValue;
			cloud->push_back(point);
		}

		return cloud;
	}

	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::PointCloudFromOpenCVList(const std::vector<cv::Point> &cvPointList, const cv::Mat &colorImage)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
		for (auto itr = cvPointList.begin(); itr != cvPointList.end(); itr++)
		{
			pcl::PointXYZRGB currentPoint(0, 0, 0);
			currentPoint.x = itr->x / (float)colorImage.size().width;
			currentPoint.y = itr->y / (float)colorImage.size().height;
			currentPoint.z = 0.0f;
			cloud->push_back(currentPoint);
		}
		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::ScalePointCloudInZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double factor)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (auto itr = cloud->begin(); itr != cloud->end(); itr++)
		{
			pcl::PointXYZRGB newPoint(*itr);
			newPoint.z *= (float)factor;
			resultPointCloud->push_back(newPoint);
		}
		return resultPointCloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::FlatPointCloudInZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
	{
		return PointCloudHelper::ScalePointCloudInZ(cloud, 0.0);
	}

	double PointCloudHelper::AverageZValue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
	{
		double zSum = 0.0;
		for (auto itr = cloud->begin(); itr != cloud->end(); itr++)
		{
			zSum += (double)(itr->z);
		}
		return zSum / (double)(cloud->size());
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::StatisticalOutlierFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int mean, double threshold)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(mean);
		sor.setStddevMulThresh(threshold);
		sor.filter(*cloudFiltered);

		return cloudFiltered;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudHelper::PointCloud(const std::vector<Eigen::Vector3f> &vectorList)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (auto itr = vectorList.begin(); itr != vectorList.end(); itr++)
		{
			newCloud->push_back(PointCloudHelper::PointXYZRGB(*itr));
		}
		return newCloud;
	}
}