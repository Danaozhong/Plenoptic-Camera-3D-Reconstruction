#ifndef _RANSAC_H_
#define _RANSAC_H_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/distances.h>


namespace sgMain
{
	namespace Segmentation
	{
		std::vector<size_t> CreateRandomSample(size_t numOfElements, size_t elementsToPick);

		int RANSAC2D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rest,
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &segmentedObjects,
			double treshold, unsigned int maxNumberOfIterations, unsigned int triesPerIteration);

		int RANSAC3D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rest,
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &segmentedObjects,
			float treshold, pcl::SacModel modelType = pcl::SACMODEL_PLANE, int methodType = pcl::SAC_RANSAC);
	}
}
#endif