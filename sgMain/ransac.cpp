#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "sgMain/3d/transformation_3d.h"
#include "sgMain/ransac.h"

namespace sgMain
{
	std::vector<size_t> Segmentation::CreateRandomSample(size_t numOfElements, size_t elementsToPick)
	{
		assert(numOfElements >= elementsToPick);

		std::vector<unsigned int> indicesList;
		for (unsigned int i = 0; i != elementsToPick; i++)
		{
			unsigned int currentIndex = std::rand() % numOfElements;
			while (std::find(indicesList.begin(), indicesList.end(), currentIndex) != indicesList.end())
			{
				// element already exists, retry
				currentIndex = std::rand() % numOfElements;
			}
			indicesList.push_back(currentIndex);
		}
		return indicesList;
	}

	int Segmentation::RANSAC2D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rest,
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &segmentedObjects,
		double treshold, unsigned int maxNumberOfIterations, unsigned int triesPerIteration)
	{
		if (pointCloud->size() < 2)
		{
			return -1;
		}

		// first, convert point cloud into list of points
		unsigned int maxIterations = maxNumberOfIterations;
		unsigned int maxSubIterations = triesPerIteration;

		double distanceTreshold = 0.005;
		double sqrDistanceTreshold = pow(distanceTreshold, 2.0);

		rest = pointCloud;

		while (rest->size() > treshold * pointCloud->size() && maxIterations > 0 && rest->size() > 2)
		{

			std::vector<size_t> bestLinePointsIndices;
			unsigned int subIterations = maxSubIterations;
			maxIterations--;
			while (subIterations > 0)
			{
				subIterations--;

				// select a random sample of points
				std::vector<size_t> randomSampleIndices = CreateRandomSample(rest->size(), 2);
				std::vector<size_t> currentLinePointsIndices;

				// create a line from points
				Eigen::Vector2f linePoint0(rest->at(randomSampleIndices[0]).x, rest->at(randomSampleIndices[0]).y);
				Eigen::Vector2f linePoint1(rest->at(randomSampleIndices[1]).x, rest->at(randomSampleIndices[1]).y);

				float squaredDirectionLength = (linePoint1 - linePoint0).squaredNorm();
				//Color averageColor()


				unsigned int numberOfInliers = 0;
				for (size_t i = 0; i != rest->size(); i++)
				{
					Eigen::Vector2f currentPoint(rest->at(i).x, rest->at(i).y);
					double sqrDistance = Transformation3D::sqrDistancePointLine(currentPoint, linePoint0, linePoint1 - linePoint0);
					if (sqrDistance <= sqrDistanceTreshold)
					{
						currentLinePointsIndices.push_back(i);
					}
				}

				if (currentLinePointsIndices.size() > bestLinePointsIndices.size())
				{
					bestLinePointsIndices = currentLinePointsIndices;
				}
			}

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedObject(new pcl::PointCloud<pcl::PointXYZRGB>);

			for (size_t index = bestLinePointsIndices.size(); index != 0; index--)
			{
				segmentedObject->push_back(rest->at(bestLinePointsIndices[index - 1]));
				rest->at(bestLinePointsIndices[index - 1]) = rest->back();
				rest->resize(rest->size() - 1);
			}
			segmentedObjects.push_back(segmentedObject);
		}
		return 0;
	}



	int Segmentation::RANSAC3D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rest,
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &segmentedObjects,
		float treshold, pcl::SacModel modelType, int methodType)
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());


		pcl::SACSegmentation<pcl::PointXYZRGB> seg;

		seg.setOptimizeCoefficients(true);
		seg.setModelType(modelType);
		seg.setMethodType(methodType);
		seg.setMaxIterations(300);
		seg.setDistanceThreshold(10.0);


		// clear the rest point cloud
		rest->clear();
		pcl::copyPointCloud(*cloud, *rest);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;

		int i = 0, nr_points = (int)rest->points.size();
		// While some perof the original cloud is still there
		while (rest->points.size() > treshold * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud(rest);
			seg.segment(*inliers, *coefficients);
			if (inliers->indices.size() == 0)
			{
				// segmentation failed.
				return -1;
			}

			// Extract the inliers
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedObject(new pcl::PointCloud<pcl::PointXYZRGB>);

			extract.setInputCloud(rest);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*segmentedObject);

			segmentedObjects.push_back(segmentedObject);


			// Create the filtering object
			extract.setNegative(true);
			extract.filter(*cloud_f);
			rest.swap(cloud_f);
			i++;
		}
		return 0;
	}
}