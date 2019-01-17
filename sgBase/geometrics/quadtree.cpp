#include "sgBase/geometrics/quadtree.h"

namespace sgBase
{
	namespace QuadtreeHelper
	{
		int GetNumberOfPointsInArea(const QuadtreeLeafNode<pcl::PointXYZRGB> &leafNode, const Eigen::Vector2d &bottomLeft, const Eigen::Vector2d &upperRight)
		{
			int numberOfPoints = 0;
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr list = leafNode.getContainerPtr();

			for (auto itr = list->begin(); itr != list->end(); itr++)
			{
				if (itr->x > bottomLeft.x() && itr->x < upperRight.x() && itr->y > bottomLeft.y() && itr->y < upperRight.y())
				{
					numberOfPoints++;
				}
			}
			return numberOfPoints;
		}

		double QuadtreeLeafNodePointDensityVariance(const QuadtreeLeafNode<pcl::PointXYZRGB> &leafNode)
		{
			int numberOfSubdivisionX = 5;
			int numberOfSubdivisionY = 5;
			double leafWidth = (leafNode.upperRight - leafNode.bottomLeft).x();
			double leafHeight = (leafNode.upperRight - leafNode.bottomLeft).y();

			std::vector<double> numberOfPoints;

			for (int X = 0; X != numberOfSubdivisionX; X++)
			{
				for (int Y = 0; Y != numberOfSubdivisionY; Y++)
				{
					Eigen::Vector2d bottomLeft(leafNode.bottomLeft.x() + (double)X / (double)numberOfSubdivisionX * leafWidth, leafNode.bottomLeft.y() + (double)Y / (double)numberOfSubdivisionY * leafHeight);
					Eigen::Vector2d upperRight(leafNode.bottomLeft.x() + (double)(X + 1) / (double)numberOfSubdivisionX * leafWidth, leafNode.bottomLeft.y() + (double)(Y + 1) / (double)numberOfSubdivisionY * leafHeight);
					numberOfPoints.push_back((double)GetNumberOfPointsInArea(leafNode, bottomLeft, upperRight));
				}
			}

			return MathStatistics::Variance(numberOfPoints);
		}
	}
}