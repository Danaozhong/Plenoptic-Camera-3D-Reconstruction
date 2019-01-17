/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* line_3d.cpp
* Implementation of line_3d.h.
*/

#include <cassert>
#include <cmath>

#include "sgBase/geometrics/line_3d.h"


namespace sgBase
{
	Line3D::Line3D()
	{}

	Line3D::Line3D(const Eigen::Vector3f &begin, const Eigen::Vector3f &end)
		: p1(begin), p2(end)
	{}

	double Line3D::SqrLength() const
	{
		return (p2 - p1).squaredNorm();
	}

	double Line3D::Length() const
	{
		return std::sqrt(this->SqrLength());
	}

	Eigen::Vector3f Line3D::Direction() const 
	{
		return (this->p2 - this->p1).normalized();
	}

	std::ostream& operator<<(std::ostream& stream, const Line3D &line)
	{
		stream << "Line (p1: " << line.p1 << ", p2: " << line.p2 << ") length: " << line.Length();
		return stream;
	}

	bool Line3DHelper::ArePointsIdentical(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
	{
		if ((v1 - v2).squaredNorm() > Line3DHelper::squaredDistanceTreshold)
		{
			return false;
		}
		return true;
	}

	bool Line3DHelper::AreLinesIdentical(const Line3D &l1, const Line3D &l2)
	{
		if ((ArePointsIdentical(l1.p1, l2.p1) && ArePointsIdentical(l1.p2, l2.p2)) ||
			(ArePointsIdentical(l1.p1, l2.p2) && ArePointsIdentical(l1.p2, l2.p1)))
		{
			return true;
		}
		return false;
	}

	double Line3DHelper::DistancePointToLine(const Line3D &l1, const Eigen::Vector3f &point)
	{
		Eigen::Vector3f linePoint = l1.p1;
		Eigen::Vector3f lineDirection = (l1.p2 - l1.p1);
		float lineEndPos = lineDirection.norm();
		lineDirection.normalize();

		Eigen::Vector3f currentPointVector = point - linePoint;

		// project point onto the line
		float currentPos = currentPointVector.dot(lineDirection);
		double pointLineDistance = (point - (linePoint + currentPos * lineDirection)).norm();
		double distance = pointLineDistance;

		if (currentPos < 0.0f || currentPos > lineEndPos)
		{
			// the projection of the point is outside of the defined line, thus we need to calculate the distance using Phytagoras.
			distance = std::sqrt(currentPos * currentPos + distance * distance);
		}
		assert(distance >= 0.0);
		return distance;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Line3DHelper::FindClosePoints(const Line3D &line2D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloud, double distance)
	{
		// find points within the point cloud which are close to the line
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (auto itr = depthCloud->begin(); itr != depthCloud->end(); itr++)
		{
			Eigen::Vector3f currentPoint(itr->x, itr->y, 0.0f);
			if (distance > Line3DHelper::DistancePointToLine(line2D, currentPoint))
			{
				pcl::PointXYZRGB newPoint(*itr);
				newCloud->push_back(newPoint);
			}
		}
		return newCloud;
	}

}