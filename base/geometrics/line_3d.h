/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* line_3d.h
* Class to store a line in 3D space with an begin and end point.
*/


#ifndef _LINE_3D_H_
#define _LINE_3D_H_

#include <Eigen/Dense>
#include <iostream>
#include "base/geometrics/point_cloud.h"

namespace sgBase
{
	/**
	 Class to store a non-infinite line in 3D space with an begin and end point. Within this program, this class is also used
	 to represent 2D lines which are used on 2D images. In this case, the z coordinate is always set to 0.0f.
	*/
	class Line3D
	{
	public:
		/**
		Constructor.
		*/
		Line3D();

		/** 
		Parameter constructor.
		*/
		Line3D(const Eigen::Vector3f &begin, const Eigen::Vector3f &end);

		/**
		Returns the squared length of the line.
		*/
		double SqrLength() const;

		/**
		Returns the length of the line.
		*/
		double Length() const;

		/**
		Returns the direction vector of the line.
		*/
		Eigen::Vector3f Direction() const;

		/** 
		The begin point of the line. 
		*/
		Eigen::Vector3f p1;

		/**
		The end point of the line.
		*/
		Eigen::Vector3f p2;
	};

	/** 
	output stream print operator of a line. Will print out line coordinates and line length.
	*/
	std::ostream& operator<<(std::ostream& stream, const Line3D &line);

	namespace Line3DHelper
	{
		/**
		Value to perform distance calculations. Equals the minimum squared distance between 3D objects before they are threated as touching.
		*/
		const double squaredDistanceTreshold = 0.005;

		/**
		Checks whether two points are identical.
		*/
		bool ArePointsIdentical(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);

		/**
		Check whether two lines are identical (have their endings at the same coordinates).
		*/
		bool AreLinesIdentical(const Line3D &l1, const Line3D &l2);

		/**
		Returns the minimum distance between two non-infinite lines. This is not the minimum distance between two infinite long lines!
		*/
		double DistancePointToLine(const Line3D &l1, const Eigen::Vector3f &point);

		/**
		Returns a point cloud consisting of points of a depth cloud in camera space which are close to a 2D line.
		*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr FindClosePoints(const Line3D &line2D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloud, double distance);
	}
}

#endif

