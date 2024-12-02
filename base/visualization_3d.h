#ifndef _VISUALIZATION_3D_H_
#define _VISUALIZATION_3D_H_

#include <math.h>
#include <string>
#include <cassert>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>

#include "base/misc/color.h"
#include "base/geometrics/line_3d.h"
#include "base/geometrics/point_cloud.h"

using namespace std;

namespace sgBase
{
	class Visualization3D
	{
		unsigned int width;
		unsigned int height;

		//boost::unique_ptr<pcl::visualization::PCLVisualizer> viewer;
		std::vector<int> viewPortIndices;
	public:

		std::unique_ptr<pcl::visualization::PCLVisualizer> viewer;

		Visualization3D(unsigned int columns, unsigned int rows);
		int GetViewPortID(unsigned int column, unsigned int row);
		int AddPointCloudToViewPort(unsigned int column, unsigned int row, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB> &pointColorHandler, const string &title);
		int AddPointCloudToViewPort(unsigned int column, unsigned int row, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const string &title);
		int AddPointCloudToViewPort(unsigned int column, unsigned int row, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const string &title, const Color &color);
		int AddLineToViewPort(unsigned int column, unsigned int row, const Line3D &line, const string &title, const Color &color);
	};
}
#endif