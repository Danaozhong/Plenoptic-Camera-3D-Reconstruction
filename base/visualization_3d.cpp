/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* visualization_3d.cpp
* Implementation of visualization_3d.h
*/


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

#include "base/visualization_3d.h"

using namespace std;

namespace sgBase
{
	Visualization3D::Visualization3D(unsigned int columns, unsigned int rows)
		: width(columns), height(rows)
	{
		assert(columns != 0 && rows != 0);
		viewer = std::unique_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->initCameraParameters();

		this->viewPortIndices = std::vector<int>();

		// create the viewports
		double deltaX = 1.0 / (double)this->width;
		double deltaY = 1.0 / (double)this->height;

		for (unsigned int Y = 0; Y != this->height; Y++)
		{
			for (unsigned int X = 0; X != this->width; X++)
			{
				int viewPortID = 0;
				this->viewer->createViewPort(X * deltaX, Y * deltaY, (X + 1) * deltaX, (Y + 1) * deltaY, viewPortID);
				this->viewPortIndices.push_back(viewPortID);
				this->viewer->setBackgroundColor(255, 255, 255, viewPortID);
			}
		}
	}

	int Visualization3D::GetViewPortID(unsigned int column, unsigned int row)
	{
		assert(row * this->width + column < this->height * this->width);
		return this->viewPortIndices[row * this->width + column];
	}

	int Visualization3D::AddPointCloudToViewPort(unsigned int column, unsigned int row, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB> &pointColorHandler, const string &title)
	{
		int id = GetViewPortID(column, row);
		this->viewer->addText(title, 10, 10, std::to_string((long long)id) + " text" + title, id);
		this->viewer->addPointCloud<pcl::PointXYZRGB>(cloud, pointColorHandler, title, id);
		this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, title);
		return 0;
	}

	int Visualization3D::AddPointCloudToViewPort(unsigned int column, unsigned int row, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const string &title)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pointColorHandler(cloud);
		return AddPointCloudToViewPort(column, row, cloud, pointColorHandler, title);
	}

	int Visualization3D::AddPointCloudToViewPort(unsigned int column, unsigned int row, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const string &title, const Color &color)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> pointColorHandler(cloud, color.b(), color.g(), color.r());
		return AddPointCloudToViewPort(column, row, cloud, pointColorHandler, title);
	}

	int Visualization3D::AddLineToViewPort(unsigned int column, unsigned int row, const Line3D &line, const string &title, const Color &color)
	{
		int id = GetViewPortID(column, row);
		this->viewer->addLine(PointCloudHelper::PointXYZRGB(line.p1), PointCloudHelper::PointXYZRGB(line.p2), color.b(), color.g(), color.r(), title, id);
		this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, title);
		return 0;
	}
}