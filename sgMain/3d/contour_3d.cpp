/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* contour_3d.cpp
* Definition of contour_3d.h.
*/

#include "sgMain/3d/contour_3d.h"
#include "sgMain/3d/transformation_3d.h"


namespace sgMain
{
	int Contour3D::PostprocessContours(const cv::Mat &intrinsicParameters)
	{
		for (int i = 0; i < this->contourLines.size() - 1; i++)
		{
			if (false == this->contourLines[i]->IsFitted() || false == this->contourLines[i + 1]->IsFitted())
			{
				continue;
			}
			// average line joints
			pcl::PointXYZRGB line1p2 = Transformation3D::TransformFromWordCoordinatesToCameraCoordinates(PointCloudHelper::PointXYZRGB(this->contourLines[i]->fittedLine.p2), intrinsicParameters);
			pcl::PointXYZRGB line2p1 = Transformation3D::TransformFromWordCoordinatesToCameraCoordinates(PointCloudHelper::PointXYZRGB(this->contourLines[i + 1]->fittedLine.p1), intrinsicParameters);

			line2p1.z = (line1p2.z + line2p1.z) / 2.0;

			pcl::PointXYZRGB mergedPoint = Transformation3D::TransformFromCameraToWorldCoordinates(line2p1, intrinsicParameters);

			this->contourLines[i]->fittedLine.p2 = PointCloudHelper::Vector3f(mergedPoint);
			this->contourLines[i + 1]->fittedLine.p1 = PointCloudHelper::Vector3f(mergedPoint);
		}
		return 0;
	}

	void Contour3D::Render(Visualization3D &viewport3D, int contourIndex) const
	{
		for (int i = 0; i != this->contourLines.size(); i++)
		{
			this->contourLines[i]->Render(viewport3D, contourIndex, i);
		}
	}

	std::ostream& operator<< (std::ostream& stream, const Contour3D &contour3D)
	{
		stream << "number of lines per contour: " << contour3D.contourLines.size() << std::endl;
		for (int i = 0; i != contour3D.contourLines.size(); i++)
		{
			stream << "Line " << i << " of " << contour3D.contourLines.size() - 1 << ": " << *contour3D.contourLines[i] << std::endl;
		}

		return stream;
	}

}
