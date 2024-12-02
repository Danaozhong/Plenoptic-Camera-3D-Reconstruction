/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* camera_3d.cpp
* Definition of camera_3d.h.
*/

#include "base/camera_3d.h" 

namespace sgBase
{
	Camera3D::Camera3D(const std::string &cameraParamFile)
	{	
		cv::FileStorage fs;
	
		fs.open(cameraParamFile, cv::FileStorage::READ);
		if (false == fs.isOpened())
		{
			throw;
		}

		fs["Intrinsic"] >> this->intrinsic;
		fs["Distortion"] >> this->distortion;

		assert(this->intrinsic.size().width == 3);
		assert(this->intrinsic.size().height == 3);

	}

	Camera3D::Camera3D(const cv::Mat &intrinsic, const cv::Mat &distortion)
	{
		intrinsic.copyTo(this->intrinsic);
		distortion.copyTo(this->distortion);
	}

	void Camera3D::Store(const std::string &cameraParamFile)
	{
		cv::FileStorage fs;
		fs.open(cameraParamFile, cv::FileStorage::WRITE);

		fs << "Intrinsic" << this->intrinsic;
		fs << "Distortion" << this->distortion;
		fs.release();
	}
}