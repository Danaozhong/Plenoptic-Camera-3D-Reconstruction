/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* camera_3d.h
* This file contains the functions for the camera 3D extensions,
* allowing the 3D reconstruction of the flat camera image by storing the intrinsic
*and distortion parameters.
*/

#ifndef _CAMERA_3D_H_
#define _CAMERA_3D_H_

#include <string>
#include <opencv2/opencv.hpp>

#include "sgBase/camera_3d.h" 
	

namespace sgBase
{
	/**
	A class to manage the camera's intrinsic parameters. It stores the matrix of intrinsic parameters and the
	dirstortion coefficients, as given by OpenCV.
	*/
	class Camera3D
	{
	private:
		/*
		projectionMatrix:
		fx 	0	oax
		0	fy	oay
		0	0	1

		f = focal length in pixel related units
		oa = optical axis
		*/

		cv::Mat intrinsic;				/// the matrix of intrinsic parameters, stored in a OpenCV matrix		
		cv::Mat distortion;				/// the distortion coefficients, stored in a OpenCV matrix
		cv::Mat mapx, mapy;				/// The image maps, mapping from a distorted to an udistorted image. They need to have the same size as the capture image.
		cv::Mat undistortedImage;		/// The udistorted output image

	public:

		/**
		Constructor to create an instance of the Camera3D class out of XML files.These files need to be in OpenCV format.
		\param[in]	cameraParamFile	String containing the path of the XML file containing the intrinsic parameters and the distortion coefficients.
		*/
		Camera3D(const std::string &cameraParamFile);

		/**
		Constructor to create an instance of the Camera3D class out of loaded OpenCV matrices.
		\param[in]	intrinsic	The OpenCV matrix containing the intrinsic parameters.
		\param[in]	distortion	The OpenCV matrix containing the camera distortion parameters.
		*/
		Camera3D(const cv::Mat &intrinsic, const cv::Mat &distortion);

		/**
		Copy constructor for the Camera3D class.
		*/
		Camera3D(const Camera3D &source);

		/**
		Return the projection matrix of the camera (matrix of intrinsic parameters of the camera) in OpenCV format.
		*/
		cv::Mat GetCameraProjectionMatrix()
		{
			cv::Mat intrinsicMat(this->intrinsic);
			return intrinsicMat;
		}

		/**
		Stores the camera parameters (intrinics and distortion) in an XML file in OpenCV format.
		*/
		void Store(const std::string &cameraParamFile);
	};
}

#endif
