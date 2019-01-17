/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* color.h
* Contains several basic image processing algorithms functions which are useful when working with images.
*/

#ifndef _IMAGE_H_
#define _IMAGE_H_

#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "sgBase/misc/color.h"

namespace sgBase
{
	namespace ImageHelper
	{
		/**
		Converts an image to a black and white image. All completely black pixels will remain black, all others will become white.
		\return The back and white image of cvImage.
		*/
		cv::Mat ConvertToBlackAndWhite(const cv::Mat &cvImage);

		/**
		Inverts each channel of each pixel.
		*/
		int Invert(cv::Mat &cvImage);

		/**
		Returns an average color value of all color values on an image within a radius at a defined position.
		\param[in] sourceImage The image.
		\param[in] position The center position of the circle of the values to select.
		\param[in] radius The radius of the selection circle.
		\return An average color of all selected pixels.
		*/
		Color GetAverageColorValueAtPosition(const cv::Mat &sourceImage, const Eigen::Vector2i &position, int radius);

		/**
		Erodes an image. Black areas will become bigger, white areas smaller.
		*/
		cv::Mat Erosion(const cv::Mat &src, int erosionSize, bool showResult);
	}
}

#endif