/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* contour_3d.h
* This class is used to represent object contours in a 3D environment.
*/

#ifndef _CONTOUR_3D_H_
#define _CONTOUR_3D_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "sgMain/3d/fit_line_3d.h"

namespace sgMain
{
	class Contour3D
	{
	public:
		std::vector<std::shared_ptr<sgMain::FitLine3D>> contourLines;

		int PostprocessContours(const cv::Mat &intrinsicParameters);
		void Render(Visualization3D &viewport3D, int contourIndex) const;
	};

	std::ostream& operator<< (std::ostream& stream, const Contour3D &contour3D);
}

#endif