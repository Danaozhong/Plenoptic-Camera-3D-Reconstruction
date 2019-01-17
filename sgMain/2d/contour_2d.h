/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* contour_2d.h
* This class is used to represent a contour in a 2D picture as a set of connected lines.
*/

#ifndef _CONTOUR_2D_H_
#define _CONTOUR_2D_H_

#include <vector>

#include "sgBase/geometrics/line_3d.h"

namespace sgMain
{
	class Contour2D
	{
	public:
		Contour2D();
		Contour2D(sgBase::Line3D line);
		std::vector<sgBase::Line3D> contourLines;
	};


	std::ostream& operator<< (std::ostream&, const Contour2D &contour2D);
}

#endif