/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* contour_2d.cpp
* Definition of contour_2d.h.
*/

#include "main/2d/contour_2d.h"

namespace sgMain
{
	Contour2D::Contour2D()
	{}

	Contour2D::Contour2D(sgBase::Line3D line)
	{
		this->contourLines.push_back(line);
	}

	std::ostream& operator<< (std::ostream& stream, const Contour2D &contour2D)
	{
		stream << "NumOfLines: " << contour2D.contourLines.size() << std::endl;
		for (size_t i = 0; i != contour2D.contourLines.size(); i++)
		{
			stream << "Line " << i << ": " << contour2D.contourLines[i] << std::endl;
		}
			
		return stream;
	}
}