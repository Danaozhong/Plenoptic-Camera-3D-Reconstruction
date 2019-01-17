/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* color.cpp
* Implementation of color.h
*/

#include "sgBase/misc/color.h"


namespace sgBase
{
	Color ColorHelper::GetAverageColor(const Color &c1, const Color &c2)
	{
		return Color((c1.r() + c2.r()) / 2, (c1.g() + c2.g()) / 2, (c1.b() + c2.b()) / 2);
	}

	int ColorHelper::GetColorDifference(const Color &c1, const Color &c2)
	{
		return (c1.r() - c2.r()) * (c1.r() - c2.r()) + (c1.g() - c2.g()) * (c1.g() - c2.g()) + (c1.b() - c2.b()) * (c1.b() - c2.b());
	}

	static int HighestOrderBitPosition(unsigned int number)
	{
		int position = 0;

		while (number != 0)
		{
			number = number >> 1;
			position++;
		}
		return position;
	}

	Color ColorHelper::GetColorFromIndex(unsigned int index)
	{
		index = index % 1500;

		unsigned int level;
		unsigned int pattern;

		unsigned int majorLevel;
		level = index / 6;
		pattern = index % 6;

		// determine the position of the highest bit (returns us the logarithm dualis as integral type)
		majorLevel = HighestOrderBitPosition(level) - 1;
		unsigned char colorIndex = (unsigned char)(255 / pow(2.0, (int)(majorLevel)));

		unsigned int multiplicator = 0;
		if (majorLevel > 0)
		{
			multiplicator = level - (unsigned int)pow(2.0, ((int)majorLevel));
		}
		multiplicator++;
		colorIndex *= multiplicator;

		if (colorIndex == 0)
		{
			// if this ever happens, above calculation must be wrong :(
			throw;
		}
		switch (pattern)
		{
		case 0:
			return Color(0, 0, colorIndex);
		case 1:
			return Color(0, colorIndex, 0);
		case 2:
			return Color(colorIndex, 0, 0);
		case 3:
			return Color(0, colorIndex, colorIndex);
		case 4:
			return Color(colorIndex, 0, colorIndex);
		case 5:
		default:
			return Color(colorIndex, colorIndex, 0);
		}
	}
}