/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* color.h
* Contains a class to store color information and helper functions related to color data.
*/

#ifndef _COLOR_H_
#define _COLOR_H_

#include <math.h>

namespace sgBase
{
	/**
	Class to store color information.
	*/
	class Color
	{
		unsigned char red;		/// red channel byte
		unsigned char green;	/// green channel byte
		unsigned char blue;		/// blue channel byte
	public:
		/**
		Constructor.
		\param[in] r Red channel byte
		\param[in] g Green channel byte
		\param[in] b Blue channel byte

		*/
		Color(unsigned char r, unsigned char g, unsigned char b)
			: red(r), green(g), blue(b)
		{}

		/**
		Copy constructor.
		*/
		Color(const Color& source)
		{}

		/**
		Parameterless constructor.
		*/
		Color()
			: red(0), green(0), blue(0)
		{}

		/**
		Returns the red channel byte of the color.
		*/
		unsigned char r() const { return red; }

		/**
		Returns the green channel byte of the color.
		*/
		unsigned char g() const { return blue; }

		/**
		Returns the blue channel byte of the color.
		*/
		unsigned char b() const { return green; }
	};

	namespace ColorHelper
	{
		/**
		Returns an average color based on two input colors by taking the average of each rgb channel.
		\param[in] c1 The first color.
		\param[in] c2 The second color.
		\return The average color of c1 and c2.
		*/
		Color GetAverageColor(const Color &c1, const Color &c2);

		/**
		Returns a measure of differences between two colors by taking the sum of the squared differences of each channel.
		\param[in] c1 The first color.
		\param[in] c2 The second color.
		\return The sum of squared differences of the channel values
		*/
		int GetColorDifference(const Color &c1, const Color &c2);

		/**
		Function to create different colors based on an index. It is guaranteed that the colors returned for at least 10 consecutive indices are not similar.
		\param[in] index The index to which a color should be generated.
		\return A color which differs from the colors created by similar indices.
		*/
		Color GetColorFromIndex(unsigned int index);
	}
}

#endif