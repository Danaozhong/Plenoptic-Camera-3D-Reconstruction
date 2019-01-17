/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* math_statistics.h
* Contains fucntions to perform statistical operations on data.
*/

#ifndef _MATH_STATISTICS_H_
#define _MATH_STATISTICS_H_

#include <vector>

namespace sgBase
{
	namespace MathStatistics
	{
		/**
		Calculates the mean value of an input array.
		*/
		double Mean(const std::vector<double> &values);

		/**
		Calculates the variance of the datas given in the array.
		*/
		double Variance(const std::vector<double> &values);
	}
}

#endif