/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* math_statistics.cpp
* Implementation of math_statistics.h.
*/


#include <math.h>

#include "sgBase/misc/math_statistics.h"

namespace sgBase
{
	namespace MathStatistics
	{
		double Mean(const std::vector<double> &values)
		{
			double sum = 0.0;

			if (0 == values.size())
			{
				return 0.0;
			}

			for (auto itr = values.begin(); itr != values.end(); itr++)

				sum += *itr;

			return (sum / (double)values.size());
		}

		double Variance(const std::vector<double> &values)
		{
			if (0 == values.size())
			{
				return 0.0;
			}

			double mean = Mean(values);
			double varianceSum = 0.0;

			for (size_t i = 0; i < values.size(); i++)
			{
				varianceSum += (values[i] - mean) * (values[i] - mean);
			}
			return (varianceSum / (double)values.size());
		}
	}
}