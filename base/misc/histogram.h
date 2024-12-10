/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* histogram.h
* Class to calculate a histogram data
*/

#ifndef _HISTOGRAM_H_
#define _HISTOGRAM_H_

#include <iostream>
#include <vector>

namespace sgBase
{
	/**
	Class to create and store a histogram
	*/
	class Histogram
	{
		unsigned int numOfBins;
		double minValue;
		double maxValue;
		double stepSize;
		std::vector<unsigned int> binContent;
	public:
		/**
		Constructor.
		\param[in] numberOfBins How many bins are used in this histogram
		\param[in] values The values out of which the histogram should be created
		*/
		Histogram(double minValue, double maxValue, unsigned int numberOfBins, std::vector<double> values);

		/**
		Copy constructor.
		*/
		//Histogram(const Histogram& source)
		//{}

		/**
		Destructor.
		*/
		~Histogram()
		{}

		friend std::ostream& operator<< (std::ostream& stream, const Histogram& histogram);
	};

	std::ostream& operator<< (std::ostream& stream, const Histogram& histogram);

	namespace HistogramHelper
	{
		int StoreInFile(const Histogram& histogram, std::string fileName);
	}
}

#endif