/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* histogram.cpp
* Implementation of histogram.h
*/

/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* histogram.h
* Class to calculate a histogram data
*/

#include <algorithm>
#include <fstream>

#include "base/misc/histogram.h"

namespace sgBase
{
	Histogram::Histogram(double minValue, double maxValue, unsigned int numberOfBins, std::vector<double> values)
	{
		this->numOfBins = numberOfBins;
		// allocate enough memory to store the histogram. Add one because the highest element would be placed outside of the array boundaries
		this->binContent.resize(this->numOfBins + 1, 0);

		this->minValue = minValue;
		this->maxValue = maxValue;

		this->stepSize = (this->maxValue - this->minValue) / (double)this->numOfBins;

		for (auto itr = values.begin(); itr != values.end(); itr++)
		{
			if (*itr > this->maxValue)
			{
				this->binContent[this->numOfBins]++;
				continue;
			}
			if (*itr < this->minValue)
			{
				this->binContent[0]++;
				continue;
			}

			this->binContent[(size_t)(((*itr) - this->minValue) / this->stepSize)]++;
		}

		// take the last bin and add it to the pre-last bin
		this->binContent[this->numOfBins - 1] += this->binContent[this->numOfBins];
		this->binContent.pop_back();
	}

	std::ostream& operator<< (std::ostream& stream, const Histogram& histogram)
	{
		stream << "Histogram begin." << std::endl;
		for (size_t i = 0; i != histogram.numOfBins; i++)
		{
			stream << histogram.minValue + (double)i * histogram.stepSize << "\t" << histogram.binContent[i] << std::endl;
		}

		stream << "End of histogram.";
		return stream;
	}

	int HistogramHelper::StoreInFile(const Histogram& histogram, std::string fileName)
	{
		std::ofstream outputFile;
		outputFile.open(fileName);
		if (outputFile.is_open())
		{
			outputFile << histogram;
			outputFile.close();
			return 0;
		}
		return -1;
	}
}


