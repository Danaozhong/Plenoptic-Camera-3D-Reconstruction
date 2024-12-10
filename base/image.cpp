/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* image.cpp
* Implementation of image.h
*/

#include "base/image.h"

namespace sgBase
{
	cv::Mat ImageHelper::ConvertToBlackAndWhite(const cv::Mat &cvImage)
	{
		cv::Mat output;
		output.create(cvImage.size(), cvImage.type());
		cv::cvtColor(cvImage, output, cv::COLOR_BGR2GRAY);
		for (int i = 0; i != output.size().height * output.size().width * output.channels(); i++)
		{
			if (output.data[i] != 0)
			{
				output.data[i] = 255;
			}
		}
		return output;
	}
	
	int ImageHelper::Invert(cv::Mat &cvImage)
	{
		for (int i = 0; i != cvImage.size().height * cvImage.size().width * cvImage.channels(); i++)
		{
			cvImage.data[i] = 255 - cvImage.data[i];
		}

		return 0;
	}

	Color ImageHelper::GetAverageColorValueAtPosition(const cv::Mat &sourceImage, const Eigen::Vector2i &position, int radius)
	{
		cv::Rect copyRect(position.x() - radius, position.y() - radius, 2 * radius + 1, 2 * radius + 1);

		int centerX = copyRect.width / 2;
		int centerY = copyRect.height / 2;

		// check boundaries of image 
		if (0 > copyRect.x)
		{
			centerX += copyRect.x;
			copyRect.width += copyRect.x;
			copyRect.x = 0;
		}
		if (0 > copyRect.y)
		{
			centerY += copyRect.y;
			copyRect.height += copyRect.y;
			copyRect.y = 0;
		}
		if (copyRect.x + copyRect.width > sourceImage.size().width)
		{
			copyRect.width = sourceImage.size().width - copyRect.x;
		}
		if (copyRect.y + copyRect.height > sourceImage.size().height)
		{
			copyRect.height = sourceImage.size().height - copyRect.y;
		}
		assert(0 < copyRect.width && 0 < copyRect.height);

		// the average color value is calculated by multiplying the extracted part of the image with a bit mask
		cv::Mat imageExtract = sourceImage(copyRect);
		cv::Mat1b mask(imageExtract.rows, imageExtract.cols);

		// create circle mask
		for (int i = 0; i != mask.rows; i++)
		{
			for (int k = 0; k != mask.cols; k++)
			{
				int diffX = centerX - k;
				int diffY = centerY - i;

				if (diffX * diffX + diffY * diffY > radius * radius)
				{
					mask.at<uchar>(i, k) = 0;
				}
				else
				{
					mask.at<uchar>(i, k) = 255;
				}
			}
		}

		cv::Scalar mean = cv::mean(imageExtract, mask);
		return Color((uchar)mean[0], (uchar)mean[1], (uchar)mean[2]);
	}

	cv::Mat ImageHelper::Erosion(const cv::Mat &src, int erosionSize, bool showResult)
	{
			int erosion_type;
			int erosion_elem = 2;
			if (erosion_elem == 0){ erosion_type = cv::MORPH_RECT; }
			else if (erosion_elem == 1){ erosion_type = cv::MORPH_CROSS; }
			else if (erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

			cv::Mat erosion_dst;

			cv::Mat element = getStructuringElement(erosion_type,
				cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1),
				cv::Point(erosionSize, erosionSize));

			/// Apply the erosion operation
			cv::erode(src, erosion_dst, element);
			if (showResult)
			{
				cv::imshow("Erosion results", erosion_dst);
			}

			return erosion_dst;
		}
}