#ifndef _IMAGE_PREPROCESSING_2D_H_
#define _IMAGE_PREPROCESSING_2D_H_

#include <opencv2/opencv.hpp>

namespace sgExecution
{
	namespace ImagePreprocessing2D
	{
		cv::Mat CreateContourImageUsingCanny(cv::Mat colorImage, int lowerThreshold);
	}
}
#endif