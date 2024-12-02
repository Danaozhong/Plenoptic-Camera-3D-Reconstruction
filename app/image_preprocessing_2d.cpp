#include "main/2d/edge_detection_2d.h"

#include "app/image_preprocessing_2d.h"

using namespace sgMain;

namespace sgExecution
{
	cv::Mat ImagePreprocessing2D::CreateContourImageUsingCanny(cv::Mat colorImage, int lowerThreshold)
	{
		// Canny edge detection
		cv::Mat colorImageAfterEdgeDetection = EdgeDetection2D::CannyEdgeDetector(colorImage, lowerThreshold);


		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat edgeImageBlackWhite = ImageHelper::ConvertToBlackAndWhite(colorImageAfterEdgeDetection);

		return edgeImageBlackWhite;
	}
}