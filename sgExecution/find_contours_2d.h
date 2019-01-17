#ifndef _FIND_CONTOURS_2D_H_
#define _FIND_CONTOURS_2D_H_

#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "sgBase/geometrics/line_3d.h"
#include "sgMain/2d/contour_2d.h"


using namespace sgBase;
using namespace sgMain;

namespace sgExecution
{
	enum Segmentation3DContourDetectionMethod
	{
		DETECTION_RANSAC,
		DETECTION_RANSAC_WITH_QUADTREE,
		DETECTION_HOUGH_TRANSFORM,
		DETECTION_SUZUKI_ABE,
		DETECTION_MANUAL_SELECTION
	};

	class FindContours2D
	{
	private:
		cv::Mat colorImage;
		cv::Mat edgeImage;
		bool showResults;

		std::vector<Contour2D> FindContoursSuzukiAbe();
		std::vector<Contour2D> FindContoursRANSAC2D();
		std::vector<Contour2D> FindContoursRANSAC2DWithQuadtree();
		std::vector<Contour2D> FindContoursHoughTransform();
		std::vector<Contour2D> FindContoursUsingManualSelection();

		Eigen::Vector2i manualLineSelectionP1;
		Eigen::Vector2i manualLineSelectionP2;
		bool manualLineSelectionCurrentPoint;
		bool manualLineSelected;
		std::string manualLineSelectionWindowName;
		std::vector<Line3D> selectedLines;

		static void MouseClick(int event, int x, int y, int flags, void *param);
		void ManualLineSelectionUpdate();
	public:
		FindContours2D(const cv::Mat &colorImage, const cv::Mat &edgeImage, bool showResults);

		int FindContours(Segmentation3DContourDetectionMethod method, std::vector<Contour2D> &contours);
	};
}

#endif