#ifndef _SEGMENTATION_3D_H_
#define _SEGMENTATION_3D_H_

#include <ostream>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/common/distances.h>

#include "sgBase/camera_3d.h"
#include "sgBase/geometrics/line_3d.h"

#include "sgMain/2d/contour_2d.h"
#include "sgMain/3d/contour_3d.h"

#include "sgExecution/find_contours_2d.h"
#include "sgExecution/contour_fitting_3d.h"

using namespace sgBase;
using namespace sgMain;

namespace sgExecution
{
	enum Segmentation3DApproach
	{
		APPROACH_CLASSICAL,
		APPROACH_CONTOUR_FITTING
	};

	class Segmentation3D
	{
	private:
		Camera3D plenopticCamera;
		cv::Mat matrixOfIntrinsicParameters;
		cv::Mat colorImage;
		cv::Mat depthImage;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloud;
		std::vector<Contour3D> ExecuteDepthFitting(ContourFitting3D contourFitting, Segmentation3DDepthFittingMethod depthFittingMethod, double radius, int id);

	public:
		cv::Mat contourImage;
		std::vector<Contour2D> contours2D;
		std::vector<Contour3D> contours3D;
		bool showResults;

		Segmentation3D(std::string cameraParameterFileName, std::string colorImageFileName, std::string depthImageFileName);

		int Execute();
		int ExecuteContourFittingApproach(Segmentation3DContourDetectionMethod lineDetectionMethod, Segmentation3DDepthFittingMethod depthFittingMethod);
		int ExecuteClassicalApproach();

		
		//void EdgeLinkingOnColorInformation();
		//void EdgeLinkingOnMouseInput();


		void RANSAC3D();
		void RANSAC2DWithQuadtree();
		void RANSAC2DWithoutQuadtree();
	};

	namespace Segmentation3DHelper
	{
		const std::string DefaultOutputPath = "output.txt";
		Segmentation3DApproach QuerySegmentationApproach();
		Segmentation3DContourDetectionMethod QueryContourDetectionMethod();
		Segmentation3DDepthFittingMethod QueryLineFittingMethod();
		bool QueryBool();
		bool QueryStoreInFile(std::ofstream &fileStream);
	}
}

#endif