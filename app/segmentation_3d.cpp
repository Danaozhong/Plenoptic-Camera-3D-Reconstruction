#include <vector>

#include <chrono>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "base/image.h"
#include "base/misc/color.h"
// pcl:visualization is not available on Bazel, therefore visualization is temporarily disabled
//#include "base/visualization_3d.h"


#include "main/2d/edge_detection_2d.h"
#include "main/3d/fit_line_3d.h"
#include "main/3d/transformation_3d.h"
#include "app/ransac.h"
#include "app/find_contours_2d.h"

#include "app/contour_fitting_3d.h"
#include "app/segmentation_3d.h"
#include "app/image_preprocessing_2d.h"


namespace sgExecution
{
	Segmentation3D::Segmentation3D(std::string cameraParameterFileName, std::string colorImageFileName, std::string depthImageFileName)
		:plenopticCamera(cameraParameterFileName), showResults(true)
	{
		//this->plenopticCamera = Camera3D(cameraParameterFileName);
		this->matrixOfIntrinsicParameters = this->plenopticCamera.GetCameraProjectionMatrix();

		// load image data...
		this->colorImage = cv::imread(colorImageFileName);
		this->depthImage = cv::imread(depthImageFileName, cv::IMREAD_ANYDEPTH);

		// ...and generate a 3D point cloud
		this->depthCloud = PointCloudHelper::StatisticalOutlierFilter(PlenopticCameraHelper::CalculateRealDepthValue(PointCloudHelper::ColoredPointCloudFromImages(colorImage, depthImage)), 50, 1.0);
	}

	int Segmentation3D::Execute()
	{
		Segmentation3DApproach approach = Segmentation3DHelper::QuerySegmentationApproach();

		if (APPROACH_CLASSICAL == approach)
		{
			return ExecuteClassicalApproach();
		}

		Segmentation3DContourDetectionMethod contourDetectionMethod = Segmentation3DHelper::QueryContourDetectionMethod();
		Segmentation3DDepthFittingMethod depthFittingMethod = Segmentation3DHelper::QueryLineFittingMethod();

		return ExecuteContourFittingApproach(contourDetectionMethod, depthFittingMethod);
	}

	std::vector<Contour3D> Segmentation3D::ExecuteDepthFitting(ContourFitting3D contourFitting, Segmentation3DDepthFittingMethod depthFittingMethod, double radius, int id)
	{
		std::vector<Contour3D> currentContours3D = contourFitting.FitContours(depthFittingMethod, radius);

		for (size_t i = 0; i != currentContours3D.size(); i++)
		{
			currentContours3D[i].PostprocessContours(this->matrixOfIntrinsicParameters);
		}

		std::ofstream outputFile;
		outputFile.open("C:\\method" + std::to_string((long long)id) + ".txt");
		if (false == outputFile.is_open())
		{
			throw(0);
		}

		for (size_t i = 0; i != currentContours3D.size(); i++)
		{
			outputFile << "Contour " << i << " of " << currentContours3D.size() << std::endl;
			for (size_t k = 0; k != currentContours3D[i].contourLines.size(); k++)
			{
				outputFile << "Line " << k << " of " << currentContours3D[i].contourLines.size() << std::endl;
				if (false == currentContours3D[i].contourLines[k]->IsFitted())
				{
					outputFile << "Line fitting FAILED." << std::endl;
					continue;
				}
				outputFile << "Length : " << currentContours3D[i].contourLines[k]->fittedLine.Length() << ", Variance: " << currentContours3D[i].contourLines[k]->GetVariance() << std::endl;
				Histogram currentDistanceHistogram = currentContours3D[i].contourLines[k]->GetHistogram(250);
				outputFile << currentDistanceHistogram << std::endl;
			}
		}
		outputFile.close();
		return currentContours3D;
	}

	int Segmentation3D::ExecuteContourFittingApproach(Segmentation3DContourDetectionMethod lineDetectionMethod, Segmentation3DDepthFittingMethod depthFittingMethod)
	{
		
		std::ofstream fileOutputStream;
		bool saveResultsInFile = Segmentation3DHelper::QueryStoreInFile(fileOutputStream);

		// perform canny 
		{
			std::cout << "Starting canny edge detection algorithm..." << std::endl;
			auto contourDetectionStartTime = std::chrono::system_clock::now();

			this->contourImage = ImagePreprocessing2D::CreateContourImageUsingCanny(this->colorImage, 35);
			auto contourDetectionEndTime = std::chrono::system_clock::now();
			std::cout << "Canny edge detection complete (duration " << std::chrono::duration_cast<std::chrono::milliseconds>(contourDetectionEndTime - contourDetectionStartTime).count() << " ms)." << std::endl;
		}

		if (true == this->showResults)
		{
			cv::imshow("Color image after Canny edge detection", this->contourImage);
			//cv::imwrite("C:\\canny.png", this->contourImage);
		}

		{
			std::cout << "Starting contour detection algorithm..." << std::endl;
			auto contourDetectionStartTime = std::chrono::system_clock::now();

			FindContours2D findContours(this->colorImage, this->contourImage, true);
			if (0 != findContours.FindContours(lineDetectionMethod, this->contours2D))
			{
				return -1;
			}

			auto contourDetectionEndTime = std::chrono::system_clock::now();
			std::cout << "Contour detection complete (duration " << std::chrono::duration_cast<std::chrono::milliseconds>(contourDetectionEndTime - contourDetectionStartTime).count() << " ms)." << std::endl;
		}

		if (true == this->showResults)
		{
			// display overlay of 2D image with contours
			cv::Mat colorImageWithLines;
			this->colorImage.copyTo(colorImageWithLines);

			for (size_t i = 0; i != this->contours2D.size(); i++)
			{
				Color currentColor = ColorHelper::GetColorFromIndex(i);
				for (size_t k = 0; k != this->contours2D[i].contourLines.size(); k++)
				{
					cv::Point p1((int)(this->contours2D[i].contourLines[k].p1.x() * (double)colorImageWithLines.size().width), (int)(this->contours2D[i].contourLines[k].p1.y() * (double)colorImageWithLines.size().height));
					cv::Point p2((int)(this->contours2D[i].contourLines[k].p2.x() * (double)colorImageWithLines.size().width), (int)(this->contours2D[i].contourLines[k].p2.y() * (double)colorImageWithLines.size().height));

					cv::line(colorImageWithLines, p1, p2, cv::Scalar(currentColor.r(), currentColor.g(), currentColor.b()), 2, cv::LINE_AA);
				}
			}
			cv::imshow("Detected lines", colorImageWithLines);
			//cv::imwrite("C:\\detected_lines.png", colorImageWithLines);
		}

		ContourFitting3D contourFitting(this->contours2D, this->depthCloud, this->matrixOfIntrinsicParameters);

		double radius = 0.20;

#if 0
		// use this code to compare the different depth fitting algorithms with each other
		std::vector<Contour3D> contoursEndPointAvg = ExecuteDepthFitting(contourFitting, FITTING_BEGIN_AND_END_POINT_AVERAGING, radius, 0);
		std::vector<Contour3D> contoursLS = ExecuteDepthFitting(contourFitting, FITTING_LEAST_SQUARES, radius, 1);
		std::vector<Contour3D> contoursMV = ExecuteDepthFitting(contourFitting, FITTING_MINIMUM_VARIANCE, radius, 2);

		this->contours3D = contoursMV;
#else
		{
			std::cout << "Starting depth fitting algorithm..." << std::endl;
			auto contourDetectionStartTime = std::chrono::system_clock::now();

			this->contours3D = contourFitting.FitContours(depthFittingMethod, radius);

			auto contourDetectionEndTime = std::chrono::system_clock::now();
			std::cout << "Depth fitting complete (duration " << std::chrono::duration_cast<std::chrono::milliseconds>(contourDetectionEndTime - contourDetectionStartTime).count() << " ms)." << std::endl;
		}
#endif
		
		{
			std::cout << "Starting contour postprocessing..." << std::endl;
			auto contourDetectionStartTime = std::chrono::system_clock::now();

			for (size_t i = 0; i != this->contours3D.size(); i++)
			{
				this->contours3D[i].PostprocessContours(this->matrixOfIntrinsicParameters);
			}

			auto contourDetectionEndTime = std::chrono::system_clock::now();
			std::cout << "Contour postprocessing complete (duration " << std::chrono::duration_cast<std::chrono::milliseconds>(contourDetectionEndTime - contourDetectionStartTime).count() << " ms)." << std::endl;
		}

		// store results in file, if required
		if (true == saveResultsInFile)
		{
			for (auto itr = this->contours3D.begin(); itr != this->contours3D.end(); itr++)
			{
				fileOutputStream << *itr;
			}
			fileOutputStream.close();

		}

		if (true == this->showResults)
		{
			/*
			Show results in a 2x1 viewport: First, the unmodified point cloud, second, the fitted planes
			*/
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloudInWorldCoordinates = Transformation3D::TransformFromCameraToWorldCoordinates(this->depthCloud, this->matrixOfIntrinsicParameters);
#if 0 // pcl:visualization is not available on Bazel, therefore visualization is temporarily disabled
			Visualization3D viewport3D(2, 1);
			viewport3D.AddPointCloudToViewPort(0, 0, depthCloudInWorldCoordinates, "cloud_org");
			for (size_t i = 0; i != this->contours3D.size(); i++)
			{
				this->contours3D[i].Render(viewport3D, i);
			}

			while (false == viewport3D.viewer->wasStopped())
			{
				if (false == viewport3D.viewer->wasStopped())
				{
					viewport3D.viewer->spinOnce(100);
				}
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
#endif
		}
		return 0;
	}

	int Segmentation3D::ExecuteClassicalApproach()
	{
		// transform point cloud to world coordinates
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloudInWorldCoordinates = Transformation3D::TransformFromCameraToWorldCoordinates(this->depthCloud, this->matrixOfIntrinsicParameters);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr rest(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentedPlanes;
		Segmentation::RANSAC3D(depthCloudInWorldCoordinates, rest, segmentedPlanes, 0.8, pcl::SACMODEL_PLANE, pcl::SAC_RANSAC);

#if 0 // pcl:visualization is not available on Bazel, therefore visualization is temporarily disabled
		Visualization3D viewport3D(2, 1);
		viewport3D.AddPointCloudToViewPort(0, 0, depthCloudInWorldCoordinates, "cloud_org");
		for (size_t i = 0; i != segmentedPlanes.size(); i++)
		{
			viewport3D.AddPointCloudToViewPort(1, 0, segmentedPlanes[i], "segmentedPlane_" + std::to_string((long long)i), ColorHelper::GetColorFromIndex(i));
		}

		viewport3D.AddPointCloudToViewPort(1, 0, rest, "rest");

		while (false == viewport3D.viewer->wasStopped())
		{
			if (false == viewport3D.viewer->wasStopped())
			{
				viewport3D.viewer->spinOnce(100);
			}
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
#endif
		return 0;
	}

	Segmentation3DApproach Segmentation3DHelper::QuerySegmentationApproach()
	{
		std::cout << "WELCOME TO THE PLENOPTIC CAMERA DATA SEGMENTATION DEMONSTRATION" << std::endl << std::endl
			<< "Version 0.3" << std::endl
			<< "(c) 2014 University of Applied Sciences, Karlsruhe" << std::endl << std::endl
			<< "This program uses the following libraries: Boost, Eigen, FLANN, OpenCV, PointCloudLibrary, VTK" << std::endl
			<< std::endl
			<< "Please select the approach how you would like to perform the segmentation:" << std::endl
			<< "    1 - Contour detection and then depth fitting " << std::endl
			<< "    2 - Classical approaches (RANSAC on 3D point data)" << std::endl
			<< std::endl

			<< "Please enter a number (1...2) to select an algorithm:" << std::endl;

		unsigned int selection = 0;

		while (!(std::cin >> selection) || selection == 0 || selection > 2)
		{
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			std::cout << "Invalid input. Please select again." << std::endl;
		}

		switch (selection)
		{
		case 1:
			return APPROACH_CONTOUR_FITTING;
		default:
			return APPROACH_CLASSICAL;
		}
	}


	Segmentation3DContourDetectionMethod Segmentation3DHelper::QueryContourDetectionMethod()
	{
		std::cout << "Please select the method to perform the contour detection on the color image:" << std::endl
			<< "    1 - Suzuki/Abe-contour detection algorithm" << std::endl
			<< "    2 - Hough-Transform" << std::endl
			<< "    3 - RANSAC with Quadtree" << std::endl
			<< "    4 - RANSAC (not recommended)" << std::endl
			<< "    5 - Select contours manually" << std::endl
			<< std::endl
			<< "Please enter a number (1...5) to select an algorithm:" << std::endl;

		unsigned int selection = 0;


		while (!(std::cin >> selection) || selection == 0 || selection > 5)
		{
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			std::cout << "Invalid input. Please select again." << std::endl;
		}

		switch (selection)
		{
		case 1:
			return DETECTION_SUZUKI_ABE;
		case 2:
			return DETECTION_HOUGH_TRANSFORM;
		case 3:
			return DETECTION_RANSAC_WITH_QUADTREE;
		case 4:
			return DETECTION_RANSAC;
		default:
			return DETECTION_MANUAL_SELECTION;
		}
	}

	Segmentation3DDepthFittingMethod Segmentation3DHelper::QueryLineFittingMethod()
	{
		std::cout << "Please select the method to fit contours into the 3D data:" << std::endl
			<< "    1 - Minimum variance fitting" << std::endl
			<< "    2 - Iterative minimum variance fitting" << std::endl
			<< "    3 - Least squares fitting" << std::endl
			<< "    4 - Line begin- and end point anveraging" << std::endl
			<< "    5 - No depth fitting, just perform 2D contour detection" << std::endl
			<< std::endl 
			<< "Please enter a number (1...5) to select an algorithm:" << std::endl;

		unsigned int selection = 0;


		while (!(std::cin >> selection) || selection == 0 || selection > 5)
		{
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			std::cout << "Invalid input. Please select again." << std::endl;
		}

		switch (selection)
		{
		case 1:
			return FITTING_MINIMUM_VARIANCE;
		case 2:
			return FITTING_MINIMUM_VARIANCE;
		case 3:
			return FITTING_LEAST_SQUARES;
		case 4:
			return FITTING_BEGIN_AND_END_POINT_AVERAGING;
		default:
			return FITTING_NONE;
		}
	}

	bool Segmentation3DHelper::QueryBool()
	{
		char selection = 'n';
		while (!(std::cin >> selection) || (selection != 'n' && selection != 'y'))
		{
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			std::cout << "Invalid input. Please select again." << std::endl;
		}
		if ('y' == selection)
		{
			return true;
		}
		return false;
	}

	bool Segmentation3DHelper::QueryStoreInFile(std::ofstream &fileStream)
	{
		std::cout << "Would you like to store the results in a text file (y/n)? " << std::endl;

		if (false == QueryBool())
		{
			return false;
		}
		for (;;)
		{
			std::cout << "Please enter a file to store the output data (or press ENTER for the default path)." << std::endl;
			std::string filePath;
			std::cin.clear();
			std::cin.sync();
			std::getline(std::cin, filePath); 

			if (filePath == "")
			{
				filePath = Segmentation3DHelper::DefaultOutputPath;
			}
			if (boost::filesystem::exists(filePath))
			{
				std::cout << "File already exists! Would you like to overwrite (y/n)?" << std::endl;
				if (false == QueryBool())
				{
					continue;
				}
			}

			// try to open file for writing
			fileStream.open(filePath, std::ios::out | std::ios::trunc);

			if (false == fileStream.is_open())
			{
				// file couldn't be opened.
				std::cout << "File could not be opened. Please select a different file." << std::endl;
				continue;
			}
			return true;
		}
	}
}
