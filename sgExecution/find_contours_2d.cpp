#include <algorithm>

#include "sgBase/misc/color.h"
#include "sgBase/geometrics/quadtree.h"

#include "sgMain/3d/transformation_3d.h"
#include "sgMain/3d/fit_line_3d.h"

#include "sgMain/ransac.h"

#include "sgExecution/find_contours_2d.h"

using namespace sgBase;
using namespace sgMain;

namespace sgExecution
{
	FindContours2D::FindContours2D(const cv::Mat &colorImage, const cv::Mat &edgeImage, bool showResults)
		: colorImage(colorImage), edgeImage(edgeImage), showResults(showResults)
	{}

	std::vector<Contour2D> FindContours2D::FindContoursSuzukiAbe()
	{
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(this->edgeImage, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		int minNumOfPoints = 85;


		if (this->showResults)
		{
			// for debug output purpose
			cv::Mat colorImageWithEdges;
			colorImageWithEdges.create(colorImage.size(), colorImage.type());
			colorImage.copyTo(colorImageWithEdges, colorImage);
			int counter = 0;

			for (int i = 0; i < contours.size(); i++)
			{
				// ignore small surfaces
				if (contours[i].size() < minNumOfPoints)
				{
					continue;
				}
				Color currentColor = ColorHelper::GetColorFromIndex(counter);
				counter++;
				cv::Scalar color(currentColor.r(), currentColor.g(), currentColor.b());
				drawContours(colorImageWithEdges, contours, i, color, 2, 8, hierarchy);
			}
			cv::imshow("Detected Contours", colorImageWithEdges);
#if 1
			cv::imwrite("C:\\contours_suzuki_abe.png", colorImageWithEdges);
#endif
		}

		// improve contours by filtering them by size. After that, approximate them by a polygon line
		std::vector<std::vector<cv::Point>> filteredContours;
		for (int i = 0; i < contours.size(); i++)
		{
			// ignore small surfaces
			if (contours[i].size() < minNumOfPoints)
			{
				continue;
			}
			std::vector<cv::Point> newContour;

			cv::approxPolyDP(contours[i], newContour, 7.0, true);
			filteredContours.push_back(newContour);
		}

		std::vector<Contour2D> contours2D;

		for (int i = 0; i < filteredContours.size(); i++)
		{
			Contour2D currentContour2D;

			for (int k = 0; k != filteredContours[i].size() - 1; k++)
			{
				Eigen::Vector3f p1;
				Eigen::Vector3f p2;
				p1[0] = (double)filteredContours[i][k].x / (double)this->edgeImage.size().width;
				p1[1] = (double)filteredContours[i][k].y / (double)this->edgeImage.size().height;
				p1[2] = 0.0f;
				p2[0] = (double)filteredContours[i][k + 1].x / (double)this->edgeImage.size().width;
				p2[1] = (double)filteredContours[i][k + 1].y / (double)this->edgeImage.size().height;
				p2[2] = 0.0f;

				Line3D currentLine(p1, p2);
				currentContour2D.contourLines.push_back(currentLine);
			}
			contours2D.push_back(currentContour2D);
		}
		return contours2D;
	}


	std::vector<Contour2D> FindContours2D::FindContoursRANSAC2D()
	{
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;

		// find contours in the edge image using the openCV method
		cv::findContours(this->edgeImage, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentedLines;

		// use RANSAC to segment lines out of the contour image
		for (int i = 0; i < contours.size(); i++)
		{
			if (contours[i].size() < 100)
			{
				continue;
			}
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = PointCloudHelper::PointCloudFromOpenCVList(contours[i], this->edgeImage);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr rest(new pcl::PointCloud<pcl::PointXYZRGB>);
			Segmentation::RANSAC2D(cloud, rest, segmentedLines, 0.01, 50, 100);
		}

		std::vector<Contour2D> contours2D;

		for (int i = 0; i != segmentedLines.size(); i++)
		{
			Eigen::Vector2f linePoint;
			Eigen::Vector2f lineDirection;

			// search for begin and end points
			if (0 == FitLine3DHelper::FitLine2DIntoPoints2DLeastSquares(segmentedLines[i], linePoint, lineDirection))
			{
				Eigen::Vector3f linePoint3(linePoint.x(), linePoint.y(), 0.0f);
				Eigen::Vector3f lineDirection3(lineDirection.x(), lineDirection.y(), 0.0f);
				Eigen::Vector3f lineBegin;
				Eigen::Vector3f lineEnd;

				Transformation3D::FindLineStartAndEndPoint(segmentedLines[i], linePoint3, lineDirection3, lineBegin, lineEnd);

				Line3D currentLine(lineBegin, lineEnd);
				bool lineExists = false;

				for (size_t k = 0; k != contours2D.size(); k++)
				{
					if (Line3DHelper::AreLinesIdentical(contours2D[k].contourLines[0], currentLine))
					{
						lineExists = true;
						break;
					}
				}

				if (false == lineExists)
				{
					contours2D.push_back(Contour2D(currentLine));
				}
			}
		}
		return contours2D;
	}

	std::vector<Contour2D> FindContours2D::FindContoursRANSAC2DWithQuadtree()
	{
		// convert 2d image to point cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud = PointCloudHelper::ColoredPointCloudFromImage(this->edgeImage);

		// build two quadtrees - one for RANSAC segmentation, another for point density calculations
		Quadtree<pcl::PointXYZRGB> quadtree(CONDITION_LEAF_SIZE, 0, 0.2);

		quadtree.setInputCloud(originalCloud);
		std::vector<double> pointDensityVariances;

		for (int i = 0; i != quadtree.leafNodes.size(); i++)
		{
			pointDensityVariances.push_back(QuadtreeHelper::QuadtreeLeafNodePointDensityVariance(*(quadtree.leafNodes[i])));
			cout << "Point  density variance node " << i << ": " << QuadtreeHelper::QuadtreeLeafNodePointDensityVariance(*(quadtree.leafNodes[i])) << std::endl;
		}


		double maxVariance = *std::max_element(pointDensityVariances.begin(), pointDensityVariances.end());
		double minVariance = *std::min_element(pointDensityVariances.begin(), pointDensityVariances.end());

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr restMap2D(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentedLines;

		for (int i = 0; i != quadtree.leafNodes.size(); i++)
		{
			Segmentation::RANSAC2D(quadtree.leafNodes[i]->getContainerPtr(), restMap2D, segmentedLines, 0.01, 20, 500);
		}

		std::vector<Contour2D> contours2D;

		for (size_t i = 0; i != segmentedLines.size(); i++)
		{
			Eigen::Vector2f linePoint;
			Eigen::Vector2f lineDirection;

			if (0 == FitLine3DHelper::FitLine2DIntoPoints2DLeastSquares(segmentedLines[i], linePoint, lineDirection))
			{

				Eigen::Vector3f linePoint3(linePoint.x(), linePoint.y(), 0.0f);
				Eigen::Vector3f lineDirection3(lineDirection.x(), lineDirection.y(), 0.0f);
				Eigen::Vector3f lineBegin;
				Eigen::Vector3f lineEnd;

				Transformation3D::FindLineStartAndEndPoint(segmentedLines[i], linePoint3, lineDirection3, lineBegin, lineEnd);

				Line3D currentLine(lineBegin, lineEnd);
				bool lineExists = false;

				// try to merge lines, if possible
				for (size_t k = 0; k != contours2D.size(); k++)
				{
					if (Line3DHelper::AreLinesIdentical(contours2D[k].contourLines[0], currentLine))
					{
						lineExists = true;
						break;
					}
				}

				if (false == lineExists)
				{
					Contour2D currentContour(currentLine);
					contours2D.push_back(currentContour);
				}
			}
		}
		return contours2D;
	}

	std::vector<Contour2D> FindContours2D::FindContoursHoughTransform()
	{
		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(this->edgeImage, lines, 1, CV_PI / 180, 60, 50, 10);
		
		std::vector<Contour2D> resultLines;
		double width = (double)this->edgeImage.size().width;
		double height = (double)this->edgeImage.size().height;

		for (size_t i = 0; i < lines.size(); i++)
		{
			cv::Vec4i currentLine = lines[i];
			Line3D currentLineNormalized(Eigen::Vector3f((double)currentLine[0] / width, (double)currentLine[1] / height, 0.0f), Eigen::Vector3f((double)currentLine[2] / width, (double)currentLine[3] / height, 0.0f));
			resultLines.push_back(Contour2D(currentLineNormalized));
		}
		return resultLines;
	}

	std::vector<Contour2D> FindContours2D::FindContoursUsingManualSelection()
	{
		this->manualLineSelectionCurrentPoint = false;
		this->manualLineSelected = false;

		std::cout << "To select a line, select the begin and end point on the color image. Press ENTER to store the selected line. When you are finished, press Q to contiue."
			<< std::endl;

		this->manualLineSelectionWindowName = "Line Selection Window";

		cv::namedWindow(this->manualLineSelectionWindowName);
		cv::setMouseCallback(this->manualLineSelectionWindowName, &FindContours2D::MouseClick, this);

		ManualLineSelectionUpdate();

		//std::vector<Contour2D> contours2D;
		this->selectedLines.clear();

		char keyCode = 0;
		while (keyCode != 'q')
		{
			// wait for the user to select a line
			keyCode = cv::waitKey(0);

			if (keyCode != 'q' && this->manualLineSelected)
			{
				// add the current line to the selection
				cout << this->manualLineSelectionP1 << ", " << this->manualLineSelectionP2;
				Eigen::Vector3f lineP1((float)this->manualLineSelectionP1.x(), (float)this->manualLineSelectionP1.y(), 0.0f);
				Eigen::Vector3f lineP2((float)this->manualLineSelectionP2.x(), (float)this->manualLineSelectionP2.y(), 0.0f);
				Line3D currentLine(lineP1, lineP2);

				std::cout << "Line added" << currentLine << std::endl;

				this->selectedLines.push_back(Line3D(currentLine));
			}
		}

		// close window
		cv::destroyWindow(this->manualLineSelectionWindowName);

		std::vector<Contour2D> contours2D;

		for (auto itr = this->selectedLines.begin(); itr != this->selectedLines.end(); itr++)
		{
			float width = (float)this->colorImage.size().width;
			float height = (float)this->colorImage.size().height;
			
			contours2D.push_back(Contour2D(Line3D(Eigen::Vector3f(itr->p1.x() / width, itr->p1.y() / height, 0.0f), Eigen::Vector3f(itr->p2.x() / width, itr->p2.y() / height, 0.0f))));
		}
		return contours2D;
	}

	static void DrawCrosshair(cv::Mat image, Eigen::Vector2i position)
	{
		cv::line(image, cv::Point(position.x() - 5, position.y()), cv::Point(position.x() + 5, position.y()), cv::Scalar(0, 255, 0), 2, 1);
		cv::line(image, cv::Point(position.x(), position.y() - 5), cv::Point(position.x(), position.y() + 5), cv::Scalar(0, 255, 0), 2, 1);
	}

	void FindContours2D::ManualLineSelectionUpdate()
	{
		cv::Mat currentColorImage;
		currentColorImage.create(this->colorImage.size(), this->colorImage.type());
		this->colorImage.copyTo(currentColorImage);

		if (false == this->manualLineSelected)
		{
			// draw only start point
			DrawCrosshair(currentColorImage, this->manualLineSelectionP1);
		}
		else
		{
			// draw line
			DrawCrosshair(currentColorImage, this->manualLineSelectionP1);
			DrawCrosshair(currentColorImage, this->manualLineSelectionP2);
			cv::line(currentColorImage, cv::Point((int)this->manualLineSelectionP1.x(), (int)this->manualLineSelectionP1.y()), cv::Point((int)this->manualLineSelectionP2.x(), (int)this->manualLineSelectionP2.y()), cv::Scalar(0, 255, 0), 2, 1);
		}

		for (auto itr = this->selectedLines.begin(); itr != this->selectedLines.end(); itr++)
		{
			cv::line(currentColorImage, cv::Point(itr->p1.x(), itr->p1.y()), cv::Point(itr->p2.x(), itr->p2.y()), cv::Scalar(0, 255, 0), 2, 1);
		}
		cv::imshow(this->manualLineSelectionWindowName, currentColorImage);
	}

	void FindContours2D::MouseClick(int event, int x, int y, int flags, void *param)
	{
		FindContours2D* findContours2D = (FindContours2D*)param;
		switch (event)
		{
		case CV_EVENT_LBUTTONDOWN:
			if (false == findContours2D->manualLineSelectionCurrentPoint)
			{
				findContours2D->manualLineSelectionP1[0] = x;
				findContours2D->manualLineSelectionP1[1] = y;
			}
			else
			{
				findContours2D->manualLineSelectionP2[0] = x;
				findContours2D->manualLineSelectionP2[1] = y;
				findContours2D->manualLineSelected = true;
			}
			findContours2D->manualLineSelectionCurrentPoint = !findContours2D->manualLineSelectionCurrentPoint;
			findContours2D->ManualLineSelectionUpdate();
			break;
		}

	}

	int FindContours2D::FindContours(Segmentation3DContourDetectionMethod method, std::vector<Contour2D> &contours)
	{
		switch (method)
		{
		case DETECTION_RANSAC:
			contours = FindContoursRANSAC2D();
			return 0;
		case DETECTION_RANSAC_WITH_QUADTREE:
			contours = FindContoursRANSAC2DWithQuadtree();
			return 0;
		case DETECTION_HOUGH_TRANSFORM:
			contours = FindContoursHoughTransform();
			return 0;
		case DETECTION_SUZUKI_ABE:
			contours = FindContoursSuzukiAbe();
			return 0;
		case DETECTION_MANUAL_SELECTION:
		default:
			contours = FindContoursUsingManualSelection();
			return 0;
		}
	}
}
