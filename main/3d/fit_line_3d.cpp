#include <fstream>
#include <string>

#include "base/geometrics/point_cloud.h"
#include "main/3d/transformation_3d.h"
#include "main/3d/fit_line_3d.h"


using namespace sgBase;

namespace sgMain
{
	void FitLine3D::print(std::ostream& os) const
	{
		if (isFitted)
		{
			os << "Line 3D length: " << this->fittedLine.Length() << ", 2D length: " << this->line2D.Length() << ", num of depth points: " << this->lineCloud->size() <<  ", 3D variance: " << this->GetVariance();
		}
		else
		{
			os << "Line has not been fitted!";
		}
	}

	FitLine3D::FitLine3D(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double radius)
		: line2D(line2D), depthCloud(depthCloud), intrinsics(intrinsicParameters), radius(radius), isFitted(false)
	{
		this->lineCloud = Line3DHelper::FindClosePoints(line2D, depthCloud, this->radius);
	}

	FitLine3D::~FitLine3D()
	{}

	double FitLine3D::GetVariance() const
	{
		if (0 == this->lineCloud->size())
		{
			return 0.0;
		}
		//calculate variance of the initial fitting
		double fittedLineVariance = 0.0;

		for (auto itr = this->lineCloud->begin(); itr != this->lineCloud->end(); itr++)
		{
			double distance = Line3DHelper::DistancePointToLine(this->fittedLine, PointCloudHelper::Vector3f(*itr));
			fittedLineVariance += distance * distance;
		}

		return fittedLineVariance / (double)this->lineCloud->size();
	}

	Histogram FitLine3D::GetHistogram(unsigned int numberOfBins) const
	{
		if (false == this->IsFitted())
		{
			return Histogram(0.0, 0.0, 1, std::vector<double>());
		}

		std::vector<double> distances;
		double minDistance = std::numeric_limits<double>::max();
		double maxDistance = std::numeric_limits<double>::min();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloudInWorldCoordinates = Transformation3D::TransformFromCameraToWorldCoordinates(this->lineCloud, this->intrinsics);
		for (auto itr = lineCloudInWorldCoordinates->begin(); itr != lineCloudInWorldCoordinates->end(); itr++)
		{
			Eigen::Vector3f currentPoint = PointCloudHelper::Vector3f(*itr);
			double distance = Line3DHelper::DistancePointToLine(this->fittedLine, currentPoint);
			if (distance < minDistance)
			{
				minDistance = distance;
			}
			if (distance > maxDistance)
			{
				maxDistance = distance;
			}
			distances.push_back(distance);
		}

		return Histogram(0.0, 1000.0, numberOfBins, distances);
	}
	bool FitLine3D::IsFitted() const
	{
		return this->isFitted;
	}

#if 0
	void FitLine3D::Render(Visualization3D &viewport3D, int contourIndex, int lineIndex) const
	{
		if (false == this->IsFitted())
		{
			return;
		}

		Color currentColor = ColorHelper::GetColorFromIndex(contourIndex);

		viewport3D.AddLineToViewPort(1, 0, this->fittedLine, "fitted_line_" + std::to_string((long long)contourIndex) + "_" + std::to_string((long long)lineIndex), currentColor);
		viewport3D.AddLineToViewPort(0, 0, this->fittedLine, "fitted_line_org_" + std::to_string((long long)contourIndex) + "_" + std::to_string((long long)lineIndex), currentColor);
	}
#endif

	std::ostream& operator<< (std::ostream& os, const FitLine3D& line)
	{
		line.print(os);
		return os;
	}

	namespace FitLine3DHelper
	{
		int CreateNearestNeighborHistogram(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const pcl::PointXYZRGB &centerPoint, const std::vector<double> &radii, std::string fileName)
		{
			// find min and max depth
			double maxRadii = (*std::max_element(radii.begin(), radii.end()));
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr maxRadiusCloud = PointCloudHelper::NearestNeighbourSearch2D(depthCloud, centerPoint, maxRadii);
			std::vector<double> maxRadiusCloudDepthValues;
			maxRadiusCloudDepthValues.reserve(maxRadiusCloud->size());
			for (auto itr = maxRadiusCloud->begin(); itr != maxRadiusCloud->end(); itr++)
			{
				maxRadiusCloudDepthValues.push_back((double)itr->z);
			}
			double minDepth = (*std::min_element(maxRadiusCloudDepthValues.begin(), maxRadiusCloudDepthValues.end()));
			double maxDepth = (*std::max_element(maxRadiusCloudDepthValues.begin(), maxRadiusCloudDepthValues.end()));

			std::ofstream outputFile;
			outputFile.open(fileName);
			if (false == outputFile.is_open())
			{
				return -1;
			}

			// create a histogram for each given radius value
			for (auto itr = radii.begin(); itr != radii.end(); itr++)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr radiusPointCloud = PointCloudHelper::NearestNeighbourSearch2D(depthCloud, centerPoint, *itr);
				std::vector<double> depthValuesOfBeginCloud;
				depthValuesOfBeginCloud.reserve(radiusPointCloud->size());
				for (auto itr = radiusPointCloud->begin(); itr != radiusPointCloud->end(); itr++)
				{
					depthValuesOfBeginCloud.push_back((double)itr->z);
				}
				Histogram depthHistogram(minDepth, maxDepth, 50, depthValuesOfBeginCloud);

				outputFile << depthHistogram;

			}
			outputFile.close();
			return 0;
		}

		int FitLineIntoPointCloudByAveragingBeginAndEndPoints(const Line3D &line2D, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &depthCloud, const cv::Mat &intrinsicParameters, double maxDistancePointToLine, Line3D &result)
		{
			// find points close to the 2D line on the depth map
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr beginPointCloudOriginal = PointCloudHelper::NearestNeighbourSearch2D(depthCloud, PointCloudHelper::PointXYZRGB(line2D.p1), maxDistancePointToLine);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr endPointCloudOriginal = PointCloudHelper::NearestNeighbourSearch2D(depthCloud, PointCloudHelper::PointXYZRGB(line2D.p2), maxDistancePointToLine);

#if 0
			// compare different search radii and use them to create histograms
			std::vector<double> radii;
			radii.push_back(0.01);
			radii.push_back(0.05);
			radii.push_back(0.07);
			radii.push_back(0.10);
			radii.push_back(0.15);
			radii.push_back(0.20);
			radii.push_back(0.30);
			radii.push_back(0.40);
			CreateNearestNeighborHistogram(depthCloud, PointCloudHelper::PointXYZRGB(line2D.p1), radii, "C:\\histogram.txt");
#endif

			if (beginPointCloudOriginal->size() == 0 || endPointCloudOriginal->size() == 0)
			{
				return -1;
			}


			// transform from camera space to 3D space
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr beginPointCloud = Transformation3D::TransformFromCameraToWorldCoordinates(beginPointCloudOriginal, intrinsicParameters);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr endPointCloud = Transformation3D::TransformFromCameraToWorldCoordinates(endPointCloudOriginal, intrinsicParameters);

			double beginPointCloudAverageZ = PointCloudHelper::AverageZValue(beginPointCloud);
			double endPointCloudAverageZ = PointCloudHelper::AverageZValue(endPointCloud);

			pcl::PointXYZRGB beginPoint = PointCloudHelper::PointXYZRGB(line2D.p1);
			beginPoint.z = PointCloudHelper::AverageZValue(beginPointCloud);

			pcl::PointXYZRGB endPoint = PointCloudHelper::PointXYZRGB(line2D.p2);
			endPoint.z = PointCloudHelper::AverageZValue(endPointCloud);

			Eigen::Vector3f beginPointTransformed = PointCloudHelper::Vector3f(Transformation3D::TransformFromCameraToWorldCoordinates(beginPoint, intrinsicParameters));
			Eigen::Vector3f endPointTransformed = PointCloudHelper::Vector3f(Transformation3D::TransformFromCameraToWorldCoordinates(endPoint, intrinsicParameters));


			result = Line3D(beginPointTransformed, endPointTransformed);
			return 0;
		}

		int FitLine2DIntoPointCloud3DLeastSquares(const Line3D &line2D, Line3D &output, const cv::Mat &intrinsic, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud)
		{
			// just use the minimum variance estimator, but set the importance of each point to 1 - will result in a least-squares estimator
			std::vector<Eigen::Vector3f> points;
			std::vector<unsigned int> pointImportance;
			
			
			for (auto itr = pointCloud->begin(); itr != pointCloud->end(); itr++)
			{
				Eigen::Vector3f currentPoint = PointCloudHelper::Vector3f(*itr);
				points.push_back(currentPoint);
				pointImportance.push_back(1);
			}
			return FitLine2DIntoPointCloud3DMinimumVariance(line2D, output, intrinsic, points, pointImportance);
		}

		int FitLine2DIntoPointCloud3DMinimumVariance(const Line3D &line2D, Line3D &output, const cv::Mat &intrinsic, const std::vector<Eigen::Vector3f> &points, const std::vector<unsigned int> &pointImportance)
		{
			// step 1: create 3D plane by projecting the 2D plane into 3D space
			pcl::PointXYZRGB p1 = PointCloudHelper::PointXYZRGB(line2D.p1);
			pcl::PointXYZRGB p2 = PointCloudHelper::PointXYZRGB(line2D.p1);
			pcl::PointXYZRGB p3 = PointCloudHelper::PointXYZRGB(line2D.p2);
			pcl::PointXYZRGB p4 = PointCloudHelper::PointXYZRGB(line2D.p2);
			p1.z = 1000.0;
			p2.z = 5000.0;
			p3.z = 1000.0;
			p4.z = 5000.0;

			Eigen::Vector3f p1Transformed = PointCloudHelper::Vector3f(Transformation3D::TransformFromCameraToWorldCoordinates(p1, intrinsic));
			Eigen::Vector3f p2Transformed = PointCloudHelper::Vector3f(Transformation3D::TransformFromCameraToWorldCoordinates(p2, intrinsic));
			Eigen::Vector3f p3Transformed = PointCloudHelper::Vector3f(Transformation3D::TransformFromCameraToWorldCoordinates(p3, intrinsic));
			Eigen::Vector3f p4Transformed = PointCloudHelper::Vector3f(Transformation3D::TransformFromCameraToWorldCoordinates(p4, intrinsic));


			// create plane
			Eigen::Hyperplane<float, 3> projectedPlane = Eigen::Hyperplane<float, 3>::Through(p1Transformed, p2Transformed, p3Transformed);


			// step 2: project the point cloud points belonging to the line onto the plane
			std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> projectedPoints;
			Eigen::Vector3f planeNormal = projectedPlane.normal();

			for (auto itr = points.begin(); itr != points.end(); itr++)
			{
				Eigen::Vector3f projectedPoint = Transformation3D::ProjectPointOnPlane(*itr, p1Transformed, planeNormal);
				projectedPoints.push_back(projectedPoint);
			}

			// step 3: create a local coordinate system to reduce the complexity of the fitting from 3D to 2D
			Eigen::Vector3f xNew = (p2Transformed - p1Transformed);
			Eigen::Vector3f zNew = planeNormal;
			Eigen::Vector3f yNew = xNew.cross(zNew);
			xNew.normalize();
			yNew.normalize();

			/*
			This rotation matrix is actually a description of a local coordinate system placed on the projected plane. It is used to
			get x- and y-coordinates for every point of the point cloud on the plane (z is 0 because these points have been
			projected onto the plane).
			use the rotation matrix to transform from the local coordinate system to the world coordinate system and the inverse to
			transform from the world coordinate system to the local coordinate system.
			*/
			Eigen::Matrix<float, 3, 3> rotationMatrix;
			rotationMatrix(0, 0) = xNew.x();
			rotationMatrix(1, 0) = xNew.y();
			rotationMatrix(2, 0) = xNew.z();

			rotationMatrix(0, 1) = yNew.x();
			rotationMatrix(1, 1) = yNew.y();
			rotationMatrix(2, 1) = yNew.z();

			rotationMatrix(0, 2) = zNew.x();
			rotationMatrix(1, 2) = zNew.y();
			rotationMatrix(2, 2) = zNew.z();

			Eigen::Matrix<float, 3, 3> rotationMatrixInv = rotationMatrix.inverse();

			// use p1 as the center point on the plane - coordinates (0/0)
			Eigen::Vector3f planeCenter = rotationMatrixInv * p1Transformed;

			std::vector<Eigen::Vector3f> localVectors;
			for (auto itr = projectedPoints.begin(); itr != projectedPoints.end(); itr++)
			{
				// get the current point as the offset of the plane center
				Eigen::Vector3f currentVec = *itr - p1Transformed;
				//....and rotate it to get the local coordinates
				currentVec = rotationMatrixInv * currentVec;
				localVectors.push_back(currentVec);
			}
			
			// step 4: linear least-square fitting of a line onto the points
			Eigen::Vector2f linePoint;
			Eigen::Vector2f lineDirection;

			if (0 != FitLine3DHelper::FitLine2DIntoPoints2DMinimumVariance(localVectors, pointImportance, linePoint, lineDirection))
			{
				return -1;
			}


			// transform the fitted line back from local coordinates to world coordinates
			Eigen::Vector3f fittedLinePoint = (rotationMatrix * Eigen::Vector3f(linePoint.x(), linePoint.y(), 0.0f)) + p1Transformed;
			Eigen::Vector3f fittedLineDirection = (rotationMatrix * Eigen::Vector3f(lineDirection.x(), lineDirection.y(), 0.0f)); // +p1Transformed;

			// step 5: calculate crossing of the two lines by using a QR decomposition linear equation solver
			Eigen::Vector3f linePoint1 = p1Transformed;
			Eigen::Vector3f lineDirection1 = p2Transformed - p1Transformed;

			Eigen::Vector3f linePoint2 = p3Transformed;
			Eigen::Vector3f lineDirection2 = p4Transformed - p3Transformed;

			Eigen::Matrix<float, 2, 2> A;
			Eigen::Vector2f b;

			A(0, 0) = fittedLineDirection.x();
			A(0, 1) = -lineDirection1.x();
			A(1, 0) = fittedLineDirection.y();
			A(1, 1) = -lineDirection1.y();

			b(0) = (linePoint1.x() - fittedLinePoint.x());
			b(1) = (linePoint1.y() - fittedLinePoint.y());

			Vector2f solution = A.colPivHouseholderQr().solve(b);

			Eigen::Vector3f finalLineStartPoint = linePoint1 + solution.y() * lineDirection1;

			A(0, 0) = fittedLineDirection.x();
			A(0, 1) = -lineDirection2.x();
			A(1, 0) = fittedLineDirection.y();
			A(1, 1) = -lineDirection2.y();

			b(0) = (linePoint2.x() - fittedLinePoint.x());
			b(1) = (linePoint2.y() - fittedLinePoint.y());

			solution = A.colPivHouseholderQr().solve(b);


			Eigen::Vector3f finalLineEndPoint = linePoint2 + solution.y() * lineDirection2;
			output = Line3D(finalLineStartPoint, finalLineEndPoint);

			// fitting went horribly wrong
			if (finalLineEndPoint.z() < 0.0 || finalLineStartPoint.z() < 0.0)
			{
				std::cout << "Fitting failed!" << std::endl;
				return -2;
			}

			return 0;
		}


		int FitLine2DIntoPoints2DLeastSquares(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, Eigen::Vector2f &linePoint, Eigen::Vector2f &lineDirection)
		{
			std::vector<Eigen::Vector3f> eigenPoints;
			for (auto itr = points->begin(); itr != points->end(); itr++)
			{
				Eigen::Vector3f currentPoint = PointCloudHelper::Vector3f(*itr);
				eigenPoints.push_back(currentPoint);
			}
			return FitLine2DIntoPoints2DLeastSquares(eigenPoints, linePoint, lineDirection);
		}
		

		int FitLine2DIntoPoints2DLeastSquares(const std::vector<Eigen::Vector3f> &points, Eigen::Vector2f &linePoint, Eigen::Vector2f &lineDirection)
		{
			/*
			Use a linear estimator to calculate the parameters of a 2D line with the formula
			y = a * x + b
			The estimator used is a least square estimator.
			*/
			double size = (double)points.size();
			double sumSqrX = 0.0;
			double sumX = 0.0;
			double sumXY = 0.0;
			double sumY = 0.0;

			for (auto n = points.begin(); n != points.end(); n++)
			{
				sumX += n->x();
				sumSqrX += n->x() * n->x();
				sumXY += n->x() * n->y();
				sumY += n->y();
			}

			double determinant = size * sumSqrX - (sumX * sumX);
			if (0.0 == determinant)
			{
				// system is not solveable - inverse matrix cannot be calculated!
				return -1;
			}

			// calulate best fitting slope
			double a = (1.0 / determinant) * (size * sumXY - sumX * sumY);

			// calculate a point on the line using the same method
			double b = (1.0 / determinant) * (sumSqrX * sumY - sumX * sumXY);


			if (cvIsNaN(b) || cvIsNaN(a))
			{
				return -2;
			}

			linePoint = Vector2f(0.0, (float)b);
			lineDirection = Vector2f(1.0, (float)a);
			lineDirection.normalize();
			return 0;
		}

		int FitLine2DIntoPoints2DMinimumVariance(const std::vector<Eigen::Vector3f> &points, const std::vector<unsigned int> &pointImportance, Eigen::Vector2f &linePoint, Eigen::Vector2f &lineDirection)
		{
			/* 
			the following algorithm is based on an minimum variance estimator. It is created using the following formulas:
			y = X*b + e
			b_estiamted =(X' * Ve^-1 * X)^-1 * X' * Ve^-1 * y
			y = vector of results, b = vector of unknown parameters, X = known matrix, e errors, Ve measuring errors covariance matrix 
			*/
			
			if (points.size() != pointImportance.size())
			{
				// parameter array size mismatch
				return -1;
			}

			double size = (double)points.size();
			double sumVe = 0.0;
			double sumXYVe = 0.0;
			double sumYVe = 0.0;
			double sumXVe = 0.0;
			double sumXY = 0.0;
			double sumSqrXVe = 0.0;
			double sumSqrXSqrVe = 0.0;
		

			for (int i = 0; i != points.size(); i++)
			{
				double Ve = (double)pointImportance[i];
				sumVe += Ve;
				sumXYVe += Ve * points[i].x() * points[i].y();
				sumYVe += points[i].y() * Ve; 
				sumXVe += points[i].x() * Ve;
				sumXY += points[i].x() * points[i].y(); 
				sumSqrXVe += points[i].x() * points[i].x() * Ve;
				sumSqrXSqrVe = points[i].x() * points[i].x() * Ve * Ve;
			}

			double determinant = sumSqrXVe * sumVe - sumXVe * sumXVe;
			if (0.0 == determinant)
			{
				// system is not solveable - inverse matrix cannot be calculated!
				return -1;
			}

			// calulate best fitting slope
			double a = (1.0 / determinant) * (sumVe * sumXYVe - sumXVe * sumYVe); 

			// calculate a point on the line using the same method
			double b = (1.0 / determinant) * (-sumXVe * sumXYVe + sumSqrXVe*sumYVe); 


			if (cvIsNaN(b) || cvIsNaN(a))
			{
				return -2;
			}

			linePoint = Vector2f(0.0, (float)b);
			lineDirection = Vector2f(1.0, (float)a);
			lineDirection.normalize();
			return 0;
		}
	}
}

