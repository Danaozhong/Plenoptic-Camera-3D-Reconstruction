//==============================================================================
//
// Title:       quadtree.h
// Purpose:     This class is an implementation of a QuadTree which is hold very 
//				closely to the OctTree data structure used by the PointCloudLibrary.
//				This has been done to make it as easy as possible to perform OctTree
//				or QuadTree operations on the same data without having to change
//				much of the source code.
//
// Created on:  05/11/2014 at 7:23:55 PM by Clemens Zangl.
// Copyright:   University of Applied Sciences, Karlsruhe. All Rights Reserved.
//
//==============================================================================

#ifndef _QUADTREE_H_
#define _QUADTREE_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>

#include "sgBase/misc/math_statistics.h"
#include "sgBase/geometrics/point_cloud.h"


namespace sgBase
{
	enum node_type_t
	{
		BRANCH_NODE, LEAF_NODE
	};

	class QuadtreeNode
	{
	public:
		QuadtreeNode()
		{
		}

		virtual ~QuadtreeNode()
		{
		}

		virtual node_type_t getNodeType() const = 0;

		virtual QuadtreeNode* deepCopy() const = 0;
	};

	template<typename ContainerT>
	class QuadtreeLeafNode : public QuadtreeNode
	{
	public:

		/** \brief Empty constructor. */
		QuadtreeLeafNode() 
			: QuadtreeNode()
		{
			this->container_ = pcl::PointCloud<ContainerT>::Ptr(new pcl::PointCloud<ContainerT>);
		}

		/** \brief Copy constructor. */
		QuadtreeLeafNode(const QuadtreeLeafNode& source) 
			: QuadtreeNode()
		{
			container_ = source.container_;
		}

		/** \brief Empty deconstructor. */
		virtual ~QuadtreeLeafNode()
		{
		}

		/** \brief Method to perform a deep copy of the octree */
		virtual QuadtreeLeafNode<ContainerT>* deepCopy() const
		{
			return new QuadtreeLeafNode<ContainerT>(*this);
		}

		/** \brief Get the type of quadtree node. Returns LEAVE_NODE type */
		virtual node_type_t getNodeType() const
		{
			return LEAF_NODE;
		}

		/** \brief Get const pointer to container */
		const typename pcl::PointCloud<ContainerT>::Ptr operator->() const
		{
			return container_;
		}

		/** \brief Get pointer to container */
		typename pcl::PointCloud<ContainerT>::Ptr operator-> ()
		{
			return container_;
		}


		/** \brief Get const reference to container */
		const typename pcl::PointCloud<ContainerT> getContainer() const
		{
			return *container_;
		}

		/** \brief Get reference to container */
		typename pcl::PointCloud<ContainerT> getContainer()
		{
			return *container_;
		}

		/** \brief Get const pointer to container */
		const typename pcl::PointCloud<ContainerT>::Ptr getContainerPtr() const
		{
			return container_;
		}

		/** \brief Get pointer to container */
		typename pcl::PointCloud<ContainerT>::Ptr getContainerPtr()
		{
			return container_;
		}

	protected:
		typename pcl::PointCloud<ContainerT>::Ptr container_;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Vector2d bottomLeft;
		Eigen::Vector2d upperRight;
	};

	template<typename ContainerT>
	class QuadtreeBranchNode : public QuadtreeNode
	{
	public:

		/** \brief Default constructor. */
		QuadtreeBranchNode() :
			QuadtreeNode()
		{
			// reset pointer to child node vectors
			memset(child_node_array_, 0, sizeof(child_node_array_));
		}

		/** \brief Copy constructor. TODO Implementation 
		QuadtreeBranchNode(const QuadtreeBranchNode& source) :
			QuadtreeNode()
		{
			unsigned char i;
			/*
			memset(child_node_array_, 0, sizeof(child_node_array_));

			for (i = 0; i < 8; ++i)
			if (source.child_node_array_[i])
				child_node_array_[i] = source.child_node_array_[i]->deepCopy();
				
		}
		*/

		/** \brief Copy operator. */
		inline QuadtreeBranchNode& operator = (const QuadtreeBranchNode &source)
		{
			unsigned char i;

			memset(child_node_array_, 0, sizeof(child_node_array_));

			for (i = 0; i < 8; ++i)
			if (source.child_node_array_[i])
				child_node_array_[i] = source.child_node_array_[i]->deepCopy();
			return (*this);
		}

		/** \brief Quadtree deep copy method */
		virtual QuadtreeBranchNode* deepCopy() const
		{
			return (new QuadtreeBranchNode<ContainerT>(*this));
		}

		/** \brief Empty destructor. */
		virtual ~QuadtreeBranchNode()
		{
		}

		/** \brief Access operator.
		*  \param child_idx_arg: index to child node
		*  \return QuadtreeNode pointer
		* */
		inline std::shared_ptr<QuadtreeNode> operator[] (unsigned char child_idx_arg)
		{
				assert(child_idx_arg < 4);
				return child_node_array_[child_idx_arg];
			}

		/** \brief Get pointer to child
		*  \param child_idx_arg: index to child node
		*  \return QuadreeNode pointer
		* */
		inline std::shared_ptr<QuadtreeNode> getChildPtr(unsigned char child_idx_arg) const
		{
				assert(child_idx_arg < 4);
				return child_node_array_[child_idx_arg];
			}

		/** \brief Get pointer to child
		*  \return QuadreeNode pointer
		* */
		inline void setChildPtr(std::shared_ptr<QuadtreeNode> child, unsigned char index)
		{
			assert(index < 4);
			child_node_array_[index] = child;
		}


		/** \brief Check if branch is pointing to a particular child node
		*  \param child_idx_arg: index to child node
		*  \return "true" if pointer to child node exists; "false" otherwise
		* */
		inline bool hasChild(unsigned char child_idx_arg) const
		{
			return (child_node_array_[child_idx_arg] != 0);
		}


		/** \brief Get the type of quadtree node. Returns LEAVE_NODE type */
		virtual node_type_t	getNodeType() const
		{
			return BRANCH_NODE;
		}

		// reset node
		void reset()
		{
			memset(child_node_array_, 0, sizeof(child_node_array_));
			container_.reset();
		}


		/** \brief Get const pointer to container */
		const ContainerT* operator->() const
		{
			return &container_;
		}

		/** \brief Get pointer to container */
		ContainerT* operator-> ()
		{
			return &container_;
		}

		/** \brief Get const reference to container */
		const ContainerT& operator* () const
		{
			return container_;
		}

		/** \brief Get reference to container */
		ContainerT& operator* ()
		{
			return container_;
		}

		/** \brief Get const reference to container */
		const ContainerT& ngetContainer() const
		{
			return container_;
		}

		/** \brief Get reference to container */
		ContainerT& getContainer()
		{
			return container_;
		}

		/** \brief Get const pointer to container */
		const ContainerT* getContainerPtr() const
		{
			return &container_;
		}

		/** \brief Get pointer to container */
		ContainerT* getContainerPtr()
		{
			return &container_;
		}


	protected:
		std::shared_ptr<QuadtreeNode> child_node_array_[4];

		//ContainerT container_;
	};


	enum QuadtreeSubdivisionCondition
	{
		CONDITION_LEAF_SIZE, CONDITION_NUMBER_OF_POINTS
	};

	template<typename PointT>
	class Quadtree
	{
	public:
		// public point cloud typedefs
		typedef boost::shared_ptr<pcl::PointCloud<PointT>> PointCloudConstPtr;

		Quadtree(QuadtreeSubdivisionCondition condition, unsigned int maximumNumberOfPoints, double maximumLeafSize)
			: maxNumOfPointsPerNode(maximumNumberOfPoints), maxLeafSize(maximumLeafSize), condition(condition)
		{}


		virtual ~Quadtree()
		{}

		inline void setInputCloud(const PointCloudConstPtr &cloud_arg) //, const IndicesConstPtr &indices_arg = IndicesConstPtr())
		{
			input_ = cloud_arg;
			//indices_ = indices_arg;
			std::shared_ptr<QuadtreeLeafNode<PointT>> master(new QuadtreeLeafNode<PointT>());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr nodeDataContainer = master->getContainerPtr();

			for (auto itr = input_->begin(); itr != input_->end(); itr++)
			{
				nodeDataContainer->push_back(*itr);
			}
			
			PointCloudSizeInformation cloudSize(input_);
			Eigen::Vector2d bottomLeft(cloudSize.min.x(), cloudSize.min.y());
			Eigen::Vector2d upperRight(cloudSize.max.x(), cloudSize.max.y());

			this->masterNode = CalculateQuadtree(master, bottomLeft, upperRight);

		}
		
		std::vector<std::shared_ptr<QuadtreeLeafNode<PointT>>> leafNodes;

	protected:
		std::shared_ptr<QuadtreeNode> CalculateQuadtree(std::shared_ptr<QuadtreeLeafNode<PointT>> currentNode, const Eigen::Vector2d &bottomLeft, const Eigen::Vector2d &upperRight)
		{
			pcl::PointCloud<PointT>::Ptr dataContainer = currentNode->getContainerPtr();

			Eigen::Vector2d currentLeafSize = upperRight - bottomLeft;


			if ((CONDITION_NUMBER_OF_POINTS == this->condition && dataContainer->size() < maxNumOfPointsPerNode)
				|| (CONDITION_LEAF_SIZE == this->condition && currentLeafSize.x() < this->maxLeafSize))
			{
				currentNode->bottomLeft = bottomLeft;
				currentNode->upperRight = upperRight;
				this->leafNodes.push_back(currentNode);
				return currentNode;
			}

			// Number of points is too high - replace the current node by a branch node and create four new leaf nodes
			std::shared_ptr<QuadtreeLeafNode<PointT>> childNodes[4];
			childNodes[0] = std::shared_ptr<QuadtreeLeafNode<PointT>>(new QuadtreeLeafNode<PointT>);
			childNodes[1] = std::shared_ptr<QuadtreeLeafNode<PointT>>(new QuadtreeLeafNode<PointT>);
			childNodes[2] = std::shared_ptr<QuadtreeLeafNode<PointT>>(new QuadtreeLeafNode<PointT>);
			childNodes[3] = std::shared_ptr<QuadtreeLeafNode<PointT>>(new QuadtreeLeafNode<PointT>);

			pcl::PointCloud<PointT>::Ptr nodeBottomLeftDataContainer = childNodes[0]->getContainerPtr();
			pcl::PointCloud<PointT>::Ptr nodeBottomRightDataContainer = childNodes[1]->getContainerPtr();
			pcl::PointCloud<PointT>::Ptr nodeUpperLeftDataContainer = childNodes[2]->getContainerPtr();
			pcl::PointCloud<PointT>::Ptr nodeUpperRightDataContainer = childNodes[3]->getContainerPtr();

			double height = (upperRight - bottomLeft).x();
			double width = (upperRight - bottomLeft).y();
			Eigen::Vector2d center = 0.5 * bottomLeft + 0.5 * upperRight;
			Eigen::Vector2d upperMiddle(bottomLeft.x() + 0.5 * width, bottomLeft.y() + height);
			Eigen::Vector2d bottomMiddle(bottomLeft.x() + 0.5 * width, bottomLeft.y());
			Eigen::Vector2d leftMiddle(bottomLeft.x(), bottomLeft.y() + 0.5 * height);
			Eigen::Vector2d rightMiddle(bottomLeft.x() + width, bottomLeft.y() + 0.5 * height);
			
			for (int i = 0; i != dataContainer->size(); i++) 
			{
				PointT currentPoint = dataContainer->at(i);

				if (currentPoint.x < center.x())
				{
					if (currentPoint.y < center.y())
					{
						nodeBottomLeftDataContainer->push_back(currentPoint);
					}
					else
					{
						nodeUpperLeftDataContainer->push_back(currentPoint);
					}
				}
				else
				{
					if (currentPoint.y < center.y())
					{
						nodeBottomRightDataContainer->push_back(currentPoint);
					}
					else
					{
						nodeUpperRightDataContainer->push_back(currentPoint);
					}
				}
			}

			// after all nodes have been filled, process child nodes
			std::shared_ptr<QuadtreeBranchNode<PointT>> newNode(new QuadtreeBranchNode<PointT>);

			(*newNode)[0] = CalculateQuadtree(childNodes[0], bottomLeft, center);
			(*newNode)[1] = CalculateQuadtree(childNodes[1], bottomMiddle, rightMiddle);
			(*newNode)[2] = CalculateQuadtree(childNodes[2], leftMiddle, upperMiddle);
			(*newNode)[3] = CalculateQuadtree(childNodes[3], center, upperRight);
			return newNode;
		}
		
		unsigned int maxNumOfPointsPerNode;
		double maxLeafSize;
		QuadtreeSubdivisionCondition condition;
		
		PointCloudConstPtr input_;

		std::shared_ptr<QuadtreeNode> masterNode;
	};

	
	namespace QuadtreeHelper
	{
		int GetNumberOfPointsInArea(const QuadtreeLeafNode<pcl::PointXYZRGB> &leafNode, const Eigen::Vector2d &bottomLeft, const Eigen::Vector2d &upperRight);
		double QuadtreeLeafNodePointDensityVariance(const QuadtreeLeafNode<pcl::PointXYZRGB> &leafNode);
	}
	

}
#endif