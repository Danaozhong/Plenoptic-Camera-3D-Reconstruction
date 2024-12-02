#ifndef _IMAGE_PROC_2D_H_
#define _IMAGE_PROC_2D_H_

#include <vector>
#include <string>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>


#include "base/image.h"

using namespace sgBase;

namespace sgMain
{
	namespace ImageProc2D
	{
		int SaveColorImageWithDepthMarkings(const Image &colorImage, const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &depthMaps, const std::string &fileName)
		{
			Image imageToStore(colorImage);
			for (auto vectorItr = depthMaps.begin(); vectorItr != depthMaps.end(); vectorItr++)
			{
				Color currentColor = ColorHelper::GetColorFromIndex(vectorItr - depthMaps.begin());

				for (auto mapItr = (*vectorItr)->begin(); mapItr != (*vectorItr)->end(); mapItr++)
				{
					if (mapItr->x > 1.0 || mapItr->x < 0.0 || mapItr->y > 1.0 || mapItr->y < 0.0)
					{
						// size does not match
						return -1;
					}
					size_t position = (size_t)(mapItr->y * (float)imageToStore.height) * imageToStore.width + (size_t)(mapItr->x * (float)imageToStore.width);

					// set current pixel color
					imageToStore.imageData[4 * position + 0] = 255; // currentColor.GetRed();
					imageToStore.imageData[4 * position + 1] = 255; // currentColor.g();
					imageToStore.imageData[4 * position + 2] = 255; // currentColor.b();
				}
			}

			imageToStore.Store(fileName);
			return 0;
		}
	}
}

#endif


