/*
* (c) 2014 University of Applied Sciences, Karlsruhe
* Project "Segmentation of depth data of a plenoptic camera"
* summer semester 2014
*
* misc.h
* Contains functions which are required at some places but to not fit in a specific category.
*/

#ifndef _MISC_H_
#define _MISC_H_


#include <boost/shared_ptr.hpp>

namespace sgBase
{
	template<typename T>
	boost::shared_ptr<T> makeSharedPtr(std::shared_ptr<T>& ptr)
	{
		return boost::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset(); });
	}

	template<typename T>
	std::shared_ptr<T> makeSharedPtr(boost::shared_ptr<T>& ptr)
	{
		return std::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset(); });
	}
}

#endif