//#####################################################################
// Copyright 2010-2015, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _BVH_LOADER_H
#define _BVH_LOADER_H

#include "Loader/LoaderBase.h"
#include "MocapProcessor/mgSkeleton.h"
/**
    @class		BVHLoader
    @date		2014/02/18
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		load BVH motion file.
    @warning	
*/
class BVHLoader : public LoaderBase
{
public:
	BVHLoader(){};
	~BVHLoader(){};

	int loadMotion(const char* filename, mgSkeleton *skeleton, mgData *motion);
	int loadSkeleton(const char* filename, mgSkeleton *skeleton);

private:	
	int	readSkeleton(mgSkeleton *skeleton, mgBone* bone);
	int	readMotion(mgSkeleton *skeleton, mgData *motion);
	
	void alterChannel( std::string name, unsigned int& value, std::string& orderTxt );

private:
	int nChannels;
	//int nCoords;

	// temp
	int tDataPos;
};

#endif