//#####################################################################
// Copyright 2010-2015, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _MOTION_LOADER_H
#define _MOTION_LOADER_H

#include "MocapProcessor/mgData.h"
#include "MocapProcessor/mgSkeleton.h"

/**
    @class		MotionLoader
    @date		2014/02/18
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		This class can load motion files which has skeleton information and motion.
    @warning	
*/

class MotionLoader
{
private:
	enum MOTIONTYPE {
		NONE,
		BVH,
		AMC, 
		FBX,
	};
public:
	MotionLoader():
	  _motions(NULL),
	  _skeleton(NULL),
	  _transScale(1.0),
	  _useSkeletonFile(false)
	{

	};
	~MotionLoader(){};

	int loadMotionFile(const char* filename, const bool skeletonOnly = false);
	int setSkeletonFile(const char* filename);
	
	void setTranslateScale(CoordinateType scale) { _transScale = scale; }
	double getTranslateScale() { return _transScale; }
	
	mgData* getMotion() {return _motions;}
	mgSkeleton* getSkeleton() {return _skeleton;}

	const mgData* getMotion() const { return _motions; }
	const mgSkeleton* getSkeleton() const { return _skeleton; }

private:
	MOTIONTYPE getExtension(const char* filename);

private:
	double _transScale;

	mgData* _motions;
	mgSkeleton* _skeleton;

	bool _useSkeletonFile;
};

#endif