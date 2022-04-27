//#####################################################################
// Copyright 2010-2015, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _MOCAP_SKELETON_TO_MBS_H
#define _MOCAP_SKELETON_TO_MBS_H

#include "MocapProcessor/mgSkeleton.h"
/**
    @class		mgSkeletonToBCharacter
    @date		2014/03/01
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		Converting skeleton information to MBS, and other wise.
    @warning	
*/

class mgSkeletonToBCharacter
{
	
public:
	mgSkeletonToBCharacter() {};
	virtual ~mgSkeletonToBCharacter() {};

	// skeleton to MBS: SHL modified 14-05-29
	static int saveToBCharacter(const mgSkeleton *skeleton, const char* filename, double totalMass, double minBoneLength=0.01, double defaultBoneAspectRatio=0.2);

private:

};

#endif
