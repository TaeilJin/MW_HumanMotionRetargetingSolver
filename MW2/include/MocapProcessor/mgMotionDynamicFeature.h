//#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _MOTION_DYNAMIC_FEATURE_H
#define _MOTION_DYNAMIC_FEATURE_H

#include "Character/bCharacter.h"
#include "MocapProcessor/mgSkeleton.h"
/**
    @class		mgMotionDynamicFeature
    @date		2014/03/08
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		Motion MBS Interface Header. MMI is an interface between MBS and Motion data. Moving MBS bones and exchanging each format, and extract useful features from motion.
    @warning	
*/

class mgMotionDynamicFeature
{
	
public:
	mgMotionDynamicFeature(bCharacter* ch, mgSkeleton* skel):_character(ch), _skeleton(skel) {};
	virtual ~mgMotionDynamicFeature() {};

	//int getContactBodyPatch(boolArrayMap& contactFlag, mgData* motion);
	
	//2014.12.30 SHL: commented out due to referring to MotionData of MotionGraph.
	//This should be fixed so as not to use MotionGraph functions
	
	//int getContactBodyPatch(MotionData *motionData); 	
	//int getMomentumTrajectory(std::vector<gWrench> &traj, MotionData *motionData);

private:
	bCharacter* _character;
	mgSkeleton* _skeleton;

};

#endif
