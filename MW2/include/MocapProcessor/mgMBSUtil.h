//#####################################################################
// Copyright 2017, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _MOTION_MBS_UTILITY_H
#define _MOTION_MBS_UTILITY_H

//#if defined(WIN32) || defined(WIN64)
//	#include <Windows.h>
//#endif

#include "Base/gMath.h"
#include "MocapProcessor/mgSkeleton.h"
#include "mbs/gMultibodySystem.h"

/**
    @class		mgMBSUtil
    @date		2017/09/13
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		Static Utility class for motion processing
    @warning	
*/

namespace mgMBSUtil
{
	int setPoseFromRawData(gMultibodySystem *mbs, mgSkeleton* skeleton, CoordinateType *data);
	int setPoseFromRawData(gMultibodySystem *mbs, double* data);
	int getCoordArrayFromRawData(arma::vec& coord, const gMultibodySystem *mbs, mgSkeleton* skeleton, const CoordinateType *data, const arma::vec* ref = NULL);

	int getCoordArrayFromRawData2(arma::vec& coord, gMultibodySystem *mbs, mgSkeleton* skeleton, const CoordinateType *data, const arma::vec* ref = NULL);

	int getCompactCoordFromPose(arma::vec& coord, gMultibodySystem *mbs, const arma::vec& ref);
	
	int setPoseFromRawExpData(gMultibodySystem *mbs, mgSkeleton* skeleton, CoordinateType *data);


	int getQuatCoordArrayFromRawData(arma::vec& coord, const gMultibodySystem *mbs, const mgSkeleton* skeleton, const CoordinateType *data);
};

#endif