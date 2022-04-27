//#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _MOTION_UTILITY_H
#define _MOTION_UTILITY_H

#include "Base/gMath.h"
#include "MocapProcessor/mgSkeleton.h"
//#include "mbs/gMultibodySystem.h"

/**
    @class		mgUtility
    @date		2014/03/01
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		Static Utility class for motion processing
    @warning	
*/

namespace mgUtility
{
	// Utils
	gVec3 getXYZFromRotation(const gRotMat &rotMat);
	gVec3 getXZYFromRotation(const gRotMat &rotMat);
	gVec3 getYXZFromRotation(const gRotMat &rotMat);
	gVec3 getYZXFromRotation(const gRotMat &rotMat);
	gVec3 getZXYFromRotation(const gRotMat &rotMat);
	gVec3 getZYXFromRotation(const gRotMat &rotMat);
	
	int getAngleFromRotMat(gVec3 Axis, const gRotMat &R, double& outAngle);

	int getTransAndRotMatFromData(const CoordinateType* data, mgBone* bone, gVec3& transVec, gRotMat& rotMat, double scale=1.0);
	int getTransAndRotVecFromData(const CoordinateType* data, mgBone* bone, gVec3& transVec, gVec3& rotVec, double scale=1.0);
	int getRotMatFromRotVec(const mgBone* bone, const gVec3& rotVec, gRotMat& rotMat);
	int setRotMatToRotVec(const mgBone* bone, gVec3& rotVec, const gRotMat& rotMat);

	int getRotMatFromRotVec(const char* buf, mgBone::_EULERANGLECONV eulerConv, const gVec3& rotVec, gRotMat& rotMat);

	//int getRotVecFromRotMat(const mgBone* bone, gVec3& rotVec, const gRotMat& rotMat);
	int getRotVecFromRotMat(const mgBone* bone, double* data, const gRotMat& rotMat);

	// test
	int getStringFromMat(std::string& buf, gXMat& mat, const char* delim=" ");
	int getStringFromVec(std::string& buf, gVec3& vec, const char* delim=" ");

	// set
	int setTransAndRotMatToData(CoordinateType* data, const mgBone* bone, const gVec3& transVec, const gRotMat& rotMat, double scale=1.0);
	int setTransAndRotVecToData(CoordinateType* data, const mgBone* bone, const gVec3& transVec, const gVec3& rotVec, double scale=1.0);

	int saveMotionToAMCFile(const char* filename, const mgSkeleton* skeleton, const mgData* motion);
	int saveMotionToAMCFile(const char* filename, const mgSkeleton* skeleton, const arma::mat& motion);
	int convertQuaternionPose(double* quatPose, const double* inPose, const mgSkeleton* skeleton);
};

#endif