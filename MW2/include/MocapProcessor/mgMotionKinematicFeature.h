//#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _MOTION_KINEMATIC_FEATURE_H
#define _MOTION_KINEMATIC_FEATURE_H

#include "Character/bCharacter.h"
#include "MocapProcessor/mgSkeleton.h"

/**
    @class		mgMotionKinematicFeature
    @date		2014/03/08
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		Motion MBS Interface Header. MMI is an interface between MBS and Motion data. Moving MBS bones and exchanging each format, and extract useful features from motion.
    @warning	
*/



typedef std::vector<std::string>							StanceList;
typedef std::vector<std::string>::iterator					StanceListIt;

class mgMotionKinematicFeature
{
public:
	enum {
		RT_OK			= 0x00,
		RT_NOTOK		= 0x02,
		RT_NOINIT		= 0x04,
		RT_OPENERR		= 0x08,
		RT_DUPLICATE	= 0x10,
		RT_OVERINDEX	= 0x20,
		RT_NULLP		= 0x40,
		RT_INSUFFICIENT	= 0x80
	} RT;

public:
	mgMotionKinematicFeature(mgSkeleton* skel, mgData* motion = NULL);// TODO: mgData->motion, nMotion
	//mgMotionKinematicFeature(bCharacter* avatar, const arma::mat& coordMotion);
	virtual ~mgMotionKinematicFeature() {};

	int setMotionData(mgData *motion);
	int addContactCandidate(std::string boneName);
	int addContactCandidate(int boneID);

	// Constraint
	int setContactParameter(double constVelThres, int sizeHalfWindow);
	int detectConstraint();
	int detectConstraintWithGroundLimit(double limit);
	int detectConstraint(std::string constName);
	int detectConstraintWithGroundLimit(std::string constName, double limit);

	// Draw
	int getContactFlagMap(boolArrayMap& flagMap);
	boolArrayMap& getContactFlagMap() { return contactsFlagMap; }
	int getContactPositionMap(std::vector<gVec3>& posMap);
	std::vector<gVec3Array>& getContactPositionMap() { return positionsVec; }


	// position
	//int getMotionWPositionMap(const std::vector<int>& bonelist, gVec3ArrayMap& posMap);
	int getMotionAllWPositionMap(gVec3ArrayMap& posVec);
	int getPosIdxWithCtIdx(const int boneId, const int contactIndex);

	int getMotionWMap(const std::vector<int>& bonelist, gVec3ArrayMap& posMap);
	int getMotionWMap(const std::vector<int>& bonelist, gXMatArrayMap& xMap);

	mgSkeleton* getAssignedSkeleton() { return _skeleton; }
	mgData* getAssignedMotion() { return _motion; }


	// contact prob
	int procContactProbability();
	int getContactProbability(int boneId, std::vector<double>& cp, int filterSize, int numOfApply);

//private:
	int getAllWPosition(std::string constName, gVec3Array &data);
	int getAllW(const int id, gVec3Array &data);
	int getAllW(const int id, gXMatArray &data);

	int boolFiltering(boolArray &data);
	static int boolFiltering(boolArray &data, unsigned int sizeofHalfL1);
	int detectContact(gVec3Array &position, boolArray &cFlag, bool useLimit, double limit);

private:
	mgSkeleton* _skeleton;
	mgData* _motion;

	// parameter
	double _constVelThres;
	int _sizeHalfWindow;

	//std::map<int, boolArray> contactsFlagMap;
	boolArrayMap contactsFlagMap;

	//gVec3ArrayMap positionsMap;
	std::vector<gVec3Array> positionsVec;

	//intArrayMap cIdxTopIdx;
	std::vector<intArray> cIdxTopIdx;

	StanceList _stanceCandidate;

	std::vector<doubleArray> _contactProb;
};

#endif
