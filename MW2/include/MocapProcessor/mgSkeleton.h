//#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _MOTION_SKELETON_H
#define _MOTION_SKELETON_H

#include "Base/gMath.h"
#include "MocapProcessor/mgData.h"
#include "armadillo"

typedef std::vector<int>				BoneDataPos;
typedef std::vector<int>::iterator		BoneDataPIt;

//typedef std::map<int, gXMat>			gXMatList;
//typedef std::map<int, gXMat>::iterator	gXMatListIt;
typedef std::list<gXMat>				gXMatList;
typedef std::list<gXMat>::iterator		gXMatListIt;

//typedef std::vector<gXMat>				gXMatVec;
//typedef std::vector<gXMat>::iterator	gXMatVecIt;


typedef std::vector<gVec3>				gVec3Array;
typedef std::vector<gVec3>::iterator	gVec3ArrayIt;

class mgSkeleton;

class gXMatVec
{
public:
	gXMatVec(void) :
		_nCount(0)
	{
	}

	gXMatVec(std::size_t nCount) :
		data(nCount * gXMat::SIZE_INDEX), _nCount(nCount)
	{
	}

	gXMatVec(std::size_t nCount, double *p ) :
		data(p, nCount * gXMat::SIZE_INDEX, false), _nCount(nCount)
	{
	}

	/*gXMat& operator[] (unsigned int idx)
	{
		return gXMat(data.memptr() + idx);
	}*/

	gXMat operator[] (std::size_t idx) const
	{
		return gXMat(data.memptr() + (idx*gXMat::SIZE_INDEX));
	}

	gXMat operator[] (std::size_t idx)
	{
		return gXMat(data.memptr() + (idx*gXMat::SIZE_INDEX), false);
	}

	std::size_t size() const
	{
		return _nCount;
	}

	void resize(std::size_t size)
	{
		_nCount = size;
		data.resize(_nCount * gXMat::SIZE_INDEX);
	}

private:
	arma::vec data;
	std::size_t _nCount;
};

class mgBone {
public:
	static const unsigned int Xposition = 0x01;
    static const unsigned int Yposition = 0x02;
    static const unsigned int Zposition = 0x04;
    static const unsigned int Zrotation = 0x08;
    static const unsigned int Xrotation = 0x10;
    static const unsigned int Yrotation = 0x20;
	static const unsigned int Xquat = 0x40;
	static const unsigned int Yquat = 0x80;
	static const unsigned int Zquat = 0x100;
	static const unsigned int Wquat = 0x200;
	static const unsigned int Xexp = 0x400;
	static const unsigned int Yexp = 0x800;
	static const unsigned int Zexp = 0x1000;

	static const unsigned int translateChannel = Xposition | Yposition | Zposition;
	static const unsigned int rotationChannel = Zrotation | Xrotation | Yrotation;
	static const unsigned int quaternionChannel = Xquat | Yquat | Zquat | Wquat;
	static const unsigned int expChannel = Xexp | Yexp | Zexp;

	enum _AXISORDER {
		XYZ,
		XZY,
		YXZ,
		YZX,
		ZXY,
		ZYX,
		XY, XZ,
		YX, YZ,
		ZX, ZY,
		X, Y, Z
	};

	//Euler angle convention (http://en.wikipedia.org/wiki/Euler_angles) 
	//Set intrinsic by default. ASF format useus EXTRINSIC while BVH format uses INTRINSIC convention for Euler Angles.
	enum _EULERANGLECONV{
		INTRINSIC,
		EXTRINSIC
	};

public:
	mgBone(): mgBone(-1, NULL)
	{
	};

private:
	//mgBone(int boneId) {//, mgSkeleton *skel) {
	mgBone(int boneId, mgSkeleton *skel):skeleton(skel) {
		id = boneId;
		parent = NULL;
		channel = 0;
		nChannel = 0;
		H.setIdentity();
		order = XYZ;
		length = 0;
		eulerConv = INTRINSIC;
	};

public:
	int id;

	mgBone* parent;
	std::vector<mgBone*> children;
	unsigned int channel; ///< Channel bit flag
	unsigned int nChannel; ///< # of channels
	std::string name;
	std::vector<std::pair<double,double>> limit; ///< limit each dof
	
	std::vector<gVec3> localEndPoints; //Typically, ASF format has only one end point per bone. BHV may have many end points (e.g., pelvis) per bone.
	gVec3 direction_w; ///< direction to the end point wrt world frame
	double length;

	gXMat H; ///< Initial pose wrt parent frame.

	_AXISORDER order;
	_EULERANGLECONV  eulerConv; //valid only for Euler anlges (XYZ,XZY,YXZ,YZX,ZXY,ZYX axisorder)

	const mgSkeleton *skeleton;
	friend class mgSkeleton;
};

/**
    @class		mgSkeleton
    @date		2014/02/18
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		A data structure for a skeleton. It manages bones with a id vector and name map.
    @warning	
*/
class mgSkeleton
{
public:
	// BVHBone Id -> MBS Bone Id
	typedef std::map<int, int>				RetargetMap;
	typedef RetargetMap::iterator			RetargetMIt;

	typedef std::vector<int>				RetargetVec;
	typedef std::vector<int>::iterator		RetargetVIt;


	typedef std::vector <mgBone*>			BONEVec;
	typedef std::vector <mgBone*>::iterator	BONEVIt;

	typedef std::map	<std::string, int>				BONEMap;
	typedef std::map	<std::string, int>::iterator	BONEMIt;
	
	enum {
		RT_OK		= 0x00,
		RT_NULLP	= 0x02,
		RT_NOINIT	= 0x04,
		RT_OPENERR	= 0x08
	} RT;

public:
	mgSkeleton() {
		boneRoot = NULL;
		nTotalChannel = 0;
		numT = numR = 0;
	};

	mgSkeleton(const mgSkeleton& skel);

	virtual ~mgSkeleton() {};

	mgBone* createBone(std::string name);

	int addBone(mgBone* bone);
	int addChild(mgBone* parent, mgBone* child);

	// IK
	int ik_with_jacobian(int endEffectorId, int baseId, gXMat tgtMat, CoordinateType* mFrame, bool onlyTrn = false);
	int get_jacobian(arma::mat& J, int endEffectorId, int baseId, CoordinateType *mFrame);
	
	int ik_with_elbowModel(int endEffectorId, int baseId, gXMat tgtMat, CoordinateType* mFrame);
	int get_elbow_jacobian(arma::mat& J, int endEffectorId, int baseId, CoordinateType *mFrame);

	int ik_with_baseJacobian(int endEffectorId, gXMat tgtMat, CoordinateType* mFrame);
	int get_base_jacobian(arma::mat& J, int endEffectorId, CoordinateType *mFrame);

	int ik_with_jacobian_test(int endEffectorId, int baseId, gXMat tgtMat, CoordinateType* mFrame, bool fDebug = false , CoordinateType*** mOutData = NULL, int* mOutDataSize = NULL);

	unsigned int getTotalChannel() { return nTotalChannel; }
	std::size_t getNumBones() { return bones.size(); }

		
	// get Transform Matrix
	int getWMatrixAt(int boneId, const CoordinateType* data, gXMat& wMat, double scale = 1.) const;
	int getWMatrixAt(std::string boneName, CoordinateType* data, gXMat& wMat, double scale = 1.);
	int getWMatrixFromToAt(int decendantBoneId, int ancestorBoneId, CoordinateType *data, gXMatList &boneMats, double scale = 1.);
	int getLMatrixAt(int boneId, const CoordinateType* data, gXMat& lMat, double scale = 1.) const;
	int getAllWMatrixAt(CoordinateType* data, std::vector<gXMat> &boneMats, double scale = 1.);

	int getLMatrixAt(gXMat& lMat, int boneId, const CoordinateType* data, double scale = 1.) const;
	int getAllWMatrixAt(gXMatVec &boneMats, const CoordinateType* data, double scale = 1.) const;

	int getAllLMatrixAt(CoordinateType* data, std::vector<gXMat> &boneMats, double scale = 1.);
	int getAllCoord(CoordinateType* data, std::vector<gXMat> &boneMats, double scale = 1.);

	int getWHMatrixAt(int boneId, gXMat& WHM, double scale = 1.) const;

	static gVec3 getRootPositionFromPose(CoordinateType *pose, double scale = 1.);

	int getAllWPosition(std::string constName, mgData& motion, gVec3Array &data);
	
	//int applyTransformToPose(CoordinateType* data, gXMat trans);

	// set Transform Matrix
	int setWMatrixAt(const int boneId, CoordinateType* data, gXMat& wMat, double scale = 1.);

	//Skeleton* getLoadedSkeleton() { return _loadedSkeleton; };
	int getBoneIdFromName(std::string name) const { return boneMap.at(name); };
	
	//Skeleton
	unsigned int getNumOfTransElement() { return numT; }
	unsigned int getNumOfRotElement() { return numR; }
	std::size_t getNumOfBones() { return bones.size(); }
	inline BONEVIt getBonesBegin() { return bones.begin(); };
	inline BONEVIt getBonesEnd() { return bones.end(); };
	inline mgBone* getBone(int i) { return bones[i]; };

	inline BONEMIt getBoneMapBegin() { return boneMap.begin(); };
	inline BONEMIt getBoneMapEnd() { return boneMap.end(); };
	inline BONEMIt getBoneMapFind(std::string name) { return boneMap.find(name); };
	
	// Util
	int convertFromGlobalToLocalPose(CoordinateType* gPose, CoordinateType* lPose);
public:
	//BoneDataPos _baseDataPos;

	// mapping from tgt index to src index
	RetargetVec _retarget;
	BoneDataPos _retargetChannel;

	// 
	mgBone *boneRoot;
	BONEVec bones;
	BONEMap boneMap;
	BoneDataPos dataPos;
	unsigned int numT, numR;

	unsigned int nTotalChannel;
};
#endif