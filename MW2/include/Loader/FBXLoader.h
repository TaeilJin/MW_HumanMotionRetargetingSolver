//#####################################################################
// Copyright 2010-2017, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _FBX_LOADER_H
#define _FBX_LOADER_H

#include <Windows.h>

#include "Loader/LoaderBase.h"

#include <osgAnimation/Skeleton>
#include <osgAnimation/Bone>
#include <osgAnimation/BasicAnimationManager>
/**
    @class		FBXLoader
    @date		2017/11/02
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		load FBX motion file.
    @warning	
*/
class FBXLoader:public LoaderBase
{
	typedef std::vector<osg::ref_ptr<osgAnimation::Animation> > AnimationList;

public:
	int loadMotion(const char* filename, mgSkeleton *skeleton, mgData *motion);
	int loadSkeleton(const char* filename, mgSkeleton *skeleton);

private:
	int	readMotion(mgSkeleton *skeleton, mgData *motion);

	void alterChannel(osg::ref_ptr<osgAnimation::Bone> bone, mgBone* mBone);
	void getChannelInfo(osg::ref_ptr<osgAnimation::Bone> bone, unsigned int& channel, unsigned int& nChannel, std::string& orderBuf);
	void arrangeChannelPos(mgSkeleton *skeleton);

	void traverseAndRemoveUpdateCallbacksFromBones(osg::Node* node);
	osgAnimation::Skeleton* findFirstOsgAnimationSkeletonNode(osg::Node* node);
	osgAnimation::BasicAnimationManager* findFirstOsgAnimationManagerNode(osg::Node* node);

	mgBone* readBoneNode(osg::Node* node, mgSkeleton *skeleton);

	void takeZeroPose(osg::ref_ptr<osg::Node> node);

private:
	osg::ref_ptr<osgAnimation::BasicAnimationManager> _animationMgr;
	osg::ref_ptr<osgAnimation::Animation> _animation = NULL;
	
	//osg::Matrixd _ref_matrix;
	std::vector<std::string> _parent_names;
	std::map<std::string, int> _parent_name_map;
	std::vector<gXMat> _parent_H;
	std::vector<gXMat> _parent_Hp;
	

	// transform name list
	//std::map<std::string, int> _nameChannel;

	mgSkeleton *_skeleton;
};

#endif