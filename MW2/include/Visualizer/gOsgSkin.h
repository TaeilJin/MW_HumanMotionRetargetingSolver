//#####################################################################
// Copyright 2010-2015, Taeil Jin, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _OSG_SKIN_H
#define _OSG_SKIN_H

/**
    @class		gOsgSkin
    @date		2015/01/06
    @author		Sung-Hee Lee(leesunghee@gmail.com)
    @brief		This class loads skin data from a file (e.g., fbx file) and controls visualization of skin
    @warning	
*/

#include <osgAnimation/Bone>
#include <osgAnimation/Skeleton>
#include <osgAnimation/BasicAnimationManager>
#include <osgViewer/Viewer>

#include "mbs/gMultibodySystem.h"
//#include "Visualizer/gOsgViewHelper.h"

class gOsgSkin{
public:
	struct MBSToOsgBone {
		gLink* mb;  //MBS link
		//osgAnimation::Bone* ob;  //osgBone corresponding to mb
		osg::MatrixTransform* ob;
		gXMat displacement; // ob's world frame = mb's world frame*displacement
	};

	gOsgSkin(gMultibodySystem* mbs, const double skinScale = 1.) 
	{ 
		m_mbs=mbs;
		endPointBones.clear();
		MBSBoneOsgBoneDisplacements.clear();
		skinGroupNode = NULL;
		nodeFile = NULL;
		visState = true;
		//dummy = NULL;

		scale = skinScale;
		invScale = 1./skinScale;
	}

	virtual ~gOsgSkin()
	{

	}

	/// Load skin data from file and add it to gOSGViewHelper
	int loadSkin(osg::ref_ptr<osg::Group> scene, const char* filename, const char* name_head, const char* name_right_shoulder);

	int loadSkin(osg::ref_ptr<osg::Group> scene, const char* filename);

	osg::ref_ptr<osg::Group> getSkinGroup() { return skinGroupNode; }
	
	///Delete skin data
	/**
	Don't call this function unless you have memory shortage
	*/
	void unloadSkin(); 

	/// Update skin shape. Call this function when a character change its pose.
	void updateSkin();

	/// Hide skin
	void hideSkin();

	/// Visualize skin
	void unhideSkin();  

protected:
	void traverseAndRemoveUpdateCallbacksFromBones(osg::Node* node);
	void* findOsgAnimationManager(osg::Node* node);
	osgAnimation::Skeleton* findFirstOsgAnimationSkeletonNode(osg::Node* node);
	
protected:

	gMultibodySystem* m_mbs;	
	std::vector<osgAnimation::Bone*> endPointBones;
	std::vector<MBSToOsgBone> MBSBoneOsgBoneDisplacements;
	osg::ref_ptr<osg::Group> skinGroupNode;
	osg::ref_ptr<osg::Group> nodeFile; 
	//osg::Group* dummy; //used to keep reference count of fileNode non-zero
	bool visState;

	// test 
	double scale;
	double invScale;
};

#endif /* _OSG_SKIN_H */