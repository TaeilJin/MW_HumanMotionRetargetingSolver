#pragma once
#include <Windows.h>
#include <iostream>
#include <algorithm>

#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/LineWidth>
#include <osgDB/ReadFile>

#include "Character/bCharacterLoader.h"
#include "Loader/MotionLoader.h"
#include "Loader/mgSkeletonToBCharacter.h"
#include "MocapProcessor/mgMBSUtil.h"
#include "MocapProcessor/mgUtility.h"
#include "Visualizer/gBdOsgSystem.h"
#include "Visualizer/gOSGSkin.h"
#include "Visualizer/gOSGShape.h"

#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Animation>
#include <osgAnimation/Skeleton>
#include <osgAnimation/Bone>
#include <osgAnimation/UpdateBone>
#include <osgAnimation/StackedRotateAxisElement>
#include <osgAnimation/StackedMatrixElement>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/StackedScaleElement>
#include <osg/TriangleIndexFunctor>
#include <osgDB/Options>
#include <osg/BlendFunc>

class saveTXT
{

public:
	
	std::string puttap(int n) {
		std::string output;
		for (int i = 0; i < n; i++) {
			output += "\t";
		}
		return output;
	}
	void saveVec3(FILE* fp, gVec3 vec, bool b_enter, char* name) {
		if (b_enter == true)
			fprintf(fp, "%s %g %g %g\n", name, vec.x(), vec.y(), vec.z());
		else
			fprintf(fp, "%s %g %g %g ", name, vec.x(), vec.y(), vec.z());
	};
	void saveVec3(FILE* fp, gVec3 vec, bool b_enter) {
		if(b_enter==true)
			fprintf(fp, "%g %g %g\n", vec.x(), vec.y(), vec.z());
		else
			fprintf(fp, "%g %g %g ", vec.x(), vec.y(), vec.z());
	};
	void saveTmat(FILE* fp, gXMat gmat, char* name) {
		fprintf(fp, "%s ",name);
		saveVec3(fp, gmat.trn(), false);
		saveVec3(fp, gmat.rotX(), false);
		saveVec3(fp, gmat.rotY(), false);
		saveVec3(fp, gmat.rotZ(), true);
	};
	
};

