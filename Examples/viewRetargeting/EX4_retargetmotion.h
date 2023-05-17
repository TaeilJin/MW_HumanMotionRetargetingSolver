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
#include "mgPoseTransfer.h"
#include "saveBVH.h"

#include <fstream>

#include "Loader/Parser.h"


class EX4_retargetmotion
{

public:
	btDynamicsWorld* m_dynamicsWorld;
	osg::ref_ptr<osgViewer::Viewer> g_viewer;
	osg::ref_ptr<osg::Group> g_osg_cha;
	osg::ref_ptr<osg::Group> g_osg_debug;

	bCharacter* g_src;
	bCharacterSim* g_srcSim;
	gBDOSGSystem* g_srcVis;
	arma::mat g_refCoord;

	bCharacter* g_tar;
	bCharacterSim* g_tarSim;
	gBDOSGSystem* g_tarVis;
	float srcFrameTime = 1/60.0;

	int g_iter = 0;
	int g_simulationTime = 0;
	//btRigidBody* m_groundBody;
	bool BTsetup(btDynamicsWorld* in_dynamicworld) {
		m_dynamicsWorld = in_dynamicworld;
		return true;
	}
	bool OSGsetup(osg::ref_ptr<osgViewer::Viewer> viewer, osg::ref_ptr<osg::Group>& osg_cha, osg::ref_ptr<osg::Group>& osg_debug) {
		// 1. osgGroup for draw character on scene [osg::Group]
		// 2. osgGroup for draw debugging on scene [osg::Group]
		g_osg_cha = osg_cha;
		g_osg_debug = osg_debug;
		g_viewer = viewer;
		return true;
	}
	bool loadAvatarModelFromFile(bCharacter* in_avatar, bCharacterSim* in_avatarSim, gBDOSGSystem* visSys, const char* filename, const double scale)
	{
		bCharacterLoader loader;
		bool exist = FALSE;
		if (!loader.loadModel(filename, in_avatar, scale)) { printf("fail to load!\n"); exist = FALSE; }
		else
		{
			exist = TRUE;
			in_avatar->postLoading();

			//double lowerBodyLength = 
			//	fabs(m_avatar->baseLink()->pos().y() - m_avatar->getLFootLink()->pos().y()) //root to ankle
			//	+ fabs( (m_avatar->getLFootLink()->frame().multVec3(m_avatar->getLFootGeom().sole())).y() ); //ankle to ground
			//m_avatar->setBasePosition(gVec3(0,lowerBodyLength + 10.0 ,0)); //set initial position 10.0 centimeter off ground
			//avatar->updateKinematicsUptoPos();

			//create avatarSim
			in_avatar->setupCharacterSim(in_avatarSim, m_dynamicsWorld, btVector3(0, 0, 0));
			in_avatarSim->postLoading();

			// kinematic 
			in_avatarSim->setBtBodiesDynamicOrKinematic(btCollisionObject::CF_KINEMATIC_OBJECT);


			visSys->setDebugMode(gBDVisSystem::ALL);

			double shapeWidth = gOSGShape::_width;
			osg::Vec4 color = gOSGShape::color;
			color.a() = 0.4;
			gOSGShape::_width = 0.5;
			gOSGShape::setColor(color);
			visSys->setCharacter(in_avatarSim, "in_avatar");
			osg::ref_ptr<osg::Group> avatarGroup;
			avatarGroup = visSys->getOSGGroup("in_avatar");

			gOSGShape::_width = shapeWidth;

			//visSys.setRenderMode(gBDVisSystem::POLYGON, "in_avatar");
			visSys->setRenderMode(gBDVisSystem::WIREFRAME, "in_avatar");

			osg::ref_ptr<osg::Group> group = visSys->getOSGGroup("in_avatar");
			group->getStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			group->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

			osg::ref_ptr<osg::BlendFunc> bf = new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
			group->getStateSet()->setAttributeAndModes(bf);

		}
		return exist;
	}
	//get gVec3 to OSGVec
	osg::Vec3 gVec3_2_OsgVec(gVec3 gVec) {
		osg::Vec3 p(gVec.x(), gVec.y(), gVec.z());
		return p;
	};
	//get gVec3 to OSGVec
	gVec3 OsgVec_2_gVec3(osg::Vec3 oVec) {
		gVec3 p(oVec.x(), oVec.y(), oVec.z());
		return p;
	};

	// init calc metric
	void EX4_retargetmotion::SetupCalcMetric(char* inputFilename, char* gtFilename);

	// init function
	// 1. load src character [txt]
	void SetupScene(char* srcfilename, char* srcCharactertxt, char* tarCharactertxt);
	void SetupScene(char* srcfilename, char* srcCharactertxt, char* tarCharactertxt, float scale);
	// 1.1 initialize retargeting
	mgPoseTransfer* poseTrans;
	void SetJntRotDirOBJ(mgPoseTransfer* poseTrans, char* txt_id, const char* src_jnt, const char* tar_jnt) {
		poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));

	}
	void SetJntRotDirOBJ(mgPoseTransfer* poseTrans, char* txt_id, const char* src_jnt, gVec3 s_X, gVec3 s_Z, const char* tar_jnt, gVec3 t_X, gVec3 t_Z) {
		gLink* srcLink = poseTrans->src->findLink(src_jnt);
		gLink* tarLink = poseTrans->tar->findLink(tar_jnt);

		if (srcLink == NULL || tarLink == NULL)
			std::cout << "you should check joint names" << std::endl;
		else {
			poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
			char p_x[45]; char p_z[45]; char p_y[45];
			strcpy(p_x, txt_id); strcat(p_x, "_x");
			strcpy(p_y, txt_id); strcat(p_y, "_y");
			strcpy(p_z, txt_id); strcat(p_z, "_z");
			poseTrans->addPoint(p_x, *poseTrans->src->findLink(src_jnt), s_X, *poseTrans->tar->findLink(tar_jnt), t_X);
			poseTrans->addPoint(p_y, *poseTrans->src->findLink(src_jnt), gVec3(0, 1, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 1, 0));
			poseTrans->addPoint(p_z, *poseTrans->src->findLink(src_jnt), s_Z, *poseTrans->tar->findLink(tar_jnt), t_Z);

			poseTrans->addDirectionObjective(txt_id, p_x, 1.0);
			poseTrans->addDirectionObjective(txt_id, p_z, 1.0);
			poseTrans->addDirectionObjective(txt_id, p_y, 1.0);
		}
	}
	void SetJntRotDirOBJ(mgPoseTransfer* poseTrans, char* txt_id, const char* src_jnt, gVec3 s_X, gVec3 s_Z, gVec3 s_Y, const char* tar_jnt, gVec3 t_X, gVec3 t_Z, gVec3 t_Y) {
		gLink* srcLink = poseTrans->src->findLink(src_jnt);
		gLink* tarLink = poseTrans->tar->findLink(tar_jnt);

		if (srcLink == NULL || tarLink == NULL)
			std::cout << "you should check joint names" << std::endl;
		else {
			poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
			char p_x[45]; char p_z[45]; char p_y[45];
			strcpy(p_x, txt_id); strcat(p_x, "_x");
			strcpy(p_y, txt_id); strcat(p_y, "_y");
			strcpy(p_z, txt_id); strcat(p_z, "_z");
			poseTrans->addPoint(p_x, *poseTrans->src->findLink(src_jnt), s_X, *poseTrans->tar->findLink(tar_jnt), t_X);
			poseTrans->addPoint(p_y, *poseTrans->src->findLink(src_jnt), s_Y, *poseTrans->tar->findLink(tar_jnt), t_Y);
			poseTrans->addPoint(p_z, *poseTrans->src->findLink(src_jnt), s_Z, *poseTrans->tar->findLink(tar_jnt), t_Z);

			poseTrans->addDirectionObjective(txt_id, p_x, 1.0);
			poseTrans->addDirectionObjective(txt_id, p_z, 1.0);
			poseTrans->addDirectionObjective(txt_id, p_y, 1.0);
		}
	}
	// 1.1.1 init joint mapping
	enum RetargetPair { Mocap2MixamoRest, Axis2MixamoRest, Kinect2MixamoRest, Mixamo92MixamoRest, CMU2MixamoRest, Amass2MixamoRest, HDM2MixamoRest};
	std::vector<std::string> RetargetPair_s;
	void setAllRP() {
		RetargetPair_s.push_back("Mocap2MixamoRest");
		RetargetPair_s.push_back("Axis2MixamoRest");
		RetargetPair_s.push_back("Kinect2MixamoRest");
		RetargetPair_s.push_back("Mixamo92MixamoRest");
		RetargetPair_s.push_back("CMU2MixamoRest");
		RetargetPair_s.push_back("Amass2MixamoRest");
		RetargetPair_s.push_back("HDM2MixamoRest");
	}
	void initTransfer(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hips")->frame().trn().y();
		double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Spine1")->frame().trn().y();
		double scale = maxHeightT / maxHeight;

		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(poseTrans, "t0", "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t1", "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t2", "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t3", "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "la0", "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la1", "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la2", "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right leg chain
		SetJntRotDirOBJ(poseTrans, "ra0", "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra1", "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra2", "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));


		//left arm chain
		SetJntRotDirOBJ(poseTrans, "ll0", "LeftShoulder", "LeftShoulder");
		SetJntRotDirOBJ(poseTrans, "ll1", "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll2", "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll3", "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right arm chain
		SetJntRotDirOBJ(poseTrans, "rl0", "RightShoulder", "RightShoulder");
		SetJntRotDirOBJ(poseTrans, "rl1", "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl2", "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl3", "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10));


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


		//direction objectives
		poseTrans->addDirectionObjective("t0", "t1", weightDir);
		poseTrans->addDirectionObjective("t1", "t2", weightDir);
		poseTrans->addDirectionObjective("t2", "t3", weightDir);
		poseTrans->addDirectionObjective("t3", "t4", weightDir);
		poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		poseTrans->addDirectionObjective("la0", "la1", weightDir * 5.0);
		poseTrans->addDirectionObjective("la1", "la2", weightDir * 5.0);
		//poseTrans->addDirectionObjective("la2", "la3", weightDir*5.0);

		//right leg chain
		poseTrans->addDirectionObjective("ra0", "ra1", weightDir * 5.0);
		poseTrans->addDirectionObjective("ra1", "ra2", weightDir * 5.0);
		//poseTrans->addDirectionObjective("ra2", "ra3", weightDir*5.0);

		//right arm chain
		poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
		poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
		poseTrans->addDirectionObjective("rl2", "rl3", weightDir);

		//right arm chain
		poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
		poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
		poseTrans->addDirectionObjective("ll2", "ll3", weightDir);

		//pelvis position
		poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}
	void initTransfer_Kinect(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hip")->frame().trn().y();
		double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Hips")->frame().trn().y();
		double scale = maxHeightT / maxHeight;

		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(poseTrans, "t0", "Hip", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t1", "LowerSpine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t2", "MiddleSpine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t3", "Chest", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "la0", "LThigh", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la1", "LShin", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la2", "LFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la3", "LToe", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right leg chain
		SetJntRotDirOBJ(poseTrans, "ra0", "RThigh", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra1", "RShin", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra2", "RFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra3", "RToe", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left arm chain
		SetJntRotDirOBJ(poseTrans, "ll0", "LClavicle", "LeftShoulder");
		SetJntRotDirOBJ(poseTrans, "ll1", "LShoulder", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll2", "LForearm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll3", "LHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right arm chain
		SetJntRotDirOBJ(poseTrans, "rl0", "RClavicle", "RightShoulder");
		SetJntRotDirOBJ(poseTrans, "rl1", "RShoulder", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl2", "RForearm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl3", "RHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10));


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


								//direction objectives
		poseTrans->addDirectionObjective("t0", "t1", weightDir);
		poseTrans->addDirectionObjective("t1", "t2", weightDir);
		poseTrans->addDirectionObjective("t2", "t3", weightDir);
		poseTrans->addDirectionObjective("t3", "t4", weightDir);
		poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		poseTrans->addDirectionObjective("la0", "la1", weightDir * 5.0);
		poseTrans->addDirectionObjective("la1", "la2", weightDir * 5.0);
		poseTrans->addDirectionObjective("la2", "la3", weightDir * 5.0);

		//right leg chain
		poseTrans->addDirectionObjective("ra0", "ra1", weightDir * 5.0);
		poseTrans->addDirectionObjective("ra1", "ra2", weightDir * 5.0);
		poseTrans->addDirectionObjective("ra2", "ra3", weightDir * 5.0);

		//right arm chain
		poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
		poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
		poseTrans->addDirectionObjective("rl2", "rl3", weightDir);

		//right arm chain
		poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
		poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
		poseTrans->addDirectionObjective("ll2", "ll3", weightDir);

		//pelvis position
		poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}
	void initTransfer_Mocap(bCharacter* src, bCharacter* tar, char* txtfile) {
		

		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = 1.0;

		std::ifstream inFile(txtfile);

		if (!inFile) {
			std::cerr << "unable to open file " << std::endl;
			exit(1);
		}

		int numLines = 0;
		std::string line;
		std::vector<float> ee_ind;

		while (getline(inFile, line)) {
			
			std::stringstream ss(line);
			
			std::vector<std::string> data;
			std::string element;
			while (ss >> element) {
				data.push_back(element);
			}
			//
			float ind = std::stof(data[0]);
			ee_ind.push_back(ind);

			// Create an output string stream
			std::ostringstream oss;

			// Write the string and float to the output stream
			oss << "j" << numLines;

			// Get the resulting string from the output stream
			std::string result = oss.str();
			const char* charPtr = result.c_str();
			// Allocate a buffer for the char* string
			char* buffer = new char[result.length() + 1];
			std::strcpy(buffer, charPtr);

			//chain
			SetJntRotDirOBJ(poseTrans, buffer, data[1].c_str(), data[2].c_str());
			/*
			gVec3 srcX(std::stof(data[3]), std::stof(data[4]), std::stof(data[5]));
			gVec3 tarX(std::stof(data[6]), std::stof(data[7]), std::stof(data[8]));

			gVec3 srcY(std::stof(data[9]), std::stof(data[10]), std::stof(data[11]));
			gVec3 tarY(std::stof(data[12]), std::stof(data[13]), std::stof(data[14]));

			gVec3 srcZ(std::stof(data[15]), std::stof(data[16]), std::stof(data[17]));
			gVec3 tarZ(std::stof(data[18]), std::stof(data[19]), std::stof(data[20]));


			SetJntRotDirOBJ(poseTrans, buffer, data[1].c_str(), srcX, srcZ, srcY, data[2].c_str(), tarX, tarZ, tarY);
			*/

			numLines++;
		}


		for (int i = 0; i < ee_ind.size()-1; i++) {
			// Create an output string stream
			std::ostringstream oss;

			// Write the string and float to the output stream
			oss << "j" << i;
			
			std::ostringstream oss2;
			// Write the string and float to the output stream
			oss2 << "j" << i+1;
			
			if(ee_ind[i] != 1)
				poseTrans->addDirectionObjective(oss.str().c_str(), oss2.str().c_str(), 1.0);
		}

		//pelvis position
		poseTrans->addDesiredObjective("j0", 1.0, gVec3(0, 0, 0));
	

		// Close the file
		inFile.close();


		/*std::ifstream fin;
		fin.open("Hello World.txt");

		int index;
		std::string src_joint;
		std::string tar_joint;

		float srcX_x, srcX_y, srcX_z;
		float tarX_x, tarX_y, tarX_z;

		float srcY_x, srcY_y, srcY_z;
		float tarY_x, tarY_y, tarY_z;

		float srcZ_x, srcZ_y, srcZ_z;
		float tarZ_x, tarZ_y, tarZ_z;

		float balance;
		while (!fin.eof())
		{
			fin >> index >> src_joint >> tar_joint >> 
				srcX_x >> srcX_y >> srcX_z >> tarX_x >> tarX_y >> tarX_z >>
				srcY_x >> srcY_y >> srcY_z >> tarY_x >> tarY_y >> tarY_z>>
				srcZ_x >> srcZ_y >> srcZ_z >> tarZ_x >> tarZ_y >> tarZ_z;;
			gVec3 srcX(srcX_x, srcX_y, srcX_z);
			gVec3 tarX(tarX_x, tarX_y, tarX_z);

			gVec3 srcY(srcY_x, srcY_y, srcY_z);
			gVec3 tarY(tarY_x, tarY_y, tarY_z);

			gVec3 srcZ(srcZ_x, srcZ_y, srcZ_z);
			gVec3 tarZ(tarZ_x, tarZ_y, tarZ_z);

			std::cout << index << ": $" << srcX << tarX << srcY << tarY << srcZ << tarZ<< std::endl;
		}

		fin.close();*/
		
	}
	void initTransfer_Mocap(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hips")->frame().trn().y();
		double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Hips")->frame().trn().y();
		double scale = maxHeightT / maxHeight;
		double save_f_height = 0.0;
		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(poseTrans, "t0", "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t1", "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t2", "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
		//SetJntRotDirOBJ(poseTrans, "t3", "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "ll0", "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll1", "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll2", "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll3", "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right leg chain
		SetJntRotDirOBJ(poseTrans, "rl0", "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl1", "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl2", "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl3", "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left arm chain
		SetJntRotDirOBJ(poseTrans, "la0", "LeftShoulder", "LeftShoulder");
		SetJntRotDirOBJ(poseTrans, "la1", "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la2", "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la3", "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0) );

		//right arm chain
		SetJntRotDirOBJ(poseTrans, "ra0", "RightShoulder", "RightShoulder");
		SetJntRotDirOBJ(poseTrans, "ra1", "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra2", "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra3", "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0) );


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


								//direction objectives
		poseTrans->addDirectionObjective("t0", "t1", weightDir);
		poseTrans->addDirectionObjective("t1", "t2", weightDir);
		poseTrans->addDirectionObjective("t2", "t4", weightDir);
		//poseTrans->addDirectionObjective("t3", "t4", weightDir);
		poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		poseTrans->addDirectionObjective("ll0", "ll1", weightDir * 5.0);
		poseTrans->addDirectionObjective("ll1", "ll2", weightDir * 5.0);
		poseTrans->addDirectionObjective("ll2", "ll3", weightDir * 5.0);

		//right leg chain
		poseTrans->addDirectionObjective("rl0", "rl1", weightDir * 5.0);
		poseTrans->addDirectionObjective("rl1", "rl2", weightDir * 5.0);
		poseTrans->addDirectionObjective("rl2", "rl3", weightDir * 5.0);

		//right arm chain
		poseTrans->addDirectionObjective("ra0", "ra1", weightDir);
		poseTrans->addDirectionObjective("ra1", "ra2", weightDir);
		poseTrans->addDirectionObjective("ra2", "ra3", weightDir);

		//right arm chain
		poseTrans->addDirectionObjective("la0", "la1", weightDir);
		poseTrans->addDirectionObjective("la1", "la2", weightDir);
		poseTrans->addDirectionObjective("la2", "la3", weightDir);

		//pelvis position
		poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}
	void initTransfer_Mocap_DK(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hips")->frame().trn().y();
		double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Hips")->frame().trn().y();
		double scale = maxHeightT / maxHeight;
		double save_f_height = 0.0;
		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(poseTrans, "t0", "Hips", "Hips");
		SetJntRotDirOBJ(poseTrans, "t1", "Spine", "Spine");
		SetJntRotDirOBJ(poseTrans, "t2", "Spine1", "Spine3");
		SetJntRotDirOBJ(poseTrans, "t4", "Neck", "Neck");
		SetJntRotDirOBJ(poseTrans, "t5", "Head", "Head");
		
		//SetJntRotDirOBJ(poseTrans, "t0", "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "t1", "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "t2", "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "t3", "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		//SetJntRotDirOBJ(poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(-10, 0, 0), gVec3(0, 10, 0));

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "ll0", "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftUpLeg", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ll1", "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftLeg", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ll2", "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ll3", "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftToeBase", gVec3(-10, 0, 0), gVec3(0, 10, 0));

		//right leg chain
		SetJntRotDirOBJ(poseTrans, "rl0", "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightUpLeg", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "rl1", "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightLeg", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "rl2", "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightFoot", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "rl3", "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightToeBase", gVec3(-10, 0, 0), gVec3(0, 10, 0));

		//left arm chain
		SetJntRotDirOBJ(poseTrans, "la0", "LeftShoulder", "LeftShoulder");
		SetJntRotDirOBJ(poseTrans, "la1", "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftArm", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "la2", "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftForeArm", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "la3", "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "LeftHand", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10));

		//right arm chain
		SetJntRotDirOBJ(poseTrans, "ra0", "RightShoulder", "RightShoulder");
		SetJntRotDirOBJ(poseTrans, "ra1", "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightArm", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ra2", "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightForeArm", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ra3", "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "RightHand", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10));


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


								//direction objectives
		poseTrans->addDirectionObjective("t0", "t1", weightDir);
		poseTrans->addDirectionObjective("t1", "t2", weightDir);
		poseTrans->addDirectionObjective("t2", "t4", weightDir);
		//poseTrans->addDirectionObjective("t3", "t4", weightDir);
		poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		poseTrans->addDirectionObjective("ll0", "ll1", weightDir * 5.0);
		poseTrans->addDirectionObjective("ll1", "ll2", weightDir * 5.0);
		poseTrans->addDirectionObjective("ll2", "ll3", weightDir * 5.0);

		//right leg chain
		poseTrans->addDirectionObjective("rl0", "rl1", weightDir * 5.0);
		poseTrans->addDirectionObjective("rl1", "rl2", weightDir * 5.0);
		poseTrans->addDirectionObjective("rl2", "rl3", weightDir * 5.0);

		//right arm chain
		poseTrans->addDirectionObjective("ra0", "ra1", weightDir);
		poseTrans->addDirectionObjective("ra1", "ra2", weightDir);
		poseTrans->addDirectionObjective("ra2", "ra3", weightDir);

		//right arm chain
		poseTrans->addDirectionObjective("la0", "la1", weightDir);
		poseTrans->addDirectionObjective("la1", "la2", weightDir);
		poseTrans->addDirectionObjective("la2", "la3", weightDir);

		//pelvis position
		poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}
	void initTransfer_Mocap_BY(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hips")->frame().trn().y();
		double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Hips")->frame().trn().y();
		double scale = maxHeightT / maxHeight;
		double save_f_height = 0.0;
		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(poseTrans, "t0", "Hips", "Hips");
		SetJntRotDirOBJ(poseTrans, "t1", "Spine", "Spine");
		SetJntRotDirOBJ(poseTrans, "t2", "Spine1", "Spine1");
		SetJntRotDirOBJ(poseTrans, "t4", "Neck", "Neck");
		SetJntRotDirOBJ(poseTrans, "t5", "Head", "Head");

		//SetJntRotDirOBJ(poseTrans, "t0", "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "t1", "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "t2", "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "t3", "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		//SetJntRotDirOBJ(poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(-10, 0, 0), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(-10, 0, 0), gVec3(0, 10, 0));

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "ll0", 
			"LeftUpLeg", src->findLink("LeftUpLeg")->frame().rotX(), src->findLink("LeftUpLeg")->frame().rotZ(), 
			"LeftUpLeg", tar->findLink("LeftUpLeg")->frame().rotX(), tar->findLink("LeftUpLeg")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "ll1", 
			"LeftLeg", src->findLink("LeftLeg")->frame().rotX(), src->findLink("LeftLeg")->frame().rotZ(), 
			"LeftLeg", tar->findLink("LeftLeg")->frame().rotX(), tar->findLink("LeftLeg")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "ll2", 
			"LeftFoot", src->findLink("LeftFoot")->frame().rotX(), src->findLink("LeftFoot")->frame().rotZ(),
			"LeftFoot", tar->findLink("LeftFoot")->frame().rotX(), tar->findLink("LeftFoot")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "ll3", 
			"LeftToeBase", src->findLink("LeftToeBase")->frame().rotX(), src->findLink("LeftToeBase")->frame().rotZ(),
			"LeftToeBase", tar->findLink("LeftToeBase")->frame().rotX(), tar->findLink("LeftToeBase")->frame().rotZ());

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "rl0",
			"RightUpLeg", src->findLink("RightUpLeg")->frame().rotX(), src->findLink("RightUpLeg")->frame().rotZ(),
			"RightUpLeg", tar->findLink("RightUpLeg")->frame().rotX(), tar->findLink("RightUpLeg")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "rl1",
			"RightLeg", src->findLink("RightLeg")->frame().rotX(), src->findLink("RightLeg")->frame().rotZ(),
			"RightLeg", tar->findLink("RightLeg")->frame().rotX(), tar->findLink("RightLeg")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "rl2",
			"RightFoot", src->findLink("RightFoot")->frame().rotX(), src->findLink("RightFoot")->frame().rotZ(),
			"RightFoot", tar->findLink("RightFoot")->frame().rotX(), tar->findLink("RightFoot")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "rl3",
			"RightToeBase", src->findLink("RightToeBase")->frame().rotX(), src->findLink("RightToeBase")->frame().rotZ(),
			"RightToeBase", tar->findLink("RightToeBase")->frame().rotX(), tar->findLink("RightToeBase")->frame().rotZ());

		//left arm chain
		SetJntRotDirOBJ(poseTrans, "la0", "LeftShoulder", "LeftShoulder");
		SetJntRotDirOBJ(poseTrans, "la1", 
			"LeftArm", src->findLink("LeftArm")->frame().rotX(), src->findLink("LeftArm")->frame().rotZ(),
			"LeftArm", tar->findLink("LeftArm")->frame().rotX(), tar->findLink("LeftArm")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "la2", 
			"LeftForeArm", src->findLink("LeftForeArm")->frame().rotX(), src->findLink("LeftForeArm")->frame().rotZ(),
			"LeftForeArm", tar->findLink("LeftForeArm")->frame().rotX(), tar->findLink("LeftForeArm")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "la3", 
			"LeftHand", src->findLink("LeftHand")->frame().rotX(), src->findLink("LeftHand")->frame().rotZ(),
			"LeftHand", tar->findLink("LeftHand")->frame().rotX(), tar->findLink("LeftHand")->frame().rotZ());

		//Right arm chain
		SetJntRotDirOBJ(poseTrans, "ra0", "RightShoulder", "RightShoulder");
		SetJntRotDirOBJ(poseTrans, "ra1",
			"RightArm", src->findLink("RightArm")->frame().rotX(), src->findLink("RightArm")->frame().rotZ(),
			"RightArm", tar->findLink("RightArm")->frame().rotX(), tar->findLink("RightArm")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "ra2",
			"RightForeArm", src->findLink("RightForeArm")->frame().rotX(), src->findLink("RightForeArm")->frame().rotZ(),
			"RightForeArm", tar->findLink("RightForeArm")->frame().rotX(), tar->findLink("RightForeArm")->frame().rotZ());
		SetJntRotDirOBJ(poseTrans, "ra3",
			"RightHand", src->findLink("RightHand")->frame().rotX(), src->findLink("RightHand")->frame().rotZ(),
			"RightHand", tar->findLink("RightHand")->frame().rotX(), tar->findLink("RightHand")->frame().rotZ());


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


								//direction objectives
		poseTrans->addDirectionObjective("t0", "t1", weightDir);
		poseTrans->addDirectionObjective("t1", "t2", weightDir);
		poseTrans->addDirectionObjective("t2", "t4", weightDir);
		//poseTrans->addDirectionObjective("t3", "t4", weightDir);
		poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		poseTrans->addDirectionObjective("ll0", "ll1", weightDir * 5.0);
		poseTrans->addDirectionObjective("ll1", "ll2", weightDir * 5.0);
		poseTrans->addDirectionObjective("ll2", "ll3", weightDir * 5.0);

		//right leg chain
		poseTrans->addDirectionObjective("rl0", "rl1", weightDir * 5.0);
		poseTrans->addDirectionObjective("rl1", "rl2", weightDir * 5.0);
		poseTrans->addDirectionObjective("rl2", "rl3", weightDir * 5.0);

		//right arm chain
		poseTrans->addDirectionObjective("ra0", "ra1", weightDir);
		poseTrans->addDirectionObjective("ra1", "ra2", weightDir);
		poseTrans->addDirectionObjective("ra2", "ra3", weightDir);

		//right arm chain
		poseTrans->addDirectionObjective("la0", "la1", weightDir);
		poseTrans->addDirectionObjective("la1", "la2", weightDir);
		poseTrans->addDirectionObjective("la2", "la3", weightDir);

		//pelvis position
		poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}

	void initTransfer_Mixamo9(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("mixamorig:Head")->frame().trn().y() - src->findLink("mixamorig:Hips")->frame().trn().y();
		double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Hips")->frame().trn().y();
		double scale = maxHeightT / maxHeight;

		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(poseTrans, "t0", "mixamorig:Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t1", "mixamorig:Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t2", "mixamorig:Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t3", "mixamorig:Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t4", "mixamorig:Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t5", "mixamorig:Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "ll0", "mixamorig:LeftUpLeg", gVec3(-10, 0, 0), gVec3(0, 0, 10), gVec3(0, -10, 0), "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ll1", "mixamorig:LeftLeg", gVec3(-10, 0, 0), gVec3(0, 0, 10), gVec3(0, -10, 0), "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ll2", "mixamorig:LeftFoot", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ll3", "mixamorig:LeftToeBase", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10), "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));

		//right leg chain
		SetJntRotDirOBJ(poseTrans, "rl0", "mixamorig:RightUpLeg", gVec3(-10, 0, 0), gVec3(0, 0, 10), gVec3(0, -10, 0), "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "rl1", "mixamorig:RightLeg", gVec3(-10, 0, 0), gVec3(0, 0, 10), gVec3(0, -10, 0), "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "rl2", "mixamorig:RightFoot", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10), "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "rl3", "mixamorig:RightToeBase", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10), "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));

		//left arm chain
		SetJntRotDirOBJ(poseTrans, "la0", "mixamorig:LeftShoulder", "LeftShoulder");
		SetJntRotDirOBJ(poseTrans, "la1", "mixamorig:LeftArm", gVec3(0, 10, 0), gVec3(-10, 0, 0), gVec3(0, 0, -10), "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "la2", "mixamorig:LeftForeArm", gVec3(0, 10, 0), gVec3(-10, 0, 0), gVec3(0, 0, -10), "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "la3", "mixamorig:LeftHand", gVec3(0, 10, 0), gVec3(-10, 0, 0), gVec3(0, 0, -10), "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));

		//right arm chain
		SetJntRotDirOBJ(poseTrans, "ra0", "mixamorig:RightShoulder", "RightShoulder");
		SetJntRotDirOBJ(poseTrans, "ra1", "mixamorig:RightArm", gVec3(0, -10, 0), gVec3(10, 0, 0), gVec3(0, 0, -10), "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ra2", "mixamorig:RightForeArm", gVec3(0, -10, 0), gVec3(10, 0, 0), gVec3(0, 0, -10), "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		SetJntRotDirOBJ(poseTrans, "ra3", "mixamorig:RightHand", gVec3(0, -10, 0), gVec3(10, 0, 0), gVec3(0, 0, -10), "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


								//direction objectives
		poseTrans->addDirectionObjective("t0", "t1", weightDir);
		poseTrans->addDirectionObjective("t1", "t2", weightDir);
		poseTrans->addDirectionObjective("t2", "t4", weightDir);
		poseTrans->addDirectionObjective("t3", "t4", weightDir);
		poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		poseTrans->addDirectionObjective("ll0", "ll1", weightDir * 5.0);
		poseTrans->addDirectionObjective("ll1", "ll2", weightDir * 5.0);
		poseTrans->addDirectionObjective("ll2", "ll3", weightDir * 5.0);

		//right leg chain
		poseTrans->addDirectionObjective("rl0", "rl1", weightDir * 5.0);
		poseTrans->addDirectionObjective("rl1", "rl2", weightDir * 5.0);
		poseTrans->addDirectionObjective("rl2", "rl3", weightDir * 5.0);

		//right arm chain
		poseTrans->addDirectionObjective("ra0", "ra1", weightDir);
		poseTrans->addDirectionObjective("ra1", "ra2", weightDir);
		poseTrans->addDirectionObjective("ra2", "ra3", weightDir);

		//right arm chain
		poseTrans->addDirectionObjective("la0", "la1", weightDir);
		poseTrans->addDirectionObjective("la1", "la2", weightDir);
		poseTrans->addDirectionObjective("la2", "la3", weightDir);

		//pelvis position
		poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}
	void initTransfer_MixamoJackie(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("mixamorig1:Head")->frame().trn().y() - src->findLink("mixamorig1:Hips")->frame().trn().y();
		double maxHeightT = tar->findLink("mixamorig1:Head")->frame().trn().y() - tar->findLink("mixamorig1:Hips")->frame().trn().y();
		double targetHeight = tar->findLink("mixamorig1:Head")->frame().trn().y() - tar->findLink("mixamorig1:RightToeBase")->frame().trn().y();
		double sourceHeight = src->findLink("mixamorig1:Head")->frame().trn().y() - src->findLink("mixamorig1:RightToeBase")->frame().trn().y();
		std::cout<<  "targetHeight "<< targetHeight + 11.89 << "head" << 11.89 << std::endl;
		std::cout << "sourceHeight " << sourceHeight + 11.89 << "head" << 11.89 << std::endl;

		//double scale = maxHeightT / maxHeight;

		//poseTrans = new mgPoseTransfer(src, tar);
		//poseTrans->scale = scale;
		//// you need to manually match the src chracter joint name and corresponding tar character joint name
		////poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		//SetJntRotDirOBJ(poseTrans, "t0", "mixamorig:Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig1:Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
		//SetJntRotDirOBJ(poseTrans, "t1", "mixamorig:Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig1:Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
		//SetJntRotDirOBJ(poseTrans, "t2", "mixamorig:Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig1:Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
		//SetJntRotDirOBJ(poseTrans, "t3", "mixamorig:Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig1:Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		//SetJntRotDirOBJ(poseTrans, "t4", "mixamorig:Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig1:Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
		//SetJntRotDirOBJ(poseTrans, "t5", "mixamorig:Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig1:Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

		////left leg chain
		//SetJntRotDirOBJ(poseTrans, "ll0", "mixamorig:LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "ll1", "mixamorig:LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "ll2", "mixamorig:LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "ll3", "mixamorig:LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));

		////right leg chain
		//SetJntRotDirOBJ(poseTrans, "rl0", "mixamorig:RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "rl1", "mixamorig:RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "rl2", "mixamorig:RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "rl3", "mixamorig:RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));

		////left arm chain
		//SetJntRotDirOBJ(poseTrans, "la0", "mixamorig:LeftShoulder", "mixamorig1:LeftShoulder");
		//SetJntRotDirOBJ(poseTrans, "la1", "mixamorig:LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "la2", "mixamorig:LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "la3", "mixamorig:LeftHand", gVec3(0, 10, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));

		////right arm chain
		//SetJntRotDirOBJ(poseTrans, "ra0", "mixamorig:RightShoulder", "mixamorig1:RightShoulder");
		//SetJntRotDirOBJ(poseTrans, "ra1", "mixamorig:RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "ra2", "mixamorig:RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
		//SetJntRotDirOBJ(poseTrans, "ra3", "mixamorig:RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0), "mixamorig1:RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));


		//double weightDir = 1.;//importance of direction vector in pose transfer
		//double weightPos = 1.01;//importance of end-effector orientation in pose transfer


		//						//direction objectives
		//poseTrans->addDirectionObjective("t0", "t1", weightDir);
		//poseTrans->addDirectionObjective("t1", "t2", weightDir);
		//poseTrans->addDirectionObjective("t2", "t4", weightDir);
		//poseTrans->addDirectionObjective("t3", "t4", weightDir);
		//poseTrans->addDirectionObjective("t4", "t5", weightDir);

		////left leg chain 
		//poseTrans->addDirectionObjective("ll0", "ll1", weightDir * 5.0);
		//poseTrans->addDirectionObjective("ll1", "ll2", weightDir * 5.0);
		//poseTrans->addDirectionObjective("ll2", "ll3", weightDir * 5.0);

		////right leg chain
		//poseTrans->addDirectionObjective("rl0", "rl1", weightDir * 5.0);
		//poseTrans->addDirectionObjective("rl1", "rl2", weightDir * 5.0);
		//poseTrans->addDirectionObjective("rl2", "rl3", weightDir * 5.0);

		////right arm chain
		//poseTrans->addDirectionObjective("ra0", "ra1", weightDir);
		//poseTrans->addDirectionObjective("ra1", "ra2", weightDir);
		//poseTrans->addDirectionObjective("ra2", "ra3", weightDir);

		////right arm chain
		//poseTrans->addDirectionObjective("la0", "la1", weightDir);
		//poseTrans->addDirectionObjective("la1", "la2", weightDir);
		//poseTrans->addDirectionObjective("la2", "la3", weightDir);

		////pelvis position
		//poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}




	void initTransfer_CMU(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hips")->frame().trn().y();
		double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Spine1")->frame().trn().y();
		double scale = maxHeightT / maxHeight;

		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(poseTrans, "t0", "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t1", "LowerBack", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t2", "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t3", "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "la0", "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la1", "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la2", "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la3", "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right leg chain
		SetJntRotDirOBJ(poseTrans, "ra0", "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra1", "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra2", "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra3", "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));


		//left arm chain
		SetJntRotDirOBJ(poseTrans, "ll0", "LeftShoulder", "LeftShoulder");
		SetJntRotDirOBJ(poseTrans, "ll1", "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll2", "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll3", "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right arm chain
		SetJntRotDirOBJ(poseTrans, "rl0", "RightShoulder", "RightShoulder");
		SetJntRotDirOBJ(poseTrans, "rl1", "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl2", "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl3", "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10));


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


								//direction objectives
		poseTrans->addDirectionObjective("t0", "t1", weightDir);
		poseTrans->addDirectionObjective("t1", "t2", weightDir);
		poseTrans->addDirectionObjective("t2", "t3", weightDir);
		poseTrans->addDirectionObjective("t3", "t4", weightDir);
		poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		poseTrans->addDirectionObjective("la0", "la1", weightDir * 5.0);
		poseTrans->addDirectionObjective("la1", "la2", weightDir * 5.0);
		poseTrans->addDirectionObjective("la2", "la3", weightDir * 5.0);

		//right leg chain
		poseTrans->addDirectionObjective("ra0", "ra1", weightDir * 5.0);
		poseTrans->addDirectionObjective("ra1", "ra2", weightDir * 5.0);
		poseTrans->addDirectionObjective("ra2", "ra3", weightDir * 5.0);

		//right arm chain
		poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
		poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
		poseTrans->addDirectionObjective("rl2", "rl3", weightDir);

		//right arm chain
		poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
		poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
		poseTrans->addDirectionObjective("ll2", "ll3", weightDir);

		//pelvis position
		poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}
	void initTransfer_Amass(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("upperneck")->frame().trn().y() - src->findLink("root")->frame().trn().y();
		double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Spine1")->frame().trn().y();
		double scale = maxHeightT / maxHeight;

		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(poseTrans, "t0", "root", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t1", "lowerback", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t2", "upperback", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t3", "chest", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t4", "lowerneck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t5", "upperneck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "la0", "lhip", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la1", "lknee", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la2", "lankle", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la3", "ltoe", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right leg chain
		SetJntRotDirOBJ(poseTrans, "ra0", "rhip", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra1", "rknee", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra2", "rankle", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra3", "rtoe", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));


		//left arm chain
		SetJntRotDirOBJ(poseTrans, "ll0", "lclavicle", "LeftShoulder");
		SetJntRotDirOBJ(poseTrans, "ll1", "lshoulder", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll2", "lelbow", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll3", "lwrist", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right arm chain
		SetJntRotDirOBJ(poseTrans, "rl0", "rclavicle", "RightShoulder");
		SetJntRotDirOBJ(poseTrans, "rl1", "rshoulder", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl2", "relbow", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl3", "rwrist", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10));


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


								//direction objectives
		poseTrans->addDirectionObjective("t0", "t1", weightDir);
		poseTrans->addDirectionObjective("t1", "t2", weightDir);
		poseTrans->addDirectionObjective("t2", "t3", weightDir);
		poseTrans->addDirectionObjective("t3", "t4", weightDir);
		poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		poseTrans->addDirectionObjective("la0", "la1", weightDir * 5.0);
		poseTrans->addDirectionObjective("la1", "la2", weightDir * 5.0);
		poseTrans->addDirectionObjective("la2", "la3", weightDir * 5.0);

		//right leg chain
		poseTrans->addDirectionObjective("ra0", "ra1", weightDir * 5.0);
		poseTrans->addDirectionObjective("ra1", "ra2", weightDir * 5.0);
		poseTrans->addDirectionObjective("ra2", "ra3", weightDir * 5.0);

		//right arm chain
		poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
		poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
		poseTrans->addDirectionObjective("rl2", "rl3", weightDir);

		//right arm chain
		poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
		poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
		poseTrans->addDirectionObjective("ll2", "ll3", weightDir);

		//pelvis position
		poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}
	void initTransfer_HDM05(bCharacter* src, bCharacter* tar) {

		double maxHeight = src->findLink("head")->frame().trn().y() - src->findLink("hip")->frame().trn().y();
		double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Spine1")->frame().trn().y();
		double scale = maxHeightT / maxHeight;

		poseTrans = new mgPoseTransfer(src, tar);
		poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(poseTrans, "t0", "hip", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t1", "lowerback", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t2", "upperback", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t3", "thorax", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t4", "upperneck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "t5", "head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left leg chain
		SetJntRotDirOBJ(poseTrans, "la0", "lfemur", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la1", "ltibia", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la2", "lfoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "la3", "ltoes", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right leg chain
		SetJntRotDirOBJ(poseTrans, "ra0", "rfemur", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra1", "rtibia", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra2", "rfoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ra3", "rtoes", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));


		//left arm chain
		SetJntRotDirOBJ(poseTrans, "ll0", "lclavicle", "LeftShoulder");
		SetJntRotDirOBJ(poseTrans, "ll1", "lhumerus", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll2", "lradius", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "ll3", "lwrist", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right arm chain
		SetJntRotDirOBJ(poseTrans, "rl0", "rclavicle", "RightShoulder");
		SetJntRotDirOBJ(poseTrans, "rl1", "rhumerus", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl2", "rradius", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(poseTrans, "rl3", "rwrist", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10));


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


								//direction objectives
		poseTrans->addDirectionObjective("t0", "t1", weightDir);
		poseTrans->addDirectionObjective("t1", "t2", weightDir);
		poseTrans->addDirectionObjective("t2", "t3", weightDir);
		poseTrans->addDirectionObjective("t3", "t4", weightDir);
		poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		poseTrans->addDirectionObjective("la0", "la1", weightDir * 5.0);
		poseTrans->addDirectionObjective("la1", "la2", weightDir * 5.0);
		poseTrans->addDirectionObjective("la2", "la3", weightDir * 5.0);

		//right leg chain
		poseTrans->addDirectionObjective("ra0", "ra1", weightDir * 5.0);
		poseTrans->addDirectionObjective("ra1", "ra2", weightDir * 5.0);
		poseTrans->addDirectionObjective("ra2", "ra3", weightDir * 5.0);

		//right arm chain
		poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
		poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
		poseTrans->addDirectionObjective("rl2", "rl3", weightDir);

		//right arm chain
		poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
		poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
		poseTrans->addDirectionObjective("ll2", "ll3", weightDir);

		//pelvis position
		poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

	}
	// 1.2. init retarget
	void initRetarget(int RP);
	void initRetarget(int RP, char* textfile);
	// 2. generate target character txt 
	void generateTarCharacterSpecfile(const char* tarCharacterMotionFile, const char* tarCharactertxt) {
		MotionLoader loader;
		loader.loadMotionFile(tarCharacterMotionFile);
		mgData* motion = loader.getMotion();
		mgSkeleton* skeleton = loader.getSkeleton();
		srcFrameTime = motion->frameTime;

		std::cout << "|--- load src character file ---|" << std::endl;
		// load src bCharacter from txt file
		g_tar = new bCharacter();
		g_tarSim = new bCharacterSim(g_tar);
		g_tarVis = new gBDOSGSystem();
		if (loadAvatarModelFromFile(g_tar, g_tarSim, g_tarVis, tarCharactertxt, 1.0) != TRUE) {
			std::cout << "|---- write src character file ---|" << std::endl;
			const double mass = 70.;
			mgSkeletonToBCharacter::saveToBCharacter(skeleton, tarCharactertxt, mass);
			std::cout << "|---- load new src character file : warning should check a free joint ---|" << std::endl;
			//loadAvatarModelFromFile(g_tar, g_tarSim, g_tarVis, tarCharactertxt, 1.0);
		}
	}

	

	// update function 
	// 1. simulation iteration (for viewer)
	// 2. pose frame iteration (for motiondata)
	void UpdateScene(int even_simulationTime, int even_iter, bool b_motionretarget, double y_offset);
	// 2.1. retarget pose
	void retargetPose(bool bool_motionretarget, double y_offset);
	// 2.2. retarget motion data
	saveBVH* g_saveBVH;
	void saveRetargetMotion(bool bool_save, char* tarBVHFILE, float frametime, double y_offset, float scale);
	// 2.3. source data 
	void savePreProcessingMotion(bool bool_save, char* tarBVHFILE);
};

