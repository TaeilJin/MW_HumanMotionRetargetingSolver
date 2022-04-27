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

class EX1_playmotion
{

public:
	//btBroadphaseInterface* m_broadphase;
	//btCollisionDispatcher* m_dispatcher;
	//btConstraintSolver* m_solver;
	//btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDynamicsWorld* m_dynamicsWorld;
	osg::ref_ptr<osgViewer::Viewer> g_viewer;
	osg::ref_ptr<osg::Group> g_osg_cha;
	osg::ref_ptr<osg::Group> g_osg_debug;

	bCharacter* g_src;
	bCharacterSim* g_srcSim;
	gBDOSGSystem* g_srcVis;
	arma::mat g_refCoord;
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
	// init function
	// 1. load src character [txt]
	// 3. load motion clip	 [txt]
	void SetupScene(char* srcfilename, char* srcCharactertxt);

	// update function 
	// 1. simulation iteration (for viewer)
	// 2. pose frame iteration (for motiondata)
	void UpdateScene(int even_simulationTime, int even_iter);
};

