//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _MBS_BD_VIS_SYSTEM_H_
#define _MBS_BD_VIS_SYSTEM_H_

#include "Base/gMath.h"
#include <btBulletDynamicsCommon.h>

#if defined(WIN32) || defined(WIN64)
	#include <Windows.h>
#endif

#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/PolygonMode>

/**
* MBS-BD Visualize System
**/
class gBDVisSystem{
	//typedef std::map<int, osg::ref_ptr<osg::MatrixTransform>>			OSGBodyMap;
	//typedef std::map<int, osg::ref_ptr<osg::MatrixTransform>>::iterator	OSGBodyMIt;

	typedef std::vector<osg::ref_ptr<osg::MatrixTransform>>				OSGBodyVec;
	typedef std::vector<osg::ref_ptr<osg::MatrixTransform>>::iterator	OSGBodyVIt;


	typedef std::map<const btRigidBody*, osg::ref_ptr<osg::MatrixTransform>>				BTBodyMap;
	typedef std::map<const btRigidBody*, osg::ref_ptr<osg::MatrixTransform>>::iterator		BTBodyMIt;
	/*
	typedef std::vector<btRigidBody*>									BTBodyVec;
	typedef std::vector<btRigidBody*>::iterator							BTBodyVIt;

	typedef std::vector<btTypedConstraint*>								BTConstVec;
	typedef std::vector<btTypedConstraint*>::iterator					BTConstVIt;
	*/


	typedef std::map<std::string, osg::ref_ptr<osg::Group>>				GROUPMap;
	typedef std::map<std::string, osg::ref_ptr<osg::Group>>::iterator	GROUPMIt;

public:
	enum {
		NONE = 0,
		NOTIFY,
		LINK,
		ALL
	} DebugMode;

	enum RENDERMODE {
		POLYGON,
		WIREFRAME,
		POINT
	} ;
public:

	gBDVisSystem ();
	virtual ~gBDVisSystem (void);

	void addVisBody(btRigidBody* btBody, std::string name="");
	void addVisConstraint(btTypedConstraint *constraint);
	void addBodyAndOSG(const btRigidBody* btBody, osg::ref_ptr<osg::MatrixTransform> osgBody, std::string name="");

	void setDebugMode(int mode) { m_debugMode = mode;}
	int update();

	int setRenderMode(RENDERMODE mode, std::string name="");
	osg::ref_ptr<osg::Group> getOSGGroup() {return systemRoot;};
	osg::ref_ptr<osg::Group> getOSGGroup(std::string name);
	osg::ref_ptr<osg::MatrixTransform> getOSGTransFromBody(btRigidBody* body) { return m_btBodies[body]; }

	void clean();

protected:
	int m_debugMode;

	osg::ref_ptr<osg::Group> systemRoot;

	osg::ref_ptr<osg::StateSet> systemState;
	osg::ref_ptr<osg::PolygonMode> polygonMode;

	GROUPMap m_groups;

	BTBodyMap m_btBodies;

	std::map<std::string, osg::ref_ptr<osg::PolygonMode>> m_polys;
};

#endif //_MBS_BD_VIS_SYSTEM_H_