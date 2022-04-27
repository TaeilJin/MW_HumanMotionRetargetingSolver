//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

//#include "Visualizer/gVisHeader.h"
#include "Visualizer/gBDVisSystem.h"
#include "Visualizer/gOSGShape.h"
#include "Visualizer/gBdOsgUtil.h"
#include "mbs/gMultibodySystem.h"

gBDVisSystem::gBDVisSystem ()
{
	systemRoot = new osg::Group;

	systemState = new osg::StateSet;
	polygonMode = new osg::PolygonMode;

	systemRoot->setStateSet(systemState);
	//systemState->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);
	systemState->setAttribute(polygonMode);
	polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL );

	m_debugMode = (int)NONE;
}

gBDVisSystem::~gBDVisSystem (void)
{
	if( systemRoot.valid() ) systemRoot.release();
	if( systemState.valid() ) systemState.release();
	if( polygonMode.valid() ) polygonMode.release();
}

void gBDVisSystem::addVisBody(btRigidBody* btBody, std::string name)
{
	osg::ref_ptr<osg::MatrixTransform> osgTrans = osgNodeFromBtCollisionShape(btBody->getCollisionShape(), btBody->getWorldTransform());

	addBodyAndOSG(btBody, osgTrans, name);
}

void gBDVisSystem::addBodyAndOSG(const btRigidBody* btBody, osg::ref_ptr<osg::MatrixTransform> osgBody, std::string name)
{
	GROUPMIt it;

	double size = 0.1;

	osg::ref_ptr<osg::Group> debugGroup = new osg::Group;
	osg::ref_ptr<osg::StateSet> debugState = new osg::StateSet;
	osg::ref_ptr<osg::PolygonMode> debugPolygonMode = new osg::PolygonMode;

	debugPolygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL );
	//debugState->setAttributeAndModes(debugPolygonMode, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
	debugState->setAttribute(debugPolygonMode);
	debugGroup->setStateSet(debugState);
	osgBody->addChild(debugGroup);

	if( m_debugMode >= LINK )
	{
		gOSGShape::setColor( osg::Vec4(1,1,1,1) );
		debugGroup->addChild( gOSGShape::createSphereShape( size ,osg::Vec3(0,0,0)) );
	} 
	if( m_debugMode >= ALL )
	{
		debugGroup->addChild( gOSGShape::createAxis(2, gOSGShape::_width) );

		gLink* link = static_cast<gLink*>(btBody->getUserPointer());
		if( link )
		{
			gOSGShape::setColor( osg::Vec4(0,1,0,1) );
			debugGroup->addChild( gOSGShape::createTextSprite( link->name(), osg::Vec3( 0, 0, 0), 0.1f ) );
		}
	}

	if( name.length() == 0 )
	{
		systemRoot->addChild(osgBody);
	} else {
		it = m_groups.find(name);
		if( it == m_groups.end() )
		{
			osg::ref_ptr<osg::Group> group = new osg::Group;
			m_groups[name] = group;

			osg::ref_ptr<osg::StateSet> state = new osg::StateSet;
			group->setStateSet(state);
			systemRoot->addChild(group);

			osg::ref_ptr<osg::PolygonMode> polyMode = new osg::PolygonMode;
			m_polys[name] = polyMode;
			state->setAttribute(polyMode);

			group->addChild(osgBody);
		} else {
			it->second->addChild(osgBody);
		}
	}

	m_btBodies[btBody] = osgBody;
}

int gBDVisSystem::update()
{
	btTransform btMat;
	osg::Matrix osgMat;

	// BT Body
	BTBodyMIt it = m_btBodies.begin();
	while( it != m_btBodies.end() )
	{
		//(it->first)->getMotionState()->getWorldTransform(btMat);
		btMat = (it->first)->getWorldTransform();
		btMat.getOpenGLMatrix(osgMat.ptr());

		(it->second)->setMatrix(osgMat);

		it++;
	}

	return 0;
}

int gBDVisSystem::setRenderMode(RENDERMODE mode, std::string name)
{
	if( name.length() == 0 )
	{
		switch(mode)
		{
		case POLYGON:
			polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL );
			break;

		case WIREFRAME:
			polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
			break;

		case POINT:
			polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT );
			break;
		}
	} else {
		std::map<std::string, osg::ref_ptr<osg::PolygonMode>>::iterator it = m_polys.find(name);
		
		if( it != m_polys.end() )
		{
			switch(mode)
			{
			case POLYGON:
				it->second->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL );
				break;

			case WIREFRAME:
				it->second->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
				break;

			case POINT:
				it->second->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT );
				break;
			}
		}
	}

	return 0;
}

osg::ref_ptr<osg::Group> gBDVisSystem::getOSGGroup(std::string name)
{
	GROUPMIt it = m_groups.find(name);
	if( it == m_groups.end() )
	{
		printf("Could not find the visualized group that name is %s in the VIS SYS.\n", name.c_str());
		return NULL;
	}

	return it->second;
}

void gBDVisSystem::addVisConstraint(btTypedConstraint *constraint)
{
	btRigidBody& A = constraint->getRigidBodyA();
	btRigidBody& B = constraint->getRigidBodyB();

	osg::ref_ptr<osg::MatrixTransform> transA = m_btBodies[&A];
	osg::ref_ptr<osg::MatrixTransform> frameA = new osg::MatrixTransform;
	transA->addChild(frameA);

	osg::ref_ptr<osg::MatrixTransform> transB = m_btBodies[&B];
	osg::ref_ptr<osg::MatrixTransform> frameB = new osg::MatrixTransform;
	transB->addChild(frameB);

	osgNodeFromBtConstraint(constraint, frameA, frameB);
}

void gBDVisSystem::clean()
{
	m_groups.clear();
	m_btBodies.clear();

	systemRoot->removeChildren(0, systemRoot->getNumChildren() - 1);
}