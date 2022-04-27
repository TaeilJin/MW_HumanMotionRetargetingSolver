//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

//#include "Visualizer/gVisHeader.h"
#include "Visualizer/gBDOSGSystem.h"
#include "Visualizer/gOSGShape.h"
#include "Visualizer/gBdOsgUtil.h"

#define X 0
#define Y 1
#define Z 2

gBDOSGSystem::gBDOSGSystem()//bCharacter *character)
	: gBDVisSystem()
{
	_mode = POLYGON;
}

int gBDOSGSystem::setCharacter(const bCharacterSim *character, std::string name)
{
	osg::ref_ptr<osg::MatrixTransform> osgTrans;
	const btRigidBody* body;
	//bShapeSet* shapeSet = character->getOrCreateVisShapeSet();
	std::string* path = NULL;
	
	for(int i=0; i< character->getNumBtBodies(); i++)
	{
		body = character->getBtBodies()[i];
		
		/*if( _mode = STICK )
		{

		} else if ( _mode = POLYGON ) {
		}*/

		//path = shapeSet->getVisShapePath( character->link(i)->name() );
		//if( path )
		//{
		//	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(path->c_str());

		//	osg::ref_ptr<osg::Node> node;
		//	//printf("%s - ", path->c_str());
		//	if( node.valid() )
		//	{
		//		//printf("valid\n");
		//		osgTrans = new osg::MatrixTransform;
		//		osgTrans->addChild(node);

		//		osg::Quat quat;
		//		osg::Matrix osgTranMat;

		//		btTransform initTrans = body->getWorldTransform();
		//		btVector3 btV = initTrans.getOrigin();
		//		btQuaternion btQ = initTrans.getRotation();
		//		quat.set( btQ.x(), btQ.y(), btQ.z(), btQ.w() );

		//		osgTranMat.setRotate(quat);
		//		osgTranMat.setTrans(btV.x(), btV.y(), btV.z());

		//		osgTrans->setMatrix(osgTranMat);

		//	} else {
		//		osgTrans = gOSGShape::osgNodeFromBtCollisionShape(body->getCollisionShape(), body->getWorldTransform());
		//	}
		//} else {
		//	osgTrans = gOSGShape::osgNodeFromBtCollisionShape(body->getCollisionShape(), body->getWorldTransform());
		//}

		osgTrans = osgNodeFromBtCollisionShape(body->getCollisionShape(), body->getWorldTransform());

		// listing body and visBody
		addBodyAndOSG(body, osgTrans, name);
	}

	return 0;
}
