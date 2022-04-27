//#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#include "MocapProcessor/mgMotionDynamicFeature.h"

extern gVec3 MW_GROUND_NORMAL;

//// Only on Box shape
//int mgMotionDynamicFeature::getContactBodyPatch(MotionData *motionData)
//{
//	gOSGViewHelper* vh = ViewerFactory::getInstance()->getViewerInstance();
//
//	//boolArrayMIt it = contactFlag.begin();
//	boolArrayMIt it = motionData->contactsFlagMap.begin();
//	btRigidBody* body;
//	btBoxShape* shape;
//	btVector3 normal;
//	//gVec3 normalVec;
//	btVector3 origin;
//	mgData* motion = &motionData->motion;
//	gXMat world;
//	btVector3 btp, btpVector;
//	btTransform worldTransform;
//
//	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
//
//	while( it != motionData->contactsFlagMap.end() )
//	{
//		body = _character->getCharacterSim()->getBtBodies()[it->first];
//		shape = static_cast<btBoxShape*>(_character->getCharacterSim()->getBtShapes()[it->first]);
//
//		for( int frame=0; frame<motion->nMotion; frame++ )
//		{
//			if( !it->second[frame] ) 
//			{
//				continue;
//			}
//
//			_skeleton->getWMatrixAt(it->first, motion->motions[frame], world);
//			//_skeleton->getWHMatrixAt(it->first, world);
//
//			world = world * _character->link(it->first)->inertia().comInVec3();
//			worldTransform = convertToBtTransform(world);
//
//			//points->push_back( osg::Vec3( world.trn().x(), world.trn().y(), world.trn().z() ) );
//
//			int bestId;
//			double bestDot = DBL_MIN;
//
//			for( int i=0; i<shape->getNumPlanes(); i++ )
//			{
//				shape->getPlane(normal, origin, i);
//
//				//origin = worldTransform * origin;
//				//points->push_back( osg::Vec3( origin.x(), origin.y(), origin.z() ) );
//
//				normal = btMatrix3x3(worldTransform.getRotation()) * normal;
//				//normalVec = world.multVec3(convertTogVec3(normal));
//
//				if( bestDot < abs(normal.dot(convertTobtVector3(MW_GROUND_NORMAL)) ) )
//				{
//					bestId = i;
//					bestDot = abs(normal.dot(convertTobtVector3(MW_GROUND_NORMAL)));
//				}
//
//				//vh->addLine( gVec3(origin.x(), origin.y(), origin.z()), gVec3( normal.x(), normal.y(), normal.z() ), 2., 0.5 );
//			}
//
//			shape->getPlane(normal, origin, bestId);
//
//			normal = btMatrix3x3(worldTransform.getRotation()) * normal;
//			origin = worldTransform * origin;
//
//			osg::ref_ptr<osg::Vec3Array> contactPlane = new osg::Vec3Array();
//			contactPlane->push_back( osg::Vec3( origin.x(), origin.y(), origin.z() ) );
//			for( int i=0; i<shape->getNumVertices(); i++ )
//			{
//				shape->getVertex(i, btp);
//
//				btp = worldTransform * btp;
//				btpVector = btp - origin;
//
//				//vh->addLine( gVec3(origin.x(), origin.y(), origin.z()), gVec3(normal.x(), normal.y(), normal.z()), normal.length(), 3., osg::Vec4(1,0,0,1) );
//				//vh->addLine( gVec3(origin.x(), origin.y(), origin.z()), gVec3(btpVector.x(), btpVector.y(), btpVector.z()), btpVector.length(), 3. );
//				btpVector.normalize();
//
//				if( abs(btpVector.dot(normal)) < gEPSILON )
//				{
//					contactPlane->push_back( osg::Vec3(btp.x(), btp.y(), btp.z()) );
//				}
//			}
//
//			printf("size : %d\n", contactPlane->size() );
//
//			vh->addLine(contactPlane, 3.);
//		}
//
//		it++;
//	}
//	vh->addPoints( points, osg::Vec4(1,0,0,1), 5.);
//
//	return 0;
//}

//int mgMotionDynamicFeature::getMomentumTrajectory(std::vector<gWrench> &traj, MotionData *motionData)
//{
//	
//	// Initialize
//	traj.clear();
//		
//	gWrench momentum;
//	gLink* link;
//	gVec3 com;
//	gXMatArray pose;
//	gTwist Jvel;
//	gXMat g;
//
//	mgData& motion = motionData->motion;
//	gXMatArray pPose(_skeleton->getNumBones());
//	gXMatArray cPose(_skeleton->getNumBones());
//
//	// pre assign
//	_skeleton->getAllWMatrixAt(motion.motions[0], pPose);
//
//	for( int i=0; i< motion.nMotion; i++ )
//	{
//		_skeleton->getAllWMatrixAt(motion.motions[i], cPose);
//
//		com.set(0,0,0);
//		momentum.set(0,0,0,0,0,0);
//				
//		for( int idx=0; idx < _skeleton->getNumBones(); idx++ )
//		{
//			link = _character->link(idx);
//			com += link->inertia().mass() * cPose[idx].multVec4(link->inertia().comInVec3());
//
//			// Velocity
//			g = pPose[idx].invMult(cPose[idx]);
//			
//			Jvel.setRot( gVec3::log(g.rot()) );
//			Jvel.setTrn( g.trn() );
//
//			Jvel = Jvel * (1/motion.frameTime);
//
//			momentum += (link->inertia() * Jvel).xform(pPose[idx]);
//			pPose[idx] = cPose[idx];
//		}
//
//		com /= _character->mass();
//		traj.push_back( momentum.xform( -com ) );
//	}
//
//	return 0;
//}