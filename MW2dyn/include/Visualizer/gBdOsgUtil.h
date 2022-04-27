//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _MBS_BD_OSG_UTIL_H_
#define _MBS_BD_OSG_UTIL_H_

//#include "Visualizer/gVisHeader.h"
#include "Visualizer/gOsgShape.h"

#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/Vec4>
#include <btBulletDynamicsCommon.h>
/**
* BULLET TO OSG STRUCTURE UTILITY
**/

osg::Matrix asOsgMatrix( const btTransform& t );

btTransform sBtTransform( const osg::Matrix& m );

osg::Matrix asOsgMatrix( const btMatrix3x3& m );

btMatrix3x3 asBtMatrix3x3( const osg::Matrix& m );

osg::Vec3 asOsgVec3( const btVector3& v );

btVector3 asBtVector3( const osg::Vec3& v );

osg::Vec4 asOsgVec4( const btVector3& v, const double w );

osg::Vec4 asOsgVec4( const btVector4& v );

btVector4 asBtVector4( const osg::Vec4& v );


osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btCollisionShape* btShape, const btTransform& trans = btTransform::getIdentity() );
osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btSphereShape* btSphere, const btTransform& trans = btTransform::getIdentity() );
osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btBoxShape* btBox, const btTransform& trans = btTransform::getIdentity() );
osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btCylinderShape * btCylinder, const btTransform& trans = btTransform::getIdentity() );
osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btTriangleMeshShape* btTriMesh, const btTransform& trans = btTransform::getIdentity() );
osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btCapsuleShape* btCapsule, const btTransform& trans = btTransform::getIdentity() );

// Constraints
int osgNodeFromBtConstraint( btTypedConstraint* constraint, osg::ref_ptr<osg::MatrixTransform> transA, osg::ref_ptr<osg::MatrixTransform> transB );


// SWL
// 2014-03-12


#endif //_MBS_BD_OSG_SYSTEM_H_