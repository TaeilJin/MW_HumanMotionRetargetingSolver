//#####################################################################
// Copyright 2010-2015, Sukwon Lee, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef MBS_BT_UTIL_H_
#define MBS_BT_UTIL_H_

#include "Base/gMath.h"
#include "btBulletDynamicsCommon.h"

///Note that gRotMat is column-major and btMatrix3x3 is row-major

gRotMat		convertToRotMat(const btMatrix3x3& in);

gXMat		convertToXMat(const btTransform& in);

btVector3	convertTobtVector3(const gVec3& in);
	
gVec3		convertTogVec3(const btVector3& in);

btMatrix3x3 convertToBtMatrix3x3(const gRotMat& in);
	
btTransform convertToBtTransform(const gXMat& in);


#if defined(MW_USE_SINGLE_PRECISION)
	inline void gXMatToBtTransfrom(btTransform& to, const gXMat& from) { to.setFromOpenGLMatrix( from.cptr() );}
	inline void btTransformTogXMat(gXMat& to,const btTransform& from) { from.getOpenGLMatrix( to.ptr() ); }
#else //defined(MW_USE_DOUBLE_PRECISION)
	inline void gXMatToBtTransfrom(btTransform& to,const gXMat& from) { to.setFromOpenGLMatrix(from.cptr());};
	inline void btTransformTogXMat(gXMat& to,const btTransform& from) { from.getOpenGLMatrix(to.ptr()); };
#endif

#endif