//#####################################################################
// Copyright 2010-2015, Sukwon Lee, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#include "Character/gBdUtil.h"

gRotMat		convertToRotMat(const btMatrix3x3& in)
{ 
	return gRotMat(in[0][0], in[1][0], in[2][0], in[0][1], in[1][1], in[2][1], in[0][2], in[1][2], in[2][2] ); 
}

gXMat		convertToXMat(const btTransform& in)
{ 
	gXMat T(convertToRotMat(in.getBasis()));	
	T.setTrn( in.getOrigin().x(), in.getOrigin().y(), in.getOrigin().z() );	
	return T;
};
	
btVector3	convertTobtVector3(const gVec3& in){ return btVector3(in.x(), in.y(), in.z()); };

gVec3		convertTogVec3(const btVector3& in){ return gVec3(in.x(), in.y(), in.z()); };

btMatrix3x3 convertToBtMatrix3x3(const gRotMat& in)
{
	return btMatrix3x3(in.e(0),in.e(3),in.e(6), in.e(1),in.e(4),in.e(7), in.e(2),in.e(5),in.e(8) );
}

btTransform convertToBtTransform(const gXMat& in)
{	
	//btMatrix3x3 R(in.e(0,0),in.e(0,1),in.e(0,2), in.e(1,0),in.e(1,1),in.e(1,2), in.e(2,0),in.e(2,1),in.e(2,2) );	
	//return btTransform(R,btVector3(in.e(0,3),in.e(1,3),in.e(2,3)));
	//Same as above.
	btTransform re;	re.setFromOpenGLMatrix(in.cptr());	return re;
}
	
