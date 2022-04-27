//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
#ifndef _MBS_UTIL_H_
#define _MBS_UTIL_H_

#include "Base/gBase.h"
#include "Base/gMath.h"
#include "armadillo"

//compute principal axes (Paxes) and principal moments (Pmoments) of a generalized inertia
//Paxes are located at CoM, and it is right-handed
void computePrincipalAxis(const gInertia& M, gRotMat& Paxes, gVec3& Pmoments);

// make 6x6 matrix of Adjoint mapping
// return 0 if success, non-zero othewise
//int make_matrix_Ad(GM* M, const gXMat& T); 
int make_matrix_Ad(arma::mat& M, const gXMat& T);

// (Ad_T^{-1})
// return 0 if success, non-zero othewise
//int make_matrix_Adi(GM* M, const gXMat& T);
int make_matrix_Adi(arma::mat& M, const gXMat& T);

//check positive definiteness of J
bool check_positive_definiteness(gInertia J);

void	invMatrix33 (gReal* K,const gReal* M); 	//K=inv(M). M[9] and K[9] are 9d vectors representing 3x3 matrices in column order 

gVec3	multMatrix33WithVec3(const gReal* M, const gVec3& v); //return M*v where M[9] is 3x3 matrix (column major order)

gVec3	proj (const gVec3 x, const gVec3 u, const gVec3 v);	// projection of x onto the plane whose base vectors are u and v ( u, v need not be unit nor orthogonal gVec3 )

gReal	distancePoint2Line (const gVec3 x, const gVec3 start, const gVec3 end); // get distance between point x and line end-start

gReal	distanceGeodesic	(const gRotMat& R1, const gRotMat& R2); 

inline	gReal	gDistance(const gVec3& p1, const gVec3& p2)	// distance between two points
				{ return gSqrt(gSqr(p1.x()-p2.x()) + gSqr(p1.y()-p2.y()) + gSqr(p1.z()-p2.z())); }

//find tangent points from point1 to sphere, and from point2 to sphere
//such that tangent points lie in a plane consisting point1,2 and the center of the sphere
//among the two tangent points from each point, select one that (t-c)'n is positive
//where t is a tangent point, c is the center, n is normal
void computeTangentialPointsFromTwoPointsToSphere(gVec3& res1, gVec3& res2, const gVec3& point1, const gVec3& point2, const gVec3& sphereCenter, gReal radius, const gVec3& normal);

//find a projection of a vector v to a plane consisting of origin, point1 and point2
//projection = c1*point1 + c2*point2
void computeProjectionOfVectorToPlane(gReal* c1, gReal* c2, const gVec3& v, const gVec3& point1, const gVec3& point2);

//return true if a circle and line contact, false or else.
//con: contact point if contacted
//center: center of circle
//dir: direction of circle, a tangent vector to the surface made by circle
//point1,2: points in line
//Between the two intersection points between a circle and a line,
//con is chosen that center-con and dir form acute angle
//let con = a*(point1-center)+b*(point2-center)
//if a+b > 1 then no-contact
//else contact
//WARNING: point1,2,and center must not be collinear.
//also, the infinite plane made by the circle is assumed to intersect with the line
bool contact_directionalCircleAndLine(gVec3& con, const gVec3& center, gReal radius, const gVec3& dir, const gVec3& point1, const gVec3& point2);


#endif