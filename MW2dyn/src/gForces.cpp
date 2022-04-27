//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------

#include "Base/gMath.h"
#include "mbs/gMbsUtil.h"
#include "mbs/gForces.h"

using namespace std;

extern gVec3 MW_GRAVITY_VECTOR;

typedef vector<gForcePoint*>::iterator gPointItor;
#define	FOR_EACH_VPOINT		for(gPointItor it = _vpoint.begin(); it!=_vpoint.end(); it++)


void gAttachedPoint::updateKinematicsUptoPos(void)		
{ 
	_posWorld = _pBody->frame().multVec4(_posLocal); 
}

void gAttachedPoint::updateKinematicsUptoVel(void)		
{ 
	updateKinematicsUptoPos(); 
	_velWorld = _pBody->frame().multVec3(_pBody->frameVel().multVec4(_posLocal)); 
}

void gGravityForce::computeForce() { 
		_forceLocal = _pBody->frame().invMultVec3( MW_GRAVITY_VECTOR * _pBody->inertia().mass()  );
		//_point.setForceInWorld( MW_GRAVITY_VECTOR * _point.body()->inertia().mass() ); 
}


//////////////////////////////////////////////////////
// class gSpring
//////////////////////////////////////////////////////





void g1dLinkSpring::addForceToBody() 
{
	gWrench F(_pLink->screw()*_force); // project joint torque to Fext (beware of sign)
	_pLink->addExtForce(F);
	_pLink->parent()->subtractExtForce(F.xform(_pLink->localFrame()));
}


void gBallLinkSpring::addForceToBody()
{
	gWrench F(_force[0],_force[1],_force[2],gZero,gZero,gZero);
	_pLink->addExtForce(F);
	_pLink->parent()->subtractExtForce(F.xform(_pLink->localFrame()));
}

void gPiecewiseLineSegmentSpring::appendVpointInLocal(gRigidBody* body, const gVec3& pos)
{
	gForcePoint* pt = new gForcePoint(body,pos); 
	_vpoint.push_back(pt);
	_nVpoints++;
}

void gPiecewiseLineSegmentSpring::appendVpointInWorld(gRigidBody* body, const gVec3& pos)
{
	gForcePoint* pt = new gForcePoint(body,body->frame().invMultVec4(pos));
	_vpoint.push_back(pt);
	_nVpoints++;
}

void gPiecewiseLineSegmentSpring::computeRestLength(gReal scale)
{
	computeLengthAndItsChangeRate();
	_len0 = scale*_len;
	assert( _len0 > gEPSILON );
}

void gPiecewiseLineSegmentSpring::updateKinematicsUptoPos()
{
	FOR_EACH_VPOINT
	{
		(*it)->updateKinematicsUptoPos();
	}

	//update _len
	_len=0;
	for(int i=0;i<(_nVpoints-1);++i)
	{
		gVec3 v(_vpoint[i+1]->posWorld(),_vpoint[i]->posWorld());
		gReal dl = v.magnitude();
		_len += dl;
	}	
}

void gPiecewiseLineSegmentSpring::updateKinematicsUptoVel()
{
	FOR_EACH_VPOINT
	{
		(*it)->updateKinematicsUptoVel();
	}
	computeLengthAndItsChangeRate();
}

void gPiecewiseLineSegmentSpring::addForceToBody()
{
	FOR_EACH_VPOINT
	{
		(*it)->addForceToBody();
	}
}




