//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
#include "mbs/gMultibodySystemSolver.h"

extern	gVec3	MW_GRAVITY_VECTOR;
extern  gVec3	MW_GROUND_NORMAL;

using namespace std;

////////////////////////////////////////////////////////////////
//	class gMultibodySystemSolver
////////////////////////////////////////////////////////////////

gMultibodySystemSolver::gMultibodySystemSolver(gMultibodySystem* mbs)
{
	_system = mbs;
}

gReal gMultibodySystemSolver::computeKineticEnergy(void)
{	
	gReal energy = 0;
	for(int i=0;i<_system->numLinks();++i){		
		energy += _system->link(i)->calcKineticEnergy();
	}
	return energy;
}

gReal gMultibodySystemSolver::computeGravityPotentialEnergy(gReal zeroHeight)
{
	gReal energy = 0;
	for(int i=0;i<_system->numLinks();++i){
		energy += _system->link(i)->calcPotentialEnergy(zeroHeight);
	}
	return energy;
}

gVec3 gMultibodySystemSolver::computeCenterOfMass(void)
{
	gVec3 com(0,0,0);
	for(int i=0;i<_system->numLinks();++i){
		gLink* l = _system->link(i);
		com += (l->frame().multVec4( gVec3(l->inertia().com()) ))*l->inertia().mass();
	}
	com /= _system->mass();
	return com;
}

void	gMultibodySystemSolver::computeMomentum(gWrench& momWorld, gWrench& momCoM, const gVec3& CoM)
{
	gWrench x;
	//momWorld
	momWorld.setZero();
	for(int i=0;i<_system->numLinks();++i){
		gLink* l = _system->link(i);
		x.makeDAdInv(l->frame(), l->inertia() * l->frameVel() );
		momWorld += x;
	}

	//momWorld = {K_0, L_0}
	//momCoM = {K_c, L_c}
	//K_c = K_0 - cross(com,L_0)
	//L_c = L_0	
	gVec3 L = momWorld.trn();
	momCoM.set(momWorld.rot() - (CoM % L ), L);	
}

void gMultibodySystemSolver::computeMomentumRateChangeFromAcc(gWrench& momRateChangeWorld, gWrench& momRateChangeCoM, const gVec3& CoM)
{
	//dP_w
	momRateChangeWorld.setZero();
	for(int i=0;i<_system->numLinks();++i){
		gLink* l = _system->link(i);
		momRateChangeWorld += gWrench::dAdInv(l->frame(), l->inertia()*l->frameAcc() - gWrench::dad(l->frameVel(),l->inertia()*l->frameVel()));
	}

	//dP_c = {dK_c, dL_c}
	//dK_c = dK_0 - cross(com,dL_0)
	//dL_c = dL_0
	gVec3 dL = momRateChangeWorld.trn();
	momRateChangeCoM.set(momRateChangeWorld.rot() - (CoM % dL ), dL);		
}

gVec3	gMultibodySystemSolver::computeGroundReactionForce(const gVec3& linMomRateChange)
{ 
	return linMomRateChange - MW_GRAVITY_VECTOR*_system->mass(); 
}
	
double 	gMultibodySystemSolver::computeGroundReactionMomentVertical(const gVec3& CoM, const gVec3& CoP, const gVec3& GRF, const gVec3& angMomRateChangeCoM)	
{ 
	return (MW_GROUND_NORMAL, angMomRateChangeCoM + ((CoM-CoP)%GRF));
}


bool	gMultibodySystemSolver::computeCenterOfPressure(
	gVec3& CoP,
	const gVec3& GRF, 
	const gVec3& CoM, 
	const gVec3& angMomRateChangeCoM, 
	const gReal groundHeight)
{
	if((GRF,MW_GROUND_NORMAL)<1e-5){
		CoP.setZero();
		return false; //center of pressure not exists.
	}

	gVec3 o(MW_GROUND_NORMAL*groundHeight); //a point on the ground
	gVec3 og(CoM,o);		//og: o to com
	gVec3 dH = angMomRateChangeCoM + (og%GRF);
	CoP = (MW_GROUND_NORMAL%dH)*gInv((MW_GROUND_NORMAL,GRF)) + o;
	return true;
}

