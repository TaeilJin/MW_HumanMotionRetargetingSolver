//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------

#ifndef		_MBS_SYSTEM_SOLVER_H_
#define		_MBS_SYSTEM_SOLVER_H_

#include	"mbs/gMultibodySystem.h"


//////////////////////////////////////////////////////
/// gMultibodySystemSolver - solver class for gMultibodySystem 
/**
* Design Principles: store variables essential for simulating system in gMultibodySystem. \n
* All other properties (eg. for momentum,CoM,CoP...) are computed by gMultibodySystemSolver. \n
*/ 
class gMultibodySystemSolver
{
public:
					 gMultibodySystemSolver(gMultibodySystem* mbs);

	virtual			~gMultibodySystemSolver(void){};
	
	///Compute kinetic energy
	virtual gReal	 computeKineticEnergy(void);		// solve kinetic energy of current state

	///Compute gravitational potential energy
	virtual gReal	 computeGravityPotentialEnergy(gReal zeroHeight); // zeroHeight is the height of zero energy

//	virtual gReal	 sol_springPotentialEnergy(void);

	///Compute center of mass
	gVec3	computeCenterOfMass(void);

	///Compute generalized momenta wrt world and CoM
	void	computeMomentum(gWrench& momWorld, gWrench& momCoM, const gVec3& CoM);

	///Compute generalized momentum rate changes wrt world and CoM, given joint accelerations in the system
	/**
	Note that the momentum rate changes computed by this function are not determined by the external forces, but by the joint accelerations.
	*/
	void	computeMomentumRateChangeFromAcc(gWrench& momRateChangeWorld, gWrench& momRateChangeCoM, const gVec3& CoM); 
	
	///Compute ground reaction force(GRF) from lin. momentum rate change
	///Set linMomRateChange = momRateChangeWorld.trn() which can be computed from computeMomentumRateChangeFromAcc.
	gVec3	computeGroundReactionForce(const gVec3& linMomRateChange);
	
	///Compute CoP from GRF and angular momentum rate change (about CoM)
	/***GRF can be computed from computeGroundReactionForce(const gVec3&) \n
	* Set angMomRateChangeCoM = momRateChangeCoM.rot() that can be computed from computeMomentumRateChangeFromAcc(gWrench&,gWrench&,const gVec3&)  \n
	* groundHeight is the height of ground
	*/
	bool	computeCenterOfPressure(gVec3& CoP,	const gVec3& CoM, const gVec3& GRF, const gVec3& angMomRateChangeCoM, const gReal groundHeight);
	
	///Compute vertical component of ground reaction moment from angular momentum rate change about CoM, CoP, and GRF
	/*** CoP can be computed from computeCenterOfPressure(gVec3&,const gVec3&,const gVec3&,const gVec3&,const gReal). \n
	* GRF can be computed from computeGroundReactionForce(const gVec3&) \n
	* Set angMomRateChangeCoM = momRateChangeCoM.rot() that can be computed from computeMomentumRateChangeFromAcc(gWrench&,gWrench&,const gVec3&)  \n
	*/
	double 	computeGroundReactionMomentVertical(const gVec3& CoM, const gVec3& CoP, const gVec3& GRF, const gVec3& angMomRateChangeCoM);
		
protected:

	gMultibodySystem		*_system;

};

#endif