//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------

#ifndef		_MBS_SYSTEM_INTEGRATOR_H_
#define		_MBS_SYSTEM_INTEGRATOR_H_

#include	"mbs/gMultibodySystem.h"

//////////////////////////////////////////////////////
/// gMultibodySystemSimulator 
/**
* This class provides a set of functions for time integrating gMultibodySystem
*/
class gMultibodySystemTimeIntegrator
{
public:
	gMultibodySystemTimeIntegrator(gMultibodySystem* sys) { 
		_system = sys; 
		#ifdef MBS_USE_COLLISION_MANIFOLD
		_considerContactForceChangeInImplicitMethods = true;
		#endif
	}

	/// Use Composite Inertia Method and expliti Euler time stepping
	void	nextStepExplicitEulerOrin(gReal timeStep);

	/// Implicit method - residual method with numerical differentiation
	void	nextStepImplicitEulerInverseDynamicsBased1stOrderHigh(gReal timeStep);

	/// Implicit method - residual method with analytic differentiation
	void	nextStepImplicitEulerInvDynTypeAnalytic(gReal timeStep);

	/// time integrate only coordinates using the explicit Euler method.
	void	timeIntegrateCoordinates(gReal timestep);

	/// time integrate coordinates and velocities using the explicit Euler method.
	void	timeIntegrateExplicitEuler(gReal timestep);

	/// time integrate  coordinates and velocities using the semi-implicit Euler method.
	void	timeIntegrateSemiImplicitEuler(gReal timestep);

#ifdef MW_USE_ARTICULATED_BODY_METHOD

	/// Use Featherstone's method and explicit Euler time stepping.
	void	nextStepExplicitEulerFeatherstone(gReal timeStep);

	/// Use Featherstone's method and semi-implicit Euler time stepping.
	void	nextStepSemiImplicitEulerFeatherstone(gReal timeStep);

	/// Use Featherstone's method and 1st order implicit time stepping.
	void	nextStepImplicitEuler1stOrder(gReal timeStep, bool assumeZeroTimeStepSquared = false);

	/// Same as nextStepImplicitEuler1stOrder, but with higer order approximation to fx,fv
	/** This method takes twice the time of nextStepImplicitEuler1stOrder, but timestep can be more than twice.
	*/
	void	nextStepImplicitEuler1stOrderHigh(gReal timeStep);
	
	/// Iteratively find the solution for the implicit method.
	void	nextStepImplicitEulerIterative(gReal h, int maxIter, gReal threshold);
	//}

#endif 

#ifdef MBS_USE_COLLISION_MANIFOLD
	/** If b=true, implicit methods will calculate contact force changes for numerical approximation of grdients.
	 * If b=false, contact forces are assumed constant againt change of generalized coordinates and velocities.
	*/
	void	setConsiderContactForceChangeInImplicitMethods(bool b) { _considerContactForceChangeInImplicitMethods = b; }
#endif

protected:

	gMultibodySystem		*_system;

#ifdef MBS_USE_COLLISION_MANIFOLD
	/** If true, implicit methods will calculate contact force changes for numerical approximation of grdients.
	 * if false, contact forces are assumed constant againt change of generalized coordinates and velocities.
	*/
	bool _considerContactForceChangeInImplicitMethods;
#endif

};

#endif