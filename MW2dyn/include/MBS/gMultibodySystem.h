//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
//////////////////////////////////////////////////
//	
//		class for MULTI-BODY SYSTEM 
//		Version:	2.0
//		Author:		Lee, Sung-Hee	-- GIST
//		LAST UPDATE:		2013-02-17
//			
//		Modification:
//				13/2/18: Move solver-related functions in gMultibodySystem to gMulstibodySystemSolver
//				10/10/17: Added Doxygen help
//				05/2/12: gVec3 class is defined in mbs_general.h
//
//////////////////////////////////////////////////

#ifndef		_MBS_SYSTEM_H_
#define		_MBS_SYSTEM_H_

#include	"mbs/gRigidBodies.h"
#include	"mbs/gForces.h"

// GSL headers
#include	"Base/gBase.h"

typedef std::vector<gCoordSpring*>			gCoordSpringList;
typedef std::vector<gStateDependentForce*>	gStateDependentForceList;
typedef std::vector<gStateIndependentForce*>	gStateIndependentForceList;
typedef std::vector<gVec3>					gVec3List;

//////////////////////////////////////////////////////
/// gMultibodySystem 
/**
* This is the most basic multi-body system class, which has the following features: \n
* - Base link can be either fixed or floating. \n
* - force inputs: \n
*       1) generalized forces (control input):  tau \n
*		2) external forces are divided into two kinds: \n
*			2-1) state dependent forces: forces dependent on the state of the system. e.g., gravity, joint spring torque, ground contact force, etc \n
*			2-2) state independent forces: forces independent of the state of the system. e.g., perturbation force. \n
* \n
* Important member variables are as follows: \n
* - arma::vec	dq: generalized velocities (joint velocity vector)\n
* -	arma::vec	ddq: generalized accelerations (joint acceleration vector)\n
* -	arma::vec	tau: generalized forces (joint torque vector)\n
* -	int	nDof: degrees of freedom \n
* -	int	nLinks: number of links	\n
* -	vector<gLink*> links: all the links of the multibody system. The first element links[0] must be set to the base link (root). \n
* -	gReal	mass: total mass \n
* -	gCoordSpringList	jSprings : You can define joint springs by adding joint spring to this vector. They are passive force elements. \n
* -	int	_nJsprings : number of joint springs \n
* -	gStateDependentForceList	stateDependentForces : state dependent forces \n
* -	gStateIndependentForceList  stateIndependentForces: state independent forces \n
*\n
* Note that you can explicitly calculate the equations of motion, i.e., tau = M*ddq + C \n
*	where  arma::mat M is the mass matrix and arma::vec C is Coriolis, centrifugal, and gravity terms.
*\n
* - [Generalized coordinates, Safe coordinate array, and Compact coordinate array] \n
*  Generalized velocities (arma::vec dq) and accelerations (arma::vec ddq) are stored as a vector. 
*  This is a design choice to make it easy to deal with the system equations of motion. \n
*  However, coordinates are stored in individual links because not every link has generalized coordinate as a vector form. 
*  In case of gBallLink and gFreeLink, their DoFs are 3 and 6, but the coordinates are represented with gRotMat and gXMat (i.e., global 
* representation). \n
* In order to represent the coordinates as a vector form, we use either "global coordinate array" or "local coordinate array". \n
* Safe coordinate array (SCA) uses quaternion to represent gBallLink's coordinates and the rotation part of gFreeLink.
* Compact coordinate array (CCA) uses exponential coordinates to represent gBallLink's coordinates and the rotation part of gFreeLink.\n
* Example: If a system consists of a free base link, n ballLinks, and m g1dLinks, its DoFs are 6+3n+m.
* Then the size of SCA is 7+4n+m, and that of CCA is 6+3n+m. \n
* CCA should be used with caution because CCA may suffer from singularity problems.
*/

class gMultibodySystem{
public:	
					 gMultibodySystem			(void);
	virtual			~gMultibodySystem			(void);

	/// Copy state variables up to acceleration from src to this. Synch src's states to this.
	virtual	void	 copyVariablesUpToAcc	(gMultibodySystem* src); 

	//{	
			/// calculate the size of safe coordinate array
			/** Safe coordinate array: an 1D array of coordinates of the links.
			 * The length of the coordinate array is greater than DOFs when there is gBallLink or gFreeLink
			 * because gQuat is used as coordinates for the orientation of these links.
			*/			
			int		 calcSizeSafeCoordArray(void);// name changed from calcSizeGlobalCoordArray

			/// get the size of safe coordinate array 
	inline	int		 sizeSafeCoordArray(void) const { return _szSafeCoordArray; }

			/// set coordinates from the safe coordinate array *gca
			void	 setFromSafeCoordArray	(const arma::vec& gca); // name changed from setFromGlobalCoordArray

			/// copy coordinates to the coordinate array *gca
			void	 getSafeCoordArray		(arma::vec& gca) const; //name changed from getGlobalCoordArray

			/// get index of link i in safe coord array
			int		 getSafeCoordArrayIndex (int linkIndex) { assert(_safeCoordArrayIndices.size()>0); return _safeCoordArrayIndices[linkIndex]; }
	//}

	//{	
			/// calculate the size of safe coordinate SUB-array of links in links
			/** links: indices of target links
			*/
			int		 calcSizeSafeCoordArrayOfLinks(const std::vector<int>& links);

			/// set coordinates from the safe coordinate array *gca
			void	 setFromSafeCoordArrayOfLinks(const std::vector<int>& links,const arma::vec& gca); 

			/// copy coordinates to the coordinate array *gca
			void	 getSafeCoordArrayOfLinks(const std::vector<int>& links, arma::vec& gca); 
	//}

	//{
	
	/// return the size of the local coordinate array
	/**
	Compact coordinate array: an 1D array of local coordinates. Its size equals to DOF.	
	For gBallLink and gFreeLink, exponential coordinates are used for the orientation.
	*/
	inline	int		 sizeCompactCoordArray(void) const { return _nDof; } //name changed from sizeLocalCoordArray

			/// copy compact coordinates to the array *lca
			void	 getCompactCoordArray (arma::vec& lca) const; //name changed from getLocalCoordArray

			/// set coordinates from the compact coordinate array lca
			void	 setFromCompactCoordArray(const arma::vec& lca); //name changed from setFromLocalCoordArray

			/// get index of link i in compact coord array
			int		 getCompactCoordArrayIndex (int linkIndex) const { assert(_compactCoordArrayIndices.size()>0); return _compactCoordArrayIndices[linkIndex]; }

			/// set coordinates from the compact coordinate array *gca
			void	 setFromCompactCoordArrayOfLinks(const std::vector<int>& links, const arma::vec& gca);

			/// copy coordinates to the coordinate array *gca
			void	 getCompactCoordArrayOfLinks(const std::vector<int>& links, arma::vec& gca);

	//}		

	//{UNIT
	inline	MW_UNIT	unit() { return _unit; }
	inline	void	setUnit(MW_UNIT unit) { _unit = unit; }
	//}

	//// TAEIL 

	/// Leg indices: list of link indices from pelvis to the foot. Used for computing Leg Jacobian.
	std::vector<int>	m_lLegIdx;
	std::vector<int>	m_rLegIdx;
	/// Arm indices: list of link indices from chest to the hand. 
	std::vector<int>	m_lArmIdx;
	std::vector<int>	m_rArmIdx;
	/// Trunk indices: list of link indices from pelvis to the chest 
	std::vector<int>	m_trunkIdx;
	/// Head indices: list of link indices from chest to the head 
	std::vector<int>	m_headIdx;

	
	bool inLLeg(int linkIdx)
	{
		std::vector<int>::iterator it = std::find(m_lLegIdx.begin(), m_lLegIdx.end(), linkIdx);
		if (it != m_lLegIdx.end()) return true;
		return false;
	}

	bool inRLeg(int linkIdx)
	{
		std::vector<int>::iterator it = std::find(m_rLegIdx.begin(), m_rLegIdx.end(), linkIdx);
		if (it != m_rLegIdx.end()) return true;
		return false;
	}

	bool inLArm(int linkIdx)
	{
		std::vector<int>::iterator it = std::find(m_lArmIdx.begin(), m_lArmIdx.end(), linkIdx);
		if (it != m_lArmIdx.end()) return true;
		return false;
	}

	bool inRArm(int linkIdx)
	{
		std::vector<int>::iterator it = std::find(m_rArmIdx.begin(), m_rArmIdx.end(), linkIdx);
		if (it != m_rArmIdx.end()) return true;
		return false;
	}

	bool inTrunk(int linkIdx)
	{
		std::vector<int>::iterator it = std::find(m_trunkIdx.begin(), m_trunkIdx.end(), linkIdx);
		if (it != m_trunkIdx.end()) return true;
		return false;
	}

	//{coordinates in individual links

			/// set coordinates to every link to zero
			void	 setZeroCoord		(void);
	
			/// memorize current coordinates before new values are set to the coordinates. Use restoreCoord() to restore the coordinates.
			void	 storeCoord();
			
			/// set the value of the coordinates to the memorized values.
			void	 restoreCoord();
	
	/// set base frame position&orientation. 
	inline  void	 setBasePose(const gXMat& T){_links[0]->localFrame().set(T);}

	/// set base frame position only
	inline 	void	 setBasePosition(const gVec3& pos){_links[0]->localFrame().setTrn(pos);}

	/// set base frame orientation only
	inline 	void	 setBaseOrientation(const gRotMat& R){_links[0]->localFrame().setRot(R);}
	inline 	void	 setBaseOrientation(const gQuat& Q){_links[0]->localFrame().setRot(Q.inRotMatrix());}

	/// set generalized velocity vector
	inline	void	 setVel				(const arma::vec& dq)	{ _dq = dq; }

	/// set i-th element of generalized velocity vector
	inline	void	 setVel				(int index, gReal v) { _dq(index)=v; }

	/// set generalized velocity sub-vector [startIdx, startIdx+size]
			void	 setVel				(int startIdx, int size, gReal* v);

	/// set generalized acceleration vector
	inline	void	 setAcc				(const arma::vec& ddq)	{ _ddq = ddq; }

	/// set i-th element of generalized acceleration vector
	inline	void	 setAcc				(int index, gReal a) { _ddq(index)=a; }

	/// set generalized acc sub-vector [startIdx, startIdx+size]
			void	 setAcc				(int startIdx, int size, gReal* a);

	/// set generalized force vector
	inline	void	 setForce			(const arma::vec& tau)	{ _tau = tau; }

	/// set i-th generalized force
	inline	void	 setForce			(int index, gReal torq) { _tau(index)=torq; }

	/// set generalized force sub-vector [startIdx, startIdx+size]
			void	 setForce			(int startIdx, int size, gReal* torq);

	/// return the DOFs of the system
	inline	int		 dof				(void) const	{ return _nDof; }		

	/// return the numer of links
	inline	int		 numLinks			(void) const	{ return _nLinks; }

	/// return the number of joint springs
	inline	int		 numJointSprings	(void) const	{ return _nJsprings; }

	/// return the pointer to the base link, which is ancestor of all other links. Baselink must be the first element of links.
	inline	gLink*	 baseLink			(void)			{ return _links[0]; }

	/// return the point to i'th link
	inline	gLink	*link				(int i)			{ return _links[i]; }	// get link[i], i=[0:number of links-1]

	inline 	const gLink	*link			(int i)	const	{ return _links[i]; }	// get link[i], i=[0:number of links-1]

	/// return the reference to links
	inline	std::vector<gLink*>& links	(void)			{ return _links; }

	/// return the pointer to the i'th joint spring
	inline	gCoordSpring *jointSpring	(int i)			{ return _jSprings[i]; }

	/// return the pointer to the generalized velocity vector
	inline	arma::vec&		vel			(void)			{ return _dq; }

	/// return i-th element of the generalized velocity vector
	inline	gReal&	 vel				(int index)		{ return _dq(index); }
	
	/// return the pointer to the generalized acceleration vector
	inline	arma::vec&		acc			(void)			{ return _ddq; }

	/// return i-th element of the generalized acceleration vector
	inline	gReal&	 acc				(int index)		{ return _ddq(index); }	

	/// return the pointer to the generalized force vector
	inline	arma::vec&		force		(void)			{ return _tau; }

	/// return i-th element of the generalized force vector
	inline	gReal&	 force				(int index)		{ return _tau(index); }
		
	/// return total mass
	inline	gReal	 mass				(void)			{ return _mass;}
	inline	gReal	 mass				(void) const 	{ return _mass;}

	/// return joint space mass matrix
	/**
	Note that computeJointSpaceEquationsOfMotion function updates _M
	*/
	inline  arma::mat&		massMatrix			(void)	{ return _M; }

	/// return joint space C vector in equation of motion
	/**
	Note that computeJointSpaceEquationsOfMotion function updates _C
	*/
	inline  arma::vec&		CVector			(void)			{ return _C; }
	
	/// Placeholder for the tasks after loading a model
	virtual	void	 postLoading		(void);	

	/// Randomize coordinates
	virtual void	 randomizeCoord		(gReal rangeScale);		// set random joint angle

	/// Print topology of the system for inspection
	void			 printHierarchy		(char* filename);	

	/// Find a link given name
	gLink*			 findLink			(const char* name);
	const gLink*	 findLink			(const char* name) const;

	/// return the index of a link inside vector::_links
	//int				 getLinkIndex		(gLink* link); //deprecated.
	int				 findLinkIndex		(gLink* link); ///Note: avoid using this function where speed is important

	/// find a joint spring with the pointer to the child link
	gCoordSpring*	 findJointSpringOfLink	(gLink* link);

	//{ construction functions
	/// add a link to links
	inline	void	 addLink			(gLink* link)	{ _links.push_back(link); }

	/// remove a link from links. You must make sure that nothing (e.g., joint spring) points to the removed link before removing a link.
			void	 removeLink			(gLink* link);  
	
	/// add a joint spring
	inline	void	 addJointSpring		(gCoordSpring* spring) { _jSprings.push_back(spring); }

	/// add a state dependent force
	inline	void	 addStateDependentForce		(gStateDependentForce* force) { _stateDependentForces.push_back(force); }

	/// add a state independent force
	inline	void	 addStateIndependentForce	(gStateIndependentForce* force) { _stateIndependentForces.push_back(force); }

			/// clear state independent force
			void	 clearStateIndependentForces(void);
			
			/// Connect between parent link (plink) and the child link (clink)
			void	 connect			(gLink* plink, gLink* clink);

	/// initialize the system. It must be called after constructing the system.		
	virtual void	 initialize			(void);		
			
			/// Check if l1 is an ancestor of l2
			bool	 checkAncestor		(gLink* l1, gLink* l2);

	//}
	//{ kinematic variable updater

	
	///Given each link's coordinates, update kinematic variables of startLink and its descendant links up to the position level (e.g., each link's frames)
	/** \param[in] startLink kinematic variables of startLink and its descendants will be updated. \n
	 * If you want to update kinematic variables of all the links, set startLink=NULL, instead of setting startLink=baseLink().\n
	 * The former is slightly faster as it uses loop-unrolled algorithm.
	*/
	virtual	void	 updateKinematicsUptoPos		(gLink* startLink=NULL);		

	/// Given the coordinates and generalized velocity, update kinematic variables of startLink and its descendant links up to the velocity level (each link's frames and velocities)
	/** \param[in] startLink kinematic variables of startLink and its descendants will be updated. \n
	 * If you want to update kinematic variables of all the links, set startLink=NULL, instead of setting startLink=baseLink().\n
	 * The former is slightly faster as it uses loop-unrolled algorithm.
	*/
	virtual	void	 updateKinematicsUptoVel		(gLink* startLink=NULL);		

	/// Given the coordinates, gen. velocities and gen. accelerations, update kinematic variables of startLink and its descendant links up to the acceleration level (each link's frame, velocities, and accelerations). This function is mainly used for inverse dynamics.
	/** \param[in] startLink kinematic variables of startLink and its descendants will be updated. \n
	 * If you want to update kinematic variables of all the links, set startLink=NULL, instead of setting startLink=baseLink().\n
	 * The former is slightly faster as it uses loop-unrolled algorithm.
	*/
	virtual	void	 updateKinematicsUptoAcc		(gLink* startLink=NULL);
	
	//}

	//{ external force updater
			/// Clear applied force info (external force and gen. force) of each link
			void	 clearAllForces		(void);

			/// Clear only external force info of each link. Generalized force info is kept.
			void	 clearExtForce		(void);

			/// Compute external forces and apply to the links.
	virtual void	 updateExtForce		(void);		
	//}

	/// Compute locked inertia of link and all of its desecendants.
	/** A locked inertia is the generalized inertia of a set of links as if they form a single rigid body (i.e., all the joints are locked).
	*/
	gInertia		 computeLockedInertia(gLink* link);

	/// Compute _M and _C where the equations of motion is _tau = _M*_ddq + _C. Note that _C includes all the non-inertial forces except generalized forces.
	void			 computeJointSpaceEquationsOfMotion(void); 

	/// compute first six rows of equations of motion for the floating base, i.e., 0 = M0 * _ddq + C0. See computeJointSpaceEquationsOfMotion(void).
	bool			 computeJointSpaceEquationsOfMotionFloatingBasePartOnly(arma::mat& M0, arma::vec& C0); 


	//{ forward dynamics
#ifdef MW_USE_ARTICULATED_BODY_METHOD

	/// Solve hybrid system dynamics (Featherstone)
	/**
	* If i_th joint is accDriven (_bAccDriven is set true), necessary torque (_tau[i]) is computed for the desired joint accelerations (_ddq[i]). \n
	* Otherwise(_bAccDriven is set false), i_th joint is torque-driven, and resulting acceleration (_ddq[i]) is computed for the input torques (_tau[i]).
	*/
	void	hybridDynamicsFeatherstone (bool updateOnlyBiasForce=false);			
	
#endif

	//{ inverse dynamics
	/** This function performs inverse dynamice: It computes generalized force (_tau) given desired joint accelerations(ddq) and external forces(Fext). \n
	* Computed values are stored in _tau.
	* Note that this ignores a link's accDriven flag, and every link is considered torque-driven. \n
	* Use hybridDynamicsFeatherstone() if you have accDriven joints.
	*/
	void	inverseDynamics		(void);			
	//}
		
	///Forward dynamics using Orin's method (old version of Orin's method)
	/** This function performs forward dynamics: It computes joint acclerations(_ddq) given input generalized force(_tau) and external forces(Fext). \n
	* Set computeEquationsOfMotion to true if M and C is not up to date. \n
	* Note that after this function calls, tau is corrupted. \n
	* Also note that this ignores a link's accDriven flag, and every link is considered torque-driven. \n
	* Use hybridDynamicsFeatherstone() if you have accDriven joints.
	*/
	void	computeAccelerationOrin	(bool computeEquationsOfMotion=true);
	//}

	/// Compute composite inertia of each link. This function is used to compute joint space mass matrix M
	void	computeCompositeInertiasWorld(void);
		
	////{ point forces
	//std::vector<gForcePoint*>		_contactPoints; 
	//void				(*_callbackContactForce)(gMultibodySystem*);
	//void				setCallbackContactForce( void(*fn)(gMultibodySystem*) ) { _callbackContactForce = fn; } 
	////}
	
//protected:

	/// set each link's genCoordIndex, which is each link's initial index in the generalized velocity/acceleration vectors.
	void			 setGenCoordIndex(void);	

	/// compute total mass and update mass variable
	void			 initializeMass		(void);	

	/// initialize _nLiks, _nDoFs, and link's id i
	void			 initializeHierarchy(void);


#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
	//{ collision manifold updated
	///update collision manifolds of the startLink and its descendants
	/**this function is used mainly for implicit integration methods.
	* NOT VERIFIED YET. NEED TO VALIDATE THIS FUNCTION.
	* TODO: For self-collision, manifolds are redundantly updated, which slows down the performance.
	* In future, we need to find more clever algorithm to get rid of this redundancy
	*/
	virtual void	 updateCollisionManifold( gLink* startLink=NULL );
		
	//}
#endif


protected:
	MW_UNIT			_unit;
	//{ interface
	arma::vec		_dq;
	arma::vec		_ddq;
	arma::vec		_tau;
	//}
	int				_nDof;				// degrees of freedom
	int				_nLinks;			// number of links	
	gLinkList		_links;				// links
	gReal			_mass;				// total mass
	//{ force elements
	int				_nJsprings;			// num joint springs
	gCoordSpringList	_jSprings;			// joint springs. passive force element
	gStateDependentForceList	_stateDependentForces; // state dependent forces (e.g., gravity, spring forces, etc)		
	gStateIndependentForceList  _stateIndependentForces; // state independent forces (e.g., user input forces)
	//}
	//{ used for computeAccelerationOrin which create joint space equations of motion: _tau = _M*_ddq + _C
	arma::mat			_M;
	arma::vec			_C;
	//}

	//{ variables for safe coordinate array
	int				  _szSafeCoordArray;   //size of safe coordinate array
	std::vector<int>  _safeCoordArrayIndices;   //stores index of each link in safeCoordArray. E.g., if j=_safeCoordIndices[i] and k=_safeCoordIndices[i+1], link[i]'s coordinates are stored at {j...k-1}	
	//}

	//{ variables for compact coordinate array
	int				  _szCompactCoordArray;   //size of compact coordinate array == DoF
	std::vector<int>  _compactCoordArrayIndices;   //stores index of each link in compactCoordArray. E.g., if j=_compactCoordIndices[i] and k=_compactCoordIndices[i+1], link[i]'s coordinates are stored at {j...k-1}	
	//}

#ifdef MW_USE_DIFF_NEWTON_EULER //differentiation of the NE inverse dynamics
public:
	void inverseDynamicsP (void); //compute link->_taup
	void updateExtForceP(void);
	void collectGenForceP(arma::vec& taup);
	void setZeroKVariablesP(void);
	void updateKVariablesP(gLink* startLink=NULL);
	//void testDiffNewtonEuler(void);
#endif //MW_USE_DIFF_NEWTON_EULER
};


#endif