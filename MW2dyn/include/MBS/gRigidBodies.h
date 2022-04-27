//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// MBS_RIGID_BODIES.H
//
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
//
// UPDATE HISTORY: 
//	2011-10-17: added Doxygen help.
//
// This file contains a rigid body class and various link classes that 
// constitutes a multibody system.
//
// [IMPORTANT!] NAMING CONVENTION of the VARIABLES
//
// frame = transformation matrix wrt a WORLD frame 
// frameVel = body velocity of frame ( inv(T)*T_dot if T is frame )
// frameAcc = d (frameVel) / dt
//
// localFrame = transformation matrix of a link wrt its PARENT frame
// localFrameVel = body velocity of localFrame
// localFrameAcc = d (localFrameVel ) / dt
//
// coordinate = generalized coordinates of a link
// genVel = velocity of coordinate
// genAcc = d (genVel ) / dt
// *NOTE* 
//  g1dLink, g2dLink: its coordinates are joint angles. Its genVel is the velocity of joint angles.
//	gBallLink: its coordinates are represented with a rotation matrix. Its genVel is the body velocity of the rotation matrix.
//  gRigidBody, gFreeLink: its coordinates are represented with a transformation matrix.  Its genVel is the body velocity of the xform matrix
//------------------------------------------------------------

#ifndef _MBS_ELEMENT_H_
#define _MBS_ELEMENT_H_

#include <vector>

#include "Base/gMath.h"
#include "gMbsUtil.h"

#ifdef MBS_USE_COLLISION_MANIFOLD
//For collision detection
#include "btBulletCollisionCommon.h"
#endif


class		gRigidBody;
class		gLink;
class		g1dLink;
class		g2dLink;
class		gBallLink;
class		gFreeLink;
class		gWeldedLink;

#define	DOF_WELDED_LINK		0
#define DOF_1D_LINK			1
#define DOF_2D_LINK			2
#define	DOF_BALL_LINK		3
#define DOF_RIGID_BODY		6
#define	DOF_FREE_LINK		6
#define SIZE_QUATERNION		4
#define SIZE_PARAM_SE3		7 //4 (quaternion) + 3(translation)

#define	TYPE_RIGID_BODY		0
#define	TYPE_1D_LINK		2
#define	TYPE_2D_LINK		4
#define	TYPE_BALL_LINK		5
#define	TYPE_WELDED_LINK	6
#define	TYPE_FREE_LINK		7

typedef std::vector<gLink*>				gLinkList;

//-----------------------------------------------------------------
/// base class of all objects
//-----------------------------------------------------------------
class gObject{
public:
						gObject(){_ID = 0xFFFFFFFF; };
	virtual			   ~gObject(){}

	/// return id
	inline	int			id(void) const				{ return _ID; }

	/// return name
	inline	const char*	name(void) const			{ return _name; }

	/// set id
	inline	void		setId(int id)				{ _ID = id; }

	/// set name
	inline	void		setName(char* name)			{ strcpy(_name,name); }
protected:
	int					_ID;
	char				_name[MW_SIZE_NAME];
};

// -----------------------------------------------------------------
/// class gRigidBody : class for the kinematics and dynamics of the rigid body 
/** class gRigidBody has member variables for kinematic and dynamic operations and it is important to know the names of the variables. \n
* \n
* frame (gXMat T=(R,p)) = transformation matrix wrt a WORLD frame \n
* frameVel (gTwist V) = body velocity of frame ( inv(T)*dot(T)  )  \n
* frameAcc (gTwist dV) = dV / dt \n
* \n
* We use its frame (T) for the generalized coordinate of gRigidbody. \n
* But in case writing/reading the generalized coordinates to/from a file, a quaternion+gVec3 (total 7 numbers) is used instead of 16 numbers of T.\n
* \n
* frameForce (gWrench F) = force applied to the frame. This is the control force for gRigidBody. So if you want to model an actuator for the rigid body, use F to specify the input force. \n
* extForce (gWrench Fext) = external forces. Sum of total forces from external sources(e.g., gravity, springs) 
* \n
* genInertia (gInertia J) = generalized inertia of the rigid body. 
* \n
* massless (bool bMassless): true if the rigid body is massless. You must set this true if a rigid body is massless. 
* Note that, naturally, a terminal link ( a link without child links ) cannot be massless because it ruins the dynamic simulation.
* \n
* accDriven (bool bAccDriven): This parameter specifies whether the rigid body is driven by acceleration or torque.
* If true, the rigid body moves as specified by the acceleration (dV). Otherwise, the rigid body moves by forces.
* You must use Featherstone's hybrid dynamics algorithm for simulating a system that contains accDriven links.
*/
class gRigidBody:public gObject
{
public:
	gRigidBody();
	~gRigidBody();

	/// return type
	virtual	int			type(void) const			{ return TYPE_RIGID_BODY; }

	/// return degrees of freedom
	virtual int			dof(void)  const			{ return DOF_RIGID_BODY; }


	///return configuration wrt world frame (T)
	inline	gXMat&		frame(void)  				{ return _T; }
	inline	gXMat		frame(void) const			{ return _T; }

	///return body velocity (V)
	inline	gTwist&		frameVel(void) 				{ return _V; }
	inline	gTwist		frameVel(void) const		{ return _V; }

	///return body acceleration (dV)
	inline	gTwist&		frameAcc(void)				{ return _dV; }
	inline	gTwist		frameAcc(void) const		{ return _dV; }

	///return frame force (F)
	/**
	* A rigid body has two sources of forces: One is frameForce and the other is extForce. \n
	* extForce is the total sum of external forces applied to the rigid body. For example, if a rigid body experiences
	* gravitational force and a spring force, extForce should be set to gravitational force + spring force (all expressed wrt body frame) \n
	* frameForce is the control input if the rigid body has an actuator. If a rigid body doesn't have an actuator, frameForce is simply zero.
	*/
	inline	gWrench&	frameForce(void)			{ return _F; }

	///return generalized inertia (J)
	inline	gInertia&	inertia(void)				{ return _J; }
	inline	const gInertia&	inertia(void) const		{ return _J; }

	///return external force (Fext). See frameForce(void).
	inline	gWrench&	extForce(void) 				{ return _Fext; }

			/// return velocity wrt world frame ( V.xform(T) )
			gTwist		frameVelWorld() const		{ return _V.xform(_T); }

			/// return spatial acc wrt world frame ( dV.xform(T) )
			gTwist		frameAccWorld() const		{ return _dV.xform(_T); }

			/// return angular velocity wrt world frame. ( R*(V.rot) )
			gVec3		angularVelWorld() const		{ return _T.multVec3(_V.rot()); }
			
			/// return linear velocity wrt world frame. ( R*(V.trn) )
			gVec3		linearVelWorld() const		{ return _T.multVec3(_V.trn()); }

			/// return angular acceleration wrt world frame. ( R*(dV.rot) )
			gVec3		angularAccWorld() const		{ return _T.multVec3(_dV.rot()); }

			/// return linear acceleration wrt world frame. ( R*(dV.trn) )
			gVec3		linearAccWorld() const		{ return _T.multVec3(_dV.trn()); }

	/// return position (p where T=(R,p)) of the body frame
	inline	gVec3		pos(void) const				{ return _T.trn(); }

	/// return CoM position wrt world frame
	inline	gVec3		posCenterOfMassWorld(void) const { return _T.multVec4( _J.com() ); }

	/// return momentum wrt body frame ( J*V )
	inline	gWrench		calcMomentumBody(void) const { return _J * _V; }

	/// return momentum wrt world frame ( (J*V).xform(T) )
	inline	gWrench		calcMomentumWorld(void) const { return (_J*_V).xform(_T); }

	/// return the world position of a point whose coordinate is posBody in body frame. ( R*posBody + p )
	inline  gVec3 getWorldPosOf(const gVec3& posBody) const { return frame().multVec4(posBody); }

	/// return body position of a point whose coordinate is posWorld in the world frame. ( R'*(posWorld-p) )
	inline  gVec3 getBodyPosOf(const gVec3& posWorld) const { return frame().invMultVec4(posWorld); }

	/// return velocity of a point wrt body frame whose coordinate is posBody in body frame. ( V*posBody )
	inline	gVec3 getBodyVelOfBodyPoint(const gVec3& posBody) const { return frameVel().multVec4(posBody); }

	/// return the velocity of a point wrt world frame whose coordinate is posBody in body frame
	inline  gVec3 getWorldVelOfBodyPoint(const gVec3& posBody) const { return frame().multVec3( getBodyVelOfBodyPoint(posBody) ); }

	/// return the velocity of a point wrt body frame whose coordinate is posWorld in world frame
	inline	gVec3 getBodyVelOfWorldPoint(const gVec3& posWorld) const { return frameVel().multVec4( getBodyPosOf(posWorld) ); }

	/// return the velocity of a point wrt world frame whose coordinate is posWorld in world frame
	inline	gVec3 getWorldVelOfWorldPoint(const gVec3& posWorld) const { return frameVelWorld().multVec4(posWorld); }
	
	/// set generalized inertia
	inline	void		setInertia(const gInertia& J)	{ _J = J; }

	/// return true if this is acceleration-driven. See setAccDriven(bool b)	
	/** If acceleration-driven is true, this rigid body will move according to the specified acceleration, regardless of the input force.
	* Otherwise, the rigid body will move according to the input forces.
	*/
	inline	bool		isAccDriven(void) const		{ return _bAccDriven; }

	/// return true of this is declared as massless
	inline	bool		isMassless(void) const		{ return _bMassless; }

	/// set whether or not it is acceleration-driven
	inline	void		setAccDriven(bool b)		{ _bAccDriven = b; }
	
	/// set whether or not it is massless
	inline	void		setMassless(bool b)			{ _bMassless = b; }

	/// set configuration of the frame
	inline	void		setFrame(const gXMat& T)		{ _T = T; }

	/// set body velocity of the frame
	inline	void		setFrameVel(const gTwist& V)	{ _V = V; }

	/// set body acceleration of the frame
	inline	void		setFrameAcc(const gTwist& dV)	{ _dV = dV; }	

	/// set force applied to the frame
	inline	void		setFrameForce(const gWrench& F)	{ _F = F; }

	/// set external force applied to the frame. 
	inline	void		setExtForce(const gWrench& Fext){ _Fext = Fext; }

	/// clear external force
	inline	void		clearExtForce(void)				{ _Fext.setZero(); }

	/// clear frame force
	inline	void		clearFrameForce(void)			{ _F.setZero(); }

	/// add F to the external force
	inline	void		addExtForce(const gWrench& F)	{ _Fext += F; }

	/// subtract F from the external force
	inline	void		subtractExtForce(const gWrench& F) { _Fext -= F; }

			/// add an external force
			/**
			\param[in] positionWorld  position (wrt world frame) to which a force is applied
			\param[in] forceWorld     force wrt world frame 
			*/
			void		addExtForceInWorld(const gVec3& positionWorld, const gVec3& forceWorld) { gVec3 forceLocal = _T.invMultVec3(forceWorld); _Fext += gWrench( _T.invMultVec4(positionWorld) % forceLocal, forceLocal ); }
			
			/// add an external force
			/**
			\param[in] positionLocal  position (wrt body frame) to which a force is applied
			\param[in] forceWorld     force wrt world frame 
			*/
			void		addExtForceInWorldAtRelPos(const gVec3& positionLocal, const gVec3& forceWorld) { gVec3 forceLocal = _T.invMultVec3(forceWorld); _Fext += gWrench( positionLocal % forceLocal, forceLocal ); }
		
			/** add gravity to extForce. \n
			 * IMPORTANT: this function is provided to apply gravity in case gGravityForce element is not attached to the rigid body.\n
			 * Note that gMultibodySystem automatically generate gGravityForce elements for its rigid body, so 
			 * normally you don't need to call this function for multibody dynamics. \n
			 * However, you will need to call this function if you simulate a single rigid body.
			*/
			void		addGravity(void);	

	/// function for inverse dynamics. It computes frameForce given frameAcc and extForce
	virtual	void		invDynFrameForce(void);			 
	

	/// function for forward dynamics. It computes frameAcc given frameForce, extForce
	virtual	void		solAcc(void);					
		
			/// calculate potential energy. 
			/** \param[in] height0 height of zero potential energy
			*/
			gReal		calcPotentialEnergy(gReal height0);
			
			/// calculate kinetic energy
			gReal		calcKineticEnergy(void) { return _bMassless? 0 :  gREAL(0.5)*( _J * _V , _V ); }
	
	//{ time integration
	
			/// do time integration usind the explicit Euler method
			virtual	void		timeIntegrateExplicitEuler(gReal time_step);

			/// do time integration usig semi-implicit Euler method: Update velocity using explicit Euler, then use the new velocity to update position
			virtual void		timeIntegrateSemiImplicitEuler(gReal time_step); 
	//}



	//{ functions for contact forces
#ifdef MBS_USE_COLLISION_MANIFOLD
	

	void setCollisionObject(btCollisionObject* co) { _collisionObject = co; }
	btCollisionObject* collisionObject(void) { return _collisionObject; }
	void clearCollisionManifolds(void) { _collisionManifolds.clear(); }
	void addCollisionManifold(btPersistentManifold* pm) { _collisionManifolds.push_back(pm); }
	void addContactForces(void);	
	std::size_t  numCollisionManifolds(void) { return _collisionManifolds.size(); }
	btPersistentManifold* collisionManifold(int i) { return _collisionManifolds[i]; }
#endif
	//{ Below 4 functions are used for writing(reading) configuration to(from) files	

	/// returns the size of coordinate array. It returns 7 ( 4 for quaternion + 3 for position )
	virtual int			sizeSafeCoordArray()			{ return SIZE_PARAM_SE3; } 

	///set coordinates from input array (ptr)
	/**
	*set _T by setting its orientation from ptr+0...3 (Quaternion) and position from ptr+4..6
	*/
	virtual	void		setFromSafeCoordArray(const gReal* ptr); 
	
	/// set frame to identity matrix
	virtual	void		setZeroCoord()				{ _T.setIdentity(); }

	///copy coordinates to ptr[0...6]. ptr[0..3] will hold quaternion for the rotation. ptr[4..6] will hold position.
	virtual	void		getSafeCoordArray(gReal* ptr) const;

	//}


#ifdef gUSE_MULTI_THREADING_IN_E
	HANDLE	hSemaphoreFext;	// semaphore to Fext
#endif

protected:

	//{ interface
	gTwist				_V, _dV;					// generalized body Velocity and its derivative. V = inv(T)*dot(T), dV = dot(V)
	gWrench				_F;							// Force applied to the frame (this is control force for gRigidBody)
	gWrench				_Fext;						// External Forces: Force from external sources(e.g., gravity, springs) 
	gXMat				_T;							// transformation matrix of the rigid body
	//}
	gInertia			_J;							// generalized gInertia
	
	bool				_bMassless;					// true if massless

	/*
	_bAccDriven
	This parameter specifies whether the rigid body is driven by acceleration or torque.
	If true, the rigid body moves as specified by the acceleration (_dV).
	Otherwise, the rigid body moves by forces.
	Featherstone's hybrid dynamics algorithm is used.
	For more information, refer to the algorithm.
	*/
	bool				_bAccDriven;

#ifdef MBS_USE_COLLISION_MANIFOLD
	//collision manifolds
	//each collision manifold stores collision information between *this and a colliding rigid body
	std::vector<btPersistentManifold*> _collisionManifolds;
	
	btCollisionObject* _collisionObject;
#endif

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	gTwist  _Tp;		//derivative of _T	 
	gTwist  _Vp, _dVp;	//derivative of _V, _dV
	gWrench _Fp, _Ep;	//derivative of _F, _Fext
#endif
};

/// class gLink : abstract class for a link (link : jointed rigid body)
/** Some important member variables of class gLink are explained here. \n
* \n
* localFrame (gXMat G) = transformation matrix of a link wrt its PARENT frame. Thus, frame(T) =  parent->frame(T)*localFrame(G) \n
* localFrameVel (gTwist U) = body velocity of localFrame (i.e., relative velocity of the link wrt its parent, expressed in the body frame).  U = inv(G)*dot(G). \n
* localFrameAcc = dU / dt \n
* localFrameDefault (gXMat H) = configuration of this frame wrt parent frame when generalized coodrdinates (e.g, joint angles) are zero
* \n
* parent (gLink* parent) =  parent link \n
* children(std::vector<gLink*> children) = child links \n
* numChildren (int numChildren) = number of children \n
* \n
* coordinate = generalized coordinates of a link, whose representation depends on the type of the link. \n
* genVel = velocity of coordinate \n
* genAcc = d (genVel ) / dt \n
* Note that, for convenience and efficiency, the generalized velocities/accelerations of all the links are stored as a vector in gMultibodySystem
* and each link only has pointers to its generalized velocities/accelerations.\n
* genCoordIndex : the starting index in generalized velocity/acceleration vector of gMultibodySystem that corresponds to this link's genVel and genAcc.
* \n
* frameForce: for gRigidBody, frameForce represents the control input force to the rigid body.
* For links, frameForce represents the force applied by the joint to the link.
* There are two sources of joint forces: one is the actuator force (i.e., generalized force) and the other is constraint force.
* For joint twist direction, it is the generalized force, i.e., genForce[i] = dot ( screw[i], frameForce ). 
* For other directions (link cannot move in those directions 
* because of the joint constraints), frameForce is the constraint force.
*/
class gLink:public gRigidBody{

public:

	gLink(){
		_numChildren = 0;
		_parent	= NULL;
		_genCoordIndex = -1;
	}

	///return type
	virtual	int			type(void) const =0;
	
	///return degrees of freedom
	virtual	int			dof(void) const = 0;
			
	/**return this link's index in the generalized coordinates vector.
	*Note that generalized coordinates vector is not usually used for dynamics simulation.
	*Instead, it is mainly used for storing motion data to a file.
	*/
	inline	int			genCoordIndex(void)	const	{ return _genCoordIndex; }

	///set the index of this link in the generalized coordinates vector. See genCoordIndex(void).
	inline	void		setGenCoordIndex(int idx)	{ _genCoordIndex = idx; }

	///return parent link
	inline	gLink*			parent(void)			{ return _parent; }

	inline	const gLink*	parent(void) const		{ return _parent; }

	///return i'th child i=[0..numChildren-1]
	inline	const gLink* child(int i) const			{ return _children[i]; }
	inline	gLink* child(int i)						{ return _children[i]; }

	///return number of children
	inline	int			numChildren(void) const		{ return _numChildren; }

	///add a child link
	inline	void		addChild(gLink* link)		{ _children.push_back(link); _numChildren++; } 

			///remove a child
			void		removeChild(gLink* link);	// remove if links is in _children
	
			///return local frame at default configuration (i.e., joint angles = 0)
	inline	gXMat&			localFrameDefault(void)			{ return _H; } 

	inline	const gXMat&	localFrameDefault(void) const	{ return _H; } 

			///return G = configuration wrt parent frame if parent exists. Otherwise, it is the configuration wrt world frame.
	inline	gXMat&			localFrame(void)			{ return _G; }	
	inline	const gXMat&	localFrame(void) const		{ return _G; }

			/// return body velocity of the local frame. I.e, inv(G)*dot(G)
	inline	gTwist&		localFrameVel(void)			{ return _U; } 
	inline	const gTwist& localFrameVel(void) const { return _U; }

			/// return body acceleration of the local frame
	inline	gTwist&		localFrameAcc(void)			{ return _Sddq; }
	inline	const gTwist& localFrameAcc(void) const { return _Sddq; }

protected:
			/// return Coriolis term
	inline	gTwist&		coriolisAcc(void)			{ return _C; }	

public:
	///returns transformation matrix from the base link of the system to the body frame
	inline	gXMat&		baseFrame(void)				{ return _T0; }
	inline	const gXMat& baseFrame(void) const		{ return _T0; }

	///set default local frame (i.e., local frame at joint coordinates = 0 )
	inline	void		setLocalFrameDefault(const gXMat& H)    { _H = H; }	

	///set parent link
	inline	void		setParent(gLink* link)					{ _parent = link; }			

	/// set local frame
	inline	void		setLocalFrame(const gXMat& G)			{ _G = G; }

	/// return  pointer to center of mass in body frame
	inline	const gReal* com() const { return _J.com(); }
	

	//{ get canonical velocities in world frame

	///return angular velocity of the frame wrt world frame = dR/dt * R^{-1} where R is the frame's rotation matrix
	inline gVec3	getAngVelCanonical() const { return frame().multVec3( frameVel().rot() ); }
	
	///return linear velocity of the frame wrt world frame = dp/dt where p is the position of the frame
	inline gVec3	getLinVelCanonical() const { return frame().multVec3( frameVel().trn() ); }

	///}

protected:
			/// update frame and localFrame
			void		updateFrame(void);	

			/// update localFrameVel, frameVel and coriolisAcc
			void		updateFrameVel(void);				
			
			/// update localFrameAcc and frameAcc
			void		updateFrameAcc(void);				

			// update composit inertia (in world frame) (= _Jcw). This is only used for composite inertia method.
			void		updateCompositeInertiaWorld(void);	

			// return composit inertia in world frame. This is only used for composite inertia method.
	inline	gInertia&	compositeInertiaWorld(void)	{ return _Jcw; }
	
	//{ functions for updating kinematic variables
public:
			/// update every position level kinematic variables. i.e., frameWorld(_T), frameLocal(_G). If recursive is true, it recursively calls updateKinematicsUptoPos for its child links.
			void		updateKinematicsUptoPos(bool recursive = false);	

			/// update every position and velocity level variables. i.e., frameWorld(_T), frameVel(_V), localFrame(_G), localFrameVel(_U). If recursive is true, it recursively calls updateKinematicsUptoVel for its child links.
			void		updateKinematicsUptoVel(bool recursive = false);		
			
			/// update all kinematic variables (position,velocity,acceleration). If recursive is true, it recursively calls updateKinematicsUptoAcc for its child links.
			void		updateKinematicsUptoAcc(bool recursive = false);		
	//}
	// virtual functions
			
			/// return size of the coordinates when they are written to/read from a file
			/**Coordinates are the minimal number of variables that describe the local frame of the link. \n
			*The size of coordinates are same as its DOF in many cases if the gereralized coordinates can describe
			*the local frame without singularity. If not, such as a ball joint and free joint, the size of coordinates is greater
			*than the DOF. We use quaternion for representing orientation. Therefore, the size of coordinates
			*of a ball link is 4 whereas its DOF is 3.
			*The size of coordinates are  0 for gWeldedLink, 1 for g1dLink, 2 for g2dLink, 4 for gBallLink, 7 for gFreeLink
			*/
	virtual	int			sizeSafeCoordArray() const =0;		

	/// Set coordinates from the array ptr. 
	/** g1dLink: set _q (joint angle) =ptr[0]\n
	* g2dLink: set _q[0]=ptr[0], _q[1]=ptr[1] (_q[0,1]: joint angles)\n
	* gBallLink: set _R from the quaternion made by ptr[0..3] \n
	* gFreeLink: set _T=(R,p) from ptr[0..6] (i.e., R from ptr[0..3], p from ptr[4..6])\n
	* gWeldLink: do nothing.\n
	*/
	virtual	void		setFromSafeCoordArray(const gReal* ptr)=0;  

	/// Set coordinates from the local coordinate array ptr. 
	/** g1dLink: set _q (joint angle) =ptr[0]\n
	* g2dLink: set _q[0]=ptr[0], _q[1]=ptr[1] (_q[0,1]: joint angles)\n
	* gBallLink: set _R from the exp(ptr[0..2]) \n
	* gFreeLink: set _T=(R,p) from ptr[0..5] (i.e., R from ptr[0..2], p from ptr[3..5])\n
	* gWeldLink: do nothing.\n
	*/
	virtual	void		setFromCompactCoordArray(const gReal* ptr)=0;  

	/// set coordinates to zero
	virtual	void		setZeroCoord()=0;

	/// Get coordinates from the array ptr. 
	/** g1dLink: set ptr[0] to _q \n
	* g2dLink: set ptr[0]=_q[0] and ptr[1]=_q[1]\n
	* gBallLink: set ptr[0..4] to the quaternion values corresponding to _R  \n
	* gFreeLink: set ptr[0..6] from _T=(R,p) (i.e., ptr[0..3] from R, ptr[4..6] from p)\n
	* gWeldLink: do nothing.\n
	*/
	virtual	void		getSafeCoordArray(gReal* ptr) const =0;


	
	/// set random numbers to the coordinates
	virtual void		randomizeCoord(gReal maxRange)=0;

	/// set random numbers between [low,up] to the coordinates
	virtual void		randomizeCoord(gReal* low, gReal* up)=0;

	///copy coordinates of link to this
	virtual	void		copyCoord(gLink* link)=0;

	///store coordinates
	virtual void		storeCoord(void)=0;			

	///restore coordinates
	virtual void		restoreCoord(void)=0;

	///change coordinates by moving in the "coordinate e" by epsilon 
	/** g1dLink: set _q += epsilon \n
	* g2dLink: set _q[e] += epsilon \n
	* gBallLink: if e={0,1,2}, rotate about {X,Y,Z}-axis by epsilon, respectively. \n
	* gFreeLink: if e={0,1,2}, rotate about {X,Y,Z}-axis by epsilon, respectively. 
	* if e={3,4,5}, translate {X,Y,Z}-axis by epsilon, respectively. 
	* gWeldedLink: do nothing.
	*/
	virtual void		moveEpsilon(int e,gReal epsilon)=0;	

	/// set ptr to the reduced coordinates of the link.
	/** "reduced coordinates" are same as generalized coordinates for g1dLink and g2dLink, whose dimension of generalized
	* coordinates is same as it DoFs.
	* For gBallLink and gFreeLink, while generalized coordinates use singularity-free coordinates for the rotation,
	* reduced coordinates use exponential coordinates for the rotation.	
	* Therefore, the size of the reduced coordinates is same as its DoFs. 
	* However, since exponential coordinates are local parameterization, it may have singularities. Use only in the case
	* that the link is guaranteed not to reach singularity.
	*/
	virtual void		getCompactCoordArray(gReal* ptr) const =0;
	

protected:
	/// update local frame
	virtual	void		updateLocalFrame(void)=0; 

	/// update local velocity
	virtual	void		updateLocalVel(void)=0;

	/// update local acceleration
	virtual void		updateLocalAcc(void)=0;

	// solve for Coriolis term
	virtual void		solCoriolisAcc(void)=0;

	//{ time integration
	virtual	void		timeIntegrateExplicitEuler(gReal time_step)=0;	
	
	///time integrate POSITION ONLY with explicit Euler method. This function is called to update pos when velocity is alread updated.
	virtual	void		timeIntegrateExplicitEulerPositionOnly(gReal time_step)=0;	

	virtual void		timeIntegrateSemiImplicitEuler(gReal time_step)=0;
	//}
public:
	
	/// return the i'th generalized velocity. (i=[0...dof-1])
	/** Note that, for convenience and efficiency, the generalized velocities of all the links are stored as a vector in gMultibodySystem
	* and each link merely has pointers to its generalized velocity.
	*/	
	virtual	gReal		genVel(int i) const =0;

	/// return the i'th generalized acceleration. (i=[0...dof-1])
	/** Note that, for convenience and efficiency, the generalized accelerations of all the links are stored as a vector in gMultibodySystem
	* and each link merely has pointers to its generalized accelerations.
	*/
	virtual gReal		genAcc(int i) const =0;

	/// return the i'th generalized forces. (i=[0...dof-1])
	/** Note that, for convenience and efficiency, the generalized forces of all the links are stored as a vector in gMultibodySystem
	* and each link merely has pointers to its generalized forces.
	*/
	virtual gReal		genForce(int i) const =0;

	/// set the i-th generalized velocity. (i=[0...dof-1])
	virtual	void		setGenVel(int i,gReal)=0;

	/// set the i-th generalized acceleration. (i=[0...dof-1])
	virtual void		setGenAcc(int i,gReal)=0;

	/// set the i-th generalized force. (i=[0...dof-1])
	virtual void		setGenForce(int i,gReal)=0;

	/// return the i-th joint twist (or screw parameter). (i=[0...dof-1])
	/** g1dLink, g2dLink: return screw parameter(s) \n
	* For gBallLink and gFreeLink, the joint twist is the basis vector of each direction. Hence, return i-th basis vector 
	*/
	virtual	gTwist		screw(int i) const =0;

	/// return the i-th joint twist (or screw parameter) as seen from the body frame
	/** g1dLink, gBallLink, gFreeLink: screwBody(i) is same as screw(i)
	* g2dLink: see g2dLink::screwBody(int)
	*/
	virtual gTwist		screwBody(int i) const =0;
	
	/// return pointer to the i-th generalized velocity. (i=[0...dof-1])
	virtual	const gReal* genVelPtr(int i) const =0;
	//virtual	gReal*		genVelPtr(int i) = 0;

	/// set pointer of the i-th generalized velocity. (i=[0...dof-1])
	virtual	void		setGenVelPtr(int i, gReal* ptr)=0;

	/// set pointer of the i-th generalized acceleration. (i=[0...dof-1])
	virtual	void		setGenAccPtr(int i, gReal* ptr)=0;

	/// set pointer of the i-th generalized force. (i=[0...dof-1])
	virtual	void		setGenForcePtr(int i, gReal* ptr)=0;

	/// return pointer to the i-th generalized force. (i=[0...dof-1])
	virtual const gReal* genForcePtr(int i) const =0;


	void jointLimitLo(arma::vec& lo) const { lo = _jointLimitLo; }
	void jointLimitHi(arma::vec& hi) const { hi = _jointLimitHi; }
	void setJointLimitLo(const arma::vec& lo) { _jointLimitLo = lo; }
	void setJointLimitHi(const arma::vec& hi) { _jointLimitHi = hi; }
	gReal jointLimitLo(int idx) const { return _jointLimitLo(idx); }
	gReal jointLimitHi(int idx) const { return _jointLimitHi(idx); }
	void	setJointLimitLo(int idx, gReal lo) { _jointLimitLo(idx) = lo; }
	void	setJointLimitHi(int idx, gReal hi) { _jointLimitHi(idx) = hi; }

protected:

	/// clear generalized forces
	virtual void		clearGenForce()=0;

	/// compute genenralized forces by projecting frameForce to screw parameters
	virtual void		frameForceToGenForce(void)=0;		

	/// inverse dynamics: compute frameForce given extForce and acceleration
	virtual	void		invDynFrameForce(void);
	
public:
	// for computing mass matrix using composite inertia method. This is only used for composite inertia method.
	virtual gTwist		screwWorld(int) const =0; 
protected:

	// for computing mass matrix using composite inertia method. This is only used for composite inertia method.
	virtual	void		updateScrewWorld(void)=0;
	//}
public:

	/// translate body frame by p wrt the reference (parent) frame, and modify its generalized inertia accordingly.
	void translateBodyFrame(const gVec3& p)
	{
		//update _H,_J,child link's _H
		_H.setTrn(_H.trn()+p);
		_J.translateFrame(p);

		for(int i=0;i<_children.size();++i)
		{
			gXMat H = _children[i]->localFrameDefault();
			H.setTrn(H.trn()-p);
			_children[i]->setLocalFrameDefault(H);
		}
	}


protected:
	int					_genCoordIndex;				// starting index of this in the generalized coordinates

	gXMat				_H;							// localFrameDefault: configuration of this frame wrt parent frame when q = 0
    gLink*				_parent;					// parent link
	gLinkList			_children;					// child links
	int					_numChildren;				// size of _children
	
	gXMat				_G;							// localFrame: configuration of this frame wrt parent frame. For base link, parent frame is world frame.
	gTwist				_C;							// coriolisAcc: Coriolli term _c = ad(_V,_U)
	gTwist				_U;							// localFrameVel: body velocity of localFrame(_G). i.e., _U=inv(G)*dG/dt
	gTwist				_Sddq;						// localFrameAcc: for revolute/ball joints, _Sddq = d_U/dt. for universal joint, _Sddq is a bit different.

	gXMat				_T0;						// configuration of this frame wrt the base frame of the system. For base link, _T0 is always identity matrix.

	gInertia			_Jcw;						// composite inertia in world frame

	arma::vec			_jointLimitLo, _jointLimitHi; //2012-02-14: joint limit added, 2017-05-06: (swl) moved. arma::vec has local storage for the speed issue.



#ifdef MW_USE_ARTICULATED_BODY_METHOD
protected:
	inline	void		setArtInertia(const gAInertia& A)		{ _A = A; }
	inline	gAInertia&	artInertia(void)			{ return _A; }
	inline	gWrench&	biasForce(void)				{ return _b; }
	virtual void		updateArtInertiaAndBiasForce(bool updateOnlyBiasForce=false);
	virtual void		solAcc(void)=0;				// compute frameAcc and genAcc
	//{ forward dynamics
	virtual gAInertia	lockedArtInertia()=0;		// a function for articulated body method
	inline	gWrench&	x(void) { return _x; }		
	virtual gWrench		sol_xx(void)=0;				// a function for articulated body method. 
	//}
protected:
	gAInertia			_A;							// artInertia: articulated-body inertia
	gWrench				_b;							// biasForce: bias force in Articulated-Body Method
	gWrench				_x;							// _x = A*C+b. It is just a temporary variable used in Articulated-Body Method
#endif //MW_USE_ARTICULATED_BODY_METHOD

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual gReal	coordP(int idx) const =0;
	virtual gReal	genVelP(int idx) const =0;
	virtual gReal	genAccP(int idx) const =0;
	virtual gReal	genForceP(int idx) const =0;
	virtual void	setCoordP(int idx, gReal x)=0;
	virtual void	setGenVelP(int idx, gReal x)=0;
	virtual void	setGenAccP(int idx, gReal x)=0;

	virtual void	updateKVariablesP(bool recursive=false)=0;
	virtual void	computeGenForceP()=0;

	void computeFrameForceP()
	{
		_Fp = _J*_dVp - gWrench::dad(_Vp,_J*_V)-gWrench::dad(_V,_J*_Vp)-_Ep;
		for(gLinkList::iterator it = _children.begin(); it!=_children.end(); it++)
		{
			_Fp += gWrench::dAdInv( (*it)->localFrame(), (*it)->_Fp - gWrench::dad( (*it)->_Gp, (*it)->frameForce() ) );
		}
	}

	void setZeroKVariablesP()
	{
		_Gp.setZero();
		_Tp.setZero();
		_Vp.setZero();
		_dVp.setZero();
	}

	gTwist _Gp;	//derivative of _G 

#endif

	friend class gMultibodySystem;
	friend class gMultibodySystemTimeIntegrator;

	public:
		void* userPointer;	// userPointer is added to maintain a btShape information.

};

/// class gWeldedLink : a link connected to its parent with a Weled joint
/** gWeldedLink is rigidly attached to its parent. So it's DoF is zero.
*/
class gWeldedLink:public gLink
{
public:
	gWeldedLink(){}

	int		type(void) const			{ return TYPE_WELDED_LINK; }
	int		dof(void)  const			{ return DOF_WELDED_LINK; }

	int		sizeSafeCoordArray() const	{ return DOF_WELDED_LINK; }
	
	///do nothing
	void	getSafeCoordArray(gReal* ptr) const {}
	///do nothing
	void	getCompactCoordArray(gReal* ptr) const {}
	///do nothing
	void	setFromSafeCoordArray(const gReal* ptr)	{}
	///do nothing
	void	setFromCompactCoordArray(const gReal* ptr) {}  
	///do nothing
	void	setZeroCoord() {}
	///do nothing
	void	copyCoord(gLink* src) {}
	///do nothing
	void	randomizeCoord(gReal maxRange)			{}
	///do nothing
	void	randomizeCoord(gReal* low, gReal* up)	{}
	///do nothing
	void	clearGenForce(void)						{}

	///do nothing
	void	setGenVelPtr(int, gReal* ref)	{}
	///do nothing
	void	setGenAccPtr(int, gReal* ref)	{}
	///do nothing
	void	setGenForcePtr(int, gReal* ref)	{}
	///return NULL
	const gReal*	genForcePtr(int) const { return NULL; }
	///do nothing
	gReal	genVel(int) const { assert(NULL); return gINFINITY;} //shuldn't be called 
	///do nothing
	gReal	genAcc(int) const { assert(NULL); return gINFINITY;} //shuldn't be called 
	///do nothing
	gReal	genForce(int) const { assert(NULL); return gINFINITY;} //shuldn't be called 
	///do nothing
	const gReal*	genVelPtr(int) const { assert(NULL); return NULL; }//shuldn't be called 
	///do nothing
	void	setGenVel(int,gReal){assert(NULL); }
	///do nothing
	void	setGenAcc(int,gReal){assert(NULL); }
	///do nothing
	void	setGenForce(int,gReal){assert(NULL); }
	///do nothing
	gTwist	screw(int) const {assert(NULL); return gTwistRotX*gINFINITY; }
	gTwist	screwBody(int i)  const {assert(NULL); return gTwistRotX*gINFINITY; }
	///do nothing
	void	updateLocalFrame(void)		{ /*_G = _H;*/}
	
	void	updateLocalVel(void)		{ _U.setZero(); }
	void	updateLocalAcc(void)		{ _Sddq.setZero(); }
	void	solCoriolisAcc(void)		{ _C.setZero(); }
	
	///do nothing
	void	moveEpsilon(int,gReal eps)	{}
	///do nothing
	void	storeCoord(void)			{}
	///do nothing
	void	restoreCoord(void)			{}
	///do nothing
	void	frameForceToGenForce (void)	{}

	//{ time integration
	///do nothing
	void	timeIntegrateExplicitEuler(gReal time_step)				{}	
	///do nothing
	void	timeIntegrateExplicitEulerPositionOnly(gReal time_step)	{}
	///do nothing
	void	timeIntegrateSemiImplicitEuler(gReal time_step)			{}
	//}


	gTwist		screwWorld(int) const { return gTwistZero; };
	void		updateScrewWorld(void){};

#ifdef MW_USE_ARTICULATED_BODY_METHOD
	
protected:
	//{ forward dynamics
	gAInertia lockedArtInertia(){ return _A; }
	void	updateArtInertiaAndBiasForce(bool updateOnlyBiasForce=false) {	gLink::updateArtInertiaAndBiasForce(updateOnlyBiasForce); _x = _b; }
	void	solAcc(void);
	gWrench	sol_xx(void){ return gWrenchZero; }
	//}	


#endif //MW_USE_ARTICULATED_BODY_METHOD

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	gReal coordP(int idx) const { assert(NULL); return 0.0; }
	gReal genVelP(int idx) const { assert(NULL); return 0.0;}
	gReal genAccP(int idx) const { assert(NULL); return 0.0;}
	gReal genForceP(int idx) const { assert(NULL); return 0.0; }
	void setCoordP(int idx, gReal x)	{ assert(NULL);}
	void setGenVelP(int idx, gReal x)	{ assert(NULL);}
	void setGenAccP(int idx, gReal x)	{ assert(NULL);}

	void updateKVariablesP(bool recursive = false)	
	{ 
		_Gp.setZero(); 
		_Tp = ( _parent ? _parent->_Tp.xformInv(_G) : gTwistZero );
		_Vp = ( _parent ? _parent->_Vp.xformInv(_G) : gTwistZero );
		_dVp = ( _parent ? _parent->_dVp.xformInv(_G) : gTwistZero );

		if(recursive){
			for(gLinkList::iterator it = _children.begin(); it!=_children.end(); it++) (*it)->updateKVariablesP(true);
		}
	}

	void computeGenForceP(){}
	
#endif

};


/// class g1dLink : a link connected to its parent with a 1 Dof joint
/** It can be used as any 1-dof constant screw joints (e.g. prismatic joints and revolute joints), hence its DoF is 1. \n
* Some important member variables are as follows. \n
* \n
* coordinate (gReal q): joint angle \n
* genVel (gReal* dq): joint velocity. \n
* genAcc (gReal* ddq): joint acceleration. \n
* screw (gTwist S): screw parameter (or joint twist) of the link wrt body frame. It is constant. \n
* S determines the type of g1dLink. For example, S = (1,0,0,0,0,0) defines a revolute joint with rotation axis being X-axis. \n
* S = (0,0,0,0,1,0) defines a prismatic joint with sliding axis being Y-axis.
*/
class g1dLink:public gLink
{
public:
	g1dLink()
	{
		_q = 0;
		_dq = NULL;
		_ddq = NULL;
		_tau = NULL;
		_jointLimitLo.set_size(DOF_1D_LINK);
		_jointLimitHi.set_size(DOF_1D_LINK);
		_jointLimitLo(0) = -HUGE_VAL; //very low number
		_jointLimitHi(0) = HUGE_VAL; //very high number
	}

	///return generalized coordinate _q
	inline	gReal		coord(void) const				{ return _q; }
	///return generalized velocity 
	inline	gReal		genVel(void) const				{ return *_dq; }
	///return generalized acceleration
	inline	gReal		genAcc(void) const				{ return *_ddq; }
	///return generalized force
	inline	gReal		genForce(void) const			{ return *_tau; }
	///return pointer to generalized velocity 
	inline	const gReal* genVelPtr(void) const			{ return _dq; }
	
	gReal		genVel(int idx)	const			{ assert(idx==0); return *_dq; }
	gReal		genAcc(int idx) const			{ assert(idx==0); return *_ddq; }
	gReal		genForce(int idx) const			{ assert(idx==0); return *_tau; }
	const gReal* genVelPtr(int idx) const		{ assert(idx==0); return _dq; }

	//2012-02-14: joint limit functions added
	gReal jointLimitLo(void) const { return _jointLimitLo(0); }
	gReal jointLimitHi(void) const { return _jointLimitHi(0); }
	void	setJointLimitLo(gReal lo) { _jointLimitLo(0) = lo; }
	void	setJointLimitHi(gReal hi) { _jointLimitHi(0) = hi; }


	///set generalized coordinate to q
	inline	void		setCoord(gReal q)				{ _q = q; }
	///set generalized velocity to dq_
	inline	void		setGenVel(gReal dq_ )			{ *_dq=dq_; }
	///set generalized acceleration to ddq_
	inline	void		setGenAcc(gReal ddq_	)		{ *_ddq=ddq_; }
	///set generalized force to t
	inline	void		setGenForce(gReal t )			{ *_tau=t; }
	///add generalized force by t
	inline	void		addGenForce(gReal t )			{ *_tau+=t; }

	void		setGenVel(int idx,gReal dq)		{ assert(idx==0); *_dq = dq; }
	void		setGenAcc(int idx,gReal ddq)	{ assert(idx==0); *_ddq = ddq; }
	void		setGenForce(int idx,gReal t)	{ assert(idx==0); *_tau = t; }

	///return screw parameter
	gTwist		screw(void) const				{ return _S; }
	///set screw parameter
	void		setScrew(const gTwist& S)		{ _S = S; }
	gTwist		screw(int idx) const			{ assert(idx==0); return _S; }
	gTwist		screwBody(int idx) const		{ return screw(idx);}

	void		copyCoord(gLink* src)			{ assert(dynamic_cast<g1dLink*>(src) != NULL); _q = ((g1dLink*)src)->_q; }
	int			sizeSafeCoordArray() const		{ return DOF_1D_LINK; }
	void		setFromSafeCoordArray(const gReal* ptr);
	void		setFromCompactCoordArray(const gReal* ptr);  
	void		setZeroCoord()					{ _q = 0; }
	void		getSafeCoordArray(gReal* ptr) const;
	void		getCompactCoordArray(gReal* ptr) const;
	const gReal* genForcePtr(int) const			{ return _tau; }

	//{ link vectorized parameter to particular variables	
	void		setGenVelPtr(int, gReal* ref)	{ _dq=ref; }
	void		setGenAccPtr(int, gReal* ref)	{ _ddq=ref; }
	void		setGenForcePtr(int, gReal* ref)	{ _tau=ref; }
	//}
	int			type(void) const				{ return TYPE_1D_LINK; }
	int			dof(void) const					{ return DOF_1D_LINK; }
	void		clearGenForce(void)				{ *_tau = 0; }
	void		randomizeCoord(gReal maxRange){ _q = maxRange*((gTwo*rand())/RAND_MAX - gOne); } // [-maxRange maxRange]
	void		randomizeCoord(gReal* low, gReal* up);
	//{ time integration
	void		timeIntegrateExplicitEuler(gReal time_step);	
	void		timeIntegrateExplicitEulerPositionOnly(gReal time_step);
	void		timeIntegrateSemiImplicitEuler(gReal time_step);
	//}
	//{ forward dynamics
	void		frameForceToGenForce (void)		{ *_tau = (_F,_S); } // given F, compute tau
	//}
	//{ values set from joint parameters	(kinematic variables)
	void		updateLocalFrame(void)			{ _G = _H * gXMat::exp( _S, _q ); }
	void		updateLocalVel(void)			{ _U = _S*genVel(); }
	void		solCoriolisAcc(void)			{ _C = gTwist::ad(_V,_U); }
	void		updateLocalAcc(void)			{ _Sddq = _S*genAcc(); }
	void		moveEpsilon(int e,gReal eps) { _q += eps; }
	void		storeCoord(void)
						{
							_q_old = _q;
						}
	void		restoreCoord(void)
						{
							_q = _q_old;
						}	

	gTwist		screwWorld(int idx) const	{ assert(idx==0); return _Sw; };
	void		updateScrewWorld(void)		{ _Sw = _S.xform(_T); };

protected:
	gReal			_q;		// generalized coordinate
	gTwist			_S;		// screw parameter in home position..constant
	gReal			_q_old;	// variable for temporarily storing _q
	gTwist			_Sw;	// screw parameter in world frame

	//{ interface
	gReal*			_dq;	// gen. coord. velocity
	gReal*			_ddq;	// gen. coodr. acceleration
	gReal*			_tau;	// torque 
	//}

#ifdef MW_USE_ARTICULATED_BODY_METHOD
protected:
	gAInertia	lockedArtInertia();
	void		updateArtInertiaAndBiasForce(bool updateOnlyBiasForce=false);
	void		solAcc(void);						// compute ddq, dV
	gWrench		sol_xx(void);
protected:
	gReal			_W;		// _W = 1/SAS
	gWrench			_AS;	// _AS = A*S

#endif //MW_USE_ARTICULATED_BODY_METHOD

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	gReal coordP(int idx) const		{assert(idx==0); return _qp;}
	gReal genVelP(int idx) const	{assert(idx==0); return _dqp;}
	gReal genAccP(int idx) const	{assert(idx==0); return _ddqp;}
	gReal genForceP(int idx) const	{assert(idx==0); return _taup;}

	void setCoordP(int idx, gReal x)	{ assert(idx==0); _qp = x;}
	void setGenVelP(int idx, gReal x)	{ assert(idx==0); _dqp = x; }
	void setGenAccP(int idx, gReal x)	{ assert(idx==0); _ddqp = x; }

	void updateKVariablesP(bool recursive = false)		
	{ 
		_Gp = _S*_qp; 
		_Tp = _Gp;
		_Vp = _S*_dqp; 
		_dVp = _S*_ddqp - gTwist::ad(_S, _Vp*(*_dq) + _V*_dqp );

		if(_parent){
			_Tp += _parent->_Tp.xformInv(_G);

			_Vp += _parent->_Vp.xformInv(_G);
			_Vp -= gTwist::ad(_Gp,_parent->frameVel().xformInv(_G));

			_dVp += _parent->_dVp.xformInv(_G);
			_dVp -= gTwist::ad( _Gp, _parent->frameAcc().xformInv(_G) );
		}

		if(recursive){
			for(gLinkList::iterator it = _children.begin(); it!=_children.end(); it++) (*it)->updateKVariablesP(true);
		}
	}

	void computeGenForceP()
	{
		_taup = (_S , _Fp);
	}

	gReal _qp, _dqp, _ddqp, _taup; // q prime, dq prime, ddq prime
#endif

};

/// class g2dLink : a link connected to its parent with a 2 Dof joint (e.g. universal joint)
/** Some important member variables are as follows. \n
* \n
* coordinate (gReal q[2]): joint angles \n
* genVel (gReal* dq[2]): joint velocities. \n
* genAcc (gReal* ddq[2]): joint accelerations. \n
* screw (gTwist S[2]): screw parameters (or joint twists) of the link wrt body frame AT DEFAULT configuration. It is constant. \n
* current1stScrew(gTwist S0): 1st screw parameter as seen from current body frame. \n
* Note that since there are two joint axes, the first screw parameter (wrt body frame) changes as the second joint angle changes,
* while the second screw parameter is always constant wrt the body frame. 
*/
class g2dLink:public gLink
{
public:
	g2dLink()
	{
		_q[0] = _q[1] = 0;
		_dq[0] = _dq[1] = NULL;
		_ddq[0] = _ddq[1] = NULL;
		_tau[0] = _tau[1] = NULL;

		_jointLimitLo.set_size(DOF_2D_LINK);
		_jointLimitHi.set_size(DOF_2D_LINK);
		_jointLimitLo.fill(-HUGE_VAL); //very low number
		_jointLimitHi.fill(HUGE_VAL); //very high number
	}

	/// return idx-th generalized coordinate. idx={0,1}
	inline	gReal	coord(int idx) const		{ assert(idx<DOF_2D_LINK); return _q[idx]; }
	gReal			genVel(int idx) const		{ assert(idx<DOF_2D_LINK); return *_dq[idx]; }
	const gReal*	genVelPtr(int idx) const	{ assert(idx<DOF_2D_LINK); return _dq[idx]; }
	gReal			genAcc(int idx) const		{ assert(idx<DOF_2D_LINK); return *_ddq[idx]; }
	gTwist			screw(int idx) const		{ assert(idx<DOF_2D_LINK); return _S[idx]; }
	gTwist			screwBody(int idx) const	{ assert(idx<DOF_2D_LINK); return (idx == 0) ? _S0 : _S[1]; }

	/** return the 1st screw parameter as seen from the body frame. \n
	* Note that since there are two joint axes, the first screw parameter (wrt body frame) changes as the second joint angle changes,
	* while the second screw parameter is always constant wrt the body frame. Therefore, this function returns the screw
	* parameter of the 1st joint as seen from the current body frame.
	*/
	inline	gTwist&		current1stScrew(void)				{ return _S0; }
	inline	gTwist		current1stScrew(void) const			{ return _S0; }

	/// set idx-th generalized coordinate to q. idx={0,1}
	inline	void		setCoord(int idx,gReal q)		{ assert(idx<DOF_2D_LINK); _q[idx] = q; }
	void		setGenVel(int idx,gReal dq_)	{ assert(idx<DOF_2D_LINK); *_dq[idx]=dq_; }
	void		setGenAcc(int idx,gReal ddq_)	{ assert(idx<DOF_2D_LINK); *_ddq[idx]=ddq_; }
	void		setGenForce(int idx,gReal t )	{ assert(idx<DOF_2D_LINK); *_tau[idx]=t; }
	
	/// add t to idx-th generalized force. idx={0,1}
	inline	void		addGenForce(int idx,gReal t )	{ assert(idx<DOF_2D_LINK); *_tau[idx]+=t; }

	/// set screw parameter of idx-th joint. idx={0,1}
	inline	void		setScrew(int idx,const gTwist& S)	{ assert(idx<DOF_2D_LINK); _S[idx] = S; }

	int			sizeSafeCoordArray() const			{ return DOF_2D_LINK; }
	void		setFromSafeCoordArray(const gReal* ptr)	{ _q[0] = ptr[0];	_q[1] = ptr[1]; }
	void		setFromCompactCoordArray(const gReal* ptr)	{ _q[0] = ptr[0];	_q[1] = ptr[1]; }
	void		setZeroCoord()						{ _q[0]=_q[1]=0; }
	void		getSafeCoordArray(gReal* ptr) const;
	void		getCompactCoordArray(gReal* ptr) const;
	void		copyCoord(gLink* src)				{ assert(dynamic_cast<g2dLink*>(src) != NULL); _q[0] = ((g2dLink*)src)->_q[0]; _q[1] = ((g2dLink*)src)->_q[1]; }
	gReal		genForce(int idx) const				{ assert(idx<DOF_2D_LINK); return *_tau[idx]; }

	//{ link vectorized parameter to particular variables	
	void		setGenVelPtr(int idx, gReal* ref)	{ _dq[idx]=ref; }
	void		setGenAccPtr(int idx, gReal* ref)	{ _ddq[idx]=ref; }
	void		setGenForcePtr(int idx, gReal* ref)	{ _tau[idx]=ref; }
	const gReal* genForcePtr(int idx) const			{ return _tau[idx]; }
	//}
	
	int			type(void) const					{ return TYPE_2D_LINK; }
	int			dof(void)  const					{ return DOF_2D_LINK; }
	void		clearGenForce(void)					{ *_tau[0] = *_tau[1] = 0; }
	void		randomizeCoord(gReal maxRange)
						{
							_q[0] = maxRange*(((gReal)2*rand())/RAND_MAX - 1); // [-maxRange maxRange]
							_q[1] = maxRange*(((gReal)2*rand())/RAND_MAX - 1); // [-maxRange maxRange]
						}
	void		randomizeCoord(gReal* low, gReal* up);
	//{ time integration
	void		timeIntegrateExplicitEuler(gReal time_step)
						{
							_q[0] += (*_dq[0])*time_step;
							_q[1] += (*_dq[1])*time_step;
							*_dq[0] += (*_ddq[0])*time_step;
							*_dq[1] += (*_ddq[1])*time_step;	
						}
	void		timeIntegrateExplicitEulerPositionOnly(gReal time_step)
						{
							_q[0] += (*_dq[0])*time_step;
							_q[1] += (*_dq[1])*time_step;
						}
	void		timeIntegrateSemiImplicitEuler(gReal time_step)
						{
							*_dq[0] += (*_ddq[0])*time_step;
							*_dq[1] += (*_ddq[1])*time_step;	
							_q[0] += (*_dq[0])*time_step;
							_q[1] += (*_dq[1])*time_step;							
						}
	//}

	//{ functions for inverse dynamics (dynamic variables)
	void		frameForceToGenForce (void)					// given F, compute tau
				{ 
					*_tau[0] = (_F,_S0 );
					*_tau[1] = (_F,_S[1]); 							
				}
	//}
	//{ values set from joint parameters	(kinematic variables)
	void		updateLocalFrame(void)						
				{ 
					gXMat t = gXMat::exp( _S[1], _q[1] );
					_G = _H * gXMat::exp( _S[0], _q[0] ) * t;	
					_S0 = _S[0].xformInv(t);
				}
	void		solCoriolisAcc(void)
				{
					_C = gTwist::ad(_V,_U) + gTwist::ad(_S0,_S[1])*((*_dq[0])*(*_dq[1]));
				}
	void		updateLocalVel(void)	
				{ 					
					_U = _S0*(*_dq[0]) + _S[1]*(*_dq[1]);
				}
	void		updateLocalAcc(void)
				{ 
					_Sddq = _S0*(*_ddq[0]) + _S[1]*(*_ddq[1]); 
				}
	void		moveEpsilon(int e,gReal eps)
				{
					_q[e] += eps;
				}
	void		storeCoord(void)
				{
					_q_old[0] = _q[0];
					_q_old[1] = _q[1];
				}

	void		restoreCoord(void)
				{
					_q[0] = _q_old[0];
					_q[1] = _q_old[1];
				}	
	gTwist		screwWorld(int idx) const	{ assert(idx<2); return _Sw[idx]; };
	void		updateScrewWorld(void)		{ _Sw[0] = _S0.xform(_T); _Sw[1] = _S[1].xform(_T); };

protected:
	gReal			_q[DOF_2D_LINK];		// generalized coordinate
	gTwist			_S[DOF_2D_LINK];		// screw parameter in home position..constant
	gTwist			_S0;					// 1st screw parameter as seen from current body frame
	gReal			_q_old[DOF_2D_LINK];	// variable for temporarily storing _q
	gTwist			_Sw[DOF_2D_LINK];

	//{ interface
	gReal*			_dq[DOF_2D_LINK];		// gen. coord. velocity
	gReal*			_ddq[DOF_2D_LINK];	// gen. coodr. acceleration
	gReal*			_tau[DOF_2D_LINK];	// torque 
	//}
#ifdef MW_USE_ARTICULATED_BODY_METHOD
protected:
	//{ forward dynamics
	gAInertia	lockedArtInertia();
	void		updateArtInertiaAndBiasForce(bool updateOnlyBiasForce=false);
	void		solAcc(void);				// compute dV
	gWrench		sol_xx(void);
	//}	

protected:
	gReal			_W[3];					// _W = 1/SAS
	gWrench			_AS0, _AS1;

#endif //MW_USE_ARTICULATED_BODY_METHOD

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	gReal coordP(int idx) const		{assert(idx<2); return _qp[idx];}
	gReal genVelP(int idx) const	{assert(idx<2); return _dqp[idx];}
	gReal genAccP(int idx) const	{assert(idx<2); return _ddqp[idx];}
	gReal genForceP(int idx) const	{assert(idx<2); return _taup[idx];}

	void setCoordP(int idx, gReal x)	{ assert(idx<2); _qp[idx] = x;}
	void setGenVelP(int idx, gReal x)	{ assert(idx<2); _dqp[idx] = x; }
	void setGenAccP(int idx, gReal x)	{ assert(idx<2); _ddqp[idx] = x; }

	void updateKVariablesP(bool recursive=true)		
	{ 
		_z = gTwist::ad(_S0,_S[1]);

		_Gp = _S0*_qp[0] + _S[1]*_qp[1]; 
		_Tp = _Gp;

		_Vp = _S0*_dqp[0] + _S[1]*_dqp[1] + _z*((*_dq[0])*_qp[1]); 

		_dVp = _S0*_ddqp[0] + _S[1]*_ddqp[1] + _z*(_dqp[0]*(*_dq[1]) + (*_ddq[0])*_qp[1]);
		_dVp -= gTwist::ad(_S[1],_z)*((*_dq[0])*(*_dq[1])*_qp[1]);
		_dVp += gTwist::ad(_V, _z*((*_dq[0])*_qp[1]) +_S0*_dqp[0]+_S[1]*_dqp[1]);
		_dVp += gTwist::ad(_Vp,_S0*(*_dq[0])+_S[1]*(*_dq[1]));

		if(_parent){
			_Tp += _parent->_Tp.xformInv(_G);

			_Vp += _parent->_Vp.xformInv(_G);
			_Vp -= gTwist::ad(_Gp , _parent->frameVel().xformInv(_G));

			_dVp += _parent->_dVp.xformInv(_G);
			_dVp -= gTwist::ad( _Gp, _parent->frameAcc().xformInv(_G));
		}

		if(recursive){
			for(gLinkList::iterator it = _children.begin(); it!=_children.end(); it++) (*it)->updateKVariablesP(true);
		}
	}

	void computeGenForceP()
	{
		_taup[0] = (_z,_F)*(_qp[1])+(_S0,_Fp);
		_taup[1] = (_S[1],_Fp);
	}

	gReal _qp[DOF_2D_LINK], _dqp[DOF_2D_LINK], _ddqp[DOF_2D_LINK], _taup[DOF_2D_LINK]; // q prime, dq prime, ddq prime
	gTwist _z; 
#endif
};

/// class gBallLink : a link connected to its parent with a spherical (ball-and-socket) joint
/** Some important member variables are as follows. \n
* \n
* coordinate (gRotMat R): rotation matrix wrt parent frame\n
* genVel (gReal* dq[3]): angular velocity relative to its parent, expressed in body frame \n
* genAcc (gReal* ddq[3]): angular accelerations\n
* Note that for a ball joint, screw axis can be any arbitrary 3D vector.
* screw(int i) function outputs the three basis vectors for rotation: (1,0,0,0,0,0), (0,1,0,0,0,0), (0,0,1,0,0,0)
*/
class gBallLink:public gLink
{
public:
	gBallLink()
	{
		_dq[0]= _dq[1] = _dq[2] = NULL;
		_ddq[0]= _ddq[1] = _ddq[2] = NULL;
		_tau[0]= _tau[1] = _tau[2] = NULL;

		_jointLimitLo.set_size(DOF_BALL_LINK);
		_jointLimitHi.set_size(DOF_BALL_LINK);
		_jointLimitLo.fill(-HUGE_VAL); //very low number
		_jointLimitHi.fill(HUGE_VAL); //very high number
	}

	/// return generalized coordinates (gRotMat R)
	inline	gRotMat&		coord(void)			{ return _R; }
	inline	const gRotMat&	coord(void) const	{ return _R; }

	gReal		genVel(int idx) const		{ assert(idx<DOF_BALL_LINK); return *_dq[idx]; }
	gVec3		genVel(void) const			{ return gVec3(*_dq[0],*_dq[1],*_dq[2]); }
	const gReal* genVelPtr(int idx)	const	{ assert(idx<DOF_BALL_LINK); return _dq[idx]; }
	gReal		genAcc(int idx) const		{ assert(idx<DOF_BALL_LINK); return *_ddq[idx]; }
	gVec3		genAcc(void) const			{ return gVec3(*_ddq[0],*_ddq[1],*_ddq[2]); }
	gReal		genForce(int idx) const		{ assert(idx<DOF_BALL_LINK); return *_tau[idx]; }
	gVec3		genForce(void) const		{ return gVec3(*_tau[0],*_tau[1],*_tau[2]); }
	gTwist		screw(int idx) const		{ assert(idx<DOF_BALL_LINK); switch(idx){ case 0: return gTwistRotX; case 1: return gTwistRotY; case 2: default: return gTwistRotZ; } }
	gTwist		screwBody(int idx) const	{ return screw(idx);}

	/// set generalized coordinates from quat
	void		setCoord(gQuat& quat)		{ quat.normalize();_R = quat.inRotMatrix(); }
	
	/// set generalized coordinates from rotation matrix R
	inline	void		setCoord(const gRotMat& R)		{ _R = R; }

			void		setGenVel(int idx, gReal dq_ )	{ assert(idx<DOF_BALL_LINK); *_dq[idx] = dq_; }
			void		setGenVel(const gVec3& dq_ )	{ *_dq[0]=dq_.x(); *_dq[1]=dq_.y();*_dq[2]=dq_.z(); }
			void		setGenAcc(int idx,gReal ddq_)	{ assert(idx<DOF_BALL_LINK); *_ddq[idx] = ddq_; }
			void		setGenAcc(const gVec3& ddq_ )	{ *_ddq[0]=ddq_.x(); *_ddq[1]=ddq_.y();*_ddq[2]=ddq_.z(); }
			void		setGenForce(int idx,gReal t )	{ assert(idx<DOF_BALL_LINK); *_tau[idx] = t; }
			void		setGenForce(const gVec3& t )	{ *_tau[0]=t.x(); *_tau[1]=t.y();*_tau[2]=t.z(); }
	/// add t to idx-th generalized force idx=[0..2]
	inline	void		addGenForce(int idx,gReal t )	{ *_tau[idx]+=t; }
			void		addGenForce(const gVec3& t )	{ *_tau[0]+=t.x(); *_tau[1]+=t.y();*_tau[2]+=t.z(); }

			int			type(void) const			{ return TYPE_BALL_LINK; }
			int			dof(void)  const			{ return DOF_BALL_LINK; }
	
	//when gBallLink's coordinate (gRotMat) is stored in an array, it is converted to quaternion and stored. Hence the size of coordinate array is 4(SIZE_QUATERNION).
			int			sizeSafeCoordArray() const	{ return SIZE_QUATERNION; }
			void		setFromSafeCoordArray(const gReal* ptr); //read four numbers (quaternion) from ptr and set _R
			void		setFromCompactCoordArray(const gReal* ptr); //read three numbers (exponential coord.) from ptr and set _R
			void		getSafeCoordArray(gReal* ptr) const; //convert _R to quaternion and copy the four numbers to ptr
			void		getCompactCoordArray(gReal* ptr) const; //convert _R to exponential coordinates (3 numbers) and copy to ptr. Beware that this may lead to singularity.
			void		setZeroCoord()					{ _R.setIdentity(); }
			void		copyCoord(gLink* src)		{ assert(dynamic_cast<gBallLink*>(src) != NULL); _R = ((gBallLink*)src)->_R; }
			void		clearGenForce(void)				{ *_tau[0] = *_tau[1] = *_tau[2] = 0; }
	//{ link vectorized parameter to particular variables	
			void		setGenVelPtr(int idx, gReal* ref)	{ _dq[idx]=ref; }
			void		setGenAccPtr(int idx, gReal* ref)	{ _ddq[idx]=ref; }
			void		setGenForcePtr(int idx, gReal* ref)	{ _tau[idx]=ref; }
			const gReal* genForcePtr(int idx) const			{ return _tau[idx]; }
	//}	
			void		randomizeCoord(gReal maxRange);
			void		randomizeCoord(gReal* low, gReal* up);
	//{ time integration
			void		timeIntegrateExplicitEuler(gReal time_step);
			void		timeIntegrateExplicitEulerPositionOnly(gReal time_step)	{  _R *= gRotMat::exp(gVec3(*_dq[0]*time_step,*_dq[1]*time_step,*_dq[2]*time_step));	}
			void		timeIntegrateSemiImplicitEuler(gReal time_step);
	//}

	//{ forward dynamics
			void		frameForceToGenForce (void) { *_tau[0] = _F.e(0); *_tau[1] = _F.e(1); *_tau[2] = _F.e(2); } // given F, compute tau
	//}	
	//{ values set from joint parameters	(kinematic variables)
			void		updateLocalFrame(void)						// given joint angle, solve f
						{ 
							_G = _H.multRotMat(_R);	
						}

			void		updateLocalVel(void)	
						{ 
							_U.setRot(genVel(0),genVel(1),genVel(2)); 
						}

			void		solCoriolisAcc(void)
						{
							_C = gTwist::ad(_V,_U);		
						}

			void		updateLocalAcc(void)
						{ 
							_Sddq.setRot(genAcc(0),genAcc(1),genAcc(2)); 
						}

			void		moveEpsilon(int e,gReal eps)
						{
							gRotMat E;
							switch(e)
							{
							case 0:
								E.makeRotateX(eps); break;
							case 1:
								E.makeRotateY(eps); break;
							case 2:
								E.makeRotateZ(eps); break;
							}								
							_R *= E;
						}
			
			void		moveEpsilon(const gVec3 eps)
						{
							_R *= gRotMat::exp(eps);
						}

			void		storeCoord(void)
						{
							_R_old = _R;
						}

			void		restoreCoord(void)
						{
							_R = _R_old;
						}

			gTwist		screwWorld(int idx) const	{ assert(idx<3); return _Sw[idx]; };

			void		updateScrewWorld(void)
	{
		_Sw[0] = gTwist(1,0,0,0,0,0).xform(_T);
		_Sw[1] = gTwist(0,1,0,0,0,0).xform(_T);
		_Sw[2] = gTwist(0,0,1,0,0,0).xform(_T);
	};

protected:
	gRotMat			_R, _R_old; // think about using Quarternion

	gTwist			_Sw[3];
	//{ interface
	gReal*			_dq[DOF_BALL_LINK];	// gen. coord. velocity
	gReal*			_ddq[DOF_BALL_LINK];	// gen. coodr. acceleration
	gReal*			_tau[DOF_BALL_LINK];	// torque 
	//}
#ifdef MW_USE_ARTICULATED_BODY_METHOD
protected:
			void		updateArtInertiaAndBiasForce(bool updateOnlyBiasForce=false);
			void		solAcc(void);				// compute dV
			gAInertia	lockedArtInertia();
			gWrench		sol_xx(void);

protected:
	gReal			_W[6];	// _W = 1/SAS. 6 vector represent 3x3 symmetric matrix
	gReal			/*_a00,_a01,_a02,_a10,_a11,_a12,_a20,_a21,_a22,*/_a30,_a31,_a32,_a40,_a41,_a42,_a50,_a51,_a52; //elements of _A*_S*_W where _S=[rotX,rotY,rotZ]

#endif

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
			gReal coordP(int idx) const			{assert(idx<3); return _qp[idx];}
			gReal genVelP(int idx) const		{assert(idx<3); return _dqp[idx];}
			gReal genAccP(int idx) const		{assert(idx<3); return _ddqp[idx];}
			gReal genForceP(int idx) const		{assert(idx<3); return _taup[idx];}

			void setCoordP(int idx, gReal x)	{ assert(idx<3); _qp[idx] = x;}
			void setGenVelP(int idx, gReal x)	{ assert(idx<3); _dqp[idx] = x; }
			void setGenAccP(int idx, gReal x)	{ assert(idx<3); _ddqp[idx] = x; }

			void updateKVariablesP(bool recursive=false)		
	{ 
		_Gp.set( _qp[0], _qp[1], _qp[2], 0, 0, 0);
		_Tp = _Gp;
		_Vp.set(_dqp[0],_dqp[1],_dqp[2],0,0,0); 
		_dVp.set(_ddqp[0],_ddqp[1],_ddqp[2],0,0,0);
		_dVp += gTwist::ad( _Vp, gTwist(*_dq[0],*_dq[1],*_dq[2],0,0,0) );
		_dVp += gTwist::ad( _V , gTwist(_dqp[0],_dqp[1],_dqp[2],0,0,0) );


		if(_parent){
			_Tp += _parent->_Tp.xformInv(_G);

			_Vp -= gTwist::ad(_Gp,_parent->frameVel().xformInv(_G));
			_Vp += _parent->_Vp.xformInv(_G);

			_dVp += _parent->_dVp.xformInv(_G);
			_dVp -= gTwist::ad( _Gp, _parent->frameAcc().xformInv(_G));
		}

		if(recursive){
			for(gLinkList::iterator it = _children.begin(); it!=_children.end(); it++) (*it)->updateKVariablesP(true);
		}
	}
	void computeGenForceP()
	{
		_taup[0] = _Fp.e(0);
		_taup[1] = _Fp.e(1);
		_taup[2] = _Fp.e(2);
	}

	gReal _qp[DOF_BALL_LINK], _dqp[DOF_BALL_LINK], _ddqp[DOF_BALL_LINK], _taup[DOF_BALL_LINK]; // q prime, dq prime, ddq prime
#endif

};


/// class gFreeLink : a link freely moving in space
/** Unlike other links such as g1dLink(q), g2dLink(q[2]), and gBallLink(R), this link does not define
* a separate coordinate variable. Instead, it uses G as its coordinate. \n
* But it still has separate genVel(dq[6]) and genAcc(ddq[6]) variables. \n
* Thus, dq[6] is equal to U and ddq[6] is equal to dU/dt. \n
* \n
* Additionally, gFreeLink must be only a root. It must not be defined as a child link. \n
* Thus, it should be that dq[6] = U = V and ddq[6] = dV = Sddq \n
* screw(int i) function outputs the six basis vectors for rotation/translation.
*/
class gFreeLink:public gLink
{
public:
	gFreeLink()
	{
		_dq[0]=_dq[1]=_dq[2]=_dq[3]=_dq[4]=_dq[5]=NULL;
		_ddq[0]=_ddq[1]=_ddq[2]=_ddq[3]=_ddq[4]=_ddq[5]=NULL;
		_tau[0]=_tau[1]=_tau[2]=_tau[3]=_tau[4]=_tau[5]=NULL;

		_jointLimitLo.set_size(DOF_FREE_LINK);
		_jointLimitHi.set_size(DOF_FREE_LINK);
		_jointLimitLo.fill(-HUGE_VAL); //very low number
		_jointLimitHi.fill(HUGE_VAL); //very high number
	}

	/// return generalized coordinates (It is local frame (G) for gFreeLink )
	inline	gXMat		coord(void) const			{ return _G; }
			gReal		genVel(int idx) const		{ assert(idx<DOF_FREE_LINK); return *_dq[idx]; }
			const gReal* genVelPtr(int idx) const	{ assert(idx<DOF_FREE_LINK); return _dq[idx]; }
			gReal		genAcc(int idx) const		{ assert(idx<DOF_FREE_LINK); return *_ddq[idx]; }
			gReal		genForce(int idx) const		{ assert(idx<DOF_FREE_LINK); return *_tau[idx]; }
	///output the six basis vectors for rotation/translation.
			gTwist		screw(int idx) const		{ assert(idx<DOF_FREE_LINK); gTwist t; t.setBasis(idx); return t; }
			gTwist		screwBody(int idx) const	{ return screw(idx);}

	/// set generalized coordinates 
	inline	void		setCoord(const gXMat& G)		{ _G = G; }

			/// set generalized velocity to (rot,lin), where rot: rotational velocity, lin: linear velocity
			void		setGenVel(const gVec3& rot, const gVec3& lin) { *_dq[0]=rot.x(), *_dq[1]=rot.y(), *_dq[2]=rot.z(), *_dq[3]=lin.x(), *_dq[4]=lin.y(), *_dq[5]=lin.z(); }
			void		setGenVel(const gTwist& vel) { *_dq[0]=vel.e(0), *_dq[1]=vel.e(1), *_dq[2]=vel.e(2), *_dq[3]=vel.e(3), *_dq[4]=vel.e(4), *_dq[5]=vel.e(5); }
			void		setGenVel(int idx, gReal dq_ )	{ assert(idx<DOF_FREE_LINK); *_dq[idx] = dq_; }
			
			/// set generalized acceleration to (rot,lin), where rot: rotational acceleration, lin: linear acceleration
			void		setGenAcc(const gVec3& rot, const gVec3& lin) { *_ddq[0]=rot.x(), *_ddq[1]=rot.y(), *_ddq[2]=rot.z(), *_ddq[3]=lin.x(), *_ddq[4]=lin.y(), *_ddq[5]=lin.z(); }
			void		setGenAcc(int idx,gReal ddq_)	{ assert(idx<DOF_FREE_LINK); *_ddq[idx] = ddq_; }
			void		setGenForce(int idx,gReal t )	{ assert(idx<DOF_FREE_LINK); *_tau[idx] = t; }
			/// add t to idx-th generalized force. idx=[0..5]
	inline	void		addGenForce(int idx,gReal t )	{ assert(idx<DOF_FREE_LINK); *_tau[idx]+=t; }
	
			int			type(void) const				{ return TYPE_FREE_LINK; }
			int			dof(void)  const				{ return DOF_FREE_LINK; } 
			int			sizeSafeCoordArray() const		{ return SIZE_PARAM_SE3; } //7 because 4 for quaternion + 3 for position
			void		setFromSafeCoordArray(const gReal* ptr);	
			void		setFromCompactCoordArray(const gReal* ptr);

			void		getSafeCoordArray(gReal* ptr) const;	//convert rotation to quaternion, and copy 7 numbers (quaternion,position) to ptr
			void		getCompactCoordArray(gReal* ptr) const; //convert rotation to exponential coordinates, and copy 6 numbers (exp. coord., position) to ptr
			void		setZeroCoord()					{ _G.setIdentity(); }
			void		copyCoord(gLink* src)			{ assert(dynamic_cast<gFreeLink*>(src) != NULL); _G = ((gFreeLink*)src)->_G; }
			void		clearGenForce(void)				{ *_tau[0]=*_tau[1]=*_tau[2]=*_tau[3]=*_tau[4]=*_tau[5]=0; }
			void		randomizeCoord(gReal maxRange);
			void		randomizeCoord(gReal* low, gReal* up);
	//{ time integration
			void		timeIntegrateExplicitEuler(gReal time_step);
			void		timeIntegrateSemiImplicitEuler(gReal time_step);

	//this integrator only updates position and orientation, but not velocities.
			void		timeIntegrateExplicitEulerPositionOnly(gReal time_step); 
	//}

	//{ link vectorized parameter to particular variables	
			void		setGenVelPtr(int idx, gReal* ref)	{ assert(idx<DOF_FREE_LINK); _dq[idx]=ref; }
			void		setGenAccPtr(int idx, gReal* ref)	{ assert(idx<DOF_FREE_LINK); _ddq[idx]=ref; }
			void		setGenForcePtr(int idx, gReal* ref)	{ assert(idx<DOF_FREE_LINK); _tau[idx]=ref; }
			const gReal* genForcePtr(int idx) const			{ assert(idx<DOF_FREE_LINK);  return _tau[idx]; }
	//}
	// given F, compute tau
			void		frameForceToGenForce (void) { *_tau[0] = _F.e(0); *_tau[1] = _F.e(1); *_tau[2] = _F.e(2); *_tau[3] = _F.e(3); *_tau[4] = _F.e(4); *_tau[5] = _F.e(5); }
	//{ values set from joint parameters	(kinematic variables)
			void		updateLocalFrame(void){ }
			void		updateLocalVel(void){ copyDqToU();	}
			void		solCoriolisAcc(void){ _C.setZero();	}
			void		updateLocalAcc(void){ copyDdqToSddq(); }
			void		moveEpsilon(int e,gReal eps);
			void		storeCoord(void)
				{
					_G_old = _G;
				}
			void		restoreCoord(void)
				{
					_G = _G_old;
				}

protected:
	//{ functions to synchronize variables
	void copyDqToU()
	{
		_U.set(*_dq[0],*_dq[1],*_dq[2],*_dq[3],*_dq[4],*_dq[5]);
	}
	void copyUToDq()
	{
		*_dq[0] = _U.e(0);*_dq[1] = _U.e(1);*_dq[2] = _U.e(2);
		*_dq[3] = _U.e(3);*_dq[4] = _U.e(4);*_dq[5] = _U.e(5);
	}
	void copyDdqToSddq()
	{
		_Sddq.set(*_ddq[0],*_ddq[1],*_ddq[2],*_ddq[3],*_ddq[4],*_ddq[5]);
	}
	void copyDVToDdq()
	{
		*_ddq[0] = _dV.e(0);*_ddq[1] = _dV.e(1);*_ddq[2] = _dV.e(2);
		*_ddq[3] = _dV.e(3);*_ddq[4] = _dV.e(4);*_ddq[5] = _dV.e(5);
	}
	void copyTauToF()
	{
		_F.set(*_tau[0],*_tau[1],*_tau[2],*_tau[3],*_tau[4],*_tau[5]);
	}
	//}
	void definedAsChildError()
	{
		std::cout << "gFreeLink error: defined as a child link." << std::endl;
		assert(0); /*since gFreeLink cannot be a child, it must not be called*/
	}

	gTwist		screwWorld(int idx) const { assert(idx<6); return _Sw[idx]; };
	void		updateScrewWorld(void)
	{
		_Sw[0] = gTwist(1,0,0,0,0,0).xform(_T);
		_Sw[1] = gTwist(0,1,0,0,0,0).xform(_T);
		_Sw[2] = gTwist(0,0,1,0,0,0).xform(_T);
		_Sw[3] = gTwist(0,0,0,1,0,0).xform(_T);
		_Sw[4] = gTwist(0,0,0,0,1,0).xform(_T);
		_Sw[5] = gTwist(0,0,0,0,0,1).xform(_T);
	};

protected:
	gXMat			_G_old; //for store/restoreCoord
	//{ interface
	gReal*			_dq[DOF_FREE_LINK];	// gen. coord. velocity
	gReal*			_ddq[DOF_FREE_LINK];	// gen. coodr. acceleration
	gReal*			_tau[DOF_FREE_LINK];	// torque 
	//}
	gTwist			_Sw[DOF_FREE_LINK];

#ifdef MW_USE_ARTICULATED_BODY_METHOD
protected:
	//{ forward dynamics
	void		updateArtInertiaAndBiasForce(bool updateOnlyBiasForce=false)
	{ 
		gLink::updateArtInertiaAndBiasForce(updateOnlyBiasForce); 
	}
	void		solAcc(void);
	gAInertia	lockedArtInertia(){ return gAInertia(); /* zero matrix */ }
	gWrench		sol_xx(void){ return gWrenchZero; }
	//}	
protected:
#endif

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	gReal coordP(int idx) const			{assert(idx<6); return _qp[idx];}
	gReal genVelP(int idx) const		{assert(idx<6); return _dqp[idx];}
	gReal genAccP(int idx) const		{assert(idx<6); return _ddqp[idx];}
	gReal genForceP(int idx) const		{assert(idx<6); return _taup[idx];}

	void setCoordP(int idx, gReal x)	{ assert(idx<6); _qp[idx] = x;}
	void setGenVelP(int idx, gReal x)	{ assert(idx<6); _dqp[idx] = x; }
	void setGenAccP(int idx, gReal x)	{ assert(idx<6); _ddqp[idx] = x; }

	void updateKVariablesP(bool recursive=false)		
	{ 
		_Gp.set( _qp[0], _qp[1], _qp[2], _qp[3], _qp[4], _qp[5]);
		_Tp = _Gp;
		_Vp.set(_dqp[0],_dqp[1],_dqp[2],_dqp[3],_dqp[4],_dqp[5]); 
		_dVp.set(_ddqp[0],_ddqp[1],_ddqp[2],_ddqp[3],_ddqp[4],_ddqp[5]);
		_dVp += gTwist::ad( _Vp, gTwist(*_dq[0],*_dq[1],*_dq[2],*_dq[3],*_dq[4],*_dq[5]) );
		_dVp += gTwist::ad( _V , gTwist(_dqp[0],_dqp[1],_dqp[2],_dqp[3],_dqp[4],_dqp[5]) );

		if(_parent){
			_Tp += _parent->_Tp.xformInv(_G);

			_Vp -= gTwist::ad(_Gp, _parent->frameVel().xformInv(_G));
			_Vp += _parent->_Vp.xformInv(_G);

			_dVp += _parent->_dVp.xformInv(_G);
			_dVp -= gTwist::ad( _Gp, _parent->frameAcc().xformInv(_G));
		}

		if(recursive){
			for(gLinkList::iterator it = _children.begin(); it!=_children.end(); it++) (*it)->updateKVariablesP(true);
		}
	}
	void computeGenForceP()
	{
		_taup[0] = _Fp.e(0);
		_taup[1] = _Fp.e(1);
		_taup[2] = _Fp.e(2);
		_taup[3] = _Fp.e(3);
		_taup[4] = _Fp.e(4);
		_taup[5] = _Fp.e(5);
	}

	gReal _qp[DOF_FREE_LINK], _dqp[DOF_FREE_LINK], _ddqp[DOF_FREE_LINK], _taup[DOF_FREE_LINK]; // q prime, dq prime, ddq prime
#endif
};


#endif














