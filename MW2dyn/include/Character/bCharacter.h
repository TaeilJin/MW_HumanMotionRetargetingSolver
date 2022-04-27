//#####################################################################
// Copyright 2010-2015, Hynchul Choi, Sukwon Lee, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _BCHARACTER_H
#define _BCHARACTER_H

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "mbs/gMultibodySystem.h"
#include <btBulletDynamicsCommon.h>
#include "Character/bShapeSet.h"

class gFootGeom;
class bCharacter;
class bCharacterSim;




// enum will include update mode to reduce errors
enum	gUpdateMode
{
	G_USE_JOINT_CORRECTION_CCD = 1,
	G_USE_JOINT_CORRECTION_FOOT = 2
};


//////////////////////////////////////////////////////
/// bCharacterSim
/**
* Actual character that is simulated by bullet dynamics engine. \n
*/	
class bCharacterSim
{
	friend class bCharacter;

public:
	/*enum collisionTypes {
		CHARACTER = 1,
		NON_CHARACTER = 1 << 1
	};*/ //SHL removed (2015.12.22) because this collisonTypes does not allow different characters to belong to different collision groups.

	bCharacterSim(bCharacter* mbs);
	virtual ~bCharacterSim();

	

	/// Get pointer to btBodies
	inline btRigidBody** getBtBodies(){ return m_btBodies; };
	//inline const btRigidBody** getBtBodies() const { return m_btBodies; };
	inline btRigidBody** getBtBodies() const { return m_btBodies; };

	/// Get pointer to btJoints
	inline btTypedConstraint** getBtJoints(){ return m_joints; };

	/// Get index of RigidBodyA of joint[idx]
	inline int getIndexJointRigidBodyA(int idx) { return m_jointRigidBodyA[idx]; }

	/// Get index of RigidBodyB of joint[idx]
	inline int getIndexJointRigidBodyB(int idx) { return m_jointRigidBodyB[idx]; }

	/// Get Bullet owner world
	inline btDynamicsWorld* getOwnerWorld(){ return m_btOwnerWorld; };

	/// Get Bullet Collision Shape
	inline btCollisionShape** getBtShapes() { return m_btShapes; };

	/// Get pointer to foot
	btRigidBody* getLFootBtBody() { return m_lFootBtBody; };
	btRigidBody* getRFootBtBody() { return m_rFootBtBody; };
	
	/// Get number of Bodies
	int getNumBtBodies() const { return m_numBodies; }

	/// Get number of joints
	int getNumBtJoints() const { return m_numJoints; }

	/// Set all bullet bodies kinematic or dynamic
	/// parameter
	/// option = 0 for dynamic simulation
	/// option = btCollisionObject::CF_KINEMATIC_OBJECT for kinematic motion
	void setBtBodiesDynamicOrKinematic(int option);

	/// Initialize member variables
	void postLoading();

protected:

	/// Pointer to the character model
	bCharacter* m_mbs;

	/// Pointer to Bullet Dynamics World
	btDynamicsWorld*		m_btOwnerWorld;
	
	/// Pointer to collision shapes for each rigid body
	btCollisionShape**		m_btShapes;
	
	/// Pointer to corresponding btRigidBody's. m_btBodies[i] corresponds to mbs->_links[i]
	/***
	For a single link, there are two entities: _links[i] and m_btBodies[i]. \n
	Consider m_btBodies[i] as the "physical link" that is simulated by the physics engine. \n
	In contrast, _links[i] is the (imaginary) model for the physical link, and it is used by solver and controller. \n
	Therefore, once a simulation step for m_btBodies[i] is finished, _links[i] is updated to synchronize with m_btBodies (for this, updateStatesFromCharacterSim() is used ). \n
	Then a controller performs to determine suitable control output (joint torques), which is then applied to m_btBodies[i] through applyJointTorquesToCharacterSim() function. \n
	*/
	btRigidBody**			m_btBodies;			// m_btBodies[i] corresponds to mbs->_links[i]

	/// Size of m_btBodies ( = mbs->_nLinks )
	int m_numBodies; 

	/// Pointer to Bullet joints.
	/***If m_btBodies[0] is a free link, m_btJoints[i] is the joint between m_btBodies[i+1] and its parent. \n
	* If m_btBodies[0] is constrained, m_btJoints[i] is the joint between m_btBodies[i] and its parent.
	*/
	btTypedConstraint** m_joints; 

	/// Size of m_btJoints
	int m_numJoints; 

	///m_jointRigidBodyA[i] is the index of m_btJoints[i]->rigidBodyA
	int* m_jointRigidBodyA; 

	///m_jointRigidBodyB[i] is the index of m_btJoints[i]->rigidBodyB
	int* m_jointRigidBodyB; 

	/// Foot links
	btRigidBody *m_rFootBtBody;
	btRigidBody *m_lFootBtBody;

	/// Hand links
	btRigidBody *m_rHandBtBody;
	btRigidBody *m_lHandBtBody;

};


//////////////////////////////////////////////////////
/// gFootGeom
/** foot geometry info. it can be expanded to a compound shape; for now, we're dealing with a box type only.
*/
class gFootGeom
{
public:

	gFootGeom(){}

	virtual ~gFootGeom(){}

	// getter
	inline gVec3&			lowerBound(){ return m_lb; }
	inline gVec3&			upperBound(){ return m_ub; }
	
	inline int				idxFrontal(){ return m_idxFrontal; }
	inline int				idxLateral(){ return m_idxLateral; }
	inline int				idxUp(){ return m_idxUp; }

	inline gVec3&			dirFrontal(){ return m_dirFrontal; }
	inline gVec3&			dirLateral(){ return m_dirLateral; }
	inline gVec3&			dirUp(){ return m_dirUp; }

	// check if the input point is inside the foot region
	// point is wrt foot body frame
	inline bool		isInsideFootBase(gVec3 point, double margin = 0.1)
	{
		if( point.e(m_idxFrontal) < m_lb.e(m_idxFrontal)+margin || point.e(m_idxFrontal) > m_ub.e(m_idxFrontal)-margin ) return false;
		if( point.e(m_idxLateral) < m_lb.e(m_idxLateral)+margin || point.e(m_idxLateral) > m_ub.e(m_idxLateral)-margin ) return false;
		return true; 	
	}


	/// set m_lb, m_ub, m_idxUp/Frontal/Lateral variables
	/***this function must be called after the character takes the default pose.
	* link: foot link
	* localCoMFrame: xform matrix from the foot's body frame to bullet body frame (refer to bCharacter::localCoMFrame) \n
	* shape: foot geometry \n
	* frontalDirection, upDirection: frontal and up directions for the charcter at the given pose
	*/
	void setFromGeometry(gLink* link,const gXMat& localCoMFrame,const btBoxShape* shape,gVec3& frontalDirection, gVec3& upDirection)
	{
		btVector3 _halfLen = shape->getHalfExtentsWithoutMargin();		
		gVec3 halfLenLocal = localCoMFrame.multVec3(  gVec3(_halfLen.x(), _halfLen.y(), _halfLen.z()) );
		if(halfLenLocal.x()<0) halfLenLocal.setX(-halfLenLocal.x());
		if(halfLenLocal.y()<0) halfLenLocal.setY(-halfLenLocal.y());
		if(halfLenLocal.z()<0) halfLenLocal.setZ(-halfLenLocal.z());
		gVec3 cen = localCoMFrame.trn();
		m_lb = cen - halfLenLocal;
		m_ub = cen + halfLenLocal;
		
		gRotMat R = link->frame().rot();
		double thresh = 0.9;
		if( fabs((gVec3(R.col(0)), upDirection)) > thresh ){
			m_idxUp = 0;
	
			if( fabs((gVec3(R.col(1)), frontalDirection)) > thresh ){
				m_idxFrontal = 1;
				m_idxLateral = 2;
			}else{
				m_idxFrontal = 2;
				m_idxLateral = 1;
			}
		}
		else if ( fabs((gVec3(R.col(1)), upDirection)) > thresh ){
			m_idxUp = 1;
			if( fabs((gVec3(R.col(0)), frontalDirection)) > thresh ){
				m_idxFrontal = 0;
				m_idxLateral = 2;
			}else{
				m_idxFrontal = 2;
				m_idxLateral = 0;
			}
		}
		else if ( fabs((gVec3(R.col(2)), upDirection)) > thresh ){
			m_idxUp = 2;
			if( fabs((gVec3(R.col(0)), frontalDirection)) > thresh ){
				m_idxFrontal = 0;
				m_idxLateral = 1;
			}else{
				m_idxFrontal = 1;
				m_idxLateral = 0;
			}
		}
		else{
			assert(0);
		}

		m_dirFrontal.setZero(); m_dirFrontal.set(m_idxFrontal, 1.0);
		m_dirLateral.setZero(); m_dirLateral.set(m_idxLateral, 1.0);
		m_dirUp.setZero(); m_dirUp.set(m_idxUp, 1.0 );
	}

private:
	//
	// bounds and directions
	// m_lb.e(m_idxFrontal), m_ub.e(m_idxFrontal) : (lower, upper) bounds in frontal direction
	// m_lb.e(m_idxLateral), m_ub.e(m_idxLateral) : (lower, upper) bounds in lateral direction
	// m_lb.e(m_idxUp), m_ub.e(m_idxUp) : (lower, upper) bounds in up-direction. m_lb.e(m_idxUp) is the coordinate of the foot base
	//
	gVec3 m_lb, m_ub;	//lower/upper bound of the foot base. If Y-up, m_lb.y=m_ub.y=Y-coordinate of the foot base, and m_lb/m_ub.x and z refer to coordinates of the corners.
	int m_idxFrontal;	// 0(X),1(Y),or 2(Z) for the front direction of the foot base
	int m_idxLateral;	// 0(X),1(Y),or 2(Z) for the lateral direction of the foot base
	int m_idxUp;		// 0(X),1(Y),or 2(Z) for the up-vector
	
	gVec3 m_dirFrontal; // frontal direction
	gVec3 m_dirLateral; // lateral direction
	gVec3 m_dirUp;		// up direction

};



//////////////////////////////////////////////////////
/// bCharacter 
/**
* A character model for bCharacterSim. \n
* Specifically, bCharacter represents a model of a real (virtual or physical) character, i.e., bCharacterSim. \n
* When Bullet simulates bCharacterSim, bCharacter synchronizes its states to bCharacterSim via updateStatesFromCharacterSim() function.\n
* Then a controller determines proper control input and store it in bCharacter->_tau. After that, bCharacter applies \n
* the control torque via applyJointTorquesToCharacterSim() function. \n
* If a bCharacterSim is declared as a kinematic body, updateKinematicBodiesOfCharacterSim() function synchronizes \n
* bCharacterSim's posture to bCharacter. \n
* setupCharacterSim() function initializes bCharacterSim per this.
*/	
class bCharacter:public gMultibodySystem
{

public: //public for now
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

	bool inLLeg(int linkIdx); //return true if linkIdx is included in m_lLegIdx
	bool inRLeg(int linkIdx); //return true if linkIdx is included in m_rLegIdx
	bool inLArm(int linkIdx);
	bool inRArm(int linkIdx);
	bool inTrunk(int linkIdx);

protected:
	/// Foot links
	gLink	*m_rFootLink;
	gLink	*m_lFootLink;
	/// Hand links
	gLink	*m_rHandLink;
	gLink	*m_lHandLink;

	/// GRF and CoP at each foot
	gFootGeom m_lFootGeom;
	gFootGeom m_rFootGeom;

	/// Foot geometry information
	gVec3 m_lgrf;
	gVec3 m_rgrf;
	gVec3 m_lcop;
	gVec3 m_rcop;

	/// Contact info
	bool	m_isLFootContact;
	bool	m_isRFootContact;
	int		m_numContactLeft;
	int		m_numContactRight;

	// ShapeSet For Visualizing
	bShapeSet *m_ShapeSet;

	///{INTERFACE TO bCharacterSim
	///Pointer to simulated character
	bCharacterSim* m_sim;

	/// Configuration of CoM frame (used by Bullet) wrt body frame (used by MBS). 
	/**
	* Bullet's convention is to fix reference frame at CoM, aligned to the principal axis.  \n
	* While MBS doesn't have such restriction, it usually fixes reference frame at the joints. \n
	* m_localCoMFrames relates reference frames used by Bullet and MBS. \n
	* m_localCoMFrames[i] is the bullet body[i]'s body frame as seen from the link[i]'s body frame.
	*/
	gXMat* m_localCoMFrames; 
	
	/// Inverse of m_localCoMFrames
	gXMat* m_localCoMFramesInv; 
	///}END OF INTERFACE TO bCharacterSim

public:
	
	bCharacter();
	
	virtual ~bCharacter();
	
	/// Get pointer to CharacterSim
	inline bCharacterSim* getCharacterSim() { return m_sim; }

	/// Setup CharacterSim corresponding to this
	/*** This funtion must be called after MBS multibodySystem has been initialized.
	*/
	//int setupBulletBodies(bCharacterSim* sim, btDynamicsWorld* ownerWorld, const btVector3& positionOffset);
	int setupCharacterSim(bCharacterSim* sim, btDynamicsWorld* ownerWorld, const btVector3& positionOffset,	int collisionGroup = 1, int collideWithMask = 2, double frictinCoeff = 0.99,	double restitutionCoeff = 0.0,	double jointParamStopERP = 0.8,	double jointParamStopCFM = 0.0,	double jointParamERP = 0.8,	double jointParamCFM = 0.0);

	/// initialization after loading	
	//virtual void postLoading(const char* NAME_LEFT_FOOT, const char* NAME_RIGHT_FOOT); //deprecated
	virtual void postLoading();

	/// Recalculate inertia using collision shapes and update inertia of btRigidBody and MBS links. \n
	/// This function assumes that m_btShapes are located at m_btBodies' CoM.
	void recalculateInertiaUsingCollisionShapesAndUpdateMBS(void);
	//void recalculateInertiaAndUpdateMBS(void);

	/// Redistribute the segment weight based on cadaver work.
	void reDistributeSegmentWeight( std::map<std::string, double> &weightProportion, double totalWeightKg );

	/// Initialize foot geometry
	void initializeFootGeometryInBoxShape();

	/// Update mbs' states according to btBodies
	/***
	option: currently not used.
	*/
	//void updateMBSStatesFromBtBodies(int option=0);
	void updateStatesFromCharacterSim(int option=0);

	/// Update xform of kinematic bodies of CharacterSim from MBS
	//void updateKinematicBtBodiesFromMBS(void);
	void updateKinematicBodiesOfCharacterSim(void);

	/// Forcefully update xform of bodies of CharacterSim from MBS	(shouldn't be called during simulation)
	void enforcePoseToCharacterSim(void);


	///Set force to CharacterSim according to mbs
	/*** 
	* applyWrenchToBaselink: if true, the first six components of the generalized force are applied to the base link. \n
	* option: 0 = joint torques are applied as impulses to CharacterSim (default), 1 = joint torques are applied as torques to CharacterSim.\n
	*/
	void applyJointTorquesToCharacterSim(double timeStep, bool applyWrenchToBaselink=false, int option=0);

	///Set force and torque to CharacterSim according to mbs. This is for debugging purpose.
	///wrench(I): input wrench, wrt base frame.
	void applyBaseLinkWrenchToCharacterSimForDebug(const gWrench& wrench);

	/// 2012-02-15 : detect foot contact and measure GRF/CoP
	void measureFootContactForce( double timeStep );

	/// detect foot contact. GRF/CoP are not measured.
	void detectFootContact();
	

	/// grf and cop
	inline gVec3 getLGRF(){ return m_lgrf; };
	inline gVec3 getLCOP(){ return m_lcop; };
	inline gVec3 getRGRF(){ return m_rgrf; };
	inline gVec3 getRCOP(){ return m_rcop; };
	
	/// foot geometry info
	gFootGeom& getLFootGeom(){ return m_lFootGeom; }
	gFootGeom& getRFootGeom(){ return m_rFootGeom; }

	/// get foot/hand link
	gLink* getLFootLink() { return m_lFootLink; }
	gLink* getRFootLink() { return m_rFootLink; }
	gLink* getLHandLink() { return m_lHandLink; }
	gLink* getRHandLink() { return m_rHandLink; }

	/// friction coefficients (must be determined from the environment)
	double m_lFootMuF; //left foot linear friction coeff
	double m_lFootMuT; //left foot angular friction coeff
	double m_rFootMuF; //right foot linear friction coeff
	double m_rFootMuT; //right foot linear friction coeff

	/// Get localCoMFrames
	gXMat* getLocalCoMFrames(){ return m_localCoMFrames; }
	gXMat* getLocalCoMFramesInv(){ return m_localCoMFramesInv; }

	inline bool	isLeftFootContact(){ return m_isLFootContact; }
	inline bool	isRightFootContact(){ return m_isRFootContact; }

	inline bool	isLeftFootContactStable(){ return m_isLFootContact; }
	inline bool	isRightFootContactStable(){ return m_isRFootContact; }

	int getNumContactPoints( bCharacter *sys, gLink *link );

	//inline int getNumContactsLeft(){ gLink* link = findLink("L_FOOT"); return getNumContactPoints(this, link); }
	//inline int getNumContactsRight(){ gLink* link = findLink("R_FOOT"); return getNumContactPoints(this, link); }


	bShapeSet* getOrCreateVisShapeSet();
	

protected:		// internal methods will be placed here
	
	//rb: source rigid body from mbs
	//localCoMFrame: configuration of CoM frame wrt body frame of rb
	//shape: collision shape
	//positionOffset: position offset wrt world frame
	//collisionGroup: collision group that this rigid body belongs to
	//collideWithMask: mask for collisiongroup to check collision with this rigid body
	btRigidBody* createRigidBodySim (gRigidBody& rb, gXMat& localCoMFrame, btCollisionShape* shape, const btVector3& positionOffset, int collisionGroup, int collideWithMask);

	
};



#endif