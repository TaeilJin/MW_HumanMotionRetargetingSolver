//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

//-----------------------------------------------------------
// MBS_FORCES.H
//
// author: Sung-Hee Lee	
//-----------------------------------------------------------
//
// This file contains various force generating components such as joint springs
//
//-----------------------------------------------------------

#ifndef _MBS_FORCES_H_
#define _MBS_FORCES_H_

#include "Base/gMath.h"
#include "gMbsUtil.h"
#include "gRigidBodies.h"

#define FORCE_TYPE_1D_LINK_SPRING		0
#define FORCE_TYPE_2D_LINK_SPRING		1
#define FORCE_TYPE_BALL_LINK_SPRING		2

#define FORCE_TYPE_1D_LINK_LIMIT_SPRING		3
#define FORCE_TYPE_2D_LINK_LIMIT_SPRING		4	
#define FORCE_TYPE_BALL_LINK_LIMIT_SPRING	5

#define FORCE_TYPE_GRAVITY			6
#define FORCE_TYPE_GROUND_CONTACT	7

#define FORCE_TYPE_LINE_EXPONENTIAL_SPRING 8
#define FORCE_TYPE_PLS_EXPONENTIAL_SPRING 9
#define FORCE_TYPE_LINE_LINEAR_SPRING 10

#define FORCE_TYPE_MUSCLE_LINEAR_HILL 11

#define	FORCE_TYPE_STATE_INDEPENDENT_FORCE 12

class		gSpring;
class		gLineSpring;
class		gForcePoint;

typedef std::vector<gForcePoint*> gForcePointList;

// -----------------------------------------------------------------
// class gAttachedPoint: 
//	a point attached to gRigidBody. 
// -----------------------------------------------------------------
class gAttachedPoint:public gObject
{
public:				
			 gAttachedPoint(void) {_pBody = NULL;};
			 gAttachedPoint(gRigidBody* pBody, const gVec3& posLocal) { _pBody = pBody; _posLocal = posLocal; }
	virtual	~gAttachedPoint(void) {};
	//{ get/set
	inline	gRigidBody*		body(void) const		{ return _pBody; }
	inline  const gVec3&	posLocal(void) const	{ return _posLocal; }
	inline	const gVec3&	posWorld(void) const	{ return _posWorld; }
	inline	const gVec3&	velWorld(void) const	{ return _velWorld; }
	inline	const gVec3&	accWorld(void) const	{ return _accWorld; }
			void setLocalPositionInLocal(gRigidBody* pBody, const gVec3& posLocal) { _pBody = pBody; _posLocal = posLocal; }
			void setLocalPositionInWorld(gRigidBody* pBody, const gVec3& posWorld) { _pBody = pBody; _posLocal = pBody->frame().invMultVec4(posWorld); }
	//}
	//{ kinematics updater
	virtual	void		updateKinematicsUptoPos(void);
	virtual	void		updateKinematicsUptoVel(void);
	virtual void		updateKinematicsUptoAcc(void)
	{
		updateKinematicsUptoVel();
		_accWorld = _pBody->frame().multVec3( _pBody->frameVel().multVec3( _pBody->frameVel().multVec4(_posLocal) ) + 
			_pBody->frameVel().multVec4(_posLocal) );
	}
	//}
protected:
	//{ interface
	gVec3				_posWorld;	// position wrt world frame
	gVec3				_velWorld;	// velocity wrt world frame
	gVec3				_accWorld;  // acceleration wrt world frame
	//}
	gRigidBody*			_pBody;		// parent rigid body
	gVec3				_posLocal;  // position wrt body frame
#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	void	updateKVariablesP(void)
	{
		_posWorldP = _pBody->frame().multVec3( _pBody->_Tp.multVec4(_posLocal) );
		_velWorldP = _pBody->frame().multVec3( _pBody->_Tp.multVec3(_pBody->frameVel().multVec4(_posLocal)) + _pBody->_Vp.multVec4(_posLocal) );
	}
	gVec3	_posWorldP, _velWorldP;

#endif
};

// -----------------------------------------------------------------
// class gForcePoint: 
//	an attached point that can exert a force to a rigid body 
// -----------------------------------------------------------------
class gForcePoint:public gAttachedPoint
{
public:
			 gForcePoint() {}
			 gForcePoint(gRigidBody* pBody, const gVec3& posLocal):gAttachedPoint(pBody,posLocal){}
	virtual ~gForcePoint(){}
	//{ get/set
	virtual	void	setForceInWorld(const gVec3& forceInWorld) { _forceLocal = _pBody->frame().invMultVec3(forceInWorld); }
	virtual	void	addForceInWorld(const gVec3& forceInWorld) { _forceLocal += _pBody->frame().invMultVec3(forceInWorld); }
	virtual	void	clearForce(void) { _forceLocal.setZero(); }
	virtual gVec3&  forceLocal(void) { return _forceLocal; }
	//}
	//{ add force to gRigidBody->_Fext
	virtual	void	addForceToBody(void) const { _pBody->addExtForce(gWrench(_posLocal%_forceLocal,_forceLocal)); }
	//}
protected:
	gVec3			_forceLocal; // output: force wrt body frame

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	gVec3			_forceLocalP; 
	virtual	void	addForcePToBody(void) const { _pBody->_Ep += gWrench(_posLocal%_forceLocalP,_forceLocalP); }
	virtual void	setForcePInWorld(const gVec3& forceWorldP) { _forceLocalP = _pBody->frame().invMultVec3(forceWorldP); }
	virtual void	addForcePInWorld(const gVec3& forceWorldP) { _forceLocalP += _pBody->frame().invMultVec3(forceWorldP); }
#endif

};

// -----------------------------------------------------------------
// class gForce 
// base class of all force generating elements 
// It has three derived classes: gStateDependentForce, gStateIndependentForce, gCoordSpring
// -----------------------------------------------------------------
class gForce:public gObject
{
public:
	virtual int		type(void) const		=0;
	virtual void	addForceToBody(void)	=0;	// add force to gRigidBody->_Fext
};

// -----------------------------------------------------------------
// class gStateIndependentForce: 
// a force that does not depend on the state of the system
// e.g., arbitrary user input force
// -----------------------------------------------------------------
class gStateIndependentForce:public gForce
{
public:

	gStateIndependentForce(){}
	
	int type(void) const
	{ 
		return FORCE_TYPE_STATE_INDEPENDENT_FORCE; 
	}
	
	void setForceInLocalAtPosInLocal(gRigidBody* pBody, const gVec3& posLocal, const gVec3& forceLocal)
	{
		_pBody = pBody;
		_posLocal = posLocal;
		_forceLocal = forceLocal;
	}

	void setForceInWorldAtPosInLocal(gRigidBody* pBody, const gVec3& posLocal, const gVec3& forceWorld)
	{
		_pBody = pBody;
		_posLocal = posLocal;
		_forceLocal = _pBody->frame().invMultVec3(forceWorld);			
	}

	void setForceInWorldAtPosInWorld(gRigidBody* pBody, const gVec3& posWorld, const gVec3& forceWorld)
	{
		_pBody = pBody;
		_posLocal = _pBody->frame().invMultVec4(posWorld);
		_forceLocal = _pBody->frame().invMultVec3(forceWorld);			
	}

	void setForceInLocalAtPosInWorld(gRigidBody* pBody, const gVec3& posWorld, const gVec3& forceLocal)
	{
		_pBody = pBody;
		_posLocal = _pBody->frame().invMultVec4(posWorld);
		_forceLocal = forceLocal;
	}

	virtual void addForceToBody()
	{ 
		_pBody->addExtForce(gWrench(_posLocal%_forceLocal,_forceLocal)); 		
	}

protected:
	gRigidBody*	_pBody;	//target rigid body
	gVec3		_posLocal; //position in body coordinate that a force is applied
	gVec3		_forceLocal;//force in body coordinate	
};

// -----------------------------------------------------------------
// class gStateDependentForce 
// a class of forces whose forces are dependent on the state of the multibody system
// e.g., gravity, ground contact force
// -----------------------------------------------------------------
class gStateDependentForce:public gForce
{
public:
	virtual int		type(void) const		=0;
	virtual	void	computeForce(void)		=0;	// compute force
	virtual	void	updateKinematicsUptoPos(void)=0;
	virtual	void	updateKinematicsUptoVel(void)=0;

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	updateKVariablesP(void)	=0;	// update derivative of variables
	virtual void	computeForceP(void)		=0;	// compute derivative of forces
	virtual void	addForcePToBody(void)	=0;	// add force derivative to a rigid body
#endif
};

class gGravityForce:public gStateDependentForce
{
public:
	virtual int		type(void) const { return FORCE_TYPE_GRAVITY; }
	void setRigidBody(gRigidBody* body) 
	{ 
		_pBody = body;
	}
	virtual void updateKinematicsUptoPos() {}
	virtual void updateKinematicsUptoVel() {}
	virtual void addForceToBody(){ 
		/*gReal re[3];
		gCROSSeq(re,_pBody->inertia().com(),_forceLocal.cptr());
		_pBody->addExtForce(gWrench(re,_forceLocal.cptr()));*/
		_pBody->addExtForce(gWrench(_pBody->inertia().comInVec3()%_forceLocal,_forceLocal)); 		
	}
	virtual void computeForce();

protected:
	gRigidBody*	_pBody;	
	gVec3		_forceLocal;//gravity force in body coordinate

#ifdef MW_USE_DIFF_NEWTON_EULER
	gVec3		_forceLocalP;
public:
	virtual void	updateKVariablesP(void)	{}
	virtual void	computeForceP(void)		{ _forceLocalP = _forceLocal % _pBody->_Tp.rot(); }
	virtual void	addForcePToBody(void)	{ 
		//_pBody->_Ep += gWrench(_com%_forceLocalP,_forceLocalP); 
		gReal re[3];
		gCROSSeq(re,_pBody->inertia().com(),_forceLocalP.cptr());
		_pBody->_Ep += gWrench(re,_forceLocalP.cptr());
	}
#endif

};

class gContactForceFlatGround:public gStateDependentForce
{
public: 
	virtual int type(void)  const { return FORCE_TYPE_GROUND_CONTACT; }

	void addContactPointInLocal(gRigidBody* body, gVec3& posLocal)
	{
		gForcePoint* fp = new gForcePoint();
		fp->setLocalPositionInLocal(body,posLocal);
		_points.push_back(fp);
	}

	void addContactPointInWorld(gRigidBody* body, gVec3& posWorld)
	{
		gForcePoint* fp = new gForcePoint();
		fp->setLocalPositionInWorld(body,posWorld);
		_points.push_back(fp);
	}

	void addContactPoint(gForcePoint* fp){ _points.push_back(fp); }

	virtual void updateKinematicsUptoPos() 
	{ 
		for(gForcePointList::iterator it = _points.begin(); it != _points.end(); it++)
		{
			(*it)->updateKinematicsUptoPos();
		}
	}
	
	virtual void updateKinematicsUptoVel()
	{ 
		for(gForcePointList::iterator it = _points.begin(); it != _points.end(); it++)
		{
			(*it)->updateKinematicsUptoVel();
		}
	}

	virtual void computeForce()
	{
		gReal Ks = 1e1;
		gReal Kd = 1*Ks;
		bool contact = false;
		int idx = 0;
		gReal height = -1;
		for(gForcePointList::iterator pit=_points.begin();pit!=_points.end();pit++)
		{
			gReal fy=0;
			gReal posy=(*pit)->posWorld().y();
			gReal vely = (*pit)->velWorld().y();
			if(posy < height)
			{
				gReal velx = (*pit)->velWorld().x();
				gReal velz = (*pit)->velWorld().z();

				gReal fyv = Kd*(0-vely);
				//fzv = gMin(fzv,1);
				fy = Ks*(0-posy) + fyv; 
				fy = gMax(0,fy);
				
				(*pit)->setForceInWorld(gVec3(0.1*Kd*(-velx),fy,0.1*Kd*(-velz)));
			}
			else
			{
				(*pit)->clearForce();
			}
		}
	}

	virtual void addForceToBody()
	{
		for(gForcePointList::iterator it = _points.begin(); it != _points.end(); it++)
		{
			(*it)->addForceToBody();
		}
	}

protected:
	std::vector<gForcePoint*> _points;

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	updateKVariablesP(void)
	{
		for(gForcePointList::iterator it = _points.begin(); it != _points.end(); it++)
		{
			(*it)->updateKVariablesP();
		}
	}

	virtual void	addForcePToBody(void)
	{
		for(gForcePointList::iterator it = _points.begin(); it != _points.end(); it++)
		{
			(*it)->addForcePToBody();
		}
	}

	virtual void	computeForceP(void)
	{
		gReal Ks = 1e5;
		gReal Kd = 0.1*Ks;
		bool contact = false;
		int idx = 0;
		for(gForcePointList::iterator pit=_points.begin();pit!=_points.end();pit++)
		{
			gReal fz=0;
			gReal posz=(*pit)->posWorld().z();
			gReal velz = (*pit)->velWorld().z();
			gReal poszP = (*pit)->_posWorldP.z();
			gReal velzP = (*pit)->_velWorldP.z();
			if(posz<0)
			{
				gReal fzv = Kd*(0-velz);
				//fzv = gMin(fzv,1);
				fz = Ks*(0-posz) + fzv; 
				fz = gMax(0,fz);

				gReal fzP=0;
				if(fz > 0){
					fzP = -Ks*poszP -Kd*velzP;
				}

				gReal velx = (*pit)->velWorld().x();
				gReal vely = (*pit)->velWorld().y();
				gReal velxP = (*pit)->_velWorldP.x();
				gReal velyP = (*pit)->_velWorldP.y();

				gReal fxP = 0.1*Kd*(-velxP);
				gReal fyP = 0.1*Kd*(-velyP);
				
				(*pit)->setForcePInWorld(gVec3(fxP,fyP,fzP));

			}
			else
			{
				(*pit)->_forceLocalP.setZero();
			}
		}
	}
#endif
};

// -----------------------------------------------------------------
// class gCoordSpring: virtual class for spring element that applies to coordinates of gLink
// It defines angular spring for a revolue joint, linear spring for a prismatic joint, etc.
// this is also a state dependent force, but defined separately because of its unique nature
// -----------------------------------------------------------------
class gCoordSpring:public gForce
{
public:
	virtual gLink*	link(void)				=0;
	virtual	void	setLink(gLink*)			=0;
	virtual void	setStiffness(int,gReal) =0;
	virtual void	setDamping(int,gReal)	=0;
	virtual	void	computeForce(void)		=0;	// compute force
	

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	computeForceP(void)		=0;	// compute derivative of forces
	virtual void	addForcePToBody(void)	=0;	// add force derivative to a rigid body
#endif
};




// -----------------------------------------------------------------
// class g1dLinkSpring: virtual class for angular spring
// restAngle and limit around it are defined
// -----------------------------------------------------------------
class g1dLinkSpring:public gCoordSpring{
public:
	virtual	int			type(void) const			{ return FORCE_TYPE_1D_LINK_SPRING; }
						g1dLinkSpring(void)			{ _ks = _kd = 0; _q0=0; _force=0; }
	//{ get/set
			gLink*		link(void)					{ return _pLink; }
	inline	gReal		stiffness(void)				{ return _ks; }
	inline	gReal		damping(void)				{ return _kd; }
	virtual	void		setLink(gLink* pLink)		{ _pLink=dynamic_cast<g1dLink*>(pLink); assert(_pLink != NULL); }
	inline	void		setRestCoord(gReal q0)		{ _q0 = q0; }	
	virtual void		setStiffness(int idx, gReal ks)	{ assert( idx== 0 ); _ks = ks; }
	inline  void		setStiffness(gReal ks)		{ _ks = ks; }
	virtual void		setDamping(int idx, gReal kd)	{ assert( idx== 0 ); _kd = kd; }
	inline	void		setDamping(gReal kd)		{ _kd = kd; }
	//}
	//{ force functions
	//virtual	void		computeForce(void)=0;
	virtual	void		addForceToBody(void);
	//}
protected:
	//{ interface
	gReal			_force;		// spring torque
	//}
	g1dLink*		_pLink;		// Spring is inserted between _pLink and its parent
	gReal			_q0;		// rest coordinate
	gReal			_ks,_kd;	// spring coeff. and damping coeff.
#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	gReal			_forceP;

	virtual void	computeForceP(void)		=0;	// compute derivative of forces
	virtual void	addForcePToBody(void)
	{
		gWrench fP(_pLink->screw()*_forceP); // project joint torque to Fext (beware of sign)
		_pLink->_Ep += fP;
		_pLink->parent()->_Ep -= fP.xform(_pLink->localFrame());
	}
#endif
};

// -----------------------------------------------------------------
// class g1dLinkLinearSpring
// rotational spring, linear stiffness
// -----------------------------------------------------------------
class g1dLinkLinearSpring:public g1dLinkSpring
{
public:
	virtual	void	computeForce(void)
	{	
		_force = -_ks*(_pLink->coord() - _q0) - _kd*_pLink->genVel(); 
	}
#ifdef MW_USE_DIFF_NEWTON_EULER
	virtual void	computeForceP(void)
	{
		_forceP = -_ks*_pLink->coordP(0) - _kd*_pLink->genVelP(0);
	}
#endif
};

// -----------------------------------------------------------------
// class g1dLinkLimitSpring
// g1dLinkSpring that has joint limits
// -----------------------------------------------------------------
class g1dLinkLimitSpring:public g1dLinkSpring
{
public:
	virtual	int	type(void) const				  { return FORCE_TYPE_1D_LINK_LIMIT_SPRING; }
				g1dLinkLimitSpring(void)		  { _limit[0] = _limit[1] =0; _ksb=0; }
	//{ get/set
	gReal		jointLimit(int i)				  { return _limit[i]; }
	void		setJointLimitStiffness(gReal ksb) { _ksb = ksb; }	
	void		setJointLimitPositive(gReal pos) { _limit[0] = pos; }
	void		setJointLimitNegative(gReal neg) { _limit[1] = neg; }
	//}
	void		computeForce(void)
	{
		gReal ang = _pLink->coord();
		_force = -_kd*_pLink->genVel() - _ks * ( ang - _q0 );
		if( ang  > _limit[0] )
			_force -=  _ksb*gSqr( ang - _limit[0]);
		else if ( ang < _limit[1] )
			_force += _ksb*gSqr( ang - _limit[1]);
	}

protected:
	gReal		_limit[2];	// zone: free motion zone, 0: -ve, 1: +ve
	gReal		_ksb;		// stiffness off limit
#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	computeForceP(void)
	{
		gReal ang = _pLink->coord();
		gReal angP = _pLink->coordP(0);
		_forceP = -_kd*_pLink->genVelP(0) - _ks * angP;
		if( ang > _limit[0] ){
			_forceP -= _ksb*2*(ang-_limit[0])*angP;
		}
		else if( ang < _limit[1] ){
			_forceP +=  _ksb*2*(ang-_limit[1])*angP;
		}
	}
#endif
};

// -----------------------------------------------------------------
// class g2dLinkSpring
// class for Angular Spring for g2dLink 
// -----------------------------------------------------------------
class g2dLinkSpring:public gCoordSpring
{
public:
	virtual	int		type(void) const				{ return FORCE_TYPE_2D_LINK_SPRING; }
					g2dLinkSpring(void)				{_pLink = NULL; _force[0]=_force[1]=0; _q0[0]=_q0[1]=0; }
	//{ get/set
			gLink*	link(void)						{ return _pLink; }
			void	setLink(gLink* pLink)			{ _pLink=(g2dLink*)pLink; }
	inline	void	setRestCoord(int idx, gReal oriAngle){ _q0[idx] = oriAngle; }	
	virtual void	setStiffness(int idx, gReal ks)	{ assert( idx < 2 ); _ks[idx] = ks; }
	virtual void	setDamping(int idx, gReal kd)	{ assert( idx < 2 ); _kd[idx] = kd; }
			gReal	force(int i)					{ assert(i < DOF_2D_LINK); return _force[i]; }
	//}
	//{ force functions
	//virtual void	computeForce(void)=0;
	void			addForceToBody(void)
	{	
		gWrench F = gWrench( _pLink->current1stScrew()*_force[0] + _pLink->screw(1)*_force[1] );
		_pLink->addExtForce(F);
		_pLink->parent()->subtractExtForce(F.xform(_pLink->localFrame()));
	}
	//}
protected:
	//{ interface
	gReal			_force[DOF_2D_LINK];	// spring torque
	//}
	g2dLink*		_pLink;					// parent link	
	gReal			_q0[DOF_2D_LINK];
	gReal			_kd[DOF_2D_LINK], _ks[DOF_2D_LINK];
#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	void			computeForceP(void)=0;
	void			addForcePToBody(void)
	{	
		gWrench fp = gWrench( _pLink->current1stScrew()*_forceP[0] + _pLink->screw(1)*_forceP[1] );
		_pLink->_Ep += fp;
		_pLink->parent()->_Ep -= fp.xform(_pLink->localFrame());
	}

	gReal _forceP[DOF_2D_LINK];
#endif
};

// -----------------------------------------------------------------
// class g2dLinkLimitSpring
// -----------------------------------------------------------------
class g2dLinkLimitSpring:public g2dLinkSpring
{
public:
	virtual	int			type(void) const							{ return FORCE_TYPE_2D_LINK_LIMIT_SPRING; }
						g2dLinkLimitSpring(void)					{ _ksb[0] = _ksb[1] = 0;_kd[0]=_kd[1]=0; _ks[0]=_ks[1]=0; }	
	//{ get/set
	gReal				jointLimit(int i,int j)						{ return _limit[i][j]; }
	inline  void		setJointLimitStiffness(int idx, gReal ksb)	{ _ksb[idx] = ksb; }	
	inline	void		setJointLimitPositive(int idx, gReal z)		{ _limit[idx][0] = z; }
	inline	void		setJointLimitNegative(int idx, gReal z)		{ _limit[idx][1] = z; }
	//}
	//{ force functions
	void				computeForce(void)
	{
		for(int i=0;i<DOF_2D_LINK;++i)
		{
			gReal ang = _pLink->coord(i);
			gReal vel = _pLink->genVel(i);
			
			_force[i] = -_kd[i]*vel - _ks[i]*(ang - _q0[i]);
			
			if(ang > _limit[i][0]){
				_force[i] -= _ksb[i]*gSqr(ang-_limit[i][0]);
			}
			else if(ang < _limit[i][1]){
				_force[i] += _ksb[i]*gSqr(ang-_limit[i][1]);
			}
		}
	}
	//}
protected:
	gReal		_limit[DOF_2D_LINK][2];	
	gReal		_ksb[DOF_2D_LINK];
#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	computeForceP(void)
	{
		for(int i=0;i<DOF_2D_LINK;++i)
		{
			gReal ang = _pLink->coord(i); 
			gReal angP = _pLink->coordP(i);
			_forceP[i] = -_kd[i]*_pLink->genVelP(i) - _ks[i]*angP;
			if( ang > _limit[i][0] ){
				_forceP[i] -= _ksb[i]*2*(ang-_limit[i][0])*angP;
			}
			else if(ang < _limit[i][1]){
				_forceP[i] += _ksb[i]*2*(ang-_limit[i][1])*angP;
			}
		}
	}
#endif
};

// -----------------------------------------------------------------
// class gBspring: class for Angular Spring for gBallLink (Ball Joint)
// -----------------------------------------------------------------
class gBallLinkSpring:public gCoordSpring
{
public:
	virtual	int		type(void) const				{ return FORCE_TYPE_BALL_LINK_SPRING; }
					gBallLinkSpring(void)			{ _pLink = NULL; _force[0]=_force[1]=_force[2]=0; }
	//{ get/set
			gLink*	link(void)						{ return _pLink; }
			void	setLink(gLink* pLink)			{ _pLink=(gBallLink*)pLink; }
	virtual void	setDamping(int idx, gReal kd)	{ assert(idx<3); _kd[idx] = kd; }
	virtual	void	setStiffness(int idx, gReal ks)	{ assert(idx<3); _ks[idx] = ks; }
	//}
	//{ force functions
	//virtual void	computeForce(void)=0;
	void			addForceToBody(void);
	//}
protected:
	//{ interface
	gReal			_force[DOF_BALL_LINK];		// spring torque
	//}
	gBallLink*		_pLink;		// parent link	
	gReal _ks[DOF_BALL_LINK];
	gReal _kd[DOF_BALL_LINK];

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	void			computeForceP(void)=0;
	void			addForcePToBody(void)
	{	
		gWrench fp(_forceP[0],_forceP[1],_forceP[2],gZero,gZero,gZero);
		_pLink->_Ep += fp;
		_pLink->parent()->_Ep -= fp.xform(_pLink->localFrame());
	}
	gReal _forceP[DOF_BALL_LINK];
#endif
};

// -----------------------------------------------------------------
// class gBallLinkLimitSpring
// -----------------------------------------------------------------
class gBallLinkLimitSpring: public gBallLinkSpring
{
public:
	virtual	int			type(void) const							{ return FORCE_TYPE_BALL_LINK_LIMIT_SPRING; }
	gBallLinkLimitSpring(){		
		memset(_kd,0,sizeof(gReal)*DOF_BALL_LINK);
		memset(_ks,0,sizeof(gReal)*DOF_BALL_LINK);
		memset(_ksb,0,sizeof(gReal)*DOF_BALL_LINK);
	}
	//{ get/set
			gReal		jointLimit(int i,int j)						{ return _limit[i][j]; }	
	inline  void		setJointLimitStiffness(int idx, gReal ksb)	{ _ksb[idx] = ksb; }	
	inline	void		setJointLimitPositive(int idx, gReal z)		{ _limit[idx][0] = z; }
	inline	void		setJointLimitNegative(int idx, gReal z)		{ _limit[idx][1] = z; }
	//}
	//{ force functions
	void				computeForce(void)
	{
		gVec3 angles = gVec3::log(_pLink->coord());
		for(int i=0;i<DOF_BALL_LINK;++i)
		{	
			_force[i] = -_kd[i]*_pLink->localFrameVel().e(i) -_ks[i]*angles.e(i);
			
			if(angles.e(i) > _limit[i][0])
				_force[i] -= _ksb[i]*gSqr(angles.e(i)-_limit[i][0]);
			else if(angles.e(i) < _limit[i][1])
				_force[i] += _ksb[i]*gSqr(angles.e(i)-_limit[i][1]);
		}		
	}
	//}
protected:
	gReal _ksb[DOF_BALL_LINK];
	gReal _limit[DOF_BALL_LINK][2];

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	computeForceP(void)
	{
		gVec3 angles = gVec3::log(_pLink->coord());
		for(int i=0;i<DOF_BALL_LINK;++i)
		{
			_forceP[i] = -_kd[i]*_pLink->genVelP(i) - _ks[i]*_pLink->coordP(i);

			if(angles.e(i) > _limit[i][0])
				_forceP[i] -= _ksb[i]*2*(angles.e(i)-_limit[i][0])*_pLink->coordP(i);
			else if(angles.e(i) < _limit[i][1])
				_forceP[i] += _ksb[i]*2*(angles.e(i)-_limit[i][1])*_pLink->coordP(i);

		}
	}
#endif
};

// -----------------------------------------------------------------
// class gSpring: virtual class for line spring element
//
// note that spring's ks (and maybe kd )is proportional to cross-sectional area,
// and inversely proportional to length. 
// i.e. ks = AE/L , A: area, L: length, E: young's modulus 
// -----------------------------------------------------------------
class gSpring : public gStateDependentForce
{
public:
	gSpring(void)	{ _ks = _kd = 0; _len0 = _len = _dlen = 0; }

	//{ get/set
	inline	gReal	restLength(void)		{ return _len0; }
	inline	gReal	length(void)			{ return _len; }
	inline	gReal	lengthChangeRate(void)	{ return _dlen; }
	inline	gReal	stiffness(void)			{ return _ks; }
	inline	gReal	damping(void)			{ return _kd; }
	inline  void	setStiffness(gReal ks)	{ _ks = ks; }
	inline	void	setDamping(gReal kd)	{ _kd = kd; }
	//}
	//{ force functions
	//virtual	void	computeForce(void)=0;	// solve force applied at each gForcePoint wrt gRigidBody's local coord.
	virtual	void	addForceToBody(void)=0; // add force to gRigidBody->Fext
	//}
	//{ kinematics updaters
	virtual void	updateKinematicsUptoPos(void)=0;  
	virtual void	updateKinematicsUptoVel(void)=0;
	//}
	virtual void	computeRestLength(gReal scale)=0;	
	virtual void	computeLengthAndItsChangeRate(void)=0;
protected:
	gReal			_len0;			// original length
	gReal			_len, _dlen;	// length and its change rate
	gReal			_ks,_kd;		// spring coeff. and damping coeff.

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	updateKVariablesP(void)	=0;	// compute derivative of variables
	virtual void	computeForceP(void)		=0;	// compute derivative of forces
	virtual void	addForcePToBody(void)	=0;	// add force derivative to a rigid body

	gReal			_lenP, _dlenP;	// derivative of _len, _dlen
#endif
};

// -----------------------------------------------------------------
// class gLineSpring: virtual class for Line Spring
//
// gLineSpring has two gForcePoint's at each end, which are
// be attached to gRigidBody
// -----------------------------------------------------------------
class gLineSpring:public gSpring
{
public:
	gLineSpring(void)	{}

	//{ get/set
	inline	gForcePoint&		pointL(void)		{ return _ptL; }
	inline	gForcePoint&		pointR(void)		{ return _ptR; }
	//}

	// connect to two gRigidBody, left and right. lPos and rPos are in body coordinate.
	void				setLinkInLocal(gRigidBody* left, const gVec3& lPos, gRigidBody* right, const gVec3& rPos) 
	{ 
		_ptL.setLocalPositionInLocal(left, lPos); 
		_ptR.setLocalPositionInLocal(right,rPos);	
	}
	// connect to two gRigidBody, left and right. lPos and rPos are in world coordinate.
	void				setLinkInWorld(gRigidBody* left, const gVec3& lPos, gRigidBody* right, const gVec3& rPos) 
	{
		_ptL.setLocalPositionInWorld(left, lPos);
		_ptR.setLocalPositionInWorld(right, rPos);
	}
	// set (current_length*scale) as original length. scale defines initial tension setting.
	virtual void		computeRestLength(gReal scale=1.0) 
	{ 
		_len0 = scale*gDistance(_ptL.posWorld(),_ptR.posWorld());
		assert( _len0 > gEPSILON );
	}
	virtual void		computeLengthAndItsChangeRate(void)
	{
		gVec3 x = _ptR.posWorld()-_ptL.posWorld();
		gVec3 v = _ptR.velWorld()-_ptL.velWorld();
		_len = x.magnitude();
		_dlen = ( _len < gEPSILON ? v.magnitude() : (v,x)/_len );	
	}

	//{ kinematics updater
	virtual void		updateKinematicsUptoPos(void)
	{	
		_ptR.updateKinematicsUptoPos();
		_ptL.updateKinematicsUptoPos(); 
		_len = (_ptR.posWorld()-_ptL.posWorld()).magnitude();
	}	
	virtual void		updateKinematicsUptoVel(void)
	{
		_ptR.updateKinematicsUptoVel();
		_ptL.updateKinematicsUptoVel(); 
		computeLengthAndItsChangeRate();
	}	

	//}
	// compute force and assign to gForcePoint
    //void				computeForce(void)=0;
	// apply force to gRigidBody->Fext
	virtual void		addForceToBody(void)
	{
		_ptL.addForceToBody();
		_ptR.addForceToBody();
	}	

protected:
	gForcePoint	_ptL, _ptR;		// two end points 
	gReal _force;				// magnitude of force
	gVec3 _forceVector;			// force vector

#ifdef MW_USE_DIFF_NEWTON_EULER

	virtual void	updateKVariablesP(void)
	{
		_ptR.updateKVariablesP();
		_ptL.updateKVariablesP();

		gVec3 x = _ptR.posWorld() - _ptL.posWorld();
		gVec3 v = _ptR.velWorld() - _ptL.velWorld();
		gVec3 xp = _ptR._posWorldP - _ptL._posWorldP;
		gVec3 vp = _ptR._velWorldP - _ptL._velWorldP;
		
		_lenP = (x,xp)/_len;
		_dlenP = ( _len < gEPSILON ? (v,vp)/_dlen : ( (x,vp)+(v,xp)-_dlen*_lenP )/_len );
	}

	virtual void	addForcePToBody(void)	// add force derivative to a rigid body
	{
		_ptL.addForcePToBody();
		_ptR.addForcePToBody();
	}

#endif
};

// -----------------------------------------------------------------
// class gLineLinearSpring: Linear Spring
//
// ks,kd: should be set to proportional to cross sectional area,
// and inversely proportional to length of the spring 
// (ks = AE/L, where A:cross-sect. area, E: Young's modulus, L: length of the spring)
// fs = ks*(l-lo)
// -----------------------------------------------------------------
class gLineLinearSpring:public gLineSpring
{
public:
	virtual int type(void)  const { return FORCE_TYPE_LINE_LINEAR_SPRING; }
	void		computeForce(void)
	{
		_force = _ks*(_len-_len0) + _kd*_dlen;
		if(_len > gEPSILON) {
			_forceVector = (_ptR.posWorld() - _ptL.posWorld())*(_force/_len);
		}else if( !IsZero(_dlen) ) {
			_forceVector =  (_ptR.velWorld() - _ptL.velWorld())*(_force/_dlen);
		}else {
			_forceVector.setZero();
		}
		_ptL.setForceInWorld(_forceVector);
		_ptR.setForceInWorld(-_forceVector);
	}

protected:


#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	computeForceP(void)
	{
		gReal forceP = _ks*_lenP + _kd*_dlenP;
		gVec3 forcePv(0,0,0);
		if(_len>gEPSILON){
			forcePv = ( (_ptR.posWorld() - _ptL.posWorld())*forceP + (_ptR._posWorldP - _ptL._posWorldP)*_force - _forceVector*_lenP )*gInv(_len);
		}else if( !IsZero(_dlen) ){
			forcePv = ( (_ptR.velWorld() - _ptL.velWorld())*forceP + (_ptR._velWorldP - _ptL._velWorldP)*_force - _forceVector*_dlenP)*gInv(_dlen);
		}
		_ptL.setForcePInWorld(forcePv);
		_ptR.setForcePInWorld(-forcePv);
	}
#endif
};

// -----------------------------------------------------------------
// class gLineExponentialSpring: Exponential Line Spring
//
// force = ks*(exp(kc*e)-1) + kd*de
//
// ks : stiffness when e=0. It should be set to proportional to cross sectional area,
// but irrelavant to length of the spring
// kd : same as ks. prop to area. irrelevant to length.
// 
// derivation:
//	stress=E*(exp(kc*e)-1) (<= compare with linear spring, in which stree = E*e )
//	f = AE*(exp(kc*e)-1).
// 
// -----------------------------------------------------------------
class gLineExponentialSpring:public gLineSpring
{
public:
	virtual int		type()  const { return FORCE_TYPE_LINE_EXPONENTIAL_SPRING; }
	inline	void	setStiffnessExponent(gReal kc)		{ _kc = kc; }
	inline gReal	stiffnessExponent(void) const		{ return _kc; }
	inline gReal	strain(void)		{ return _e; }
	inline gReal	strainRate(void)	{ return _de; }

	void			computeForce(void)
	{
		_e = (_len - _len0)/_len0;	// strain
		_de = _dlen/_len0;			// strain rate
		_force = _ks*(exp(_kc*_e)-1) + _kd*_de;
		
		if(_len > gEPSILON){
			_forceVector = (_ptR.posWorld()-_ptL.posWorld())*(_force/_len);
		}else if(!IsZero(_dlen)){
			_forceVector = (_ptR.velWorld()-_ptL.velWorld())*(_force/_dlen);
		}else{
			_forceVector.setZero();		
		}
		_ptL.setForceInWorld(_forceVector);
		_ptR.setForceInWorld(-_forceVector);
	}
protected:
	gReal _kc;			// exponential coeff. 
	gReal _e,_de;		//strain, strain rate

#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	computeForceP(void)
	{
		gReal ep = _lenP/_len0;
		gReal dep = _dlenP/_len0;
		gReal forceP = _ks*_kc*exp(_kc*_e)*ep + _kd*dep;
		gVec3 forcePv(0,0,0);
		if(_len>gEPSILON){
			forcePv = ( (_ptR.posWorld()-_ptL.posWorld())*forceP + (_ptR._posWorldP-_ptL._posWorldP)*_force - _forceVector*_lenP )*gInv(_len);
		}else if(!IsZero(_dlen)){
			forcePv = ( (_ptR.velWorld()-_ptL.velWorld())*forceP + (_ptR._velWorldP-_ptL._velWorldP)*_force - _forceVector*_dlenP )*gInv(_dlen);
		}
		_ptL.setForcePInWorld(forcePv);
		_ptR.setForcePInWorld(-forcePv);

	}
#endif
};

// -----------------------------------------------------------------
// class gPiecewiseLineSegmentSpring : linear spring with multiple via points
//
// gPiecewiseLineSegmentSpring consists of a set of line segments, and each line segment
// connects twa adjacent via points
// -----------------------------------------------------------------
class gPiecewiseLineSegmentSpring: public gSpring
{
public:
						gPiecewiseLineSegmentSpring(void)	{ _nVpoints = 0;}
	inline gForcePoint*	vpoint(int i)						{ return _vpoint[i]; }
	inline	int			nVpoints(void)						{ return _nVpoints; }
	/* append a gForcePoint to _vpoint */
	void				appendVpointInLocal(gRigidBody* body, const gVec3& pos);	//pos in local coordinate of body
	void				appendVpointInWorld(gRigidBody* body, const gVec3& pos);	//pos in world coordinate
	/* set (current_length*scale) as original length.
	 change scale for initial tension setting */
	void				computeRestLength(gReal scale=1.0);
	//{ kinematics updater
	virtual	void		updateKinematicsUptoPos(void);
	virtual	void		updateKinematicsUptoVel(void);
	//}
	// compute force and assign to dcPoints
    //void				computeForce(void)=0;
	// apply force to gRigidBody->Fext
	void				addForceToBody(void);

	void				computeLengthAndItsChangeRate()
	{
		_len=0;
		_dlen=0;
		for(int i=0;i<(_nVpoints-1);++i)
		{
			gVec3 xi(_vpoint[i+1]->posWorld(),_vpoint[i]->posWorld());
			gVec3 vi(_vpoint[i+1]->velWorld(),_vpoint[i]->velWorld());
			gReal leni = xi.magnitude();
			_len += leni;
			_dlen += ( leni < gEPSILON ? vi.magnitude() : (xi,vi)/leni );
		}	
	}
protected:
	std::vector<gForcePoint*>	_vpoint;		// via points (including two end points)
	int				_nVpoints;
	void			setForceToVpoints(gReal force)
	{
		// assign forces to vpoint
		_vpoint[0]->clearForce();
		for(int i=0;i<(_nVpoints-1);++i)
		{
			gVec3 dir(_vpoint[i+1]->posWorld(),_vpoint[i]->posWorld()); //vector from i to i+1
			dir.normalize();
			_vpoint[i]->addForceInWorld(dir*force);
			_vpoint[i+1]->setForceInWorld(dir*(-force));
		}	
	}

#ifdef MW_USE_DIFF_NEWTON_EULER
	virtual void	updateKVariablesP(void)
	{
		for(std::vector<gForcePoint*>::iterator it=_vpoint.begin(); it != _vpoint.end(); it++)	
		{
			(*it)->updateKVariablesP();
		}

		_lenP = 0;
		_dlenP = 0;
		for(unsigned int i=0; i<_vpoint.size()-1;++i)
		{
			gVec3 xi = _vpoint[i+1]->posWorld() - _vpoint[i]->posWorld();
			gVec3 vi = _vpoint[i+1]->velWorld() - _vpoint[i]->velWorld();
			gVec3 xip = _vpoint[i+1]->_posWorldP - _vpoint[i]->_posWorldP;
			gVec3 vip = _vpoint[i+1]->_velWorldP - _vpoint[i]->_velWorldP;
			gReal leni = xi.magnitude();
			gReal lenip = (xi,xip)/leni;
			gReal dleni = (xi,vi)/leni;
			_lenP += lenip;
			_dlenP += ((xip,vi)+(xi,vip)-lenip*dleni )/leni;
		}
	}

	virtual void	addForcePToBody(void)	// add force derivative to a rigid body
	{
		for(std::vector<gForcePoint*>::iterator it=_vpoint.begin(); it != _vpoint.end(); it++)	
		{
			(*it)->addForcePToBody();
		}
	}

	void			setForcePToVpoints(gReal force, gReal forceP)
	{
		// assign forces to vpoint
		_vpoint[0]->_forceLocalP.setZero();
		for(int i=0;i<(_nVpoints-1);++i)
		{
			gVec3 di = _vpoint[i+1]->posWorld() - _vpoint[i]->posWorld();
			gVec3 xip = _vpoint[i+1]->_posWorldP - _vpoint[i]->_posWorldP;
			gReal leni = di.magnitude();
			di /= leni;
			gReal lenip = (di,xip);
			gVec3 fp = di*forceP + (xip-di*lenip)*(force/leni);
			_vpoint[i]->addForcePInWorld( fp );
			_vpoint[i+1]->setForcePInWorld( -fp );
		}	
	}
#endif
};

// -----------------------------------------------------------------
// class gPiecewiseLineSegmentLinearSpring
// -----------------------------------------------------------------
class gPiecewiseLineSegmentLinearSpring: public gPiecewiseLineSegmentSpring
{
public:
	void	computeForce(void)
	{
		_force = _ks*(_len-_len0) + _kd*_dlen;	//contractile force of spring
		setForceToVpoints(_force);
	}
#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	computeForceP(void)
	{
		gReal fp = _ks*_lenP + _kd*_dlenP;
		setForcePToVpoints(_force,fp);
	}
#endif
	gReal _force;
};

// -----------------------------------------------------------------
// class gPiecewiseLineSegmentExponentialSpring
// -----------------------------------------------------------------
class gPiecewiseLineSegmentExponentialSpring: public gPiecewiseLineSegmentSpring
{
public:	
	virtual int		type()  const { return FORCE_TYPE_PLS_EXPONENTIAL_SPRING; }
	gPiecewiseLineSegmentExponentialSpring() { _kc = 0; }
	
	inline	void	setStiffnessExponent(gReal kc) { _kc = kc; }
	inline gReal	stiffnessExponent(void) const { return _kc; }
	inline gReal	strain(void)	{ return _e; }
	inline gReal	strainRate(void) { return _de; }

	void			computeForce(void)
	{
		_force = _ks*(exp(_kc*_e)-1) + _kd*_de;	//contractile force of spring
		setForceToVpoints(_force);
	}

	virtual	void		updateKinematicsUptoPos(void)
	{
		gPiecewiseLineSegmentSpring::updateKinematicsUptoPos();
		_e = _len/_len0 - 1.0; //strain
	}

	virtual	void		updateKinematicsUptoVel(void)
	{
		gPiecewiseLineSegmentSpring::updateKinematicsUptoVel();
		_e = _len/_len0 - 1.0; //strain
		_de = _dlen/_len0;		//strain rate
	}

protected:
	gReal _kc;
	gReal _e,_de; //strain, strain rate
	gReal _force;
#ifdef MW_USE_DIFF_NEWTON_EULER
public:
	virtual void	computeForceP(void)
	{
		gReal ep = _lenP/_len0;
		gReal dep = _dlenP/_len0;
		gReal fp = _ks*_kc*exp(_kc*_e)*ep + _kd*dep;	//contractile force of spring
		setForcePToVpoints(_force,fp);
	}
#endif
};

#endif