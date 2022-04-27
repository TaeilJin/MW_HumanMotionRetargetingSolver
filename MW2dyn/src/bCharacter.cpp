//#####################################################################
// Copyright 2010-2015, Hynchul Choi, Sukwon Lee, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#include "Character/bCharacter.h"
#include "Character/gBdUtil.h"

//#include "Visualizer/gVisHeader.h"
#include "Visualizer/gOsgShape.h"
#include "Visualizer/gOsgUtil.h"

#include <algorithm>

extern double DEBUG_DRAW_CONSTRAINT_SIZE;
extern double BD_PARAM_JOINT_STOP_ERP;
extern double BD_PARAM_JOINT_STOP_CFM;

#define ENABLE_BT_ENABLE_GYROPSCOPIC_FORCE  0//1

bCharacterSim::bCharacterSim(bCharacter* mbs) {
	m_mbs = mbs;
}

bCharacter::bCharacter() : m_ShapeSet(NULL) {
	m_lFootMuF = 0.999; 
	m_lFootMuT = 0.999;
	m_rFootMuF = 0.999;
	m_rFootMuT = 0.999;	

	m_lFootLink = m_rFootLink = m_lHandLink = m_rHandLink = NULL;
};


bCharacterSim::~bCharacterSim()
{
	// remove all constraints
	for( int i = 0; i < m_numJoints; ++i)
	{
		m_btOwnerWorld->removeConstraint( m_joints[i] );
		delete m_joints[i]; m_joints[i] = 0;
	}
	delete[] m_joints;

	// Remove all bodies and shapes
	for( int i = 0; i < m_numBodies; ++i)
	{
		m_btOwnerWorld->removeRigidBody(m_btBodies[i]);
			
		delete m_btBodies[i]->getMotionState();
		delete m_btBodies[i]; m_btBodies[i] = 0;
		delete m_btShapes[i]; m_btShapes[i] = 0;
	}
	delete[] m_btBodies;

	delete[] m_jointRigidBodyA;
	delete[] m_jointRigidBodyB;	

}

bShapeSet* bCharacter::getOrCreateVisShapeSet()
{
	if( m_ShapeSet == NULL )
	{
		m_ShapeSet = new bShapeSet;
	}
	return m_ShapeSet;
}

int	bCharacter::setupCharacterSim(
	bCharacterSim* sim, 
	btDynamicsWorld* ownerWorld, 
	const btVector3& positionOffset,
	int collisionGroup, 
	int collideWithMask,
	double frictionCoeff,
	double restitutionCoeff,
	double jointParamStopERP,
	double jointParamStopCFM,
	double jointParamERP,
	double jointParamCFM
	)
{
	// link number assertion
	if( sim == NULL || _nLinks < 1 ) return -1;		// setup fail
	
	//set sim
	m_sim = sim;
	
	//Setup CharacterSim
	m_sim->m_btOwnerWorld	= ownerWorld;	
	m_sim->m_numBodies		= _nLinks;
	m_sim->m_numJoints		= _nLinks-1;
	m_sim->m_btShapes		= new btCollisionShape*[ m_sim->m_numBodies ];
	m_sim->m_btBodies		= new btRigidBody*[ m_sim->m_numBodies ];
	m_sim->m_joints			= new btTypedConstraint*[ m_sim->m_numJoints ];
	m_sim->m_jointRigidBodyA= new int[ m_sim->m_numJoints ];
	m_sim->m_jointRigidBodyB= new int[ m_sim->m_numJoints ];

	m_localCoMFrames		= new gXMat[ m_sim->m_numBodies ];
	m_localCoMFramesInv		= new gXMat[ m_sim->m_numBodies ];
	m_isLFootContact		= false;
	m_isRFootContact		= false;

	m_rFootMuF = m_lFootMuF = m_rFootMuT = m_lFootMuT = frictionCoeff;

	// 2012-02-01 : added geometry setup
	for(int i = 0; i < _nLinks; ++i)
	{
		gLink* link = this->link(i);
		m_sim->m_btShapes[i] = m_ShapeSet->getShape(link->name());

		//set localCoMFrames
		m_localCoMFrames[i] = m_ShapeSet->getShapeXform(link->name());
		m_localCoMFramesInv[i].makeInverse( m_localCoMFrames[i] );
		
		m_sim->m_btBodies[i] = createRigidBodySim( *link, m_localCoMFrames[i], m_sim->m_btShapes[i], positionOffset, collisionGroup, collideWithMask);
#if ENABLE_BT_ENABLE_GYROPSCOPIC_FORCE
		m_sim->m_btBodies[i]->setFlags(BT_ENABLE_GYROPSCOPIC_FORCE); //Bullet recommends to turn-off this flag to increase stability, but it probably would hurt simulation accuracy.
#endif
		// 2012-02-16 : link corresponding m_btBodies to mbs links - this is for later use of looking for colliding object
		m_sim->m_btBodies[i]->setUserPointer((void*)link);
		m_sim->m_btBodies[i]->setFriction(frictionCoeff);
		m_sim->m_btBodies[i]->setRestitution(restitutionCoeff);
	}

	// Setup some damping on the m_btBodies...what is this? need to know.
	for (int i = 0; i < m_sim->m_numBodies; ++i)
	{
		m_sim->m_btBodies[i]->setDamping(0.0, 0.0); ///NEVER GIVE FICTITIOUS DAMPING TO bCharacter. It degrades controller's performance.
		m_sim->m_btBodies[i]->setSleepingThresholds(0,0);
		m_sim->m_btBodies[i]->setActivationState(DISABLE_DEACTIVATION);

		// enable ccd. for now, simply use x length for all types. *TODO : the length(x) should be corrected.
		//m_sim->m_btBodies[i]->setCcdMotionThreshold(0.01);
		//m_sim->m_btBodies[i]->setCcdMotionThreshold(((btShape*)this->link(i)->userPointer)->_length.x());
		//m_sim->m_btBodies[i]->setCcdSweptSphereRadius(0.9*((btShape*)this->link(i)->userPointer)->_length.x());
	}

	// Now setup the constraints
	
	btTransform localA, localB; //transform wrt CoM frames. A is parent. B is child
	gXMat locA, locB; //config. of constraint frame wrt mbs' body frames.

	// Rest of links
	for (int i = 1; i< m_sim->m_numBodies; ++i)
	{
		gLink* p = link(i)->parent();
		gLink* c = link(i);
		int pid = findLinkIndex(p);
		int jointId = i-1;

		m_sim->m_jointRigidBodyA[jointId]=pid;
		m_sim->m_jointRigidBodyB[jointId]=i;

		double dbgDrawSize = DEBUG_DRAW_CONSTRAINT_SIZE;		
			
		switch(c->type())
		{
		case TYPE_1D_LINK:

			if( c->screw(0).trn().magnitudeSquared() < 0.0001f ) // revolute joint
			{
				btHingeConstraint* hingeC;
				//gXMat locA, locB; //config. of constraint frame wrt mbs' body frames...axis must be in Z-axis
					
				//B
				//revolute axis must be Z-axis
				gQuat q; q.makeRotate( gVec3(0,0,1), c->screw(0).rot() );
				locB.setRot( q.inRotMatrix() );					
				//A
				locA = c->localFrame()*locB;

				gXMat _localA, _localB; //config. of constraint frame wrt CoM frames
				_localA = m_localCoMFrames[pid].invMult(locA);
				_localB = m_localCoMFrames[i].invMult(locB);
					
				localA.setIdentity(); localB.setIdentity();
				q = _localA.rot().inQuat();
				localA.setRotation( btQuaternion(q.x(), q.y(), q.z(), q.w() ) );
				localA.setOrigin( btVector3(_localA.e(0,3), _localA.e(1,3), _localA.e(2,3) ) );

				q = _localB.rot().inQuat();
				localB.setRotation( btQuaternion(q.x(), q.y(), q.z(), q.w() ) );
				localB.setOrigin( btVector3(_localB.e(0,3), _localB.e(1,3), _localB.e(2,3) ) );

				hingeC =  new btHingeConstraint(*(m_sim->m_btBodies[pid]), *(m_sim->m_btBodies[i]), localA, localB, true);
				
				btScalar lo = btScalar( ((g1dLink*)c)->jointLimitLo() );
				btScalar hi = btScalar( ((g1dLink*)c)->jointLimitHi() );
				//2012-02-14: bullet dynamics' joint limit is w.r.t. B frame. So change the order.
				//hingeC->setLimit(-hi,-lo);			
				hingeC->setLimit(lo,hi);
					
				m_sim->m_joints[jointId] = hingeC;
				hingeC->setDbgDrawSize(dbgDrawSize);

				hingeC->setParam(BT_CONSTRAINT_STOP_ERP, jointParamStopERP);
				hingeC->setParam(BT_CONSTRAINT_STOP_CFM, jointParamStopCFM);
				/*hingeC->setParam(BT_CONSTRAINT_ERP, jointParamERP); 
				hingeC->setParam(BT_CONSTRAINT_CFM, jointParamCFM);			*/


				m_sim->m_btOwnerWorld->addConstraint(m_sim->m_joints[jointId], true);
			}
			else if( c->screw(0).rot().magnitudeSquared() < 0.0001f ) // slider joint
			{
				btSliderConstraint* sliderC;

				//gXMat locA, locB; //config. of constraint frame wrt mbs' body frames...axis must be in X-axis
				//B
				gQuat q; q.makeRotate( gVec3(1,0,0), c->screw(0).trn() ); //q rotates X-axis to sliding axis defined in mbs
				locB.setRot( q.inRotMatrix() );					
				//A
				locA = c->localFrame()*locB;

				gXMat _localA, _localB; //config. of constraint frame wrt CoM frames
				_localA = m_localCoMFrames[pid].invMult(locA);
				_localB = m_localCoMFrames[i].invMult(locB);
					
				localA.setIdentity(); localB.setIdentity();
				q = _localA.rot().inQuat();
				localA.setRotation( btQuaternion(q.x(), q.y(), q.z(), q.w() ) );
				localA.setOrigin( btVector3(_localA.e(0,3), _localA.e(1,3), _localA.e(2,3) ) );

				q = _localB.rot().inQuat();
				localB.setRotation( btQuaternion(q.x(), q.y(), q.z(), q.w() ) );
				localB.setOrigin( btVector3(_localB.e(0,3), _localB.e(1,3), _localB.e(2,3) ) );

				sliderC =  new btSliderConstraint(*(m_sim->m_btBodies[pid]), *(m_sim->m_btBodies[i]), localA, localB,true);
					
				//TODO: JOINT LIMITS TO BE HANDLED LATER!
				sliderC->setLowerLinLimit(-1);
				sliderC->setUpperLinLimit(1);

				//below limits controls how much the two rigid bodies can "twist" along the sliding axis					
				sliderC->setLowerAngLimit(0);
				sliderC->setUpperAngLimit(0);
					
				m_sim->m_joints[jointId] = sliderC;
				sliderC->setDbgDrawSize(dbgDrawSize);

				m_sim->m_btOwnerWorld->addConstraint(m_sim->m_joints[jointId], true);
			}
			else // both rotating and sliding...ignored...
			{
				//report error
				exit(0); //exit the program
			}
			break;

		case TYPE_2D_LINK: //TODO: universal constraint
			{
				btUniversalConstraint* univC;

				// world frame
				btVector3 anchor, axis1, axis2;
				anchor = convertTobtVector3( c->frame().trn()) + positionOffset;

				axis1 = convertTobtVector3(c->screw(0).xform(c->frame()).rot());
				axis2 = convertTobtVector3(c->screw(1).xform(c->frame()).rot());

				univC = new btUniversalConstraint(*(m_sim->m_btBodies[pid]), *(m_sim->m_btBodies[i]), anchor, axis1, axis2);

				//univC->setLimit(btScalar(M_PI*0.8f), btScalar(M_PI*0.8f), btScalar(M_PI), 0.5f);
					
				univC->setLowerLimit(-SIMD_HALF_PI * 0.5f, -SIMD_HALF_PI * 0.5f);
				univC->setUpperLimit(SIMD_HALF_PI * 0.5f,  SIMD_HALF_PI * 0.5f);

				m_sim->m_joints[jointId] = univC;
				univC->setDbgDrawSize(dbgDrawSize);
				m_sim->m_btOwnerWorld->addConstraint(m_sim->m_joints[jointId], true);
			}
			break;

		case TYPE_BALL_LINK: //cone twist constraint if there's joint limit
			{
					
				btGeneric6DofConstraint* dof6C;
				
				//gXMat locA, locB; //config. of constraint frame wrt mbs' body frames.					
				//cone must be in X-axis (i.e., X is twist axis)
				//We use a hack...assume that twist axis = vector from B's origin to its CoM

				//B
				locB.setIdentity();
				locB.setRot(m_localCoMFrames[pid].rot());
				//locB.setRot(m_localCoMFramesInv[pid].rot());
				//A
				locA = c->localFrame()*locB;

				gXMat _localA, _localB; //config. of constraint frame wrt CoM frames
				_localA = m_localCoMFrames[pid].invMult(locA);
				_localB = m_localCoMFrames[i].invMult(locB);

				localA.setIdentity(); localB.setIdentity();
				gQuat q;
				q = _localA.rot().inQuat();
				localA.setRotation( btQuaternion(q.x(), q.y(), q.z(), q.w() ) );
				localA.setOrigin( btVector3(_localA.e(0,3), _localA.e(1,3), _localA.e(2,3) ) );

				q = _localB.rot().inQuat();
				localB.setRotation( btQuaternion(q.x(), q.y(), q.z(), q.w() ) );
				localB.setOrigin( btVector3(_localB.e(0,3), _localB.e(1,3), _localB.e(2,3) ) );

				//coneC = new btConeTwistConstraint(*(m_sim->m_btBodies[pid]), *(m_sim->m_btBodies[i]), localA, localB);
				dof6C = new btGeneric6DofConstraint(*(m_sim->m_btBodies[pid]), *(m_sim->m_btBodies[i]), localA, localB, true);

				//TODO: JOINT LIMITS NEED TO BE READ FROM bCharacter!
				//coneC->setLimit(btScalar(M_PI*0.8f), btScalar(M_PI*0.8f), btScalar(M_PI), 0.5f);
					
				/*dof6C->setAngularLowerLimit(btVector3(-SIMD_PI / 2.0, -SIMD_PI / 2.0, -SIMD_PI / 2.0));
				dof6C->setAngularUpperLimit(btVector3(SIMD_PI / 2.0, SIMD_PI / 2.0, SIMD_PI / 2.0));*/
				dof6C->setAngularLowerLimit(btVector3(-SIMD_PI, -SIMD_PI, -SIMD_PI));
				dof6C->setAngularUpperLimit(btVector3(SIMD_PI, SIMD_PI, SIMD_PI));
				dof6C->setLinearLowerLimit(btVector3(0.0, 0.0, 0.0));
				dof6C->setLinearUpperLimit(btVector3(0.0, 0.0, 0.0));

				m_sim->m_joints[jointId] = dof6C;
				dof6C->setDbgDrawSize(dbgDrawSize);

				//for( int cIdx=0; cIdx<6; cIdx++ )
				/*{
					dof6C->setParam(BT_CONSTRAINT_STOP_ERP, jointParamStopERP, cIdx);
					dof6C->setParam(BT_CONSTRAINT_STOP_CFM, jointParamStopCFM, cIdx);
					dof6C->setParam(BT_CONSTRAINT_ERP, jointParamERP, cIdx);
					dof6C->setParam(BT_CONSTRAINT_CFM, jointParamCFM, cIdx);
				}*/
				
				m_sim->m_btOwnerWorld->addConstraint(m_sim->m_joints[jointId], true);
			}

			break;

		case TYPE_WELDED_LINK: //cone twist constraint if there's joint limit
			{		
				//cone must be in X-axis (i.e., X is twist axis)
				//We use a hack...assume that twist axis = vector from B's origin to its CoM
				//B
				gVec3 com = c->inertia().comInVec3(); com.normalize();
				gQuat q; q.makeRotate( gVec3(1,0,0), com );
				locB.setRot( q.inRotMatrix() );
				//A
				locA = c->localFrame()*locB;

				gXMat _localA, _localB; //config. of constraint frame wrt CoM frames
				_localA = m_localCoMFrames[pid].invMult(locA);
				_localB = m_localCoMFrames[i].invMult(locB);

				localA.setIdentity(); localB.setIdentity();
				q = _localA.rot().inQuat();
				localA.setRotation( btQuaternion(q.x(), q.y(), q.z(), q.w() ) );
				localA.setOrigin( btVector3(_localA.e(0,3), _localA.e(1,3), _localA.e(2,3) ) );

				q = _localB.rot().inQuat();
				localB.setRotation( btQuaternion(q.x(), q.y(), q.z(), q.w() ) );
				localB.setOrigin( btVector3(_localB.e(0,3), _localB.e(1,3), _localB.e(2,3) ) );

				btGeneric6DofConstraint* pGen6DOF = new btGeneric6DofConstraint(*(m_sim->m_btBodies[pid]), *(m_sim->m_btBodies[i]), localA, localB, true);

				pGen6DOF->setLinearLowerLimit(btVector3(0., 0., 0.));
				pGen6DOF->setLinearUpperLimit(btVector3(0., 0., 0.));

				pGen6DOF->setAngularLowerLimit(btVector3(0., 0., 0.));
				pGen6DOF->setAngularUpperLimit(btVector3(0., 0., 0.));

				m_sim->m_joints[jointId] = pGen6DOF;
				pGen6DOF->setDbgDrawSize(dbgDrawSize);

				m_sim->m_btOwnerWorld->addConstraint(m_sim->m_joints[jointId], true);
			}

			break;

		default:
			printf("Warning: unsupported joint constraint.\n");
			break;
		}
	}

	return 1;		// setup completed
}


bCharacter::~bCharacter()
{
	delete[] m_localCoMFrames;
	delete[] m_localCoMFramesInv;
}

void bCharacter::postLoading()
{
	gMultibodySystem::postLoading();
	
	/*
	//register foot/hand links
	if(m_lLegIdx.size()>0){		
		m_lFootLink = _links[m_lLegIdx.back()]; 
	}else{
		std::cout << "WARNING: bCharacter::m_lFootLink not defined." << std::endl;
	}
	if(m_rLegIdx.size()>0){
		m_rFootLink = _links[m_rLegIdx.back()]; 
	}else{
		std::cout << "WARNING: bCharacter::m_rFootLink not defined." << std::endl;
	}
	if(m_lArmIdx.size()>0){
		m_lHandLink = _links[m_lArmIdx.back()]; 
	}else{
		std::cout << "WARNING: bCharacter::m_lHandLink not defined." << std::endl;
	}
	if(m_rArmIdx.size()>0){
		m_rHandLink = _links[m_rArmIdx.back()]; 
	}else{
		std::cout << "WARNING: bCharacter::m_rHandLink not defined." << std::endl;
	}

	//check unit
	if( _unit == MW_UNDEFINED ) std::cout << "WARNING: Unit undefined." << std::endl;
	*/
}

extern gVec3 MW_GRAVITY_VECTOR;

void bCharacter::initializeFootGeometryInBoxShape()
{
	assert( m_lFootLink && m_rFootLink );

	gVec3 upDirection = -1*MW_GRAVITY_VECTOR; upDirection.normalize();
	gVec3 r2l = m_lFootLink->pos() - m_rFootLink->pos(); r2l.normalize();
	gVec3 frontalDirection = r2l%upDirection; frontalDirection.normalize();

	int lFootIdx = findLinkIndex(m_lFootLink);
	m_lFootGeom.setFromGeometry(m_lFootLink, m_localCoMFrames[lFootIdx], (btBoxShape*)m_sim->m_btShapes[lFootIdx], frontalDirection, upDirection);

	int rFootIdx = findLinkIndex(m_rFootLink);
	m_rFootGeom.setFromGeometry(m_rFootLink, m_localCoMFrames[rFootIdx], (btBoxShape*)m_sim->m_btShapes[rFootIdx], frontalDirection, upDirection);

}

void bCharacterSim::postLoading()
{
	//register foot/hand links
	if(m_mbs->getLFootLink()){
		int id = m_mbs->findLinkIndex(m_mbs->getLFootLink());
		m_lFootBtBody = m_btBodies[ id ];
	}
	if(m_mbs->getRFootLink()){
		int id = m_mbs->findLinkIndex(m_mbs->getRFootLink());
		m_rFootBtBody = m_btBodies[ id ];
	}	
	if(m_mbs->getLHandLink()){
		int id = m_mbs->findLinkIndex(m_mbs->getLHandLink());
		m_lHandBtBody = m_btBodies[ id ];
	}
	if(m_mbs->getRHandLink()){
		int id = m_mbs->findLinkIndex(m_mbs->getRHandLink());
		m_rHandBtBody = m_btBodies[ id ];
	}
}


void bCharacter::updateStatesFromCharacterSim(int gUpdateMode)
{
	assert( baseLink()->type() == TYPE_FREE_LINK );

	//update base link
	gFreeLink* base = (gFreeLink*)baseLink();
	gXMat C = convertToXMat( m_sim->m_btBodies[0]->getWorldTransform() );
	C.rectify(0); //ensure SE(3) structure
	base->setLocalFrame( C * ~m_localCoMFrames[0] );
	base->setFrame( base->localFrame() );

	//velocity of CoM frame expressed in world frame
	btVector3 vel = m_sim->m_btBodies[0]->getLinearVelocity();
	gVec3 linV(vel.x(),vel.y(),vel.z());
	vel = m_sim->m_btBodies[0]->getAngularVelocity();
	gVec3 angV(vel.x(),vel.y(),vel.z());

	//velocity of mbs frame expressed in itself
	gTwist velBodyFrame;
	velBodyFrame.setRot( baseLink()->frame().invMultVec3(angV) );
	velBodyFrame.setTrn( baseLink()->frame().invMultVec3(linV) );		
	base->setGenVel(velBodyFrame);
	
	//update descendants
	for(int i=0; i<m_sim->m_numJoints;++i)
	{
		int plinkIdx = m_sim->m_jointRigidBodyA[i];
		int clinkIdx = m_sim->m_jointRigidBodyB[i];
		gLink* plink = link(plinkIdx); //parent link
		gLink* clink = link(clinkIdx); //child link
		

		btTypedConstraintType ctype =  m_sim->m_joints[i]->getConstraintType();

		if( ctype == HINGE_CONSTRAINT_TYPE )
		{
			btHingeConstraint* hingeC = (btHingeConstraint*)(m_sim->m_joints[i]);
			
			btScalar angle = hingeC->getHingeAngle();
			
			//get relative angular velocity of B with respect to A
			//Note that the angular velocity of BD is expressed with respect to world frame			
			btVector3 velB = m_sim->m_btBodies[clinkIdx]->getAngularVelocity();
			btVector3 velA = m_sim->m_btBodies[plinkIdx]->getAngularVelocity();
			btVector3 relativeAngVelWorld = velB - velA;

			//get hinge axis wrt world frame
			//below 2 lines are same
			//btVector3 swingAxisWorld = m_btBodies[m_jointRigidBodyB[i]]->getWorldTransform().getBasis() * hingeC->getBFrame().getBasis().getColumn(2);			
			btVector3 swingAxisWorld = 
				hingeC->getRigidBodyB().getWorldTransform().getBasis() * hingeC->getBFrame().getBasis().getColumn(2);

			//velocity of joint angle
			gReal jointRate = swingAxisWorld.dot(relativeAngVelWorld);
			
			//update corresponding mbs link
			((g1dLink*)clink)->setCoord(angle);
			clink->setGenVel(0,jointRate);
		}
		//else if ( ctype == CONETWIST_CONSTRAINT_TYPE )
		//{
		//	//compute coord
		//	btConeTwistConstraint* coneC = (btConeTwistConstraint*)m_sim->m_joints[i];
	
		//	gXMat B = convertToXMat( coneC->getRigidBodyB().getWorldTransform() );
		//	B *= m_localCoMFramesInv[ clinkIdx ]; //B is now configuration of mbs frame

		//	gXMat A = convertToXMat( coneC->getRigidBodyA().getWorldTransform() );
		//	A *= m_localCoMFramesInv[ plinkIdx ]; //A is now configuration of mbs frame
		//	A *= clink->localFrameDefault(); 
		//	gRotMat coord = A.invMult(B).rot();
		//	coord.rectify(0);
		//	
		//	//get relative angular velocity of B with respect to A
		//	//Note that the angular velocity of BD is expressed with respect to world frame			
		//	btVector3 relativeAngVel = m_sim->m_btBodies[clinkIdx]->getAngularVelocity() - m_sim->m_btBodies[plinkIdx]->getAngularVelocity();
		//	
		//	//transform to B's CoM frame
		//	relativeAngVel =  relativeAngVel * m_sim->m_btBodies[clinkIdx]->getCenterOfMassTransform().getBasis();
		//	
		//	//transform to B's mbs frame
		//	gVec3 rAV(relativeAngVel.x(),relativeAngVel.y(),relativeAngVel.z());
		//	rAV = m_localCoMFrames[clinkIdx].multVec3(rAV);

		//	//update corresponding mbs link
		//	((gBallLink*)clink)->setCoord(coord);
		//	clink->setGenVel(0,rAV.x());
		//	clink->setGenVel(1,rAV.y());
		//	clink->setGenVel(2,rAV.z());
		//}
		else if ( ctype == D6_CONSTRAINT_TYPE )
		{
			//compute coord
			btGeneric6DofConstraint* dof6C = (btGeneric6DofConstraint*)m_sim->m_joints[i];

			btRotationalLimitMotor* motor = dof6C->getRotationalLimitMotor(0);
			motor->m_enableMotor = true;

	
			gXMat B = convertToXMat( dof6C->getRigidBodyB().getWorldTransform() );
			B *= m_localCoMFramesInv[ clinkIdx ]; //B is now configuration of mbs frame

			gXMat A = convertToXMat( dof6C->getRigidBodyA().getWorldTransform() );
			A *= m_localCoMFramesInv[ plinkIdx ]; //A is now configuration of mbs frame
			A *= clink->localFrameDefault(); 
			gRotMat coord = A.invMult(B).rot();
			coord.rectify(0);
			
			//get relative angular velocity of B with respect to A
			//Note that the angular velocity of BD is expressed with respect to world frame			
			btVector3 relativeAngVel = m_sim->m_btBodies[clinkIdx]->getAngularVelocity() - m_sim->m_btBodies[plinkIdx]->getAngularVelocity();
			
			//transform to B's CoM frame
			relativeAngVel =  relativeAngVel * m_sim->m_btBodies[clinkIdx]->getCenterOfMassTransform().getBasis();
			
			//transform to B's mbs frame
			gVec3 rAV(relativeAngVel.x(),relativeAngVel.y(),relativeAngVel.z());
			rAV = m_localCoMFrames[clinkIdx].multVec3(rAV);

			//update corresponding mbs link
			((gBallLink*)clink)->setCoord(coord);
			clink->setGenVel(0,rAV.x());
			clink->setGenVel(1,rAV.y());
			clink->setGenVel(2,rAV.z());
		}
		else
		{
			//not handled constraint type
			assert(0);			
		}			
	}
	updateKinematicsUptoVel();

	/* 
	// Below experimental code tries to correct the joint-constraint violation of bullet simulation.
	if(G_USE_JOINT_CORRECTION_FOOT == gUpdateMode)
	{
		// fix foot frame
		gVec3 dRot, dTrn;
		gVec3 bdRot, bdTrn;
		int	idxFoot = 16;

		gVec3 mbRot = link( idxFoot )->frameVel().rot();
		gVec3 mbTrn = link( idxFoot )->frameVel().trn();

		btTransform bdFrame = m_sim->m_btBodies[ idxFoot ]->getWorldTransform().inverse();
		gXMat mbFrame = convertToXMat(  m_sim->m_btBodies[ idxFoot ]->getWorldTransform().inverse() );
		gXMat frame = m_localCoMFrames[ idxFoot ] * mbFrame;

		bdRot = frame.multVec3( convertTogVec3(m_sim->m_btBodies[ idxFoot ]->getAngularVelocity()) );
		bdTrn = frame.multVec3( convertTogVec3(m_sim->m_btBodies[ idxFoot ]->getLinearVelocity()) );

		dRot = bdRot - mbRot;
		dTrn = bdTrn - mbTrn;
		gTwist dVf(dRot, dTrn);

		gXMat T_of = link(0)->frame().invMult(link( idxFoot )->frame());
		gTwist dV0 = gTwist::Ad(T_of, dVf);

		for(int i = 0; i < link(0)->dof(); ++i)
			link(0)->setGenVel(i, link(0)->genVel(i) + dV0.e(i));

		updateKinematicsUptoVel();
	}
	*/
}

void bCharacter::applyBaseLinkWrenchToCharacterSimForDebug(const gWrench& wrench)
{
	gWrench wren = wrench.xformInv( m_localCoMFrames[0] );	
	m_sim->m_btBodies[0]->applyTorque(btVector3(wren.e(0),wren.e(1),wren.e(2)));
	m_sim->m_btBodies[0]->applyCentralForce(btVector3(wren.e(3),wren.e(4),wren.e(5)));
}

void bCharacter::applyJointTorquesToCharacterSim(double timeStep, bool applyWrenchToBaselink, int option)
{
	if(applyWrenchToBaselink)
	{
		gFreeLink* l = (gFreeLink*)link(0);
		gWrench wrench(l->genForce(0),l->genForce(1),l->genForce(2),l->genForce(3),l->genForce(4),l->genForce(5));
		gWrench wren = wrench.xformInv( m_localCoMFrames[0] );	
		if (option){
			m_sim->m_btBodies[0]->applyTorque(btVector3(wren.e(0),wren.e(1),wren.e(2)));
			m_sim->m_btBodies[0]->applyCentralForce(btVector3(wren.e(3),wren.e(4),wren.e(5)));
		}else{
			m_sim->m_btBodies[0]->applyTorqueImpulse(btVector3(wren.e(0)*timeStep,wren.e(1)*timeStep,wren.e(2)*timeStep));
			m_sim->m_btBodies[0]->applyCentralImpulse(btVector3(wren.e(3)*timeStep,wren.e(4)*timeStep,wren.e(5)*timeStep));
		}
	}

	for(int i=0; i<m_sim->m_numJoints;++i)
	{
		int plinkIdx = m_sim->m_jointRigidBodyA[i];
		int clinkIdx = m_sim->m_jointRigidBodyB[i];
		gLink* plink = link(plinkIdx); //parent link
		gLink* clink = link(clinkIdx); //child link
		

		btTypedConstraintType ctype =  m_sim->m_joints[i]->getConstraintType();

		if( ctype == HINGE_CONSTRAINT_TYPE )
		{
			if (option){
				//torque applied to B, expressed in world frame
				gVec3 torqueW = clink->frame().multVec3( ((g1dLink*)clink)->screw().rot() ) * clink->genForce(0);
				btVector3 torque(torqueW.x(),torqueW.y(),torqueW.z());
				m_sim->m_btBodies[ plinkIdx ]->applyTorque(-torque);
				m_sim->m_btBodies[ clinkIdx ]->applyTorque(torque);
			}else{
				gVec3 torqueW = clink->frame().multVec3( ((g1dLink*)clink)->screw().rot() ) * ( clink->genForce(0) * timeStep );
				btVector3 torque(torqueW.x(),torqueW.y(),torqueW.z());
				m_sim->m_btBodies[ plinkIdx ]->applyTorqueImpulse(-torque);
				m_sim->m_btBodies[ clinkIdx ]->applyTorqueImpulse(torque);
			}
		}
		else if ( ctype == CONETWIST_CONSTRAINT_TYPE )
		{
			if (option){
				//torque applied to B, expressed in world frame			
				gVec3 torqueW =  clink->frame().multVec3( ((gBallLink*)clink)->genForce() );
				btVector3 torque(torqueW.x(),torqueW.y(),torqueW.z());
				m_sim->m_btBodies[ plinkIdx ]->applyTorque(-torque);
				m_sim->m_btBodies[ clinkIdx ]->applyTorque(torque);
			}else{
				//torque applied to B, expressed in world frame			
				gVec3 torqueW =  clink->frame().multVec3( ((gBallLink*)clink)->genForce() );
				btVector3 torque(torqueW.x()* timeStep,torqueW.y()* timeStep,torqueW.z()* timeStep);
				m_sim->m_btBodies[ plinkIdx ]->applyTorqueImpulse(-torque);
				m_sim->m_btBodies[ clinkIdx ]->applyTorqueImpulse(torque);
			}
		}
		else if ( ctype == D6_CONSTRAINT_TYPE )
		{
			//Nothing to do.
			;
		}
		else
		{
			//not handled constraint type
			assert(0);			
		}			
	}
}

#if 1 //new contact measurement method 2012-08-20 (SHL). Useful for non-flat ground.

//#include "Visualizer\gOsgViewHelper.h"
//#define DRAW_CONTACT_FORCES

//aggrLinImpulse: total linear impulse
//aggrAngImpulse: torque induced by linear impulse about the pivot point (expressed in world frame)
static int computeAggregateImpulseOfManifold(
	btVector3* aggrLinImpulse, 
	btVector3* aggrAngImpulse,
	const btPersistentManifold* pManifold,
	const btVector3& pivot,
	bool flip
	)
{
	btVector3	ptLinImp(0,0,0);
	btVector3	ptAngImp(0,0,0);

	int sizeContactPt = pManifold->getNumContacts();
	if(sizeContactPt<1) return -1;

#ifdef DRAW_CONTACT_FORCES
	gOSGViewHelper* vh = (gOSGViewHelper*)ViewerFactory::getViewerInstance();	
	vh->groupStart( gOSGViewHelper::INSTANCE_GROUP );
#endif

	for(int j = 0; j < sizeContactPt; ++j)
	{
		const btManifoldPoint& pt = pManifold->getContactPoint(j);
		
#if (BT_BULLET_VERSION >= 282)		///Friction error has been fixed since Bullet version 2.82
#ifdef DRAW_CONTACT_FORCES
		btVector3 cpPos = pt.getPositionWorldOnA();
		btVector3 cpNor = pt.m_normalWorldOnB;

		gVec3 cp = gVec3(cpPos.x(), cpPos.y(), cpPos.z());
		gVec3 DirN = gVec3(cpNor.x(), cpNor.y(), cpNor.z());
		gVec3 Dir1 = gVec3(pt.m_lateralFrictionDir1.x(),pt.m_lateralFrictionDir1.y(),pt.m_lateralFrictionDir1.z());
		gVec3 Dir2 = gVec3(pt.m_lateralFrictionDir2.x(),pt.m_lateralFrictionDir2.y(),pt.m_lateralFrictionDir2.z());
		gVec3 lat = Dir1*pt.m_appliedImpulseLateral1 + Dir2*pt.m_appliedImpulseLateral2;
		if(flip){DirN*=-1; lat*=-1;}
		double latMag = lat.magnitude();
		lat.normalize();

		gOSGShape::setColor( osg::Vec4(0,0.5,0.5,1) );
		vh->addObject( gOSGShape::createLineShape( g2o(cp), g2o(lat), latMag, 3.f) );
		gOSGShape::setColor( osg::Vec4(0,0,1,1) );
		vh->addObject( gOSGShape::createLineShape( g2o(cp), g2o(DirN), pt.m_appliedImpulse, 3.f) );
#endif
		btVector3 linImp = (pt.m_normalWorldOnB * pt.m_appliedImpulse) //normal component
			+ (pt.m_lateralFrictionDir1*pt.m_appliedImpulseLateral1)  //friction components
			+ (pt.m_lateralFrictionDir2*pt.m_appliedImpulseLateral2);
#else
#ifdef DRAW_CONTACT_FORCES
		btVector3 cpPos = pt.getPositionWorldOnA();
		btVector3 cpNor = pt.m_normalWorldOnB;
		gVec3 cp = gVec3(cpPos.x(), cpPos.y(), cpPos.z());
		gVec3 DirN = gVec3(cpNor.x(), cpNor.y(), cpNor.z());
		gVec3 Dir1 = gVec3(pt.m_lateralFrictionDir1.x(),pt.m_lateralFrictionDir1.y(),pt.m_lateralFrictionDir1.z());
		gVec3 Dir2 = gVec3(pt.m_lateralFrictionDir2.x(),pt.m_lateralFrictionDir2.y(),pt.m_lateralFrictionDir2.z());
		gVec3 lat = Dir1*pt.m_appliedImpulseLateral2 + Dir2*pt.m_appliedImpulseLateral1;		
		if(flip){DirN*=-1; lat*=-1;}
		double latMag = lat.magnitude();
		lat.normalize();
		gOSGShape::setColor(osg::Vec4(0,0.5,0.5,1));
		vh->addObject( gOSGShape::createLineShape( g2o(cp), g2o(lat), latMag, 3.f));
		gOSGShape::setColor(osg::Vec4(0,0.0,1,1));
		vh->addObject( gOSGShape::createLineShape( g2o(cp), g2o(DirN), pt.m_appliedImpulse, 3.f));
#endif
		btVector3 linImp = (pt.m_normalWorldOnB * pt.m_appliedImpulse) //normal component
			+ (pt.m_lateralFrictionDir1*pt.m_appliedImpulseLateral2)  //friction components
			+ (pt.m_lateralFrictionDir2*pt.m_appliedImpulseLateral1);
#endif

		if(flip) linImp = -linImp;

		ptLinImp += linImp;
		ptAngImp += (pt.getPositionWorldOnB() - pivot).cross(linImp);		
	}

#ifdef DRAW_CONTACT_FORCES
	vh->groupEnd();
#endif

	*aggrLinImpulse = ptLinImp;
	*aggrAngImpulse = ptAngImp;

	return 1;
}

//compute aggregate force and torque (about the body frame) of a link 
//force and torque are expressed in world frame.
static bool computeTotalContactWrenchOfLink(btVector3* force, btVector3* torque, gLink* link, int& numContactPts, float invTimeStep)
{
	force->setValue(0,0,0);
	torque->setValue(0,0,0);
	bool contacted = false;
	numContactPts = 0;

	gVec3 p = link->frame().trn();
	btVector3 pivot(p.x(),p.y(),p.z());

	for(int i=0; i<link->numCollisionManifolds(); ++i)
	{
		btVector3 f,t;
		btPersistentManifold* pManifold = link->collisionManifold(i);
		
		bool flip = ( link == (gLink*)(((btRigidBody*)pManifold->getBody1())->getUserPointer() ) ) ? true : false;

		if( computeAggregateImpulseOfManifold(&f,&t,pManifold,pivot,flip) == -1 ) continue;
		
		numContactPts += pManifold->getNumContacts();

		contacted = true;
		*force += f;
		*torque += t;
	}

	//divide by timeStep to convert impulse to force
	*force *= invTimeStep; 
	*torque *= invTimeStep;

	return contacted;
}

//Given applied force and torque about body frame (expressed in world frame)
//compute CoP and vertTorque expressed in world frame
//vertTorque is the torqueComponent tangential to the foot sole (computed only when vertTorque is not NULL)
static int computeCOPfromWrench(gVec3* cop, gVec3* vertTorque, gLink* foot, gFootGeom& footGeom, const gVec3& force, const gVec3& torque)
{
	//foot sole is in -Y direction
	if( footGeom.idxUp() == 1 )
	{
		gRotMat R = foot->frame().rot();
		gVec3 forceLocal = R.invMult(force);
		gVec3 torqeLocal = R.invMult(torque);

		if( fabs(forceLocal.y()) < gEPSILON ){
			cop->set(-1e10,-1e10,-1e10); //non-existent
			if(vertTorque) vertTorque->setZero();
			return -1;
		}else{
			gReal cop_x = ( forceLocal.x() * footGeom.lowerBound().e( 1 ) + torqeLocal.z() )/forceLocal.y();
			gReal cop_z = ( forceLocal.z() * footGeom.lowerBound().e( 1 ) - torqeLocal.x() )/forceLocal.y();
			gVec3 copLocal(cop_x,footGeom.lowerBound().e( 1 ),cop_z);
			*cop = foot->frame().multVec4(copLocal);
			if(vertTorque){ 
				gReal vertT = torqeLocal.y() + cop_x*forceLocal.z() - cop_z*forceLocal.x();
				*vertTorque = R*gVec3(0,vertT,0); 
			}
			return 1;
		}
	}
	//foot sole is in -Z direction
	else if ( footGeom.idxUp() == 2)
	{
		gRotMat R = foot->frame().rot();
		gVec3 forceLocal = R.invMult(force);
		gVec3 torqeLocal = R.invMult(torque);

		if( fabs(forceLocal.z()) < gEPSILON ){
			cop->set(-1e10,-1e10,-1e10); //non-existent
			if(vertTorque) vertTorque->setZero();
			return -1;
		}else{
			gReal cop_x = ( forceLocal.x() * footGeom.lowerBound().e( 2 )  - torqeLocal.y() )/forceLocal.z();
			gReal cop_y = ( forceLocal.y() * footGeom.lowerBound().e( 2 ) + torqeLocal.x() )/forceLocal.z();
			gVec3 copLocal(cop_x,cop_y,footGeom.lowerBound().e( 2 ));
			*cop = foot->frame().multVec4(copLocal);
			if(vertTorque){ 
				gReal vertT = torqeLocal.z() - cop_x*forceLocal.y() + cop_y*forceLocal.x();
				*vertTorque = R*gVec3(0,0,vertT); 
			}
			return 1;
		}
	}
	//other cases are not accounted for yet
	else
	{
		assert(0);
		return -1;
	}
}

int bCharacter::getNumContactPoints( bCharacter *sys, gLink *link )
{
	int numContacts = 0;
	for(int i=0; i<link->numCollisionManifolds(); ++i)
	{
		btVector3 f,t;
		btPersistentManifold* pManifold = link->collisionManifold(i);

		numContacts += pManifold->getNumContacts();
	}

	return numContacts;
}

extern bool BD_USE_MLCP_SOLVER;

void bCharacter::detectFootContact()
{
	// 2012-02-15 :	find out corresponding contact point and apply proper contact force..
	//
	for(int i = 0; i < m_sim->m_numBodies; ++i)	link(i)->clearCollisionManifolds();

	/* iterate through manifold which has active contact points */
	int sizeManifold = m_sim->m_btOwnerWorld->getDispatcher()->getNumManifolds();
	for(int i = 0; i < sizeManifold; ++i)
	{
		btPersistentManifold* pManifold = m_sim->m_btOwnerWorld->getDispatcher()->getManifoldByIndexInternal(i);

		int sizeContactPt = pManifold->getNumContacts();
		if(sizeContactPt == 0) continue;
		
		btRigidBody* b0 = (btRigidBody*)pManifold->getBody0();
		btRigidBody* b1 = (btRigidBody*)pManifold->getBody1();

		if(b0==m_sim->m_rFootBtBody || b1==m_sim->m_rFootBtBody) m_rFootLink->addCollisionManifold(pManifold);
		if(b0==m_sim->m_lFootBtBody || b1==m_sim->m_lFootBtBody) m_lFootLink->addCollisionManifold(pManifold);

	}

	m_isRFootContact  = m_isLFootContact = false;
	gLink* link = m_rFootLink;
	m_numContactRight = 0;
	for(int i=0; i< link->numCollisionManifolds(); ++i)
	{
		btPersistentManifold* pManifold = link->collisionManifold(i);		
		m_numContactRight += pManifold->getNumContacts();
	}
	if(m_numContactRight>1) m_isRFootContact = true; //more than one contact point
		
	link = m_lFootLink;
	m_numContactLeft = 0;
	for(int i=0; i< link->numCollisionManifolds(); ++i)
	{
		btPersistentManifold* pManifold = link->collisionManifold(i);		
		m_numContactLeft += pManifold->getNumContacts();
	}
	if(m_numContactLeft>1) m_isLFootContact = true; //more than one contact point

}


void bCharacter::measureFootContactForce( double timeStep)
{
	if(timeStep > DBL_EPSILON) timeStep = 1.0/timeStep;
	// 2012-02-15 :	find out corresponding contact point and apply proper contact force..
	//
	for(int i = 0; i < m_sim->m_numBodies; ++i)	link(i)->clearCollisionManifolds();

	/* iterate through manifold which has active contact points */
	int sizeManifold = m_sim->m_btOwnerWorld->getDispatcher()->getNumManifolds();
	for(int i = 0; i < sizeManifold; ++i)
	{
		btPersistentManifold* pManifold = m_sim->m_btOwnerWorld->getDispatcher()->getManifoldByIndexInternal(i);

		int sizeContactPt = pManifold->getNumContacts();
		if(sizeContactPt == 0) continue;
		
		btRigidBody* b0 = (btRigidBody*)pManifold->getBody0();
		btRigidBody* b1 = (btRigidBody*)pManifold->getBody1();

		if(b0==m_sim->m_rFootBtBody || b1==m_sim->m_rFootBtBody) m_rFootLink->addCollisionManifold(pManifold);
		if(b0==m_sim->m_lFootBtBody || b1==m_sim->m_lFootBtBody) m_lFootLink->addCollisionManifold(pManifold);

	}

	/* detect foot contact forces for each foot */	
	btVector3 force,torque;		
	m_isLFootContact = computeTotalContactWrenchOfLink(&force,&torque,m_lFootLink,m_numContactLeft,timeStep);
	if(m_isLFootContact)
	{
		m_lgrf.set(force.x(),force.y(),force.z());
		if( computeCOPfromWrench(&m_lcop,NULL,m_lFootLink,m_lFootGeom,m_lgrf,convertTogVec3(torque)) == -1 )
		{
			m_isLFootContact = false;
		}
	}
		
	m_isRFootContact = computeTotalContactWrenchOfLink(&force,&torque,m_rFootLink,m_numContactRight,timeStep);
	if(m_isRFootContact)
	{
		m_rgrf.set(force.x(),force.y(),force.z());
		if( computeCOPfromWrench(&m_rcop,NULL,m_rFootLink,m_rFootGeom,m_rgrf,convertTogVec3(torque)) == -1 )
		{
			m_isRFootContact = false;
		}
	}

}

#else 
//Simplistic method to calculate contact force....Not accurate because CoP is determined only by normal force (i.e., friction force
//doesn't affect CoP)
//but may be reasonably useful for flat-ground

void bCharacter::measureFootContactForce( double timeStep )
{
	if(timeStep > DBL_EPSILON) timeStep = 1.0/timeStep;
	// 2012-02-15 :	find out corresponding contact point and apply proper contact force..
	//
	m_isLFootContact = false;
	m_isRFootContact = false; 
	m_lgrf.setZero();
	m_rgrf.setZero();
	m_lcop.set(-1e10,-1e10,-1e10);
	m_rcop.set(-1e10,-1e10,-1e10);

	for(int i = 0; i < m_numBodies; ++i)
		link(i)->clearCollisionManifolds();
 
	/* iterate through manifold which has active contact points */
	int sizeManifold = m_btOwnerWorld->getDispatcher()->getNumManifolds();
	for(int i = 0; i < sizeManifold; ++i)
	{		
		btPersistentManifold* pManifold = m_btOwnerWorld->getDispatcher()->getManifoldByIndexInternal(i);
		gLink* pLink0 = (gLink*)(((btRigidBody*)pManifold->getBody0())->getUserPointer());
		gLink* pLink1 = (gLink*)(((btRigidBody*)pManifold->getBody1())->getUserPointer());
 
		int sizeContactPt = pManifold->getNumContacts();
		if(sizeContactPt == 0) continue;
		if(!(pLink0 ||pLink1) || (pLink0 && pLink1) ) continue; //if non is MBS link or both are MBS links, ignore.

		/* we need to update manifolds to MBS gLink first */
		if(pLink0) pLink0->addCollisionManifold( pManifold );
		if(pLink1) pLink1->addCollisionManifold( pManifold );

		gLink* pLink; //MBS link
		bool flip=false;
		int lr=0; //1 for left foot, 2 for right foot, 0 for others
		if(pLink0){
			pLink = pLink0;
		}else{
			pLink = pLink1;
			flip = true;
		}

		if( !strcmp(pLink->name(), "L_FOOT") ) lr = 1;
		else if( !strcmp(pLink->name(), "R_FOOT") ) lr = 2;
		
		if(lr==0) continue; //ignore

		// compute interpolated center point and force magnitude
		btVector3	ptForce(0,0,0);
		btVector3	ptCenter(0,0,0);
		btVector3	ptFriction(0,0,0);
		btScalar	totalForce = 0;
		for(int j = 0; j < sizeContactPt; ++j)
		{
			const btManifoldPoint& pt = pManifold->getContactPoint(j);
 
			ptForce += (pt.m_normalWorldOnB * pt.m_appliedImpulse); 
			ptCenter += pt.getPositionWorldOnB() * pt.m_appliedImpulse;
			totalForce += (pt.m_appliedImpulse); 
			ptFriction += pt.m_lateralFrictionDir1*pt.m_appliedImpulseLateral2;
			ptFriction += pt.m_lateralFrictionDir2*pt.m_appliedImpulseLateral1;
		}
		ptForce *= timeStep;
		ptFriction *= timeStep;
		if(totalForce) ptCenter /= totalForce;
		totalForce *= timeStep;

		// apply aggregated force impulse to a body
		// we can put this inside of the loop and make updating every point individually
		// this part is no more needed; wrong implementation
		btVector3 contactForce = ptForce + ptFriction;
	
		// set contact forces of each foot	
		if(lr == 1) //left foot
		{
			m_lgrf = convertTogVec3(contactForce);
			if(flip) m_lgrf *= -1;
			m_lcop = convertTogVec3(ptCenter);
			m_isLFootContact = true; 			
		}
		else if(lr == 2) //right foot
		{
			m_rgrf = convertTogVec3(contactForce);
			if(flip) m_rgrf *= -1;
			m_rcop = convertTogVec3(ptCenter);
			m_isRFootContact = true; 			
		}
		else
		{
			//shouldn't reach here.
		}
	}
}

#endif

void bCharacter::recalculateInertiaUsingCollisionShapesAndUpdateMBS(void)
{
	//return;
	btVector3 inertia;
	gReal I[6];
	for(int i=0;i<m_sim->m_numBodies;++i)
	{
		////TEST
		//btRigidBody* rb = m_sim->m_btBodies[i];
		//btVector3 localInertiaInv = rb->getInvInertiaDiagLocal();
		//btVector3 localInertia(1./localInertiaInv.x(), 1./localInertiaInv.y(), 1./localInertiaInv.z());
		////ENDTEST
		
		//recalculate inertia
		btScalar mass = btScalar(link(i)->inertia().mass());
		m_sim->m_btShapes[i]->calculateLocalInertia(mass,inertia); 		

		//update btRigidBody's inertia
		if( link(i)->isAccDriven() )
			m_sim->m_btBodies[i]->setMassProps(0.0,inertia); 
		else
			m_sim->m_btBodies[i]->setMassProps(mass,inertia);

		//make corresponding gInertia
		I[0] = inertia.x(); I[3]=inertia.y(); I[5]=inertia.z(); I[1]=I[2]=I[4]=0.0f; 
		gInertia in(mass,gVec3Zero,I,gInertia::INERTIA_REF_BODY);
		gInertia newIn = in.xform( m_localCoMFrames[i] );
		link(i)->setInertia(newIn);		
	}
}

void bCharacter::reDistributeSegmentWeight( std::map<std::string, double> &weightProportion, double totalWeightKg )
{
	std::map<std::string, double>::iterator it;
	double *weight = new double[m_sim->m_numBodies];
	double sum=0;

	for( int i=0; i<m_sim->m_numBodies; i++ )
	{
		it = weightProportion.find( link(i)->name() );
		if( it != weightProportion.end() )
		{
			weight[i] = it->second;
		} else {
			//weight[i] = 1.0;
			weight[i] = 1e-10;
		}
		sum += weight[i];
	}

	double check = 0;
	for( int i=0; i<m_sim->m_numBodies; i++ )
	{
		//check = totalWeightKg * weight[i] / sum;
		link(i)->inertia().setMass(totalWeightKg * weight[i] / sum);

		//printf("%s : %f\n", link(i)->name(), link(i)->inertia().mass());
	}

	initializeMass();

	delete [] weight;
}


btRigidBody* bCharacter::createRigidBodySim (gRigidBody& rb, gXMat& localCoMFrame, btCollisionShape* shape, const btVector3& positionOffset, int collisionGroup, int collideWithMask)
{
	bool isKinematic = !rb.isAccDriven(); 
		
	//compute localInertia
	gXMat T; T.makeInverse(localCoMFrame); 
	gInertia principalInertia = rb.inertia().xform( T );
		
	//assert( principalInertia's Ixy=Ixz=Iyz=0 )
	
	//assert sum of off diagonal parts should be less than 5% of sum of diagonal parts
	if( (fabs(principalInertia.rotInertia(gInertia::Ixy)) 
		+   fabs(principalInertia.rotInertia(gInertia::Ixz))
		+   fabs(principalInertia.rotInertia(gInertia::Iyz))) 
		> 
		0.05f*(fabs(principalInertia.rotInertia(gInertia::Ixx))+fabs(principalInertia.rotInertia(gInertia::Iyy))+fabs(principalInertia.rotInertia(gInertia::Izz))) 
		)
	{
		printf("%s: Inertia does not match for gRigidBody and btRigidBody. You must call recalculateInertiaUsingCollisionShapesAndUpdateMBS.\n", rb.name());
	}
		

	btVector3 localInertia(
		principalInertia.rotInertia( gInertia::Ixx ),
		principalInertia.rotInertia( gInertia::Iyy ),
		principalInertia.rotInertia( gInertia::Izz ) 
		);

	//initial transform
	btTransform startTransform;
	T = rb.frame()*localCoMFrame; 
	startTransform.setOrigin( positionOffset + btVector3(T.e(gXMat::M03),T.e(gXMat::M13),T.e(gXMat::M23)) );
	gQuat rot = T.rot().inQuat();
	startTransform.setRotation( btQuaternion(rot.x(),rot.y(),rot.z(),rot.w() ) );


	btRigidBody* body;
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);	
	
	
	if(isKinematic){
		btRigidBody::btRigidBodyConstructionInfo rbInfo( btScalar(0), myMotionState, shape, localInertia);
		body = new btRigidBody(rbInfo);
		body->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT);		
		//body->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT);		//setting as CF_KINEMATIC_OBJECT ruins simulation. don't know why.
	}else{
		btRigidBody::btRigidBodyConstructionInfo rbInfo( rb.inertia().mass(), myMotionState, shape, localInertia);
		body = new btRigidBody(rbInfo);
	}

	//
	//collision filtering: http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Filtering
	//
	m_sim->m_btOwnerWorld->addRigidBody(body, collisionGroup, collideWithMask);

	return body;
}

void bCharacter::updateKinematicBodiesOfCharacterSim(void)
{
	for(int i=0;i<m_sim->m_numBodies;++i)
	{
		if( m_sim->m_btBodies[i]->isKinematicObject() ) //this wouldn't work
		//if( link(i)->isAccDriven() )
		{
			gXMat CoMTransform = link(i)->frame()*m_localCoMFrames[i];
			btTransform _CoMTransform = convertToBtTransform(CoMTransform);
			//m_btBodies[i]->getMotionState()->setWorldTransform(_CoMTransform); 
			m_sim->m_btBodies[i]->getMotionState()->setWorldTransform(_CoMTransform); 
			//test
			// sukwon 2013-05-24
			m_sim->m_btBodies[i]->setWorldTransform(_CoMTransform);
		}		
	}
}

void bCharacter::enforcePoseToCharacterSim(void)
{
	for(int i=0;i<m_sim->m_numBodies;++i)
	{
		gXMat CoMTransform = link(i)->frame()*m_localCoMFrames[i];
		btTransform _CoMTransform = convertToBtTransform(CoMTransform);
		m_sim->m_btBodies[i]->setWorldTransform(_CoMTransform);
	}
}

void bCharacterSim::setBtBodiesDynamicOrKinematic(int option)
{	
	for( int i=0; i<getNumBtBodies(); i++ ) m_btBodies[i]->setCollisionFlags(option);	
}


bool bCharacter::inLLeg(int linkIdx)
{
	std::vector<int>::iterator it = std::find(m_lLegIdx.begin(),m_lLegIdx.end(),linkIdx);
	if(it != m_lLegIdx.end()) return true;
	return false;
}

bool bCharacter::inRLeg(int linkIdx)
{
	std::vector<int>::iterator it = std::find(m_rLegIdx.begin(),m_rLegIdx.end(),linkIdx);
	if(it != m_rLegIdx.end()) return true;
	return false;
}


bool bCharacter::inLArm(int linkIdx)
{
	std::vector<int>::iterator it = std::find(m_lArmIdx.begin(),m_lArmIdx.end(),linkIdx);
	if(it != m_lArmIdx.end()) return true;
	return false;
}

bool bCharacter::inRArm(int linkIdx)
{
	std::vector<int>::iterator it = std::find(m_rArmIdx.begin(),m_rArmIdx.end(),linkIdx);
	if(it != m_rArmIdx.end()) return true;
	return false;
}

bool bCharacter::inTrunk(int linkIdx)
{
	std::vector<int>::iterator it = std::find(m_trunkIdx.begin(),m_trunkIdx.end(),linkIdx);
	if(it != m_trunkIdx.end()) return true;
	return false;
}
