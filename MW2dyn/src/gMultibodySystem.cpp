//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
//#include <stdio.h>
//#include <stdlib.h>
//#include <vector>
#include "Base/gBase.h"
#include "mbs/gMultibodySystem.h"
#include "mbs/gArmaUtil.h"
//#include <time.h>

#ifdef MBS_USE_COLLISION_MANIFOLD
//#include "mbs/CollisionWorld.h"
#endif

extern	gVec3	MW_GRAVITY_VECTOR;
const   gVec3	MW_GROUND_NORMAL (gVec3UnitY);

using namespace std;

typedef vector<gLink*>::iterator		 gLinkItorF;
//typedef vector<const gLink*>::const_iterator	 gLinkItorF_const;
typedef vector<gLink*>::const_iterator	 gLinkItorF_const;
typedef vector<gLink*>::reverse_iterator gLinkItorB;

//#define FOR_EACH_CONST_LINK_FORWARD	for(gLinkItorF_const it = _links.begin(); it != _links.end(); it++)

#define FOR_EACH_LINK_FORWARD	for(gLinkItorF it = _links.begin(); it != _links.end(); it++)
#define FOR_EACH_LINK_BACKWARD	for(gLinkItorB it = _links.rbegin(); it != _links.rend(); it++)
#define FOR_EACH_CONST_LINK_FORWARD		for(gLinkItorF_const it = _links.begin(); it != _links.end(); it++)
#define FOR_EACH_CONST_LINK_BACKWARD	for(gLinkItorF_const it = _links.rbegin(); it != _links.rend(); it++)

#define FOR_EACH_JOINT_SPRING	for(gCoordSpringList::iterator it = _jSprings.begin(); it!= _jSprings.end(); it++)
#define	FOR_EACH_STATE_DEPENDENT_FORCE	for(std::vector<gStateDependentForce*>::iterator it = _stateDependentForces.begin(); it!= _stateDependentForces.end(); it++)
#define	FOR_EACH_STATE_INDEPENDENT_FORCE	for(std::vector<gStateIndependentForce*>::iterator it = _stateIndependentForces.begin(); it!= _stateIndependentForces.end(); it++)


void	 gMultibodySystem::setVel		(int startIdx, int size, gReal* v) { vec_set(_dq,startIdx,size,v); }
void	 gMultibodySystem::setAcc		(int startIdx, int size, gReal* a) { vec_set(_ddq,startIdx,size,a); }
void	 gMultibodySystem::setForce		(int startIdx, int size, gReal* torq) { vec_set(_tau,startIdx,size,torq); }

gMultibodySystem::gMultibodySystem()
{	
	//_dq = _ddq = _tau = NULL;
	_mass = 0;
	//_C = NULL;
	//_M = NULL;
	//_callbackContactForce = NULL;
	_szSafeCoordArray = 0;
	_unit = MW_UNDEFINED;
}

gMultibodySystem::~gMultibodySystem()
{
	/*if(_dq)	GV_free(_dq);
	if(_ddq) GV_free(_ddq);
	if(_tau) GV_free(_tau);
	if(_C)	 GV_free(_C);
	if(_M)	GM_free(_M);*/

	FOR_EACH_LINK_FORWARD{
		delete (*it);
	}
	_links.clear();

	FOR_EACH_JOINT_SPRING{
		delete (*it);
	}
	_jSprings.clear();	

	FOR_EACH_STATE_DEPENDENT_FORCE{
		delete (*it);
	}
	_stateDependentForces.clear();

	FOR_EACH_STATE_INDEPENDENT_FORCE{
		delete (*it);
	}
	_stateIndependentForces.clear();

}

void gMultibodySystem::copyVariablesUpToAcc(gMultibodySystem* src)
{
	//synchronize state
	int cnt=0;
	FOR_EACH_LINK_FORWARD{
		(*it)->copyCoord(src->link(cnt++));
	}
	
	_dq = src->vel();
	_ddq = src->acc();
	_tau = src->force();	
	updateKinematicsUptoAcc();	
}

void gMultibodySystem::clearExtForce(void)
{
	FOR_EACH_LINK_FORWARD{
		(*it)->clearExtForce();	
	}
}

void gMultibodySystem::clearAllForces(void)			
{ 
	FOR_EACH_LINK_FORWARD{
		(*it)->clearExtForce();	
		(*it)->clearFrameForce();
	}
	//if(_tau) GV_set_zero(_tau);
	if(!_tau.is_empty()) _tau.zeros();
}

void gMultibodySystem::initializeMass		(void)
{
	_mass = 0;
	FOR_EACH_LINK_FORWARD{
		_mass += (*it)->inertia().mass();
	}
}

void gMultibodySystem::updateKinematicsUptoPos(gLink* startLink)
{
	//update links
	if(startLink)  
		startLink->updateKinematicsUptoPos(true);
	else 
	{
		FOR_EACH_LINK_FORWARD{
			(*it)->updateKinematicsUptoPos();
		}
	}

	FOR_EACH_STATE_DEPENDENT_FORCE{
		(*it)->updateKinematicsUptoPos();
	}

}

void gMultibodySystem::randomizeCoord(gReal maxRange)		// set random joint angle
{
	FOR_EACH_LINK_FORWARD{
		(*it)->randomizeCoord(maxRange);
	}
}

void gMultibodySystem::connect(gLink* plink, gLink* clink)
{
	assert(clink->type() != TYPE_FREE_LINK);
	plink->addChild(clink);
	clink->setParent(plink);
}

void gMultibodySystem::printHierarchy(char* filename)
{
	/*FILE* fp = fopen(filename,"w");
	if ( !fp ) return;
	
	fprintf(fp,"\n Hierarchy \n");
	for(int i=0;i<_nLink;++i)
	{
		fprintf(fp,"%s: \t",_FR[i]->name());
		for(int j=0;j<_FR[i]->numChildren();++j)
		{
			fprintf(fp,"[%s]\t",_FR[i]->child(j)->name() );
		}
		
		fprintf(fp,"\n");
	}

	fclose(fp);*/
}

void gMultibodySystem::postLoading()
{
	// set default pose
	if(!_dq.is_empty()) _dq.zeros();
	if(!_ddq.is_empty()) _ddq.zeros();
	updateKinematicsUptoAcc();
	initializeMass();
}

//find link by its name
//if failed, retun NULL
gLink*	gMultibodySystem::findLink(const char* name)
{
	FOR_EACH_LINK_FORWARD{
		if(!strcmp((*it)->name(),name))	return (*it);
	}
	return NULL;
}

const gLink*	gMultibodySystem::findLink(const char* name) const
{
	//FOR_EACH_CONST_LINK_FORWARD{
	//for(gLinkItorF_const it = _links.begin(); it != _links.end(); it++)
	for( int i=0; i<_links.size(); i++ )
	{
		if(!strcmp(_links[i]->name(),name))	return _links[i];
		//if(!strcmp((*it)->name(),name))	return (*it);
	}
	return NULL;
}

int gMultibodySystem::findLinkIndex(gLink* link)
{
	for(int i=0;i<numLinks();i++){
		if ( link == _links[i] ) return i;
	}
	return -1;
}

int gMultibodySystem::calcSizeSafeCoordArray(void)
{
	int sz=0;
	FOR_EACH_LINK_FORWARD{
		sz += (*it)->sizeSafeCoordArray();
	}
	return sz;
}

void gMultibodySystem::initializeHierarchy(void)
{
	assert( _links.size() > 0 );

	_nLinks = (int)_links.size();	
	_nDof = 0; int id=0;
	FOR_EACH_LINK_FORWARD{
		_nDof += (*it)->dof();
		(*it)->setId(id++);
	}	

}

void gMultibodySystem::initialize(void)
{
	int i,cnt=0;	
	_nJsprings = (int)_jSprings.size();
	
	initializeHierarchy();
	initializeMass();

	/*if(_dq) GV_free(_dq);
	if(_ddq) GV_free(_ddq);
	if(_tau) GV_free(_tau);*/

	if(_nDof > 0){
		_dq.zeros(_nDof);
		_ddq.zeros(_nDof);
		_tau.zeros(_nDof);

		FOR_EACH_LINK_FORWARD{
			for(i=0;i<(*it)->dof();++i){
				(*it)->setGenVelPtr(i, &_dq(cnt)); //(*it)->setGenVelPtr(i, GV_ptr(_dq,cnt) );
				(*it)->setGenAccPtr(i, &_ddq(cnt)); //(*it)->setGenAccPtr(i, GV_ptr(_ddq,cnt));
				(*it)->setGenForcePtr(i, &_tau(cnt));//(*it)->setGenForcePtr(i, GV_ptr(_tau,cnt));
				cnt++;
			}		
		}
	}

	//add gravity force element to stateDependentForces
	FOR_EACH_LINK_FORWARD{
		gGravityForce* g = new gGravityForce();
		g->setRigidBody( (*it) );
		_stateDependentForces.push_back(g);
	}

	setGenCoordIndex();

	_szSafeCoordArray = calcSizeSafeCoordArray();
	_szCompactCoordArray = _nDof;

	_safeCoordArrayIndices.clear();
	_compactCoordArrayIndices.clear();
	int offset = 0; int offset2 = 0;
	FOR_EACH_LINK_FORWARD{		
		_safeCoordArrayIndices.push_back(offset);
		_compactCoordArrayIndices.push_back(offset2);
		offset+=(*it)->sizeSafeCoordArray();
		offset2+=(*it)->dof();
	}
}

void gMultibodySystem::removeLink(gLink* link)
{
	bool removed;
	do{
		removed = false;
		FOR_EACH_LINK_FORWARD{
			if((*it)==link){
				_links.erase(it);
				_nLinks--;
				removed = true;
				break;
			}
		}
	}while(removed);
}

void gMultibodySystem::setZeroCoord	(void)
{
	FOR_EACH_LINK_FORWARD{		
		(*it)->setZeroCoord();
	}
}

void gMultibodySystem::setFromSafeCoordArray(const arma::vec& gc)
{
	int offset = 0;
	FOR_EACH_LINK_FORWARD{		
		(*it)->setFromSafeCoordArray(&gc(offset));
		offset+=(*it)->sizeSafeCoordArray();
	}
}

void gMultibodySystem::getSafeCoordArray(arma::vec& gc) const
{
	assert( gc.n_rows >= _szSafeCoordArray );
	int offset = 0;
	FOR_EACH_CONST_LINK_FORWARD
	{		
		(*it)->getSafeCoordArray(&gc(offset));
		offset+=(*it)->sizeSafeCoordArray();
	}
}

void gMultibodySystem::getCompactCoordArray(arma::vec& rc) const
{
	assert( rc.n_rows >= _nDof );
	int offset = 0;
	FOR_EACH_CONST_LINK_FORWARD
	//for (gLinkItorF_const it = _links.begin(); it != _links.end(); it++)
	{
		//(*it)->getCompactCoordArray(GV_ptr(rc,offset));
		(*it)->getCompactCoordArray(&rc(offset));
		offset += (*it)->dof();
	}
}

void gMultibodySystem::setFromCompactCoordArray(const arma::vec& gc)
{
	int offset = 0;
	FOR_EACH_LINK_FORWARD{		
		//(*it)->setFromCompactCoordArray(GV_const_ptr(gc,offset));
		(*it)->setFromCompactCoordArray(&gc(offset));
		offset+=(*it)->dof();
	}
}

void gMultibodySystem::storeCoord()
{
	FOR_EACH_LINK_FORWARD{
		(*it)->storeCoord();
	}
}



void gMultibodySystem::restoreCoord()
{
	FOR_EACH_LINK_FORWARD{
		(*it)->restoreCoord();
	}
}

gCoordSpring* gMultibodySystem::findJointSpringOfLink	(gLink* link)
{
	FOR_EACH_JOINT_SPRING{
		if(link == (*it)->link() ) return (*it);
	}
	return NULL;
}

gInertia gMultibodySystem::computeLockedInertia(gLink* link)
{
	gInertia re = link->inertia();

	for(int i=0; i<link->numChildren(); ++i)
	{
		re += computeLockedInertia(link->child(i)).xform(link->child(i)->localFrame());
	}

	return re;
}

void gMultibodySystem::updateKinematicsUptoVel(gLink* startLink)		// update f,G,T,V given q,dq
{
	//update links
	if(startLink)  
		startLink->updateKinematicsUptoVel(true);
	else 
	{
		FOR_EACH_LINK_FORWARD{
			(*it)->updateKinematicsUptoVel();
		}
	}
	
	FOR_EACH_STATE_DEPENDENT_FORCE{
		(*it)->updateKinematicsUptoVel();
	}

}


#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE

//update collision object's world transform and AABB, for link and its descendants
static void updateCollisionObjectRecursive(gLink* link)
{
	btCollisionObject* co = link->collisionObject();
	if( co )
	{
		btTransform wt;
		gXMatToBtTransfrom(wt,link->frame());
		co->setWorldTransform(wt);

		//pCollisionWorld->updateSingleAabb(co); //do we need this? we only need narrow phase collision check here.
	}

	for(int i=0;i<link->numChildren();++i)
	{
		updateCollisionObjectRecursive( link->child(i) );
	}
}

static void updateCollisionManifoldRecursive(gLink* link)
{
	int num = link->numCollisionManifolds();
	for(int i=0; i<num; ++i)
	{
		mbsCollisionWorld::computeContactForcesOfManifold( link->collisionManifold(i) , true);
	}

	for(int i=0; i<link->numChildren(); ++i)
	{
		updateCollisionManifoldRecursive( link->child(i) );
	}
}


//NOT VERIFIED YET. NEED TO VALIDATE THIS FUNCTION.
//TODO: For self-collision, manifolds are redundantly updated, which slows down the performance.
//In future, we need to find more clever algorithm to get rid of this redundancy
void gMultibodySystem::updateCollisionManifold(gLink* startLink)
{
	//if(!_considerContactForceChangeInImplicitMethods) return; //SHL commented out because this function is only called when integrator's _considerContactForceChangeInImplicitMethods=true

	//update collision objects' world transforms
	gLink* l = (startLink==NULL) ? _links[0] : startLink;
	updateCollisionObjectRecursive(l);

	//update contact forces in manifolds
	updateCollisionManifoldRecursive(l);
}

#endif

void gMultibodySystem::updateKinematicsUptoAcc(gLink* startLink)		// update f,G,T,V,dV given q,dq,ddq
{
	//update links
	if(startLink)  
		startLink->updateKinematicsUptoAcc(true);
	else 
	{
		FOR_EACH_LINK_FORWARD{
			(*it)->updateKinematicsUptoAcc();
		}
	}
	
	FOR_EACH_STATE_DEPENDENT_FORCE{
		(*it)->updateKinematicsUptoVel(); //don't need to updateKinematicsUptoAcc because it is only dependent on position and velocity
	}
}

void gMultibodySystem::clearStateIndependentForces()
{
	FOR_EACH_STATE_INDEPENDENT_FORCE{
		delete (*it);
	}
	_stateIndependentForces.clear();
}


// set Fext
// every force factors except for control input must be accounted for here.
// Since this is system dependent, child class should override this function for additional forces to be added. 
// TODO: divide forces into three groups: position-dependent forces, velocity-dependent forces, independent forces
// and update the forces only when necessary.
void gMultibodySystem::updateExtForce()
{
	clearExtForce();

	// apply spring forces
	FOR_EACH_JOINT_SPRING{
		(*it)->computeForce();
		(*it)->addForceToBody();		// add joint torque to Fext
	}

	// apply state dependent forces
	FOR_EACH_STATE_DEPENDENT_FORCE{
		(*it)->computeForce();
		(*it)->addForceToBody();
	}
			
	// apply state independent forces
	FOR_EACH_STATE_INDEPENDENT_FORCE{
		(*it)->addForceToBody();
	}

#ifdef MBS_USE_COLLISION_MANIFOLD
	// apply contact forces
	FOR_EACH_LINK_FORWARD{
		(*it)->addContactForces();
	}
#endif

	this->clearStateIndependentForces();
}









bool gMultibodySystem::checkAncestor(gLink* l1, gLink* l2)
{
	gLink* l = l2->parent();	
	while(l){
		if( l1 == l ) return true;
	}
	return false;
}

//static bool firstTime = true;
//int count = 0;
//
//bool gMultibodySystem::findOwnerLink(int i, gLink* l, int* idx)	// find l and idx such that l's _dq[*idx] = _dqp[i]
//{
//	const gReal* x = GV_const_ptr(_dq,i);
//	int j;
//
//	FOR_EACH_LINK_FORWARD{
//		switch((*it)->type()){
//		case TYPE_1D_LINK:
//			if(x == ((g1dLink*)(*it))->genVelPtr()){
//				l = *it;
//				*idx = 0;
//				return true;
//			}
//			break;
//		case TYPE_2D_LINK:
//			for(j=0;j<DOF_2D_LINK;++j){
//				if( x == ((g2dLink*)(*it))->genVelPtr(j) ){
//					l = *it;
//					*idx = j;
//					return true;
//				}
//			}
//			break;
//		case TYPE_BALL_LINK:
//			for(j=0;j<DOF_BALL_LINK;++j){
//				if( x == ((gBallLink*)(*it))->genVelPtr(j) ){
//					l = *it;
//					*idx = j;
//					return true;
//				}
//			}
//			break;
//		case TYPE_FREE_LINK:
//			for(j=0;j<DOF_FREE_LINK;++j){
//				if( x == ((gFreeLink*)(*it))->genVelPtr(j) ){
//					l = *it;
//					*idx = j;
//					return true;
//				}
//			}
//			break;
//		default:
//			break;
//		}
//	}
//
//	return false;
//}
//
//
//gTwist gMultibodySystem::jacobianVector(gLink* l, int i)
//{
//	gTwist J;
//	gLink* p = NULL;
//	int idx;
//	if( !findOwnerLink(i,p,&idx) ) return gTwist(0,0,0,0,0,0);
//	//now p->_dq[idx] == _dq[i]
//	//if p is not an ancestor of l, return zero
//	if( !checkAncestor(p,l) ) return gTwist(0,0,0,0,0,0); 
//
//	gXMat T = p->frame().invMult(l->frame()); // configuration of l->T w.r.t p->T
//	switch(p->type()){
//	case TYPE_1D_LINK:
//		J.makeAdInv(T, ((g1dLink*)p)->screw() );
//		break;
//	case TYPE_2D_LINK:
//		if( idx == 0 ) J.makeAdInv(T, ((g2dLink*)p)->current1stScrew() ); 
//		else J.makeAdInv(T, ((g2dLink*)p)->screw(idx) );
//		break;
//	case TYPE_BALL_LINK:
//		if( idx == 0 ) J.makeAdInv(T, gTwistRotX);
//		else if (idx == 1 ) J.makeAdInv(T, gTwistRotY);
//		else J.makeAdInv(T, gTwistRotZ);
//		break;
//	case TYPE_FREE_LINK:{
//			switch(idx)
//			{
//			case 0: J.makeAdInv(T,gTwistRotX); break;
//			case 1: J.makeAdInv(T,gTwistRotY); break;
//			case 2: J.makeAdInv(T,gTwistRotZ); break;
//			case 3: J.makeAdInv(T,gTwistTrnX); break;
//			case 4: J.makeAdInv(T,gTwistTrnY); break;
//			case 5: J.makeAdInv(T,gTwistTrnZ); break;
//			}
//		}		
//	}
//	return J;
//}
//
//gVec3 gMultibodySystem::dxdq	(gLink* l, int i, gVec3& x_loc)
//{
//	gTwist J = jacobianVector(l,i); // J contains angular and linear derivate of link l w.r.t q[i]
//	gVec3 dxdq = J.multVec4(x_loc); // now dxdq is expressed wrt local frame
//	return l->frame().multVec3(dxdq); 
//}

// inverse dynamics
// Fext,state, desired accelerations -> F,tau
void gMultibodySystem::inverseDynamics()		
{
	//compute necessary force
	FOR_EACH_LINK_BACKWARD{
		(*it)->invDynFrameForce();
		(*it)->frameForceToGenForce();
	}
}

#ifdef MW_USE_DIFF_NEWTON_EULER
void gMultibodySystem::inverseDynamicsP()		
{
	FOR_EACH_LINK_BACKWARD{
		(*it)->computeFrameForceP();
		(*it)->computeGenForceP();
	}
}

void gMultibodySystem::updateExtForceP()
{
	//clear
	FOR_EACH_LINK_FORWARD{ 
		(*it)->_Fp.setZero();	
		(*it)->_Ep.setZero(); 
	}

	// apply spring forces
	FOR_EACH_JOINT_SPRING{
		(*it)->computeForceP();
		(*it)->addForcePToBody();		// add joint torque to Fext
	}

	// apply state dependent forces
	FOR_EACH_STATE_DEPENDENT_FORCE{
		(*it)->computeForceP();
		(*it)->addForcePToBody();
	}		
}

void gMultibodySystem::collectGenForceP(arma::vec& taup)
{
	assert(taup.n_rows == dof());
	int i,j=0;
	FOR_EACH_LINK_FORWARD
	{
	for(i=0;i<(*it)->dof();++i)
	{
		//GV_set(taup,j++,(*it)->genForceP(i));
		taup(j++) = (*it)->genForceP(i);
	}
	}
}

void gMultibodySystem::setZeroKVariablesP(void)
{
	FOR_EACH_LINK_FORWARD{
		(*it)->setZeroKVariablesP();
	}
}

//if keepFrameP==true, gLink->updateKVariablesP() does not update _Gp,_Tp
void gMultibodySystem::updateKVariablesP(gLink* startLink)
{
	if( startLink==NULL )
		_links[0]->updateKVariablesP(true);
	else
		startLink->updateKVariablesP(true);


	FOR_EACH_STATE_DEPENDENT_FORCE{
		(*it)->updateKVariablesP();
	}

}



#endif //MW_USE_DIFF_NEWTON_EULER



// compute first six rows of equations of motion for the floating base, i.e., 0 = M0 * _ddq + C0
// return true if success, false otherwise.
// time complexity of this algorithm is O(n+6*n), first n for C0, next 6*n for M0
bool gMultibodySystem::computeJointSpaceEquationsOfMotionFloatingBasePartOnly(arma::mat& M0, arma::vec& C0)//(GM* M0, GV* C0) 
{
	if(/*M0.is_empty() || C0.is_empty() ||*/ M0.n_rows < 6 || M0.n_cols < _nDof || C0.n_cols < 6) return false;

	if(_C.is_empty()) _C.zeros(_nDof);
	arma::vec _tau0 = _tau; 	//store input forces
	
	updateExtForce(); //only state-dependent forces are included

	//compute C
	_ddq.zeros();//GV_set_zero(_ddq);
	updateKinematicsUptoAcc();
	inverseDynamics();

	_C = _tau; //GV_memcpy(_C,_tau);
	C0 = _C.subvec(0,5); //GV_memcpy(C0,&GV_subvector(_C,0,6).vector); // C0 <- first 6 elements of _C

	//compute M0
	for(int i=0;i<6;++i)
	{
		_ddq(i)=1.0;//GV_set(_ddq,i,1);
		updateKinematicsUptoAcc();
		inverseDynamics();
		_tau -= _C; //GV_sub(_tau,_C);
		M0.row(i) = _tau.t(); //GV_memcpy(&GM_row(M0,i).vector,_tau);
		_ddq(i)=0.0;//GV_set(_ddq,i,0);
	}

	_tau = _tau0;  //GV_memcpy(_tau,_tau0);  //restore input forces
	//GV_free(_tau0);


	////test
	//computeJointSpaceEquationsOfMotion();
	//GM_sub(M0, &GM_submatrix(_M,0,0,6,_nDof).matrix);
	//gReal sum=0;
	//for(int i=0;i<M0->size1;++i)
	//{
	//	for(int j=0;j<M0->size2;++j)
	//	{
	//		sum += fabs(GM_get(M0,i,j));
	//	}
	//}

	return true;
}

//compute joint space mass matrix _M and _C vector.
//where _tau = _M*_ddq + _C
//using Composite Inertia Method
//this algorithm computes _M in O(n*n/2) using the symmetry of _M
//computing _C is O(n)
void gMultibodySystem::computeJointSpaceEquationsOfMotion(void) 
{
	if(_C.is_empty()) _C.zeros(_nDof); //_C = GV_alloc(_nDof);
	if(_M.is_empty()) _M.zeros(_nDof,_nDof);
	arma::vec _tau0(_nDof); //GV* _tau0 = GV_alloc(_nDof);
		
	//compute C
	updateExtForce();
	_tau0=_tau;//GV_memcpy(_tau0,_tau); 	//store input forces
	_ddq.zeros();//GV_set_zero(_ddq);
	updateKinematicsUptoAcc();
	inverseDynamics();
	_C=_tau;//GV_memcpy(_C,_tau);
	_tau=_tau0;//GV_memcpy(_tau,_tau0);  //restore input forces

	//update composite inertia and screwWorld
	computeCompositeInertiasWorld();
	FOR_EACH_LINK_FORWARD{
		(*it)->updateScrewWorld();
	}
	
	//compute M
	gLink* p;
	_M.zeros();//GM_set_zero(_M);
	int i,j,k,l;
	j=0;
	FOR_EACH_LINK_FORWARD{
	for(k=0;k<(*it)->dof();++k)
	{
		gWrench fj = (*it)->compositeInertiaWorld() * (*it)->screwWorld(k);
		p = (*it);
		while(p)
		{
			for(l=p->dof()-1;l>=0;--l)
			{
				i = p->genCoordIndex()+l;
				if(i<=j){
					gReal Mij = (fj , p->screwWorld(l));
					_M(i,j)=Mij;//GM_set(_M,i,j,Mij);
					_M(j,i)=Mij;//GM_set(_M,j,i,Mij);
				}
			}			
			p = p->parent();
		}
		j++;
	}}

	//GV_free(_tau0);
}

//Forward dynamics using Orin's method (slower version)
// computeEquationsOfMotion must be set true if M and C is not current
//_tau is corrupted
void gMultibodySystem::computeAccelerationOrin(bool computeEquationsOfMotion)
{
	if(computeEquationsOfMotion)
	{
		//given Q,dQ,ddQ,Fext -> compute ddQ using Orin's method
		//tau = _M*ddQ + _C
		computeJointSpaceEquationsOfMotion();
	}
	//now solve _M*ddQ = _tau-_C using Cholesky decomposition
	_tau = _C; //GV_sub(_tau,_C);	
	
	//solve _M*_ddq = _tau
	//gsl_linalg_cholesky_decomp(_M);
	//gsl_linalg_cholesky_solve(_M,_tau,_ddq); 

	//R: upper tri. M = R.t * R
	//first solve R.t * z = _tau
	//then solve R *_ddq = z
	arma::mat R = chol(_M);
	arma::vec z = arma::solve(trimatl(R.t()), _tau);
	_ddq = arma::solve(trimatu(R),z);
}


void gMultibodySystem::computeCompositeInertiasWorld(void)
{
	FOR_EACH_LINK_BACKWARD
	{
		(*it)->updateCompositeInertiaWorld();
	}
}

void gMultibodySystem::setGenCoordIndex(void)	// set genCoordIndex of each link
{
	int idx = 0;
	FOR_EACH_LINK_FORWARD
	{
		if( (*it)->dof() > 0 )
		{
			(*it)->setGenCoordIndex(idx);
			idx += (*it)->dof();
		}
	}
}

#ifdef MW_USE_ARTICULATED_BODY_METHOD

//Forward dynamics using Featherstone's method
//If updateOnlyBiasForce, updates b only without updating A, thus it is faster than updateOnlyBiasForce=false.
//Set updateOnlyBiasForce to true when the coordinates are not changed but only velocities is changed from the previous state.
//Note that A is entirely determined by the coordinates regardless of velocities wheareas b is dependent on both.
void gMultibodySystem::hybridDynamicsFeatherstone(bool updateOnlyBiasForce)
{
	//backward recursion
	FOR_EACH_LINK_BACKWARD{
		(*it)->updateArtInertiaAndBiasForce(updateOnlyBiasForce);
	}
	//forward recursion
	FOR_EACH_LINK_FORWARD{
		(*it)->solAcc();
	}
}


#endif 

int gMultibodySystem::calcSizeSafeCoordArrayOfLinks(const std::vector<int>& linkIdx)
{
	int sz=0;
	for(std::vector<int>::const_iterator it = linkIdx.begin(); it != linkIdx.end(); it++)
	{
		sz += _links[(*it)]->sizeSafeCoordArray();
	}
	return sz;
}


void gMultibodySystem::setFromSafeCoordArrayOfLinks(const std::vector<int>& linkIdx,const arma::vec& gc)
{
	int offset = 0;
	for(std::vector<int>::const_iterator it = linkIdx.begin(); it != linkIdx.end(); it++)
	{			
		_links[(*it)]->setFromSafeCoordArray(&gc(offset));
		offset+=_links[(*it)]->sizeSafeCoordArray();
	}
}

void gMultibodySystem::getSafeCoordArrayOfLinks(const std::vector<int>& linkIdx,arma::vec& gc)
{
	assert( gc.n_rows >= calcSizeSafeCoordArray() );
	int offset = 0;
	for(std::vector<int>::const_iterator it = linkIdx.begin(); it != linkIdx.end(); it++)
	{		
		_links[(*it)]->getSafeCoordArray(&gc(offset));
		offset+=_links[(*it)]->sizeSafeCoordArray();
	}
}

void gMultibodySystem::setFromCompactCoordArrayOfLinks(const std::vector<int>& linkIdx, const arma::vec& gc)
{
	
	assert(gc.n_rows > linkIdx.size() * 3);
	int offset = 0;
	for (std::vector<int>::const_iterator it = linkIdx.begin(); it != linkIdx.end(); it++)
	{
		_links[(*it)]->setFromCompactCoordArray(&gc(offset));
		offset += _links[(*it)]->dof();
	}
}

void gMultibodySystem::getCompactCoordArrayOfLinks(const std::vector<int>& linkIdx, arma::vec& gc) {
	assert(gc.n_rows > linkIdx.size()*3);
	int offset = 0;
	for (std::vector<int>::const_iterator it = linkIdx.begin(); it != linkIdx.end(); it++)
	{
		_links[(*it)]->getCompactCoordArray(&gc(offset));
		offset += _links[(*it)]->dof();
	}
}