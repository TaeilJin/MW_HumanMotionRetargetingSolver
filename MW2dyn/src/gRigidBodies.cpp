//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------

#include "mbs/gRigidBodies.h"
//#include <process.h>	// for Semaphore

extern gVec3 MW_GRAVITY_VECTOR;

using namespace std;

typedef vector<gLink*>::iterator gLinkItor;

#define FOR_EACH_CHILD		for(gLinkItor it = _children.begin(); it != _children.end(); it++)

static gReal randomRange(gReal low, gReal up)
{
	return rand()*((up-low)/RAND_MAX) + low;
}

void g1dLink::randomizeCoord(gReal* low, gReal* up)
{
	_q = randomRange(low[0],up[0]);
}

void g2dLink::randomizeCoord(gReal* low, gReal* up)
{
	_q[0] = randomRange(low[0],up[0]);
	_q[1] = randomRange(low[1],up[1]);
}

void gBallLink::randomizeCoord(gReal* low, gReal* up)
{
	gReal x = randomRange(low[0],up[0]); 
	gReal y = randomRange(low[1],up[1]);
	gReal z = randomRange(low[2],up[2]);
	_R.makeRotateXYZ(x,y,z);
}

void gFreeLink::randomizeCoord(gReal* low, gReal* up)
{
	gReal rx = randomRange(low[0],up[0]); 
	gReal ry = randomRange(low[1],up[1]);
	gReal rz = randomRange(low[2],up[2]);
	gReal tx = randomRange(low[0],up[0]); 
	gReal ty = randomRange(low[1],up[1]);
	gReal tz = randomRange(low[2],up[2]);
	gRotMat R; R.makeRotateXYZ(rx,ry,rz);
	_G = gXMat(R,gVec3(tx,ty,tz));
}

gRigidBody::gRigidBody()
{
	_bAccDriven	= false;
	_bMassless	= false;		
	_ID = -1;
	//_collisionObject = NULL;
#ifdef gUSE_MULTI_THREADING_IN_E
	hSemaphoreFext = CreateSemaphore(NULL,1,1,NULL);
#endif
}

gRigidBody::~gRigidBody()
{
#ifdef gUSE_MULTI_THREADING_IN_E
	CloseHandle(hSemaphoreFext);
#endif
}

void gRigidBody::setFromSafeCoordArray(const gReal* ptr)
{
	//first four is for quaternion
	gQuat q(ptr);
	q.normalize();
	_T.setRot(q.inRotMatrix());
	_T.setTrn(ptr+4);
}

void g1dLink::setFromSafeCoordArray(const gReal* ptr)
{
	_q = *ptr;	
}

void g1dLink::setFromCompactCoordArray(const gReal* ptr)
{
	_q = *ptr;	
}

void gBallLink::setFromSafeCoordArray(const gReal* ptr)
{
	gQuat Q(ptr);
	setCoord(Q);	
}

void gBallLink::setFromCompactCoordArray(const gReal* ptr)
{
	setCoord( gRotMat::exp(gVec3(ptr[0],ptr[1],ptr[2])));	
}

void gRigidBody::getSafeCoordArray(gReal* ptr) const
{
	//first four is for quaternion
	gQuat quat(_T.rotInQuat());
	ptr[0]=quat.e(0);
	ptr[1]=quat.e(1);
	ptr[2]=quat.e(2);
	ptr[3]=quat.e(3);
	gVec3 trn(_T.trn());
	ptr[4]=trn.x();
	ptr[5]=trn.y();
	ptr[6]=trn.z();
}

void gLink::removeChild(gLink* link)	// remove if links is in _children
{
	bool removed;
	do{
		removed = false;
		FOR_EACH_CHILD{
			if( (*it) == link ){
				it = _children.erase(it);
				_numChildren--;
				removed = true;
				break;
			}
		}	
	}while(removed);
}

void gBallLink::randomizeCoord(gReal maxRange)
{
	gReal x = maxRange*(((gReal)2*rand())/RAND_MAX - 1);
	gReal y = maxRange*(((gReal)2*rand())/RAND_MAX - 1);
	gReal z = maxRange*(((gReal)2*rand())/RAND_MAX - 1);
	_R.makeRotateXYZ(x,y,z);
}

void gBallLink::timeIntegrateSemiImplicitEuler(gReal time_step)
{
	// update dq
	*_dq[0] += (*_ddq[0])*time_step;
	*_dq[1] += (*_ddq[1])*time_step;
	*_dq[2] += (*_ddq[2])*time_step;
	// update R
	_R *= gRotMat::exp(gVec3(*_dq[0]*time_step,*_dq[1]*time_step,*_dq[2]*time_step));
}

void gBallLink::timeIntegrateExplicitEuler(gReal time_step)
{
	// update R
	_R *= gRotMat::exp(gVec3(*_dq[0]*time_step,*_dq[1]*time_step,*_dq[2]*time_step));
	// update dq
	*_dq[0] += (*_ddq[0])*time_step;
	*_dq[1] += (*_ddq[1])*time_step;
	*_dq[2] += (*_ddq[2])*time_step;
}	

void g1dLink::getSafeCoordArray(gReal* ptr) const
{
	*ptr = _q;
}

void g1dLink::getCompactCoordArray(gReal* ptr) const
{
	*ptr = _q;
}

void g2dLink::getSafeCoordArray(gReal* ptr) const
{
	ptr[0] = _q[0];
	ptr[1] = _q[1];
}

void g2dLink::getCompactCoordArray(gReal* ptr) const
{
	ptr[0] = _q[0];
	ptr[1] = _q[1];
}

void gBallLink::getSafeCoordArray(gReal* ptr) const
{
	gQuat quat(_R.inQuat());
	ptr[0]=quat.e(0);
	ptr[1]=quat.e(1);
	ptr[2]=quat.e(2);
	ptr[3]=quat.e(3);
}

void gBallLink::getCompactCoordArray(gReal* ptr) const
{
	gVec3 v = gVec3::log(_R);
	ptr[0]=v.x();
	ptr[1]=v.y();
	ptr[2]=v.z();
}



// inverse dynamics
// given _V,_dV,_Fext, solve for proper F
void gRigidBody::invDynFrameForce(void)	
{
	// F + E = J*dV-dad(_V,JV) 
	_F = _Fext*(-1);

	if(!_bMassless){
		_F += _J*_dV;
		_F -= gWrench::dad(_V,_J*_V);
	}
}

void gLink::invDynFrameForce(void)
{
	// F + E = dAdInv(F(i+1))+J*dV-dad(_V,JV) 
	gRigidBody::invDynFrameForce();

	FOR_EACH_CHILD{	
		_F += (*it)->frameForce().xform( (*it)->localFrame() );
	}
}


void gRigidBody::solAcc(void)		// solve F = J*dV - dad(V,JV) - E
{
	if( isAccDriven() ){
		_dV.setZero(); 
	}
	else{
		_dV.solve(_J, _F + _Fext + gWrench::dad(_V,_J*_V) );	
	}
}

void gRigidBody::timeIntegrateSemiImplicitEuler(gReal time_step)
{
	if( _bAccDriven ) return;

	gTwist V = _V+_dV*time_step;
	
	gRotMat R = gRotMat::exp(V.rot()*time_step);
	gXMat dT(R,V.trn()*time_step);
	_T*=dT;

	_V.makeAdInv(dT,V);	

}

void gRigidBody::timeIntegrateExplicitEuler(gReal time_step)
{
	if( _bAccDriven ) return;

	////////METHOD 1 ... use T_new = T*exp(V*time_step)
	//gXMat dT; dT.exp(_V*time_step); 
	//_T*=dT;
	//gTwist V = _V+_dV*time_step;
	//_V.makeAdInv(dT,V);	
	
	////////METHOD 2...directly use the relation V = inv(T)*dot(T)
	// a couple of experiments show that this method is about twice accurate that Method 1
	gRotMat R = gRotMat::exp(_V.rot()*time_step);
	gXMat dT(R,_V.trn()*time_step);
	_T*=dT;
	gTwist V = _V+_dV*time_step;
	_V.makeAdInv(dT,V);	//coordinate transformation to next frame. this is better than simply setting _V = V;

	//////// METHOD 3...similar to method2, but R_new = R*proj(eye+[w])
	//// results are more or less same with Method 2. 
	//// This method is not better than METHOD 2 in that it needs division.
	//gVec3 w=_V.rot()*time_step;
	//gReal len = w.magnitude();
	//gRotMat R;
	//if(len > gEPSILON){
	//	w /= len;
	//	gReal th = acos(1./gSqrt(1+gSqr(len)));
	//	R = gRotMat::exp(w*th);
	//}
	//gXMat dT(R,_V.trn()*time_step);
	//_T*=dT;
	//gTwist V = _V+_dV*time_step;
	//_V.makeAdInv(dT,V);
}

void g1dLink::timeIntegrateSemiImplicitEuler(gReal time_step)
{
	// update _V
	*_dq += (*_ddq)*time_step;
	// update q
	_q += (*_dq)*time_step;
}

void g1dLink::timeIntegrateExplicitEuler(gReal time_step)
{
	// update q
	_q += (*_dq)*time_step;
	// update _V
	*_dq += (*_ddq)*time_step;
}

void g1dLink::timeIntegrateExplicitEulerPositionOnly(gReal time_step)	//called to update pos when velocity is alread updated
{
	// update q
	_q += (*_dq)*time_step;
}


void	gLink::updateCompositeInertiaWorld(void)
{
	_Jcw = _J.xform(_T); 
	for(unsigned int i=0;i<_children.size();++i)
	{
		_Jcw += _children[i]->compositeInertiaWorld();
	}
}

void	gLink::updateFrame(void)					//update _G, _T
{
	updateLocalFrame();
	if( _parent )
	{
		_T	= _parent->frame() * _G;	
		_T0 = _parent->baseFrame() * _G;
	}
	else
	{
		_T = _G;
	}
}

void	gLink::updateFrameVel(void)						//update _U, _V, _C
{ 
	updateLocalVel();
	if(_parent){
		_V = _parent->frameVel().xformInv(_G) + _U;
	}
	else{
		_V = _U;
	}
	solCoriolisAcc();
}
void	gLink::updateFrameAcc(void)						// update _Sddq, _dV
{
	updateLocalAcc();
	if(_parent){
		_dV = _parent->frameAcc().xformInv(_G) + _C;
	}
	else{
		_dV = _C;
	}
	_dV += _Sddq;
}


void gLink::updateKinematicsUptoPos(bool recursive)
{
	updateFrame();

	if(recursive)
	{
		for(gLinkList::iterator it = _children.begin(); it!=_children.end(); it++) (*it)->updateKinematicsUptoPos(true);
	}
}

void gLink::updateKinematicsUptoVel(bool recursive)
{
	updateFrame();				// f,G,T
	updateFrameVel();				// _V

	if(recursive)
	{
		for(gLinkList::iterator it = _children.begin(); it!=_children.end(); it++) (*it)->updateKinematicsUptoVel(true);
	}
}

void gLink::updateKinematicsUptoAcc(bool recursive)
{
	updateFrame();				// f,G,T
	updateFrameVel();				// _V
	updateFrameAcc();				// dV

	if(recursive)
	{
		for(gLinkList::iterator it = _children.begin(); it!=_children.end(); it++) (*it)->updateKinematicsUptoAcc(true);
	}
}

void gRigidBody::addGravity(void)				// add gravity to Fext
{
	gVec3 gravityLocal = _T.invMultVec3(MW_GRAVITY_VECTOR*_J.mass());
	addExtForce(gWrench( _J.comInVec3() % gravityLocal ,gravityLocal));
}

gReal gRigidBody::calcPotentialEnergy(gReal height0)
{
	return _bMassless? gZero : -1*_J.mass()*((MW_GRAVITY_VECTOR,_T.multVec4( _J.com() )) - height0);
}



void gFreeLink::setFromSafeCoordArray(const gReal* ptr)	
{ 	
	_G.setTrn(ptr+4);
	gQuat q(ptr);
	q.normalize();
	_G.setRot(q.inRotMatrix());	

	//_G.setTrn(ptr);
	//gQuat q(ptr+3);
	//q.normalize();
	//_G.setRot(q.inRotMatrix());	
}

void gFreeLink::setFromCompactCoordArray(const gReal* ptr)	
{ 	
	_G.setRot( gRotMat::exp(gVec3(ptr[0],ptr[1],ptr[2])));	
	_G.setTrn(ptr+3);

	//_G.setTrn(ptr);
	//_G.setRot( gRotMat::exp(gVec3(ptr[3],ptr[4],ptr[5])));	
}

void gFreeLink::getSafeCoordArray(gReal* ptr) const
{
	gQuat quat(_G.rotInQuat());
	ptr[0]=quat.e(0);
	ptr[1]=quat.e(1);
	ptr[2]=quat.e(2);
	ptr[3]=quat.e(3);	
	gVec3 trn(_G.trn());
	ptr[4]=trn.x();
	ptr[5]=trn.y();
	ptr[6]=trn.z();


	////first three are for translation
	//gVec3 trn(_G.trn());
	//ptr[0]=trn.x();
	//ptr[1]=trn.y();
	//ptr[2]=trn.z();
	////next four are for quaternion
	//gQuat quat(_G.rotInQuat());
	//ptr[3]=quat.e(0);
	//ptr[4]=quat.e(1);
	//ptr[5]=quat.e(2);
	//ptr[6]=quat.e(3);	
}

void gFreeLink::getCompactCoordArray(gReal* ptr) const
{
	gVec3 v = gVec3::log(_G.rot());
	ptr[0]=v.x();
	ptr[1]=v.y();
	ptr[2]=v.z();	
	ptr[3]=_G.e(12);
	ptr[4]=_G.e(13);
	ptr[5]=_G.e(14);

	//first three are for exponential coordinates
	//gVec3 v = gVec3::log(_G.rot());
	//ptr[0]=_G.e(12);
	//ptr[1]=_G.e(13);
	//ptr[2]=_G.e(14);
	//ptr[3]=v.x();
	//ptr[4]=v.y();
	//ptr[5]=v.z();	
}

void gFreeLink::randomizeCoord(gReal maxRange)
{
	gReal rx = maxRange*(((gReal)2*rand())/RAND_MAX - 1);
	gReal ry = maxRange*(((gReal)2*rand())/RAND_MAX - 1);
	gReal rz = maxRange*(((gReal)2*rand())/RAND_MAX - 1);
	gReal tx = maxRange*(((gReal)2*rand())/RAND_MAX - 1);
	gReal ty = maxRange*(((gReal)2*rand())/RAND_MAX - 1);
	gReal tz = maxRange*(((gReal)2*rand())/RAND_MAX - 1);
	gRotMat R; R.makeRotateXYZ(rx,ry,rz);
	_G = gXMat(R,gVec3(tx,ty,tz));
}

void gFreeLink::timeIntegrateSemiImplicitEuler(gReal time_step)
{
	copyDqToU();
	copyDdqToSddq();
	
	//refer to gRigidBody::timeIntegrateExplicitEuler()
	gTwist U = _U+_Sddq*time_step;

	gRotMat R = gRotMat::exp(U.rot()*time_step);
	gXMat dG(R,U.trn()*time_step);
	_G*=dG;

	_U.makeAdInv(dG,U);	//coordinate transformation to next frame. this is better than simply setting _V = V;

	copyUToDq();
}


void gFreeLink::timeIntegrateExplicitEuler(gReal time_step)
{
	copyDqToU();
	copyDdqToSddq();
	
	//refer to gRigidBody::timeIntegrateExplicitEuler()
	gRotMat R = gRotMat::exp(_U.rot()*time_step);
	gXMat dG(R,_U.trn()*time_step);
	_G*=dG;
	gTwist U = _U+_Sddq*time_step;
	_U.makeAdInv(dG,U);	//coordinate transformation to next frame. this is better than simply setting _V = V;

	copyUToDq();
}

void gFreeLink::timeIntegrateExplicitEulerPositionOnly(gReal time_step)
{
	copyDqToU();
	gRotMat R = gRotMat::exp(_U.rot()*time_step);
	gXMat dG(R,_U.trn()*time_step);
	_G*=dG;
}

void gFreeLink::moveEpsilon(int e,gReal eps)
{
	gXMat E;
	switch(e)
	{
	case 0:
		E.makeRotateX(eps); break;
	case 1:
		E.makeRotateY(eps); break;
	case 2:
		E.makeRotateZ(eps); break;
	case 3:
		E.setTrn(gVec3(eps,0,0)); break;
	case 4:
		E.setTrn(gVec3(0,eps,0)); break;
	case 5:
		E.setTrn(gVec3(0,0,eps)); break;
	}								
	_G *= E;
}


#ifdef MW_USE_ARTICULATED_BODY_METHOD
void	gWeldedLink::solAcc(void)
{
	if( isAccDriven() )	
	{
		if(_parent){
			_dV = _parent->frameAcc().xformInv(_G);
			_F = _A*_dV + _b;
		}
		else
		{
			_dV.setZero();
			_F = _b;
		}
	}
	else
	{
		_dV = _parent ? _parent->frameAcc().xformInv(_G) : gTwistZero;
	}
}
void gFreeLink::solAcc(void)
{
	//copyTauToF();
	//if( isAccDriven() ){
	// _dV.setZero();
	//}
	//else{
	// _dV = _A.solve(_F-_b); // using cholesky decomposition
	//}
	//copyDVToDdq();

	//REVISED 2012-7-24 BY SHL
 
	if( isAccDriven() ){
		copyDdqToSddq();
		_dV = _Sddq;
		_F = _A * _dV + _b;  
		*_tau[0] = _F.e(0); *_tau[1] = _F.e(1); *_tau[2] = _F.e(2);
		*_tau[3] = _F.e(3); *_tau[4] = _F.e(4); *_tau[5] = _F.e(5);
	}
	else{
		copyTauToF();
		_dV = _A.solve(_F-_b); // using cholesky decomposition
		copyDVToDdq();
	}
}	

void g1dLink::solAcc(void)		// compute dV, ddq
{
	if( isAccDriven() ){		//compute dV only since ddq is given	
		//_Sddq = _S*genAcc();
		_dV = _parent->frameAcc().xformInv(_G)  + _Sddq + _C;
		//below two lines compute necessary torque
		_F = _A*_dV + _b;
		frameForceToGenForce();
	}
	else{
		gTwist dVp = _parent->frameAcc().xformInv(_G);
		*_ddq = _W*( genForce() - (_S,_A*dVp+_x) ); //*_ddq = _W*( genForce() - (_S,_A*(dVp + _C ) + _b));
		_Sddq = _S*genAcc(); 
		_dV = dVp + _Sddq + _C;
	}
}

void g2dLink::solAcc(void)
{
	if( isAccDriven() ){		//compute dV only since ddq is given
		_dV = _parent->frameAcc().xformInv(_G) + _Sddq + _C;
		_F = _A*_dV + _b;
		frameForceToGenForce();
	}
	else{
		gTwist dVp = _parent->frameAcc().xformInv(_G);
		gWrench y = _A*dVp + _x;
		
		gReal t0 = *_tau[0] - (_S0, y) ;
		gReal t1 = *_tau[1] - (_S[1],y);

		*_ddq[0] = _W[0]*t0 + _W[1]*t1;
		*_ddq[1] = _W[1]*t0 + _W[2]*t1;

		updateLocalAcc();
		_dV = dVp + _Sddq + _C;
	}
}

void gBallLink::solAcc(void)		// compute dV, ddq
{
	if( isAccDriven() ){		//compute dV only since ddq is given
		_dV = _parent->frameAcc().xformInv(_G) + _Sddq + _C;
		//{
		_F = _A*_dV + _b; 
		frameForceToGenForce();
		//} remove these two lines if you don't want to compute tau
	}
	else{
		gTwist dVp = _parent->frameAcc().xformInv(_G);
		gWrench AdVp = _A*dVp;
		gReal t0 = *_tau[0] - _x.e(0) - AdVp.e(0);
		gReal t1 = *_tau[1] - _x.e(1) - AdVp.e(1);
		gReal t2 = *_tau[2] - _x.e(2) - AdVp.e(2);
		//*_ddq = _W*( genForce() - (_S,_A*dVp+_x) ); //*_ddq = _W*( genForce() - (_S,_A*(dVp + _C ) + _b));
		*_ddq[0] = _W[0]*t0+_W[1]*t1+_W[2]*t2;
		*_ddq[1] = _W[1]*t0+_W[3]*t1+_W[4]*t2;
		*_ddq[2] = _W[2]*t0+_W[4]*t1+_W[5]*t2;
		updateLocalAcc();
		_dV = dVp + _Sddq + _C;
	}
}

//compute _A, _b, _W, _c
void g1dLink::updateArtInertiaAndBiasForce(bool updateOnlyBiasForce)
{
	gLink::updateArtInertiaAndBiasForce(updateOnlyBiasForce);
	
	if(!updateOnlyBiasForce){
		_AS= _A*_S;
		_W = gInv((_S,_AS));
	}
	_x = _A*_C + _b;
}

void g2dLink::updateArtInertiaAndBiasForce(bool updateOnlyBiasForce)
{
	gLink::updateArtInertiaAndBiasForce(updateOnlyBiasForce);

	if(!updateOnlyBiasForce){
		//_W
		_AS0 = _A*_S0;
		_AS1 = _A*_S[1];
		gReal a = (_S0,_AS0);
		gReal b = (_S0,_AS1);
		gReal c = (_S[1],_AS1);
		gReal idet = gInv(a*c-b*b);
		_W[0] = idet*c;
		_W[1] = -idet*b;
		_W[2] = idet*a;
	}
	_x = _A*_C + _b;
}

void gBallLink::updateArtInertiaAndBiasForce(bool updateOnlyBiasForce)
{
	gLink::updateArtInertiaAndBiasForce(updateOnlyBiasForce);

	if(!updateOnlyBiasForce){
		// _W is the inverse of 3x3 upper-left submatrix of A
		gReal idet = gInv(( _A.e(0,0)*(_A.e(1,1)*_A.e(2,2)-_A.e(1,2)*_A.e(1,2))-_A.e(0,1)*(_A.e(0,1)*_A.e(2,2)-2*_A.e(0,2)*_A.e(1,2))-_A.e(0,2)*_A.e(0,2)*_A.e(1,1) ));
		_W[0] = (_A.e(1,1)*_A.e(2,2)-_A.e(1,2)*_A.e(1,2))*idet;
		_W[1] = (_A.e(0,2)*_A.e(1,2)-_A.e(0,1)*_A.e(2,2))*idet;
		_W[2] = (_A.e(0,1)*_A.e(1,2)-_A.e(0,2)*_A.e(1,1))*idet;
		_W[3] = (_A.e(0,0)*_A.e(2,2)-_A.e(0,2)*_A.e(0,2))*idet;
		_W[4] = (_A.e(0,1)*_A.e(0,2)-_A.e(0,0)*_A.e(1,2))*idet;
		_W[5] = (_A.e(0,0)*_A.e(1,1)-_A.e(0,1)*_A.e(0,1))*idet;
	}

	_x = _A*_C + _b;
}



gWrench gBallLink::sol_xx(void)
{
	//xx = A*S*W*(tau-S'*x)
	gReal t0 = *_tau[0]-_x.e(0);
	gReal t1 = *_tau[1]-_x.e(1);
	gReal t2 = *_tau[2]-_x.e(2);
	return gWrench(
		t0,
		t1,
		t2,
		_a30*t0+_a31*t1+_a32*t2,
		_a40*t0+_a41*t1+_a42*t2,
		_a50*t0+_a51*t1+_a52*t2 );
}


//set updateOnlyBiasForce to true only when coordinates have not been changed since last time _A was computed.
//Note that _A is dependent only on coordinates
void gLink::updateArtInertiaAndBiasForce(bool updateOnlyBiasForce)
{
	if(!updateOnlyBiasForce){
		if(_bMassless){
			_A.setZero();
			_b = _Fext*(-1);
		}
		else{
			_A = _J;
			_b = gWrench::dad(_V,_J*_V)*(-1) - _Fext;
		}
		
		FOR_EACH_CHILD{
			if((*it)->isAccDriven()){	// acc driven
				_A.add((*it)->artInertia(),(*it)->localFrame());	// _A += Adi((*it)->artInertia)
				(*it)->updateLocalAcc();
				_b += gWrench::dAdInv((*it)->localFrame(),(*it)->x() + (*it)->artInertia()*(*it)->localFrameAcc());
			}
			else{	// torque driven
				/*B = (*it)->artInertia();
				(*it)->subtractUnlockedInertia(B);*/
				_A.add((*it)->lockedArtInertia(),(*it)->localFrame());	// _A += Adi(B)
				_b += gWrench::dAdInv((*it)->localFrame(),(*it)->x() + (*it)->sol_xx());
			}		
		}
	}
	else{
		if(_bMassless){
			_b = _Fext*(-1);
		}
		else{
			_b = gWrench::dad(_V,_J*_V)*(-1) - _Fext;
		}
		
		FOR_EACH_CHILD{
			if((*it)->isAccDriven()){
				(*it)->updateLocalAcc();
				_b += gWrench::dAdInv((*it)->localFrame(),(*it)->x() + (*it)->artInertia()*(*it)->localFrameAcc());
			}
			else{
				_b += gWrench::dAdInv((*it)->localFrame(),(*it)->x() + (*it)->sol_xx());
			}		
		}
	}
}

gWrench g1dLink::sol_xx()
{
	//xx = A*S*W*(tau-S'*x)
	return _AS*(_W*(*_tau - (_S,_x)));
}

gWrench g2dLink::sol_xx()
{
	//xx = A*S*W*(tau-S'*x)
	gReal t0 = *_tau[0] - (_S0,_x);
	gReal t1 = *_tau[1] - (_S[1],_x);
	gReal w0 = _W[0]*t0 + _W[1]*t1;
	gReal w1 = _W[1]*t0 + _W[2]*t1;
	return _A*( _S0*w0 + _S[1]*w1 );
}


gAInertia g1dLink::lockedArtInertia()//subtractUnlockedInertia(gAInertia& r)
{
	//return A - A*S*W*S'*A
	gAInertia r(_A);
	r.subtractReciprocalProduct( _AS, _AS, _W);
	return r;
}

gAInertia g2dLink::lockedArtInertia()
{
	//r = A - A*S*W*S'*A
	gAInertia r(_A);
	r.subtractReciprocalProduct( _AS0, _AS0, _W[0] );
	r.subtractReciprocalProduct( _AS0, _AS1, _W[1] );
	r.subtractReciprocalProduct( _AS1, _AS0, _W[1] );
	r.subtractReciprocalProduct( _AS1, _AS1, _W[2] );
	return r;
}

gAInertia gBallLink::lockedArtInertia()
{
	//return A - A*S*W*S'*A
	//FOR OPTIMIZATION, 
	//STORE BELOW LOCAL VARIABLES TO REUSE IN sol_xx(void) FUNCTION
	/*_a00 = (_A.e(0,0)*_W[0]+_A.e(0,1)*_W[1]+_A.e(0,2)*_W[2]);
	_a01 = (_A.e(0,0)*_W[1]+_A.e(0,1)*_W[3]+_A.e(0,2)*_W[4]);
	_a02 = (_A.e(0,0)*_W[2]+_A.e(0,1)*_W[4]+_A.e(0,2)*_W[5]);
	_a10 = (_A.e(0,1)*_W[0]+_A.e(1,1)*_W[1]+_A.e(1,2)*_W[2]);
	_a11 = (_A.e(0,1)*_W[1]+_A.e(1,1)*_W[3]+_A.e(1,2)*_W[4]);
	_a12 = (_A.e(0,1)*_W[2]+_A.e(1,1)*_W[4]+_A.e(1,2)*_W[5]);
	_a20 = (_A.e(0,2)*_W[0]+_A.e(1,2)*_W[1]+_A.e(2,2)*_W[2]);
	_a21 = (_A.e(0,2)*_W[1]+_A.e(1,2)*_W[3]+_A.e(2,2)*_W[4]);
	_a22 = (_A.e(0,2)*_W[2]+_A.e(1,2)*_W[4]+_A.e(2,2)*_W[5]);
	_a30 = (_A.e(0,3)*_W[0]+_A.e(1,3)*_W[1]+_A.e(2,3)*_W[2]);
	_a31 = (_A.e(0,3)*_W[1]+_A.e(1,3)*_W[3]+_A.e(2,3)*_W[4]);
	_a32 = (_A.e(0,3)*_W[2]+_A.e(1,3)*_W[4]+_A.e(2,3)*_W[5]);
	_a40 = (_A.e(0,4)*_W[0]+_A.e(1,4)*_W[1]+_A.e(2,4)*_W[2]);
	_a41 = (_A.e(0,4)*_W[1]+_A.e(1,4)*_W[3]+_A.e(2,4)*_W[4]);
	_a42 = (_A.e(0,4)*_W[2]+_A.e(1,4)*_W[4]+_A.e(2,4)*_W[5]);
	_a50 = (_A.e(0,5)*_W[0]+_A.e(1,5)*_W[1]+_A.e(2,5)*_W[2]);
	_a51 = (_A.e(0,5)*_W[1]+_A.e(1,5)*_W[3]+_A.e(2,5)*_W[4]);
	_a52 = (_A.e(0,5)*_W[2]+_A.e(1,5)*_W[4]+_A.e(2,5)*_W[5]);

	gAInertia r(_A);
	r.set(0,0,r.e(0,0)-(_a00*_A.e(0,0)+_a01*_A.e(0,1)+_a02*_A.e(0,2)));
	r.set(0,1,r.e(0,1)-(_a00*_A.e(0,1)+_a01*_A.e(1,1)+_a02*_A.e(1,2)));
	r.set(0,2,r.e(0,2)-(_a00*_A.e(0,2)+_a01*_A.e(1,2)+_a02*_A.e(2,2)));
	r.set(0,3,r.e(0,3)-(_a00*_A.e(0,3)+_a01*_A.e(1,3)+_a02*_A.e(2,3)));
	r.set(0,4,r.e(0,4)-(_a00*_A.e(0,4)+_a01*_A.e(1,4)+_a02*_A.e(2,4)));
	r.set(0,5,r.e(0,5)-(_a00*_A.e(0,5)+_a01*_A.e(1,5)+_a02*_A.e(2,5)));
	r.set(1,1,r.e(1,1)-(_a10*_A.e(0,1)+_a11*_A.e(1,1)+_a12*_A.e(1,2)));
	r.set(1,2,r.e(1,2)-(_a10*_A.e(0,2)+_a11*_A.e(1,2)+_a12*_A.e(2,2)));
	r.set(1,3,r.e(1,3)-(_a10*_A.e(0,3)+_a11*_A.e(1,3)+_a12*_A.e(2,3)));
	r.set(1,4,r.e(1,4)-(_a10*_A.e(0,4)+_a11*_A.e(1,4)+_a12*_A.e(2,4)));
	r.set(1,5,r.e(1,5)-(_a10*_A.e(0,5)+_a11*_A.e(1,5)+_a12*_A.e(2,5)));
	r.set(2,2,r.e(2,2)-(_a20*_A.e(0,2)+_a21*_A.e(1,2)+_a22*_A.e(2,2)));
	r.set(2,3,r.e(2,3)-(_a20*_A.e(0,3)+_a21*_A.e(1,3)+_a22*_A.e(2,3)));
	r.set(2,4,r.e(2,4)-(_a20*_A.e(0,4)+_a21*_A.e(1,4)+_a22*_A.e(2,4)));
	r.set(2,5,r.e(2,5)-(_a20*_A.e(0,5)+_a21*_A.e(1,5)+_a22*_A.e(2,5)));
	r.set(3,3,r.e(3,3)-(_a30*_A.e(0,3)+_a31*_A.e(1,3)+_a32*_A.e(2,3)));
	r.set(3,4,r.e(3,4)-(_a30*_A.e(0,4)+_a31*_A.e(1,4)+_a32*_A.e(2,4)));
	r.set(3,5,r.e(3,5)-(_a30*_A.e(0,5)+_a31*_A.e(1,5)+_a32*_A.e(2,5)));
	r.set(4,4,r.e(4,4)-(_a40*_A.e(0,4)+_a41*_A.e(1,4)+_a42*_A.e(2,4)));
	r.set(4,5,r.e(4,5)-(_a40*_A.e(0,5)+_a41*_A.e(1,5)+_a42*_A.e(2,5)));
	r.set(5,5,r.e(5,5)-(_a50*_A.e(0,5)+_a51*_A.e(1,5)+_a52*_A.e(2,5)));*/

	_a30 = (_A.e(0,3)*_W[0]+_A.e(1,3)*_W[1]+_A.e(2,3)*_W[2]);
	_a31 = (_A.e(0,3)*_W[1]+_A.e(1,3)*_W[3]+_A.e(2,3)*_W[4]);
	_a32 = (_A.e(0,3)*_W[2]+_A.e(1,3)*_W[4]+_A.e(2,3)*_W[5]);
	_a40 = (_A.e(0,4)*_W[0]+_A.e(1,4)*_W[1]+_A.e(2,4)*_W[2]);
	_a41 = (_A.e(0,4)*_W[1]+_A.e(1,4)*_W[3]+_A.e(2,4)*_W[4]);
	_a42 = (_A.e(0,4)*_W[2]+_A.e(1,4)*_W[4]+_A.e(2,4)*_W[5]);
	_a50 = (_A.e(0,5)*_W[0]+_A.e(1,5)*_W[1]+_A.e(2,5)*_W[2]);
	_a51 = (_A.e(0,5)*_W[1]+_A.e(1,5)*_W[3]+_A.e(2,5)*_W[4]);
	_a52 = (_A.e(0,5)*_W[2]+_A.e(1,5)*_W[4]+_A.e(2,5)*_W[5]);

	gAInertia r;
	r.set(3,3,_A.e(3,3)-(_a30*_A.e(0,3)+_a31*_A.e(1,3)+_a32*_A.e(2,3)));
	r.set(3,4,_A.e(3,4)-(_a30*_A.e(0,4)+_a31*_A.e(1,4)+_a32*_A.e(2,4)));
	r.set(3,5,_A.e(3,5)-(_a30*_A.e(0,5)+_a31*_A.e(1,5)+_a32*_A.e(2,5)));
	r.set(4,4,_A.e(4,4)-(_a40*_A.e(0,4)+_a41*_A.e(1,4)+_a42*_A.e(2,4)));
	r.set(4,5,_A.e(4,5)-(_a40*_A.e(0,5)+_a41*_A.e(1,5)+_a42*_A.e(2,5)));
	r.set(5,5,_A.e(5,5)-(_a50*_A.e(0,5)+_a51*_A.e(1,5)+_a52*_A.e(2,5)));


	return r;
}
#endif //MW_USE_ARTICULATED_BODY_METHOD


#ifdef MBS_USE_COLLISION_MANIFOLD
//add contact forces to _Fext
void gRigidBody::addContactForces()
{

	for(std::vector<btPersistentManifold*>::iterator iter = _collisionManifolds.begin() ; 
		iter != _collisionManifolds.end() ;
		iter ++)
	{
		btPersistentManifold* contactManifold = (*iter);

		//check if this is A or B
		gReal dir = 1.0; 
		//btCollisionObject* coB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		btCollisionObject* coB = (btCollisionObject*)(contactManifold->getBody1());
		if( coB->getUserPointer() &&  (coB->getUserPointer() == this) ) //this is B
		{
			dir = -1.0;
		}
		
		int numContacts = contactManifold->getNumContacts();
		for(int j=0 ; j<numContacts ; j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			gReal depth = static_cast<gReal>(pt.getDistance()); //depth is negative if collided
			depth *= -1.0f;

			if(depth < 0.0f) continue; //this case DOES happen.

			btVector3 f = pt.m_appliedImpulse*pt.m_normalWorldOnB //normal force
				+ pt.m_lateralFrictionDir1;	//friction force

			gVec3 pos(pt.m_positionWorldOnB.x(),pt.m_positionWorldOnB.y(),pt.m_positionWorldOnB.z());
			gVec3 force(f.x(),f.y(),f.z());

			force *= dir; //invert direction if this is B

			gWrench wrench( (pos%force), force ); // wrench in world frame

			_Fext += wrench.xformInv( _T );		
		}
	}

}

#endif