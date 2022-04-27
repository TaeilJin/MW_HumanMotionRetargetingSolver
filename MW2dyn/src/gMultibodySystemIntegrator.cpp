//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
#include "mbs/gMultibodySystemIntegrator.h"
#include <time.h>

extern	gVec3	MW_GRAVITY_VECTOR;//(0,-9.8,0);
const   gVec3	MW_GROUND_NORMAL (gVec3UnitY);

using namespace std;

void gMultibodySystemTimeIntegrator::nextStepImplicitEulerInverseDynamicsBased1stOrderHigh(gReal timeStep)
{
	double h = timeStep;
	gReal eps = 0.987654321e-10;
	gReal halfEps = 0.5*eps;
	gReal invEps = gInv(eps);
	gReal hInvEps = h*invEps;
	gReal hSqrInvEps = h*h*invEps;
	unsigned int i,j;

	//static GM* A = NULL;
	//static GV* y = NULL;
	////static GV* tau = NULL;
	//static GV* acc = NULL;
	//static GV* tmp = NULL;
	//static gsl_permutation* permu = NULL;

	int _nDof = _system->dof();
	//if(!A) A = GM_alloc(_nDof,_nDof);
	//if(!y) y = GV_alloc(_nDof);
	////if(!tau) tau = GV_alloc(_nDof);
	//if(!acc) acc = GV_alloc(_nDof);
	//if(!tmp) tmp = GV_alloc(_nDof);
	//if(!permu) permu = gsl_permutation_alloc(_nDof);
	arma::mat A(_nDof,_nDof);
	arma::vec y(_nDof);
	arma::vec tau(_nDof);
	arma::vec acc(_nDof);
	arma::vec tmp(_nDof);
	
	
	//store _tau
	y = _system->force(); //GV_memcpy(y,_system->force()); 

	//compute M , C , acc 
	_system->computeJointSpaceEquationsOfMotion();
	A = _system->massMatrix(); //GM_memcpy(A,_system->massMatrix());

#ifdef MW_USE_ARTICULATED_BODY_METHOD	
	_system->hybridDynamicsFeatherstone(); //Featherstone is faster than Orin
#else
	computeAccelerationOrin(false);  //compute acc. _M is garbaged.
#endif
	acc = _system->acc(); //GV_memcpy(acc,_system->acc());
	
	//y = tau-C
	y -= _system->CVector(); //GV_sub(y,_system->CVector());
	y *= h; //GV_scale(y,h);

	//compute Cv and A = A + h*Cv
	_system->acc().zeros(); //GV_set_zero(_system->acc());
	j=0;
	
	for(int k=0;k<_system->numLinks();++k){
		gLink* link = _system->link(k);
		for(i=0;i<link->dof();++i)
		{
			gReal x = link->genVel(i); //store
			link->setGenVel(i,x+halfEps);
			_system->updateKinematicsUptoAcc(link->parent());
	#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
	#endif
			_system->updateExtForce();
			_system->inverseDynamics();
			tmp = _system->force(); //GV_memcpy(tmp,_system->force());

			link->setGenVel(i,x-halfEps);
			_system->updateKinematicsUptoAcc(link->parent());
	#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
	#endif
			_system->updateExtForce();
			_system->inverseDynamics();
			tmp -= _system->force(); //GV_sub(tmp,_system->force());
			tmp *= hInvEps; //GV_scale(tmp,hInvEps); //h*Cv_i

			A.col(j) += tmp; //GV_add(&GM_column(A,j).vector,tmp);
			_system->setVel(j,x);//GV_set(_system->vel(),j,x); //restore

			j++;
		}
	}
	

	//compute Phi
	j=0;		
	for(int k=0;k<_system->numLinks();++k){
		gLink* link = _system->link(k);
		link->storeCoord();

		for(i=0;i<link->dof();++i){
			_system->acc() = acc;//GV_memcpy(_system->acc(),acc);

			link->moveEpsilon(i,halfEps);
			_system->updateKinematicsUptoAcc( link->parent() );
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
#endif
			_system->updateExtForce();
			_system->inverseDynamics();
			link->restoreCoord();
			tmp=_system->force();//GV_memcpy(tmp,_system->force());
			
			link->moveEpsilon(i,-halfEps);
			_system->updateKinematicsUptoAcc( link->parent() );
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
#endif
			_system->updateExtForce();
			_system->inverseDynamics();
			link->restoreCoord();
			tmp-=_system->force();//GV_sub(tmp,_system->force());
			tmp*=hSqrInvEps;//GV_scale(tmp,hSqrInvEps); //tmp = Phi_j * h^2

			A.col(j)+=tmp;//GV_add(&GM_column(A,j).vector,tmp); // A_j += h*h*Phi_j

			tmp*=_system->vel(j);//GV_scale(tmp,GV_get(_system->vel(),j)); 
			y-=tmp; //GV_sub(y,tmp); // y -= h*h*Phi_j*dq_j

			j++;
		}			
	}	

	/*}
	gReal diff = ( clock() - start ) / (double)CLOCKS_PER_SEC;
	std::cout << diff << std::endl;*/

	//solve A*del_v=y
	//int signum;
	//gsl_linalg_LU_decomp(A,permu,&signum);
	//gsl_linalg_LU_svx	(A,permu,y); //now y = del_v

	//solve A*del_v = y
	arma::solve(tmp,A,y);  //now tmp = del_v

	// set next _q,_dq
	// time integrate velocity
	//GV_add (_system->vel(),y);	 // _dq += _b_
	_system->vel() += tmp;
	// time integrate position
	//FOR_EACH_LINK_FORWARD{
	for(int k=0;k<_system->numLinks();++k){
		_system->link(k)->timeIntegrateExplicitEulerPositionOnly(h);
	}

	_system->updateKinematicsUptoVel();
}


#ifdef MW_USE_ARTICULATED_BODY_METHOD

void gMultibodySystemTimeIntegrator::timeIntegrateCoordinates(gReal timestep)
{
	for(int k=0;k<_system->numLinks();++k)	_system->link(k)->timeIntegrateExplicitEulerPositionOnly(timestep);
}

void gMultibodySystemTimeIntegrator::nextStepExplicitEulerFeatherstone(gReal timeStep)
{
	_system->hybridDynamicsFeatherstone();
	timeIntegrateExplicitEuler(timeStep);
	_system->updateKinematicsUptoVel();	
}

void gMultibodySystemTimeIntegrator::nextStepSemiImplicitEulerFeatherstone(gReal timeStep)
{
	_system->hybridDynamicsFeatherstone();
	timeIntegrateSemiImplicitEuler(timeStep);
	_system->updateKinematicsUptoVel();	
}

void gMultibodySystemTimeIntegrator::nextStepImplicitEuler1stOrder(gReal h, bool assumeZeroTimeStepSquared)
{
	/*
	//equations of motion is given as below:
	x_dot = v
	v_dot = f (x,v)
	// then in implicit euler
	del_x = x(t+h)-x(t) = h*v(t+h) = h*(v+del_v)
	del_v = v(t+h)-v(t) = h*f(x(t+h),v(t+h)) = h ( f(x(t),v(t)) + fx*del_x + fv*del_v )
	where fx = partial f/partial x, fv = partial f/partial v
	
	Therefore,
	(I-h*fv-h*h*fx)*del_v = h*f(x(t),v(t))+h*h*fx*v
	del_x = h*(v+del_v)

	If assumeZeroTimeStepSquared, we even ignore h*h*fx term. So
	(I-h*fv)*del_v = h*f(x(t),v(t))
	This is less accurate, but twice faster because we don't compute fx.
	*/

	int sz = _system->dof();

	/*static GM* _A_ = NULL;
	static GV* _b_ = NULL;
	static GV* _f_ = NULL;
	static GV* _g_ = NULL;
	static gsl_permutation* _permu = NULL;

	if(!_A_) _A_ = GM_alloc(sz,sz);
	if(!_b_) _b_ = GV_alloc(sz);
	if(!_f_) _f_ = GV_alloc(sz);
	if(!_g_) _g_ = GV_alloc(sz);
	if(!_permu) _permu = gsl_permutation_alloc(sz);*/
	
	arma::mat _A_(sz,sz);
	arma::vec _b_(sz);
	arma::vec _f_(sz);
	arma::vec _g_(sz);

	int i,j;
	gReal eps = 1e-10;
	//gReal eps = 1e-15;
	gReal minuseHOverEps = -h/eps;
	gReal hSqrOverEps = h*h/eps;

	//store _ddq
	_system->hybridDynamicsFeatherstone();
	_f_ = _system->acc();//GV_memcpy(_f_,_system->acc());

	// make _b_ = h*_f_
	//GV_memcpy(_b_,_f_);
	//GV_scale(_b_,h);
	_b_ = h*_f_;

	// make _A_  = I-h*fv 
	gReal x;
	j=0;
	for(int k=0;k<_system->numLinks();++k){
		gLink* link = _system->link(k);
		for(i=0;i<link->dof();++i){
			x = link->genVel(i); //store
			link->setGenVel(i,x+eps);
			
			_system->updateKinematicsUptoVel( link->parent() );

#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
#endif

			_system->updateExtForce();

			_system->hybridDynamicsFeatherstone(true);	

			/*GV_view Aj = GM_column(_A_,j);
			GV_memcpy(&Aj.vector,_system->acc());
			GV_sub   (&Aj.vector,_f_);
			GV_scale (&Aj.vector,minuseHOverEps);
			GV_set	 (&Aj.vector,j,1+GV_get(&Aj.vector,j));
			*/

			_A_.col(j) = minuseHOverEps*(_system->acc()-_f_);
			_A_(j,j) += 1.0; 
			
			link->setGenVel(i,x);	// restore
			link->updateKinematicsUptoVel(false); 
			j++;
		}
	}

	if(!assumeZeroTimeStepSquared){
		// make _A_ -= h^2 fx
		// _b_ += h^2 fx*v
		j=0;		
		for(int k=0;k<_system->numLinks();++k){
			gLink* link = _system->link(k);
			link->storeCoord();

			for(i=0;i<link->dof();++i){
				link->moveEpsilon(i,eps);
				_system->updateKinematicsUptoVel( link->parent()  );	

#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
				if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
#endif
				_system->updateExtForce();
				_system->hybridDynamicsFeatherstone();

				//GV_memcpy(_g_,_system->acc());
				//GV_sub(_g_,_f_);
				//GV_scale(_g_,hSqrOverEps); //_g_ = h*h*fx
				//GV_sub(&GM_column(_A_,j).vector,_g_);
				//GB_axpy(GV_get(_system->vel(),j),_g_,_b_);

				_g_ = hSqrOverEps*(_system->acc()-_f_);
				_A_.col(j) -= _g_;
				_b_ += _system->vel(j)*_g_;

				link->restoreCoord();		
				link->updateKinematicsUptoVel(false);
				j++;
			}			
		}
	}

	// solve _A_*c = _b_
	//int signum;
	//gsl_linalg_LU_decomp(_A_,_permu,&signum);
	//gsl_linalg_LU_svx	(_A_,_permu,_b_); //now _b_ = del_v
	
	// solve _A_*c = _b_
	arma::solve(_g_,_A_,_b_); //now _g_ = del_v

	// set next _q,_dq
	// time integrate velocity
	//GV_add (_system->vel(),_b_);	 // _dq += _b_
	_system->vel()+=_g_;

	// time integrate position
	for(int k=0;k<_system->numLinks();++k){
		_system->link(k)->timeIntegrateExplicitEulerPositionOnly(h);
	}

	_system->updateKinematicsUptoVel();	
}



void gMultibodySystemTimeIntegrator::nextStepImplicitEuler1stOrderHigh(gReal h)
{
	/*
	same as nextStepImplicitEuler1stOrder, but with higer order approximation to fx,fv
	This method takes twice the time of nextStepImplicitEuler1stOrder, but timestep can be more than twice.
	*/

	int sz = _system->dof();
	arma::mat _A_(sz,sz);
	arma::vec _b_(sz);
	arma::vec _f_(sz);
	arma::vec _g_(sz);
	arma::vec tmp(sz);
	
	int i,j;
	gReal eps = 0.987654321e-10;

	////timer
	//clock_t start = clock();
	//int niter=100;
	//while(niter-->0){


	// make _b_ = h*_f_
	_system->hybridDynamicsFeatherstone();
	//GV_memcpy(_b_,_system->acc());
	//GV_scale(_b_,h);
	_b_ = h*_system->acc();

	// make _A_  = I-h*fv 
	gReal x;
	j=0;
	for(int k=0;k<_system->numLinks();++k){
		gLink* link = _system->link(k);
		for(i=0;i<link->dof();++i){
			//GV_view Aj = GM_column(_A_,j);
			x = link->genVel(i); // store

			link->setGenVel(i,x+0.5*eps);		
			_system->updateKinematicsUptoVel( link->parent() );
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
#endif
			_system->updateExtForce();
			_system->hybridDynamicsFeatherstone(true);	
			//GV_memcpy(&Aj.vector,_system->acc());
			tmp = _system->acc();

			link->setGenVel(i,x-0.5*eps);		
			_system->updateKinematicsUptoVel( link->parent() );
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
#endif
			_system->updateExtForce();
			_system->hybridDynamicsFeatherstone(true);	
			//GV_sub(&Aj.vector,_system->acc());
			//GV_scale (&Aj.vector,-h*gInv(eps));
			//GV_set (&Aj.vector,j,1+GV_get(&Aj.vector,j));			
			_A_.col(j) = -h*gInv(eps)*(tmp-_system->acc());			
			_A_(j,j) += 1.0;			
			
			_system->setVel(j,x); //GV_set(_system->vel(),j,x);	// restore			
	
			j++;
		}
	}

	// make _A_ -= h^2 fx
	// _b_ += h^2 fx*v
	j=0;		
	for(int k=0;k<_system->numLinks();++k){
		gLink* link = _system->link(k);
		link->storeCoord();

		for(i=0;i<link->dof();++i){
			link->moveEpsilon(i,0.5*eps);
			_system->updateKinematicsUptoVel( link->parent() );
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
#endif
			_system->updateExtForce();
			_system->hybridDynamicsFeatherstone();
			//GV_memcpy(_g_,_system->acc());
			_g_ = _system->acc();
			link->restoreCoord();

			link->moveEpsilon(i,-0.5*eps);
			_system->updateKinematicsUptoVel(  link->parent() );
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold( link->parent() );
#endif
			_system->updateExtForce();
			_system->hybridDynamicsFeatherstone();
			//GV_sub(_g_,_system->acc());
			_g_ -= _system->acc();
			link->restoreCoord();

			//GV_scale(_g_,gSqr(h)*gInv(eps)); //_g_ = h*h*fx
			//GV_sub(&GM_column(_A_,j).vector,_g_);
			//GB_axpy(GV_get(_system->vel(),j),_g_,_b_);
			_g_ *= gSqr(h)*gInv(eps);
			_A_.col(j) -= _g_;
			_b_ += _system->vel(j)*_g_;

			j++;
		}			
	}

	////timer
	//}
	//gReal diff = ( clock() - start ) / (double)CLOCKS_PER_SEC;
	//std::cout << diff << std::endl;

	// solve _A_*c = _b_
	//int signum;
	//gsl_linalg_LU_decomp(_A_,_permu,&signum);
	//gsl_linalg_LU_svx	(_A_,_permu,_b_); //now _b_ = del_v
	arma::solve(_g_,_A_,_b_); //now _g_ = del_v

	// set next _q,_dq
	// time integrate velocity
	//GV_add (_system->vel(),_b_);	 // _dq += _b_
	_system->vel() += _g_;	 // _dq += _g_

	// time integrate position
	for(int k=0;k<_system->numLinks();++k){
		_system->link(k)->timeIntegrateExplicitEulerPositionOnly(h);
	}

	_system->updateKinematicsUptoVel();	

	////check error
	//updateExtForce();
	//hybridDynamicsFeatherstone();
	//GV_scale(_ddq,-h);
	//GV_add(_ddq,_b_);
	//std::cout << "error: " << GB_nrm2(_ddq) << std::endl;
}


//Newton-raphson method to iteratively find root...
//Refer to note for the algorihm
//maxIter: maximum iterations allowed
//threshold: convergence threshold 
void gMultibodySystemTimeIntegrator::nextStepImplicitEulerIterative(gReal h, int maxIter, gReal threshold)
{
	int iter = 0;
	int sz = _system->dof();
	arma::mat _A_(sz,sz);
	arma::vec _b_(sz);
	arma::vec _g_(sz);
	arma::vec z(sz);
	arma::vec tmp(sz);	

	int i,j;
	gReal eps = 0.987654321e-10;
	gReal mag=1e10,magOld;
		
	//step 1: x = x + hv
	timeIntegrateCoordinates(h);
	_system->updateKinematicsUptoVel();
	z.zeros(); //initialize

	do
	{
		//step3-1: compute b = -z + h*f(x,v)
		//compute _f_
		_system->updateExtForce();
		_system->hybridDynamicsFeatherstone();
		//compute _b_
		/*GV_memcpy(_b_,_system->acc());
		GV_scale(_b_,h);
		GV_sub(_b_,z);*/
		_b_ = h*_system->acc() - z;

		//step3-2: compute _A_ = I-h*fv-h*h*fx 
		//_A_  = I-h*fv 
		gReal x;
		for(j=0;j<sz;j++)
		{	
			//GV_view Aj = GM_column(_A_,j);
			x=_system->vel(j);//x = GV_get(_system->vel(),j); // store

			//GV_set(_system->vel(),j,x+0.5*eps);		
			_system->setVel(j,x+0.5*eps);		

			_system->updateKinematicsUptoVel();
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold();
#endif
			_system->updateExtForce();
			_system->hybridDynamicsFeatherstone(true);	
			//GV_memcpy(&Aj.vector,_system->acc());
			tmp = _system->acc();

			//GV_set(_system->vel(),j,x-0.5*eps);		
			_system->setVel(j,x-0.5*eps);		
			_system->updateKinematicsUptoVel();
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
			if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold();
#endif
			_system->updateExtForce();
			_system->hybridDynamicsFeatherstone(true);	
			//GV_sub(&Aj.vector,_system->acc());
			_A_.col(j) = -h*gInv(eps)*(tmp - _system->acc());
			_A_(j,j) += 1.0; //GV_set	 (&Aj.vector,j,1+GV_get(&Aj.vector,j));
			_system->setVel(j,x); //GV_set(_system->vel(),j,x);	// restore
		}
		// _A_ -= h^2 fx
		j=0;		
		for(int k=0;k<_system->numLinks();++k){
			gLink* link = _system->link(k);
			link->storeCoord();

			for(i=0;i<link->dof();++i){
				link->moveEpsilon(i,0.5*eps);
				_system->updateKinematicsUptoVel();
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
				if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold();
#endif
				_system->updateExtForce();
				_system->hybridDynamicsFeatherstone();
				//GV_memcpy(_g_,_system->acc());
				_g_ = _system->acc();
				link->restoreCoord();

				link->moveEpsilon(i,-0.5*eps);
				_system->updateKinematicsUptoVel();
#ifdef MBS_USE_COLLISION_MANIFOLD_UPDATE
				if(_considerContactForceChangeInImplicitMethods) _system->updateCollisionManifold();
#endif
				_system->updateExtForce();
				_system->hybridDynamicsFeatherstone();
				//GV_sub(_g_,_system->acc());
				_g_ -= _system->acc();
				link->restoreCoord();

				//GV_scale(_g_,gSqr(h)*gInv(eps)); //_g_ = h*h*fx
				_g_ *= gSqr(h)*gInv(eps);
				//GV_sub(&GM_column(_A_,j).vector,_g_);
				_A_.col(j) -= _g_;

				j++;
			}			
		}

		// solve _A_*del_z = _b_
		//int signum;
		//gsl_linalg_LU_decomp(_A_,_permu,&signum);
		//gsl_linalg_LU_svx	(_A_,_permu,_b_); //now _b_ = del_z
		arma::solve(tmp,_A_,_b_); //now tmp = del_z
	
		// step 5: x += h*del_z, v += del_z
		//GV_add(_system->vel(),_b_);
		_system->vel() += tmp;

		j=0;
		for(int k=0;k<_system->numLinks();++k){
			gLink* link = _system->link(k);
			for(i=0;i<link->dof();++i){
				//link->moveEpsilon(i, h*GV_get(_b_,j++));
				link->moveEpsilon(i, h*tmp(j++));
			}
		}
		_system->updateKinematicsUptoVel();
		
		//z += _b_
		//GV_add(z,_b_);
		z += tmp;

		magOld = mag;
		mag = arma::norm(tmp);
		//GB_dot(_b_,_b_,&mag);
		mag = sqrt(mag/sz); //RMS
		//std::cout <<iter <<": " << mag <<" ";
		
	}while(++iter < maxIter && mag > threshold && mag < magOld ); //last condition make sure that it stops on divergence

	//std::cout << std::endl;
}


void gMultibodySystemTimeIntegrator::nextStepExplicitEulerOrin(gReal timeStep)
{
	_system->computeAccelerationOrin();
	timeIntegrateExplicitEuler(timeStep);
	_system->updateKinematicsUptoVel();	
}


#endif 


#ifdef MW_USE_DIFF_NEWTON_EULER

void gMultibodySystemTimeIntegrator::timeIntegrateExplicitEuler(gReal timeStep)
{
	for(int i=0;i<_system->numLinks();++i)	_system->link(i)->timeIntegrateExplicitEuler(timeStep);	
}

void gMultibodySystemTimeIntegrator::timeIntegrateSemiImplicitEuler(gReal timeStep)
{
	for(int i=0;i<_system->numLinks();++i)	_system->link(i)->timeIntegrateSemiImplicitEuler(timeStep);	
}

void gMultibodySystemTimeIntegrator::nextStepImplicitEulerInvDynTypeAnalytic(gReal timeStep)
{
	double h = timeStep;
	double hh = h*h;
	unsigned int i,j;

	int _nDof = _system->dof();

	arma::mat A(_nDof,_nDof);
	arma::vec y(_nDof);
	arma::vec tmp(_nDof);
	
	////timer
	//clock_t start = clock();
	//int niter=1000;
	//while(niter-->0){

	//store _tau
	//GV_memcpy(y,_system->force()); 
	y = _system->force();

	//compute M , C , acc 
	_system->computeJointSpaceEquationsOfMotion(); // takes only 0.8 ms for human skeleton
	//GM_memcpy(A,_system->massMatrix());
	A = _system->massMatrix();
#ifdef MW_USE_ARTICULATED_BODY_METHOD	
	_system->hybridDynamicsFeatherstone(); //Featherstone is faster than Orin. 0.4 ms
#else
	_system->computeAccelerationOrin(false);    //compute acc. _M is garbaged. 14 ms
#endif
		
	//y = h*(tau-C)
	//GV_sub(y,_system->CVector());
	//GV_scale(y,h);
	y -= _system->CVector();
	y *= h;

	//set qp,dqp,ddqp zeros	
	for(int k=0;k<_system->numLinks();++k){
		gLink* link = _system->link(k);
		for(int i=0;i<link->dof();++i)
		{
			link->setCoordP(i,0.0);
			link->setGenVelP(i,0.0);
			link->setGenAccP(i,0.0);
		}
	}

	//compute Phi
	//Note that q,dq,ddq,Fext are made current in computing M and C above.
	_system->inverseDynamics(); 
	j=0;		

	for(int k=0;k<_system->numLinks();++k){
		gLink* link = _system->link(k);
		for(i=0;i<link->dof();++i)
		{
			link->setCoordP(i,gOne);

			//1. compute derivative of kinematic variables (_Gp, _Tp, _Vp, _dVp, and state dependent forces' KVariables )
			_system->setZeroKVariablesP();
			_system->updateKVariablesP( link->parent() );

			//2. compute derivative of external forces
			_system->updateExtForceP();
		
			//3. compute derivative of frame forces, gen. forces
			_system->inverseDynamicsP();

			//tmp = Phi_j
			_system->collectGenForceP(tmp);
			tmp *= hh; //GV_scale(tmp,hh); 
			A.col(j) += tmp;//GV_add(&GM_column(A,j).vector,tmp); // A_j += h*h*Phi_j
			tmp *= _system->vel(j);//GV_scale(tmp,GV_get(_system->vel(),j)); 
			y -= tmp;//GV_sub(y,tmp); // y -= h*h*Phi_j*dq_j

			link->setCoordP(i,gZero);
			j++;
		}			
	}	

	//compute Cv and A = A + h*Cv
	//GV_set_zero(_system->acc());
	_system->acc().zeros();
	_system->updateKinematicsUptoAcc();
	//updateExtForce(); //external force is only dependent on the state (position and velocity).
	_system->inverseDynamics();
	j=0;

	for(int k=0;k<_system->numLinks();++k){
		gLink* link = _system->link(k);
		for(i=0;i<link->dof();++i)
		{
			link->setGenVelP(i,gOne);

			//1. compute derivative of kinematic variables (_Gp, _Tp, _Vp, _dVp,  and state dependent forces' KVariables )
			_system->setZeroKVariablesP();
			_system->updateKVariablesP( link->parent() ); // gLink->_Gp,_Tp are redundantly recomputed, but wasted computing cost is not significant.

			//2. compute derivative of external forces
			_system->updateExtForceP();

			//3. compute _Ep, _Fp
			_system->inverseDynamicsP();

			_system->collectGenForceP(tmp); 
			//GV_scale(tmp,h);
			tmp*=h;
			//GV_add(&GM_column(A,j).vector,tmp);
			A.col(j) += tmp;

			link->setGenVelP(i,gZero);
			j++;
		}
	}

	//solve A*del_v=y
	//int signum;
	//gsl_linalg_LU_decomp(A,permu,&signum);
	//gsl_linalg_LU_svx	(A,permu,y); //now y = del_v
	arma::solve(tmp,A,y); //now tmp = del_v

	// set next _q,_dq
	// time integrate velocity
	//GV_add (_system->vel(),y);	 // _dq += _y_
	_system->vel() += tmp; 
	// time integrate position
	for(int k=0;k<_system->numLinks();++k){
		_system->link(k)->timeIntegrateExplicitEulerPositionOnly(h);
	}

	_system->updateKinematicsUptoVel();
}

#endif //MW_USE_DIFF_NEWTON_EULER