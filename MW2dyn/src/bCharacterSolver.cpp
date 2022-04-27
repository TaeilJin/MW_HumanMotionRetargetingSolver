//#####################################################################
// Copyright 2010-2015, Hynchul Choi, Sukwon Lee, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

/*
*	[ bCharacterSolver ]
*	
*	TYPE		: IMPLEMENTATION
*	AUTHOR		: Hyunchul Choi
*	LAST UPDATE	: 2012 / 06 / 09
*	DESCRIPTION	:
*	this solver is designed to solve MBS system created by S.H. Lee.
*	all credits from mbs system goes to S.H. Lee.
*	class will cover the variables -
*		Momentum, CoM, CoP, GRF, and additional features.
*/

#include "Character/bCharacterSolver.h"
#include "mbs/gArmaUtil.h"

extern	gVec3 MW_GRAVITY_VECTOR;

bCharacterSolver::bCharacterSolver( bCharacter* model )
{
	m_sys = model;	
}

gVec3 bCharacterSolver::computeCenterOfMass()
{
	gVec3 com(0,0,0);

	gLink* l;
	for(int i = m_sys->numLinks()-1; i >= 0; --i)
	{
		l = m_sys->link(i);
		com += (l->pos() + l->frame().rot() * l->inertia().comInVec3()) * l->inertia().mass();
	}

	com /= m_sys->mass();
	return com;
}

gWrench	bCharacterSolver::computeMomentum(const gVec3& relPos)
{
	// compute momentum w.r.t. relPos
	gWrench mom(0,0,0,0,0,0);
	for(int i = 0; i < m_sys->numLinks(); ++i)
	{
		gLink* link = m_sys->link(i);
		mom += ( link->inertia() * link->frameVel() ).xform( link->frame() );
	}
	mom = mom.xform( -relPos );
	return mom;
}

void bCharacterSolver::computeBodyJacobianOfChain(const std::vector<int>& linkIdx, bool includeFirstLink, arma::mat& J)
{
	gLink*	end = m_sys->link( linkIdx.back() );	//end link
	
	int start = includeFirstLink ? 0 : 1;
	int col = 0;
	for(int i = start; i < linkIdx.size(); ++i)
	{
		gLink* link = m_sys->link( linkIdx[i] );		
		gXMat gb = end->frame().invMult(link->frame()); // link->frame() as seen from foot
		// set jacobian index
		for(int j = 0; j < link->dof(); ++j)
		{
			gTwist xi = link->screwBody(j).xform(gb); //link->screwBody(j) as seen from foot			
			mat_set_col(J, 0, col, 6, xi.cptr());
			++col;
		}
	}
}

int bCharacterSolver::computeNumColumnsOfBodyJacobianOfChain(const std::vector<int>& linkIdx, bool includeFirstLink)
{
	gLink*	end = m_sys->link( linkIdx.back() );	//end link	
	int start = includeFirstLink ? 0 : 1;
	int col = 0;
	for(int i = start; i < linkIdx.size(); ++i)
	{
		col += m_sys->link( linkIdx[i] )->dof();
	}
	return col;
}

void bCharacterSolver::computeEndEffectorJacobian(const std::vector<int>& linkIdx, bool includeFirstLink, arma::mat& J)
{
	assert(J.n_rows==6 && J.n_cols==m_sys->dof());

	gLink*	endEff = m_sys->link( linkIdx.back() );	//end effector	
	
	int start = includeFirstLink ? 0 : 1;
	for(int i = start; i < linkIdx.size(); ++i)
	{
		gLink* link = m_sys->link( linkIdx[i] );
		// construct g
		gXMat gb = endEff->frame().invMult(link->frame()); // link->frame() as seen from foot
		// set jacobian index
		int	jIdx = link->genCoordIndex();		
		for(int j = 0; j < link->dof(); ++j)
		{
			gTwist xi = link->screwBody(j).xform(gb); //link->screwBody(j) as seen from foot			
			//GM_set_col(J, 0, jIdx, 6, xi.cptr());
			mat_set_col(J, 0, jIdx, 6, xi.cptr());
			++jIdx;
		}
	}
}

void bCharacterSolver::computeEndEffectorJacobian(const std::vector<int>& linkIdx, bool includeFirstLink, arma::subview<gReal>& J)
{
	assert(J.n_rows==6 && J.n_cols==m_sys->dof());

	gLink*	endEff = m_sys->link( linkIdx.back() );	//end effector	
	
	int start = includeFirstLink ? 0 : 1;
	for(int i = start; i < linkIdx.size(); ++i)
	{
		gLink* link = m_sys->link( linkIdx[i] );
		// construct g
		gXMat gb = endEff->frame().invMult(link->frame()); // link->frame() as seen from foot
		// set jacobian index
		int	jIdx = link->genCoordIndex();		
		for(int j = 0; j < link->dof(); ++j)
		{
			gTwist xi = link->screwBody(j).xform(gb); //link->screwBody(j) as seen from foot			
			memcpy(J.colptr(jIdx), xi.cptr(),sizeof(gReal)*6);
			++jIdx;
		}
	}
}

void bCharacterSolver::getCompactJacobian(arma::mat& Jout, arma::mat& Jin, const std::vector<int>& linkIdx, bool includeFirstLink)
{
	int start,end;
	getRangeForCompactJacobian(start, end, linkIdx, includeFirstLink);
	assert(Jout.n_rows==6 && Jout.n_cols==(end-start+1));
	
	//GM_memcpy(Jout, &GM_submatrix(Jin,0,start,6,end-start+1).matrix);
	Jout = Jin(0,start,arma::size(6,end-start+1));
}

// Centroidal Momentum Matrix
void bCharacterSolver::computeSpatialMomentumJacobian( arma::mat& Jp, gTwist** JMbody, const gVec3& p )
{
	assert(Jp.n_rows==6 && Jp.n_cols==m_sys->dof());

	//GM_set_zero(Jp); //set all zero
	Jp.zeros();
	
	gWrench* Jp_ = new gWrench[m_sys->dof()]; //corresponding to Jp
	for(int i=0;i<m_sys->dof();++i) Jp_[i].setZero(); //initialize to zero

	for(int j=0;j<m_sys->numLinks();++j) //for each link
	{
		gLink* link = m_sys->link(j);
		//for link j and its ancestors
		gLink* p = link; //start from itself
		while(p){ 
			int gc = p->genCoordIndex();
			for(int i=gc; i<gc+p->dof(); ++i)
			{
				Jp_[i] += (link->inertia()*JMbody[j][i]).xform(link->frame());				
			}
			p = p->parent(); //proceed to the parent
		}
	}

	//now, Jp_ holds momentum Jacobian wrt world frame. Let's transform to p. Then fill Jp
	for(int i=0;i<m_sys->dof();++i){
		Jp_[i] = Jp_[i].xform(-p);
		//GM_set_col(Jp,0,i,gWrench::SIZE_INDEX,Jp_[i].cptr());
		mat_set_col(Jp,0,i,gWrench::SIZE_INDEX,Jp_[i].cptr());
	}

	delete[] Jp_;
}

void bCharacterSolver::computeSpatialMomentumJacobian( arma::mat& Jp, const arma::mat& Jb, const gVec3& p )
{
	assert(Jp.n_rows==6 && Jp.n_cols==m_sys->dof());

	//GM_set_zero(Jp); //set all zero
	Jp.zeros();
	
	gWrench* Jp_ = new gWrench[m_sys->dof()]; //corresponding to Jp
	for(int i=0;i<m_sys->dof();++i) Jp_[i].setZero(); //initialize to zero

	for(int j=0;j<m_sys->numLinks();++j) //for each link
	{
		gTwist Jb_twist;

		gLink* link = m_sys->link(j);
		//for link j and its ancestors
		gLink* p = link; //start from itself
		while(p){ 
			int gc = p->genCoordIndex();
			for(int i=gc; i<gc+p->dof(); ++i)
			{
				Jb_twist.set( Jb.colptr(i) + 6*j );
				Jp_[i] += (link->inertia()*Jb_twist).xform(link->frame());				
			}
			p = p->parent(); //proceed to the parent
		}
	}

	//now, Jp_ holds momentum Jacobian wrt world frame. Let's transform to p. Then fill Jp
	for(int i=0;i<m_sys->dof();++i){
		Jp_[i] = Jp_[i].xform(-p);
		//GM_set_col(Jp,0,i,gWrench::SIZE_INDEX,Jp_[i].cptr());
		mat_set_col(Jp,0,i,gWrench::SIZE_INDEX,Jp_[i].cptr());
	}

	delete[] Jp_;
}

void bCharacterSolver::computeSpatialMomentumJacobianBias( arma::vec& Jp_Bias, const gVec3& p )
{
	//Jp_bias is the rate of change of the spatial momentum when generalized acc = 0
	//GV* temp = GV_alloc( m_sys->dof() );
	arma::vec temp( m_sys->dof() );

	temp = m_sys->acc();//GV_memcpy(temp, m_sys->acc()); //make a backup of current acc

	m_sys->acc().zeros();//GV_set_zero( m_sys->acc() ); //set acc to zero
	
	m_sys->updateKinematicsUptoAcc(); //update kinematics

	gWrench dh_w = computeMomentumRateChangeWorld(); //compute rate of change of the spatial momentum
	gWrench dh_c = dh_w.xform(-p);
	//GV_set(Jp_Bias,0,6,dh_c.cptr()); // set Jp_bias
	vec_set(Jp_Bias,0,6,dh_c.cptr());

	//GV_memcpy(m_sys->acc(),temp); //restore acc
	m_sys->acc() = temp;
	m_sys->updateKinematicsUptoAcc(); //update kinematics
	//GV_free(temp);
}

gWrench bCharacterSolver::computeMomentumRateChangeWorld()
{
	gWrench dMomentum;
	for(int i = 0; i < m_sys->numLinks(); ++i)
	{
		gLink* l = m_sys->link(i);
		dMomentum += ( l->inertia() * l->frameAcc() - gWrench::dad(l->frameVel(), l->inertia()*l->frameVel() ) ).xform( l->frame() );
	}
	return dMomentum;
}

void bCharacterSolver::computeMomentumRateChangeGivenAcc(gWrench& dest, const arma::vec& acc, const arma::mat& JM, const arma::vec& Jbias)
{
	// wren = Jbias + JM*acc
	//GV_view wren = GV_view_array(dest.ptr(),6);
	//GV_memcpy(&wren.vector,Jbias);	
	//GB_gemv(CblasNoTrans,1,JM,acc,1,&wren.vector);	

	//vec(aux_mem*, number_of_elements, copy_aux_mem = true, strict = true) 
	arma::vec wren(dest.ptr(),6,false);
	wren = Jbias + JM*acc;
}

void	bCharacterSolver::computeBodyJacobianMatrix(gTwist** bjm)
{
	for(int i=0;i<m_sys->numLinks();++i)
	{	
		gLink* link = m_sys->link(i);			

		//itself
		int gc = link->genCoordIndex();
		for(int j=0;j<link->dof();++j)
		{
			bjm[i][gc+j] = m_sys->link(i)->screwBody(j);

			//gTwist scr = m_sys->link(i)->screwBody(j);
		}

		//parent GC
		gLink* p = link->parent();
		while(p)
		{
			int pgc = p->genCoordIndex();
			gXMat X = link->frame().invMult(p->frame());			
			for(int j=0;j<p->dof();++j) //for each screw of p
			{
				bjm[i][pgc+j] = p->screwBody(j).xform( X );				
			}			
			p = p->parent();
		}
	}
}


void bCharacterSolver::computeBodyJacobianMatrix(arma::mat& Jb)
{
	gTwist twist;
	int dim = 6;

	for(int i=0;i<m_sys->numLinks();++i)
	{	
		gLink* link = m_sys->link(i);			

		//itself
		int gc = link->genCoordIndex();
		for(int j=0;j<link->dof();++j)
		{
			twist = m_sys->link(i)->screwBody(j);
			
			//GM_set_col(Jb, i*dim, gc+j, dim, twist.cptr() );
			mat_set_col(Jb, i*dim, gc+j, dim, twist.cptr() );
		}

		//parent GC
		gLink* p = link->parent();
		while(p)
		{
			int pgc = p->genCoordIndex();
			gXMat X = link->frame().invMult(p->frame());			
			for(int j=0;j<p->dof();++j) //for each screw of p
			{
				//bjm[i][pgc+j] = p->screwBody(j).xform( X );				
				twist = p->screwBody(j).xform( X );

				//GM_set_col(Jb, i*dim, pgc+j, dim, twist.cptr());
				mat_set_col(Jb, i*dim, pgc+j, dim, twist.cptr());
			}			
			p = p->parent();
		}
	}
}