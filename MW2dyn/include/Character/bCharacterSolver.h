//#####################################################################
// Copyright 2010-2015, Hynchul Choi, Sukwon Lee, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

/*
*	[ bCharacterSolver ]
*	
*	TYPE		: HEADER
*	AUTHOR		: Hyunchul Choi
*	LAST UPDATE	: 2012 / 06 / 09
*	DESCRIPTION	:
*	this solver is designed to solve MBS system created by S.H. Lee.
*	all credits from mbs system goes to S.H. Lee.
*	class will cover the variables -
*		Momentum, CoM, CoP, GRF, and additional features.
*/

#ifndef _BCHARACTER_SOLVER_H
#define _BCHARACTER_SOLVER_H

#include "Character/bCharacter.h"

class bCharacterSolver
{
public:
	bCharacterSolver( bCharacter* model );
	virtual	~bCharacterSolver(){}

	/// Compute center of mass
	gVec3	computeCenterOfMass();

	/// Compute BodyJacobianMatrix. 
	/*** J[nLink][nDoFs] J[i][j] = body velocity(V) of link i when dot{q}[j] (link j's joint velocity) = 1 \n
	* Note that body velocity is the "spatial velocity" of i wrt the body frame. \n
	* J[0][i]: not used. J[i][j]=0 if j is neither at i nor i's ancestor.  \n
	* \n
	* v[i] = Ad_(inv(T[i])*T[0])v[0] + J[i][1] * link[1]->genVel() + ... + J[i][n] * link[n]->genVel();	\n
	*/	
	void computeBodyJacobianMatrix(gTwist** bjm);
	void computeBodyJacobianMatrix(arma::mat& Jb);

	/// Dimension of BodyJacobianMatrix
	inline void getDimensionOfBodyJacobianMatrix(int& nRow, int& nCol) { nRow = m_sys->numLinks(); nCol = m_sys->dof(); }
	
	/// malloc BodyJacobianMatrix
	gTwist** mallocBodyJacobianMatrix(void)	
	{
		gTwist** bjm = new gTwist*[m_sys->numLinks()];
		for(int i=0;i<m_sys->numLinks();++i) bjm[i] = new gTwist[m_sys->dof()];	
		return bjm;
	}

	/// delete BodyJacobianMatrix
	void deleteBodyJacobianMatrix(gTwist** bjm)
	{
		for(int i = 0; i < m_sys->numLinks(); ++i)	delete[] bjm[i];
		delete[] bjm;
	}

	/// compute BodyJacobianMatrixBias
	void computeBodyJacobianMatrixBias(gTwist* bias)
	{
		//GV *tmpAcc = gsl_vector_alloc(m_sys->acc()->size );
		//GV_memcpy(tmpAcc, m_sys->acc()); //store
		arma::vec tmpAcc = m_sys->acc();

		//gsl_vector_set_zero( m_sys->acc() );
		m_sys->acc().zeros();
		m_sys->updateKinematicsUptoAcc();

		for(int i=0;i<m_sys->numLinks();++i) bias[i] = m_sys->link(i)->frameAcc();
	
		// restore previous values
		m_sys->setAcc(tmpAcc);
		m_sys->updateKinematicsUptoAcc(); //need to restore acc?	

		//GV_free(tmpAcc);
	}


	/// Compute body Jacobian of an end-effector	
	/*** J is the body Jacobian of an end-effector(whose link index is linkIdx.back()). \n
	* Therefore, v=J*dq refers to the body velocity of the end-effector. \n
	* linkIdx is a vector of link indices forming a chain, from ancestors to the end-effector. \n
	* If includeFirstLink is true, J contains non-zero column for the link[ linkIdx[0] ]. If false, link[ linkIdx[0] ] is ignored to compute Jacobian matrix. \n
	* J stores the Jacobian. Its dimension must be 6xDoF. \n
	* Example: if linkIdx=[0,10,11,12] (0:pelvis, 10:thigh, 11: shin, 12: foot) and includeFirstLink=false, \n
	* J contains zero-vector for the pelvis part. Thereby J can be used for inverse kinematics for the leg (while pelvis is fixed). \n
	* Note that if you already have the BodyJacobianMatrix (computed by computeBodyJacobianMatrix() ), \n
	* EndEffectorJacobian is same as a row of the BodyJacobianMatrix correponding to the end-effector.
	*/
	//void computeEndEffectorJacobian(const std::vector<int>& linkIdx, bool includeFirstLink, GM* J);
	void computeEndEffectorJacobian(const std::vector<int>& linkIdx, bool includeFirstLink, arma::mat& J);
	void computeEndEffectorJacobian(const std::vector<int>& linkIdx, bool includeFirstLink, arma::subview<gReal>& J);

	/// Get compact Jacobian matrix Jout from Jin
	/*** Jin: regular end-effector Jacobian ( 6xDoF dimension) \n
	* Jout: sub-matrix of Jin, i.e., Jout = Jin[:, index_of_first_dof_of_link[s] : index_of_last_dof_of_link[linkIdx.back()] ] \n
	* where s = linkIdx[0] if includeFirstLink is true. s=linkIdx[1], otherwise.
	*/
	//void getCompactJacobian(GM* Jout, GM* Jin, const std::vector<int>& linkIdx, bool includeFirstLink);
	void getCompactJacobian(arma::mat& Jout, arma::mat& Jin, const std::vector<int>& linkIdx, bool includeFirstLink);

	/// Get first and last column of compact Jacobian matrix from regular end-effector Jacobian, which is constructed from linkIdx. 
	/*** Regular end-effector Jacobian ( 6xDoF dimension), contains many 0-columns for irrelavant joints. \n
	* For example, foot Jacobian has many 0-columns for the upper body links. \n
	* This function gets the first (col_start) and last column (col_end) of non-zero sub-matrix of end-effector Jacobian. \n
	* col_start = index_of_first_dof_of_link[s], where s = linkIdx[0] if includeFirstLink is true. s=linkIdx[1], otherwise. \n
	* col_end = index_of_last_dof_of_link[linkIdx.back()].
	*/
	void getRangeForCompactJacobian(int& col_start, int& col_end, const std::vector<int>& linkIdx, bool includeFirstLink)
	{
		int o = includeFirstLink? 0:1;
		col_start = m_sys->link(linkIdx[o])->genCoordIndex();
		gLink* lastLink = m_sys->link(linkIdx.back());
		col_end = lastLink->genCoordIndex() + lastLink->dof()-1;
	}


	/// Compute body Jacobian of a chain formed by linkIdx
	void computeBodyJacobianOfChain(const std::vector<int>& linkIdx, bool includeFirstLink, arma::mat& J);

	/// compute number of columns of body Jacobian of a chain
	int  computeNumColumnsOfBodyJacobianOfChain(const std::vector<int>& linkIdx, bool includeFirstLink);
	

	/// Compute spatial momentum at the relative position	
	/// For example, if you want to computer spatial momentum wrt CoM, set relPos to CoM.
	gWrench	computeMomentum(const gVec3& relPos);

	/// Dimension of spatial momentum Jacobian
	inline void getDimensionOfSpatialMomentumJacobian(int& nRow, int& nCol) { nRow = 6; nCol = m_sys->dof(); }

	/// Compute Jp matrix such that h(spatial momentum about p) = Jp*dq
	/*** JMbody is BodyJacobianMatrix(computed by computeBodyJacobianMatrix(gTwist**) function) \n
	* p is reference point to compute (angular) momentum.
	*/
	//void computeSpatialMomentumJacobian( GM* Jp, gTwist** JMbody, const gVec3& p );
	void computeSpatialMomentumJacobian( arma::mat& Jp, gTwist** JMbody, const gVec3& p );
	void computeSpatialMomentumJacobian( arma::mat& Jp, const arma::mat& Jb, const gVec3& p );
	

	/// Compute Jp_Bias vector such that dot(h) = Jp*ddq + Jp_Bias. Size of Jp_Bias = Size of rows of Jp. Refer to getDimensionOfSpatialMomentumJacobian(int&,int&).
	//void computeSpatialMomentumJacobianBias( GV* Jp_Bias, const gVec3& p );
	void computeSpatialMomentumJacobianBias( arma::vec& Jp_Bias, const gVec3& p );
	
	/// Compute rate of change of spatial momentum (or generalized momentum) wrt world, given joint accelerations
	/// Note: this function must be called after gMultibodySystem::updateKinematicsUptoAcc() has been called.
	gWrench computeMomentumRateChangeWorld(void);

	/// Compute rate of change of spatial momentum given momentum Jacobian(JM) and its bias term (Jbias)
	/// In fact, this function simply calculates dest = JM*acc+Jbias
	/// The reference frame of the resulting momentum rate change is same as that of JM.
	//void computeMomentumRateChangeGivenAcc(gWrench& dest, const GV* acc, const GM* JM, const GV* Jbias);
	void computeMomentumRateChangeGivenAcc(gWrench& dest, const arma::vec& acc, const arma::mat& JM, const arma::vec& Jbias);


	

protected:
	bCharacter*		m_sys;					// character class pointer
};


#endif /* _GCHARACTER_SOLVER_H */