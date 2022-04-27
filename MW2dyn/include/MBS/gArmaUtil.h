//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
#ifndef _MBS_GSL_UTIL_H_
#define _MBS_GSL_UTIL_H_

#include "Base/gBase.h"
#include "Base/gMath.h"
#include "armadillo"
#include <vector>



//
// Least square method of over-determined problem
//
// solve min( (1-lambda)*||Ax-y||^2 + lambda*||W(x-x_d)||^2 ) 
// where W = diag(w)
//
// - if x_d = NULL, x_d is treated as zero vector
// - if A is broad matrix (Ax = y is underdetermined), lambda must be positive
// - if A is broad matrix and W is identity, then consider solving min |Ax-y|^2 instead, which
//   is a under-determined problem and x = pseudoInv(A)y will give min |x|^2 solution that exactly satisfies Ax=y.
// - lambda can be 0 if A is a tall matrix.
//
//void solve_Ax_y(GV* x, const GM* A, const GV* y, const GV* w, const GV* x_d, gReal lambda, gReal* residual=NULL);
void solve_Ax_y(arma::vec& x, const arma::mat& A, const arma::vec& y, const arma::vec& w, const arma::vec* x_d, gReal lambda, gReal* residual=NULL);



//set M[r+i,c]=src[i], 0<= i < size
//void GM_set_col(GM* M, int r, int c, int size, const gReal* src);
inline void mat_set_col(arma::mat& M, int r, int c, int size, const gReal* src)
{
	//for(int i=0;i<size;++i)	M(r+i,c)=src[i];		
	memcpy(&M(r,c),src,sizeof(double)*size);
}

//set V[r+i]=src[i], 0 <= i < size
//void GV_set(GV* V, int r, int size, const gReal* src);
inline void vec_set(arma::vec& V, int r, int size, const gReal* src)
{
	//for(int i=0;i<size;++i)	V(r+i)=src[i];
	memcpy(&V(r),src,sizeof(double)*size);
}

int makeSkewSym3x3(arma::mat& mat, gVec3 v);

int makeSkewSym3x3(arma::mat& mat, const arma::vec3& v);


#endif