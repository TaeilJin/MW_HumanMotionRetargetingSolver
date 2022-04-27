//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#include "Tools/gBspline.h"

// uniform cubic B-spline
//B[0] = B_i(u)
//B[1] = B_{i-1}(u) ...
//B[3] = B_{i-3}(u)
void gBspline::Basis(gReal u, gReal* B)
{
	gReal u2 = u*u;
	gReal u3 = u*u2;
	gReal scale = gInv(6.);
	B[0] = scale*u3;
	B[1] = scale*( -3*u3 + 3*u2 + 3*u + 1 );
	B[2] = scale*( 3*u3 - 6*u2 + 4 );
	B[3] = scale*( 1 - 3*u + 3*u2 - u3 );
}

//dB[0] = dB_i/du
//dB[1] = dB_{i-1}/du ...
//dB[3] = dB_{i-3}/du
void gBspline::dBasis(gReal u, gReal* dB)
{
	gReal u2 = u*u;
	dB[0] = 0.5*u2;
	dB[1] = 0.5*( -3*u2 + 2*u + 1 );
	dB[2] = 0.5*(  3*u2 - 4*u );
	dB[3] = 0.5*( -1*u2 + 2*u - 1 );
}

//ddB[0] = d^2B_i/du^2
//ddB[1] = d^2B_{i-1}/du^2 ...
//ddB[3] = d^2B_{i-3}/du^2
void gBspline::ddBasis(gReal u, gReal* ddB)
{
	ddB[0] = u;
	ddB[1] = -3*u + 1 ;
	ddB[2] = 3*u - 2;
	ddB[3] = 1 - u;
}

int gBspline::get_index(gReal x, gReal* u)
{
	*u = (x-_x_min)/_dx + 3;
	int i = int(*u);
	*u -= (gReal)i;
	return i;
}

gBspline::gBspline(gReal x_min, gReal x_max, int nSeg)
{
	_x_min = x_min;
	_x_max = x_max;
	_N = nSeg;
	_p.zeros(_N+3); //num knots = N+3
	_dx = (x_max-x_min)/_N;
}

gBspline::~gBspline()
{
	//if(_p) GV_free(_p);
}

gReal gBspline::eval(gReal x)
{
	gReal u;
	gReal y=0;
	int i = get_index(x,&u);
	gReal B[4];	

	Basis(u,B);	
	if( i != _p.n_cols ) y += p(i)*B[0];
	y += p(i-1)*B[1];
	y += p(i-2)*B[2];
	y += p(i-3)*B[3];

	return y;
}

gReal gBspline::deriv1(gReal x)
{
	gReal u;
	gReal y=0;
	int i = get_index(x,&u);
	gReal dB[4];

	dBasis(u,dB);
	if( i != _p.n_cols ) y += p(i)*dB[0];
	y += p(i-1)*dB[1];
	y += p(i-2)*dB[2];
	y += p(i-3)*dB[3];
	//y *= gInv(_dx);

	return y;
}

gReal gBspline::deriv2(gReal x)
{
	gReal u;
	gReal y=0;
	int i = get_index(x,&u);
	gReal ddB[4];

	dBasis(u,ddB);
	if( i != _p.n_cols ) y += p(i)*ddB[0];
	y += p(i-1)*ddB[1];
	y += p(i-2)*ddB[2];
	y += p(i-3)*ddB[3];
	y *= gInv((_dx*_dx));
	
	return y;
}

int gBspline::eval_allSplines(arma::vec& B, gReal x)
{
	if(B.n_rows != _N+3) return -1;

	gReal u;
	int i = get_index(x,&u);
	gReal B_loc[4];

	Basis(u,B_loc);
	B.zeros();
	B(i)= B_loc[0] ;
	B(i-1)= B_loc[1] ;
	B(i-2)= B_loc[2] ;
	B(i-3)= B_loc[3] ;

	return 0;
}

gBsplineND::gBsplineND(gReal x_min, gReal x_max, int nSeg, int nDim, double* aux_mem)
	:gBspline(x_min, x_max, nSeg),
	_ps( arma::mat(aux_mem, nDim, nSeg + 3, false) )
{
}

gBsplineND::gBsplineND(gReal x_min, gReal x_max, int nSeg, int nDim, const double* aux_mem)
	:gBspline(x_min, x_max, nSeg),
	_ps( arma::mat(aux_mem, nDim, nSeg + 3) )
{
}

gBsplineND::gBsplineND(gReal x_min, gReal x_max, int nSeg, int nDim)
	:gBspline(x_min, x_max, nSeg)
{
	_ps.zeros(nDim, nSeg + 3);
}

gBsplineND::~gBsplineND()
{
}

gReal gBsplineND::eval(int dim, gReal x)
{
	gReal u;
	gReal y=0;
	int i = get_index(x,&u);
	gReal B[4];	

	Basis(u,B);
	if( i != _ps.n_cols ) y += _ps(dim, i)*B[0];
	y += _ps(dim, i-1)*B[1];
	y += _ps(dim, i-2)*B[2];
	y += _ps(dim, i-3)*B[3];

	return y;
};

void gBsplineND::eval(arma::vec& y, const gReal x)
{
	gReal u;
	int i = get_index(x,&u);
	gReal B[4];

	Basis(u,B);

	//if( y.n_rows != _ps.n_rows ) y.resize(_ps.n_rows);

	y.zeros();
	if( i != _ps.n_cols ) y += _ps.col(i) * B[0];
	y += _ps.col(i-1)*B[1];
	y += _ps.col(i-2)*B[2];
	y += _ps.col(i-3)*B[3];
}

void gBsplineND::deriv1(arma::vec& yd, const gReal x)
{
	gReal u;
	int i = get_index(x,&u);
	gReal dB[4];

	dBasis(u,dB);
	
	//if( yd.n_rows != _ps.n_rows ) yd.resize(_ps.n_rows);
	yd.zeros();
	if( i != _ps.n_cols ) yd += _ps.col(i)*dB[0];
	yd += _ps.col(i-1)*dB[1];
	yd += _ps.col(i-2)*dB[2];
	yd += _ps.col(i-3)*dB[3];
	//y *= gInv(_dx);
}

void gBsplineND::deriv2(arma::vec& ydd, const gReal x)
{
	gReal u;
	int i = get_index(x,&u);
	gReal ddB[4];

	ddBasis(u,ddB);

	//if( ydd.n_rows != _ps.n_rows ) ydd.resize(_ps.n_rows);
	ydd.zeros();
	if( i != _ps.n_cols ) ydd += _ps.col(i)*ddB[0];
	ydd += _ps.col(i-1)*ddB[1];
	ydd += _ps.col(i-2)*ddB[2];
	ydd += _ps.col(i-3)*ddB[3];
	//ydd *= gInv((_dx*_dx));
}

void gBsplineND::eval(arma::vec& y, const int xi, const arma::mat& cp, const arma::mat& B)
{
	int cpIdx = (double)(B.n_cols - 3) / (double)(B.n_rows-1) * (double)xi;

	if( cpIdx == B.n_cols - 3 )
	{
		y = cp.submat(0, cpIdx, arma::SizeMat(cp.n_rows, 3)) * B.submat(xi, cpIdx, arma::SizeMat(1, 3)).t();
	}
	else 
		y = cp.submat(0, cpIdx, arma::SizeMat(cp.n_rows, 4)) * B.submat(xi, cpIdx, arma::SizeMat(1, 4)).t();
}

void gBsplineND::buildBasisMatrix(arma::mat& B)
{
	double dt = (double)(B.n_cols-3) / (double)(B.n_rows-1);
	B.zeros();
	
	#pragma omp parallel for
	for( int i=0; i<B.n_rows; ++i )
	{
		arma::vec4 B_;

		double p = dt * (double)i;
		int cpIdx = p;
		double u = p - (double)cpIdx;

		Basis(u, B_.memptr());

		if( cpIdx == B.n_cols - 3 )
		{
			B(i, cpIdx) = B_(3);
			B(i, cpIdx+1) = B_(2);
			B(i, cpIdx+2) = B_(1);
		} else B.submat(i, cpIdx, arma::SizeMat(1,4)) = arma::fliplr(B_.t());

		//B.submat( i, cpIdx, arma::SizeMat(1, 4) ) = arma::fliplr(B_.t());
	}
}

void gBsplineND::buildBasisMatrix(arma::sp_mat& B)
{
	double dt = (double)(B.n_cols - 3) / (double)(B.n_rows - 1);
	//B.zeros();

	int expSize = B.n_rows * 4 - 1;
	arma::umat locs(2, expSize);
	arma::vec vals(expSize);

	//#pragma omp parallel for
	int pos = 0;
	for (int i = 0; i < B.n_rows; ++i)
	{
		arma::vec4 B_;

		double p = dt * (double)i;
		int cpIdx = p;
		double u = p - (double)cpIdx;

		Basis(u, B_.memptr());

		if (cpIdx == B.n_cols - 3)
		{
			locs(0, pos) = i;
			locs(1, pos) = cpIdx;
			vals(pos) = B_(3);
			pos++;

			locs(0, pos) = i;
			locs(1, pos) = cpIdx + 1;
			vals(pos) = B_(2);
			pos++;

			locs(0, pos) = i;
			locs(1, pos) = cpIdx + 2;
			vals(pos) = B_(1);
			pos++;
		}
		else {
			//B.submat(i, cpIdx, arma::SizeMat(1, 4)) = arma::fliplr(B_.t());
			locs(0, pos) = i;
			locs(1, pos) = cpIdx;
			vals(pos) = B_(3);
			pos++;

			locs(0, pos) = i;
			locs(1, pos) = cpIdx + 1;
			vals(pos) = B_(2);
			pos++;

			locs(0, pos) = i;
			locs(1, pos) = cpIdx + 2;
			vals(pos) = B_(1);
			pos++;

			locs(0, pos) = i;
			locs(1, pos) = cpIdx + 3;
			vals(pos) = B_(0);
			pos++;
		}
		//B.submat( i, cpIdx, arma::SizeMat(1, 4) ) = arma::fliplr(B_.t());
	}

	locs = locs.cols(0, pos - 1);
	vals = vals.subvec(0, pos - 1);

	B = arma::sp_mat(locs, vals, B.n_rows, B.n_cols);
}

void gBsplineND::derive1(arma::vec& yd, const int xi, const arma::mat& cp, const arma::mat& BD)
{
	int cpIdx = (double)(BD.n_cols - 3) / (double)(BD.n_rows-1) * (double)xi;

	if( cpIdx == BD.n_cols - 3 )
	{
		yd = cp.submat(0, cpIdx, arma::SizeMat(cp.n_rows, 3)) * BD.submat(xi, cpIdx, arma::SizeMat(1, 3)).t();
	}
	else 
		yd = cp.submat(0, cpIdx, arma::SizeMat(cp.n_rows, 4)) * BD.submat(xi, cpIdx, arma::SizeMat(1, 4)).t();
}

void gBsplineND::derive2(arma::vec& ydd, const int xi, const arma::mat& cp, const arma::mat& BDD)
{
	int cpIdx = (double)(BDD.n_cols - 3) / (double)(BDD.n_rows-1) * (double)xi;

	if( cpIdx == BDD.n_cols - 3 )
	{
		ydd = cp.submat(0, cpIdx, arma::SizeMat(cp.n_rows, 3)) * BDD.submat(xi, cpIdx, arma::SizeMat(1, 3)).t();
	}
	else 
		ydd = cp.submat(0, cpIdx, arma::SizeMat(cp.n_rows, 4)) * BDD.submat(xi, cpIdx, arma::SizeMat(1, 4)).t();
}

void gBsplineND::buildDerivMatrix(arma::mat& BD)
{
	double dt = (double)(BD.n_cols-3) / (double)(BD.n_rows-1);
	BD.zeros();
	
	#pragma omp parallel for
	for( int i=0; i<BD.n_rows; ++i )
	{
		arma::vec4 B_;

		double p = dt * i;
		int cpIdx = p;
		double u = p - (double)cpIdx;

		dBasis(u, B_.memptr());

		if( cpIdx == BD.n_cols - 3 )
		{
			BD(i, cpIdx) = B_(3);
			BD(i, cpIdx+1) = B_(2);
			BD(i, cpIdx+2) = B_(1);
		} else BD.submat(i, cpIdx, arma::SizeMat(1,4)) = arma::fliplr(B_.t());

		//B.submat( i, cpIdx, arma::SizeMat(1, 4) ) = arma::fliplr(B_.t());
	}
}

void gBsplineND::buildDeriv2Matrix(arma::mat& BDD)
{
	double dt = (double)(BDD.n_cols-3) / (double)(BDD.n_rows-1);
	BDD.zeros();
	
	#pragma omp parallel for
	for( int i=0; i<BDD.n_rows; ++i )
	{
		arma::vec4 B_;

		double p = dt * i;
		int cpIdx = p;
		double u = p - (double)cpIdx;

		ddBasis(u, B_.memptr());

		if( cpIdx == BDD.n_cols - 3 )
		{
			BDD(i, cpIdx) = B_(3);
			BDD(i, cpIdx+1) = B_(2);
			BDD(i, cpIdx+2) = B_(1);
		} else BDD.submat(i, cpIdx, arma::SizeMat(1,4)) = arma::fliplr(B_.t());

		//B.submat( i, cpIdx, arma::SizeMat(1, 4) ) = arma::fliplr(B_.t());
	}
}


/*
gBsplineS::gBsplineS(gReal x_min, gReal x_max, int nSegX, gReal y_min, gReal y_max, int nSegY)
{
	_x_min = x_min;
	_x_max = x_max;
	_N = nSegX;
	_y_min = y_min;
	_y_max = y_max;
	_M = nSegY;

	_dx = (x_max-x_min)/_N;
	_dy = (x_max-x_min)/_M;

	_p = GM_calloc(_N+4, _M+4); //num knots = N+4 by M+4
}


gBsplineS::~gBsplineS()
{
	if(_p) GM_free(_p);
}


int gBsplineS::get_index_x(gReal x, gReal* u) // get index i and u, such that B_{i-3}..B_{i} is non-zeros.
{
	*u = (x-_x_min)/_dx + 3;
	int i = int(*u);
	*u -= (gReal)i;
	return i;
}


int gBsplineS::get_index_y(gReal y, gReal* v) // get index j and v, such that B_{j-3}..B_{j} is non-zeros.
{
	*v = (y-_y_min)/_dy + 3;
	int j = int(*v);
	*v -= (gReal)j;
	return j;
}


gReal gBsplineS::eval(gReal x, gReal y)
{
	gReal u,v;
	gReal z=0;
	int i = get_index_x(x,&u);
	int j = get_index_y(y,&v);
	gReal Bx[4], By[4];

	Basis(u,Bx);
	Basis(v,By);
	for(int k=0;k<=3;++k)
	for(int l=0;l<=3;++l)
	{
		z += p(i-k,j-l)*Bx[k]*By[l];
	}
	return z;
}


gReal gBsplineS::deriv1x(gReal x, gReal y)
{
	gReal u,v;
	gReal z=0;
	int i = get_index_x(x,&u);
	int j = get_index_y(y,&v);
	gReal dBx[4], By[4];

	dBasis(u,dBx);
	Basis(v,By);
	for(int k=0;k<=3;++k)
	for(int l=0;l<=3;++l)
	{
		z += p(i-k,j-l)*dBx[k]*By[l];
	}
	z *= gInv(_dx);
	return z;
}


gReal gBsplineS::deriv1y(gReal x, gReal y)
{
	gReal u,v;
	gReal z=0;
	int i = get_index_x(x,&u);
	int j = get_index_y(y,&v);
	gReal Bx[4], dBy[4];

	Basis(u,Bx);
	dBasis(v,dBy);
	for(int k=0;k<=3;++k)
	for(int l=0;l<=3;++l)
	{
		z += p(i-k,j-l)*Bx[k]*dBy[l];
	}
	z *= gInv(_dy);
	return z;
}


gReal gBsplineS::deriv2xx(gReal x, gReal y)
{
	gReal u,v;
	gReal z=0;
	int i = get_index_x(x,&u);
	int j = get_index_y(y,&v);
	gReal ddBx[4], By[4];

	ddBasis(u,ddBx);
	Basis(v,By);
	for(int k=0;k<=3;++k)
	for(int l=0;l<=3;++l)
	{
		z += p(i-k,j-l)*ddBx[k]*By[l];
	}
	z *= gInv((_dx*_dx));
	return z;
}


gReal gBsplineS::deriv2xy(gReal x, gReal y)
{
	gReal u,v;
	gReal z=0;
	int i = get_index_x(x,&u);
	int j = get_index_y(y,&v);
	gReal dBx[4], dBy[4];

	dBasis(u,dBx);
	dBasis(v,dBy);
	for(int k=0;k<=3;++k)
	for(int l=0;l<=3;++l)
	{
		z += p(i-k,j-l)*dBx[k]*dBy[l];
	}
	z *= gInv((_dx*_dy));
	return z;
}


gReal gBsplineS::deriv2yy(gReal x, gReal y)
{
	gReal u,v;
	gReal z=0;
	int i = get_index_x(x,&u);
	int j = get_index_y(y,&v);
	gReal Bx[4], ddBy[4];

	Basis(u,Bx);
	ddBasis(v,ddBy);
	for(int k=0;k<=3;++k)
	for(int l=0;l<=3;++l)
	{
		z += p(i-k,j-l)*Bx[k]*ddBy[l];
	}
	z *= gInv((_dy*_dy));
	return z;
}

*/


