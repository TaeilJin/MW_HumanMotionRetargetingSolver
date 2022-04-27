//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _MBS_BSPLINE_H_
#define _MBS_BSPLINE_H_

#include "Base/gBase.h"
#include "mbs/gArmaUtil.h"

#define NsToNp(n) n+3

//----------------------------------------------------
//uniform cubic B-spline
//----------------------------------------------------
class gBspline{
public:

	gBspline(gReal x_min, gReal x_max, int nSeg);	
	~gBspline();
	inline	gReal	p(int i)									{ return _p(i); }
	inline	arma::vec&		p(void)										{ return _p; }
	
	// functions to solve f(x), f'(x). f''(x)
	gReal eval(gReal x);	// evaluation at x
	gReal deriv1(gReal x);	// 1st derivative at x
	gReal deriv2(gReal x);	// 2nd derivative at x

	// functions to compute basis splines
	// this function can be used for curve fitting.
	// return 0 if success
	int eval_allSplines(arma::vec& B, gReal x);	//fill up B[0]...B[N+2] given x
	int get_index(gReal x, gReal* u); // get index i and u, such that B_{i-3}..B_{i} is non-zeros.

	static void Basis(gReal u, gReal* B);
	static void dBasis(gReal u, gReal* dB);
	static void ddBasis(gReal u, gReal* ddB);

protected:

	gReal _x_min, _x_max;	// range of x. x_min <= x < x_max
	gReal _dx;				// _dx = (_x_max-_x_min )/_N
	arma::vec _p;					// control point vector
	int _N;					// number of segments in x domain.

	//int get_index(gReal x, gReal* u); // get index i and u, such that B_{i-3}..B_{i} is non-zeros.
};

//----------------------------------------------------
//uniform cubic B-spline N-Dimension
//----------------------------------------------------
class gBsplineND : public gBspline {
public:
	gBsplineND(gReal x_min, gReal x_max, int nSeg, int nDim);
	gBsplineND(gReal x_min, gReal x_max, int nSeg, int nDim, double* aux_mem); // shallow
	gBsplineND(gReal x_min, gReal x_max, int nSeg, int nDim, const double* aux_mem); // deep
	~gBsplineND();

	inline void setP(const int dim, const arma::vec& p)		{ _ps.row(dim) = p.t(); }

	inline	gReal	p(int dim, int i)						{ return _ps(dim, i); }
	//inline	const arma::vec&		p(int dim)				{ return _ps.row(dim); }
	inline	arma::mat&		p()								{ return _ps; }
	
	gReal eval(int dim, gReal x);
	void eval(arma::vec& y, const gReal x);

	// TODO: correction
	void deriv1(arma::vec& yd, const gReal x);
	void deriv2(arma::vec& ydd, const gReal x);

	// stateless functions
	//static void eval(arma::vec& y, const gReal x, const arma::mat& cp, const arma::mat& B);
	static void eval(arma::vec& y, const int xi, const arma::mat& cp, const arma::mat& B);
	static void buildBasisMatrix(arma::mat& B);
	static void buildBasisMatrix(arma::sp_mat& B);

	static void derive1(arma::vec& yd, const int xi, const arma::mat& cp, const arma::mat& BD);
	static void derive2(arma::vec& ydd, const int xi, const arma::mat& cp, const arma::mat& BDD);
	static void buildDerivMatrix(arma::mat& BD);
	static void buildDeriv2Matrix(arma::mat& BDD);

	// functions to compute basis splines
	// this function can be used for curve fitting.
	// return 0 if success
	//int eval_allSplines(arma::vec& B, gReal x);	//fill up B[0]...B[N+2] given x

	//int get_index(gReal x, gReal* u); // get index i and u, such that B_{i-3}..B_{i} is non-zeros.

	//static void Basis(gReal u, gReal* B);
	//static void dBasis(gReal u, gReal* dB);
	//static void ddBasis(gReal u, gReal* ddB);
private:
	arma::mat _ps;			// control point matrix
};

/*
//----------------------------------------------------
//uniform cubic B-spline surface
//----------------------------------------------------
class gBsplineS{
public:

	gBsplineS(gReal x_min, gReal x_max, int nSegX, gReal y_min, gReal y_max, int nSegY);	
	~gBsplineS();
	inline	gReal	p(int i, int j)								{ return GM_get(_p,i,j); }
	inline	GM*		p(void)										{ return _p; }
	inline	void	set_p(int i, int j, gReal val)				{ GM_set(_p,i,j,val); }
	
	// functions to solve f(x,y)
	gReal eval(gReal x, gReal y);		// f(x,y). evaluation at (x,y)
	gReal deriv1x(gReal x, gReal y);	// df/dx at (x,y)
	gReal deriv1y(gReal x, gReal y);	// df/dy at (x,y)
	gReal deriv2xx(gReal x, gReal y);	// d^2f/dx^2 at (x,y)
	gReal deriv2xy(gReal x, gReal y);	// d^2f/dxdy at (x,y)
	gReal deriv2yy(gReal x, gReal y);	// d^2f/dy^2 at (x,y)

	// functions to compute basis splines
	// this function can be used for curve fitting.
	// return 0 if success
	//int eval_allSplines(GV* B, gReal x);	//fill up B[0]...B[N+2] given x

private:

	gReal _x_min, _x_max;	// range of x. x_min <= x < x_max
	gReal _y_min, _y_max;	// range of y. y_min <= y < y_max

	gReal _dx;				// _dx = (_x_max-_x_min )/_N
	gReal _dy;				// _dy = (_y_max-_y_min )/_M

	GM* _p;					// control point matrix
	int _N;					// number of segments in x domain.
	int _M;					// number of segments in y domain.

	int get_index_x(gReal x, gReal* u); // get index i and u, such that B_{i-3}..B_{i} is non-zeros.
	int get_index_y(gReal y, gReal* v); // get index j and v, such that B_{j-3}..B_{j} is non-zeros.
};
*/
#endif //_MBS_BSPLINE_H_
