#include "Tools/spline1dFit.h"
#include "Tools/gBspline.h"

int spline1dFit(
	const arma::vec& x,
	const arma::vec& y,
	const unsigned int m, // # of data point
	arma::vec& sp
	)
{
	// check
	if( x.n_elem != y.n_elem )
	{
		printf("# of element in x and y must be the same.\n");
		return -1;
	}

	if( m < 5 )
	{
		printf("m must be greater than 4.\n");
		return -1;
	}

	// build band matrix
	const arma::vec& x_s = x;

	gBspline spline(x_s.min(), x_s.max(), m-1);

	arma::mat BM(x_s.n_elem + 2, m + 2);
	BM.zeros();

	// filling
	gReal u;
	arma::vec4 B, ddB;
	int cpIdx;
	for( int i=0; i<x_s.n_elem; i++ )
	{
		cpIdx = spline.get_index(x_s(i), &u);
		spline.Basis(u, B.memptr());
		
		if( cpIdx == spline.p().n_elem )
		{
			BM(i, cpIdx-3) = B(3);
			BM(i, cpIdx-2) = B(2);
			BM(i, cpIdx-1) = B(1);
		} else BM.submat(i, cpIdx-3, arma::SizeMat(1,4)) = arma::fliplr(B.t());
	}

	// test
	double param = 5.0;
	cpIdx = spline.get_index(x_s(0), &u);
	spline.ddBasis(u, ddB.memptr());
	BM.submat(x_s.n_elem,cpIdx-3, arma::SizeMat(1,4)) = arma::fliplr(ddB.t());

	cpIdx = spline.get_index(x_s(x_s.n_elem - 1), &u);
	spline.ddBasis(u, ddB.memptr());
	//BM.submat(x_s.n_elem+1,cpIdx-3, arma::SizeMat(1,4)) = arma::fliplr(ddB.t());
	BM.submat(x_s.n_elem+1,cpIdx-3, arma::SizeMat(1,3)) = arma::fliplr(ddB.subvec(1,3).t());

	arma::mat U;
	arma::vec s;
	arma::mat V;

	if( !arma::svd(U,s,V,BM) )
	{
		printf("SVD fail.\n");
		return -1;
	}

	arma::mat S_inv(V.n_rows, U.n_rows);
	S_inv.zeros();
	s( arma::find(s < 1e-5) ).zeros();
	arma::uvec s_idx = arma::find(s>1e-10);
	s(s_idx) = 1./s(s_idx);
	S_inv.submat(0,0,s.size()-1,s.size()-1) = arma::diagmat(s);

	arma::vec y_s(y.n_elem+2);
	y_s.zeros();
	y_s.subvec(0,y.n_elem-1) = y;

	sp = V*S_inv*U.t()*y_s;

	return 0;
}

int spline1dFit(
	const arma::vec& y,
	const unsigned int m, // # of data point
	arma::vec& sp
	)
{
	if( m < 5 )
	{
		printf("m must be greater than 4.\n");
		return -1;
	}

	// build band matrix
	int r = y.n_elem;

	gBspline spline(0., r-1, m-1);

	arma::mat BM(r + 2, m + 2);
	BM.zeros();

	// filling
	gReal u;
	arma::vec4 B, ddB;
	int cpIdx;
	for( int i=0; i<r; i++ )
	{
		cpIdx = spline.get_index(i, &u);
		spline.Basis(u, B.memptr());
		
		if( cpIdx == spline.p().n_elem )
		{
			BM(i, cpIdx-3) = B(3);
			BM(i, cpIdx-2) = B(2);
			BM(i, cpIdx-1) = B(1);
		} else BM.submat(i, cpIdx-3, arma::SizeMat(1,4)) = arma::fliplr(B.t());
	}

	cpIdx = spline.get_index(0., &u);
	spline.ddBasis(u, ddB.memptr());
	BM.submat(r,cpIdx-3, arma::SizeMat(1,4)) = arma::fliplr(ddB.t());

	cpIdx = spline.get_index(r-1, &u);
	spline.ddBasis(u, ddB.memptr());
	BM.submat(r+1,cpIdx-3, arma::SizeMat(1,3)) = arma::fliplr(ddB.subvec(1,3).t());

	arma::mat U;
	arma::vec s;
	arma::mat V;

	if( !arma::svd(U,s,V,BM) )
	{
		printf("SVD fail.\n");
		return -1;
	}

	arma::mat S_inv(V.n_rows, U.n_rows);
	S_inv.zeros();
	s( arma::find(s < 1e-5) ).zeros();
	arma::uvec s_idx = arma::find(s>1e-10);
	s(s_idx) = 1./s(s_idx);
	S_inv.submat(0,0,s.size()-1,s.size()-1) = arma::diagmat(s);

	arma::vec y_s(y.n_elem+2);
	y_s.zeros();
	y_s.subvec(0,y.n_elem-1) = y;

	sp = V*S_inv*U.t()*y_s;

	return 0;
}

int splineNdFit(
	arma::mat& sp,
	const arma::mat& y,
	const unsigned int m, // # of target spline knot
	const bool rowMajor
	)
{
	if( m < 5 )
	{
		printf("m must be greater than 4.\n");
		return -1;
	}

	// build band matrix
	int r=0;

	if( rowMajor ) r = y.n_rows;
	else r = y.n_cols;

	gBspline spline(0., r-1, m-1);

	arma::mat BM(r + 2, m + 2);
	BM.zeros();

	// filling
	gReal u;
	arma::vec4 B, ddB;
	int cpIdx;
	for( int i=0; i<r; i++ )
	{
		cpIdx = spline.get_index(i, &u);
		spline.Basis(u, B.memptr());
		
		if( cpIdx == spline.p().n_elem )
		{
			BM(i, cpIdx-3) = B(3);
			BM(i, cpIdx-2) = B(2);
			BM(i, cpIdx-1) = B(1);
		} else BM.submat(i, cpIdx-3, arma::SizeMat(1,4)) = arma::fliplr(B.t());
	}

	cpIdx = spline.get_index(0., &u);
	spline.ddBasis(u, ddB.memptr());
	BM.submat(r,cpIdx-3, arma::SizeMat(1,4)) = arma::fliplr(ddB.t());

	cpIdx = spline.get_index(r-1, &u);
	spline.ddBasis(u, ddB.memptr());
	BM.submat(r+1,cpIdx-3, arma::SizeMat(1,3)) = arma::fliplr(ddB.subvec(1,3).t());

	arma::mat U;
	arma::vec s;
	arma::mat V;

	if( !arma::svd(U,s,V,BM) )
	{
		printf("SVD fail.\n");
		return -1;
	}

	
	s( arma::find(s < 1e-5) ).zeros();
	arma::uvec s_idx = arma::find(s>1e-10);
	s(s_idx) = 1./s(s_idx);
	//S_inv.submat(0,0,s.size()-1,s.size()-1) = arma::diagmat(s);

	if (rowMajor)
	{
		arma::mat S_inv(V.n_rows, U.n_rows, arma::fill::zeros);
		S_inv.diag() = s;
		//S_inv.submat(0, 0, s.size() - 1, s.size() - 1) = arma::diagmat(s);

		arma::mat invBM = V*S_inv*U.t();
		sp = (invBM.cols(0, r-1) * y).t();
	}
	else {
		arma::mat S_inv(U.n_rows, V.n_rows, arma::fill::zeros);
		S_inv.diag() = s;

		arma::mat invBM_t = U*S_inv*V.t();
		sp = y * invBM_t.rows(0, r - 1);
	}

	return 0;
}

int splineNdFit_sparse(
	arma::mat& sp,
	const arma::mat& y,
	const unsigned int m, // # of target spline knot
	const bool rowMajor
)
{
	if (m < 5)
	{
		printf("m must be greater than 4.\n");
		return -1;
	}

	// build band matrix
	int r = 0;
	int D = 0;

	if (rowMajor) {
		r = y.n_rows;
		D = y.n_cols;
	}
	else {
		r = y.n_cols;
		D = y.n_rows;
	}

	gBspline spline(0., r - 1, m - 1);

	std::vector<unsigned int> rows;
	std::vector<unsigned int> cols;
	std::vector<double> values;

	// filling
	gReal u;
	arma::vec4 B, ddB;
	int cpIdx;
	for (int i = 0; i<r; i++)
	{
		cpIdx = spline.get_index(i, &u);
		spline.Basis(u, B.memptr());

		if (cpIdx == spline.p().n_elem)
		{
			rows.push_back(i);
			rows.push_back(i);
			rows.push_back(i);

			cols.push_back(cpIdx - 3);
			cols.push_back(cpIdx - 2);
			cols.push_back(cpIdx - 1);

			values.push_back(B(3));
			values.push_back(B(2));
			values.push_back(B(1));
		}
		else {
			//BM.submat(i, cpIdx - 3, arma::SizeMat(1, 4)) = arma::fliplr(B.t());
			rows.push_back(i);
			rows.push_back(i);
			rows.push_back(i);
			rows.push_back(i);

			cols.push_back(cpIdx - 3);
			cols.push_back(cpIdx - 2);
			cols.push_back(cpIdx - 1);
			cols.push_back(cpIdx - 0);

			values.push_back(B(3));
			values.push_back(B(2));
			values.push_back(B(1));
			values.push_back(B(0));
		}
	}

	{
		cpIdx = spline.get_index(0., &u);
		spline.ddBasis(u, ddB.memptr());


		rows.push_back(r);
		rows.push_back(r);
		rows.push_back(r);
		rows.push_back(r);

		cols.push_back(cpIdx - 3);
		cols.push_back(cpIdx - 2);
		cols.push_back(cpIdx - 1);
		cols.push_back(cpIdx - 0);

		values.push_back(ddB(3));
		values.push_back(ddB(2));
		values.push_back(ddB(1));
		values.push_back(ddB(0));

		//BM.submat(r, cpIdx - 3, arma::SizeMat(1, 4)) = arma::fliplr(ddB.t());
	}

	{
		cpIdx = spline.get_index(r - 1, &u);
		spline.ddBasis(u, ddB.memptr());

		rows.push_back(r + 1);
		rows.push_back(r + 1);
		rows.push_back(r + 1);
		rows.push_back(r + 1);

		cols.push_back(cpIdx - 3);
		cols.push_back(cpIdx - 2);
		cols.push_back(cpIdx - 1);

		values.push_back(ddB(3));
		values.push_back(ddB(2));
		values.push_back(ddB(1));

		//BM.submat(r + 1, cpIdx - 3, arma::SizeMat(1, 3)) = arma::fliplr(ddB.subvec(1, 3).t());
	}
	/*
	cpIdx = spline.get_index(0., &u);
	spline.ddBasis(u, ddB.memptr());
	{
	rows.push_back(r);
	rows.push_back(r);
	rows.push_back(r);
	rows.push_back(r);

	cols.push_back(cpIdx - 3);
	cols.push_back(cpIdx - 2);
	cols.push_back(cpIdx - 1);
	cols.push_back(cpIdx - 0);

	values.push_back(ddB(3));
	values.push_back(ddB(2));
	values.push_back(ddB(1));
	values.push_back(ddB(0));
	}
	//BM.submat(r, cpIdx - 3, arma::SizeMat(1, 4)) = arma::fliplr(ddB.t());

	cpIdx = spline.get_index(r - 1, &u);
	spline.ddBasis(u, ddB.memptr());
	{
	rows.push_back(r + 1);
	rows.push_back(r + 1);
	rows.push_back(r + 1);
	rows.push_back(r + 1);

	cols.push_back(cpIdx - 3);
	cols.push_back(cpIdx - 2);
	cols.push_back(cpIdx - 1);

	values.push_back(ddB(3));
	values.push_back(ddB(2));
	values.push_back(ddB(1));
	}
	//BM.submat(r + 1, cpIdx - 3, arma::SizeMat(1, 3)) = arma::fliplr(ddB.subvec(1, 3).t());
	*/

	arma::umat locs(2, values.size());
	arma::vec vals(values.size());

	for (int rIdx = 0; rIdx<values.size(); rIdx++)
	{
		locs(0, rIdx) = rows[rIdx];
		locs(1, rIdx) = cols[rIdx];
		vals(rIdx) = values[rIdx];
	}

	//arma::sp_mat BM(locs, vals, r, m + 2);
	arma::sp_mat BM(locs, vals, r + 2, m + 2);

	if (!rowMajor)
	{
		arma::mat y_t = y.t();
		y_t = arma::join_vert(y_t, arma::zeros(2, y_t.n_cols));

		arma::sp_mat BM_t = BM.t();
		sp = arma::spsolve(BM_t * BM, BM_t * y_t);
		sp = sp.t();
	}
	else {
		arma::mat y_ = arma::join_horiz(y, arma::zeros(2, y.n_rows));
		sp = arma::spsolve(BM, y_);
	}

	return 0;
}