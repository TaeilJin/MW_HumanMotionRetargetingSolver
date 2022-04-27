#include <armadillo>

int spline1dFit(
	const arma::vec& x,
	const arma::vec& y,
	const unsigned int m, // # of target spline knot
	arma::vec& sp
	);

int spline1dFit(
	const arma::vec& y,
	const unsigned int m, // # of target spline knot
	arma::vec& sp
	);

int splineNdFit(
	arma::mat& sp,
	const arma::mat& y,
	const unsigned int m, // # of target spline knot
	const bool rowMajor = true
	);

int splineNdFit_sparse(
	arma::mat& sp,
	const arma::mat& y,
	const unsigned int m, // # of target spline knot
	const bool rowMajor = true
);

