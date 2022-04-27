//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
#include <string>
#include <iostream>
#include "mbs/gArmaUtil.h"


using namespace std;

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
// - output: 
// residual = || Ax - y ||
//
void solve_Ax_y(arma::vec& x, const arma::mat& A, const arma::vec& y, const arma::vec& w, const arma::vec* x_d, gReal lambda, gReal* residual)
{
	int i;
	gReal ww;
	int m = A.n_cols;	

	if( A.n_rows < A.n_cols && lambda < gEPSILON ) return; //fail

	gReal p = 1-lambda;
	arma::mat AA(m,m);
	arma::vec b(m);
	
	// b = p*A'*y
	b = A.t()*(p*y);

	// AA = p*A'*A
	AA = p*A.t()*A;

	// AA += lambda*w'w
	// b += lambda*w'w*x_d
	if(lambda > gEPSILON){
		for(i=0;i<m;++i)
		{
			ww = w(i);
			ww *= ww;
			ww *= lambda;
			AA(i,i)+=ww;
			if(x_d) b(i)+=ww*(*x_d)(i); 
		}
	}

	// solve AA*u = b
	x = arma::solve(AA,b);
	
	if(residual)
	{		
		arma::vec res = y - A*x;
		*residual = arma::norm(res);
	}		
}

int makeSkewSym3x3(arma::mat& mat, gVec3 v)
{
	// assert if it is 3x3 matrix
	if(!(mat.n_rows == 3) || !(mat.n_cols == 3)) return -1;
	mat(0,0)=0.0;
	mat(0,1)=-v.z();
	mat(0,2)=v.y();
	mat(1,0)=v.z();
	mat(1,1)=0.0;
	mat(1,2)=-v.x();
	mat(2,0)=-v.y();
	mat(2,1)=v.x();
	mat(2,2)=0.0;
	return 1;
}

int makeSkewSym3x3(arma::mat& mat, const arma::vec3& v)
{
	// assert if it is 3x3 matrix
	if( mat.n_rows != 3 || mat.n_cols != 3)
	{
		mat.set_size(3,3);
	}

	mat(0,0)=0.0;
	mat(0,1)=-v(2);
	mat(0,2)=v(1);
	mat(1,0)=v(2);
	mat(1,1)=0.0;
	mat(1,2)=-v(0);
	mat(2,0)=-v(1);
	mat(2,1)=v(0);
	mat(2,2)=0.0;
	return 1;
}

#if 0 //GSL IS DEPRECATED



int GV_gemv_partial(CBLAS_TRANSPOSE_t TransA,
                                gReal alpha,
                                const GM * A,
                                const GV * X,
                                gReal beta,
                                GV * Y,
								std::vector<int>& index
								)
{
	GV_scale(Y,beta); //Y = beta*Y

	if(TransA == CblasNoTrans){
		for(std::vector<int>::iterator it = index.begin(); it != index.end(); it++)
		{
			GB_axpy(alpha * GV_get(X,*it), &GM_const_column(A,*it).vector, Y); // Y += alpha*X(i)*A(i)
		}
	}else if(TransA == CblasTrans){
		for(std::vector<int>::iterator it = index.begin(); it != index.end(); it++)
		{
			GB_axpy(alpha * GV_get(X,*it), &GM_const_row(A,*it).vector, Y); // Y += alpha*X(i)*A^T(i)
		}
	}else{
		return GSL_FAILURE;
	}

	return GSL_SUCCESS;
}



// compute sum of gVec3 v
gReal GV_sum(const GV* v)
{
	gReal sum=0;
	for(size_t i=0;i<v->size;++i)	sum+=GV_get(v,i);
	return sum;
}

// compute sum of column col of matrix m
gReal GM_colsum(const GM* m, size_t c)
{
	return GV_sum(&GM_const_column(m,c).vector);
}

// compute sum of row r of matrix m
gReal GM_rowsum(const GM* m, size_t r)
{
	return GV_sum(&GM_const_row(m,r).vector);
}

// dompute sum of all elements of matrix m
gReal GM_sum(const GM* m)
{
	return GV_sum(&GV_const_view_array(m->data,m->size1*m->size2).vector);
}

// divide each element with sum
void GV_normalize(GV* v)
{
	gReal sum = gSqrt(GB_nrm2(v));
	GB_scal(1.0/sum,v);	
}

// cross product
void GV_cross(const GV* v1, const GV* v2, GM* m)
{
	size_t dim = v1->size;
	GB_gemm(
		CblasNoTrans,
		CblasTrans,
		1.0,
		&GM_view_array(v1->data,dim,1).matrix,
		&GM_view_array(v2->data,dim,1).matrix,
		0,
		m);	
}

//0: success , -1: EOF
static int	read_word	(FILE* fp, char* buf)
{
	int ch, i = 0;
	do {
		ch = fgetc(fp);
		if ( ch == EOF ) return -1;
	} while ( ch == '\0' || ch == '\t' || ch == '\n' || ch == ' ' );
	do {
		buf[i++] = (char)ch;
		ch = fgetc(fp);
	} while ( ch != '\0' && ch != '\t' && ch != '\n' && ch!=' ' && ch != EOF );
	buf[i] = '\0'; 
	return 0;
}

//start with number
static bool isNumeric(char* ch)
{
	if(ch[0] == '-' || ( ch[0] >= '0' && ch[0] <= '9' ) )	return true;
	return false;
}

static int readNumber (FILE* fp, char* buf)
{
	int re;
	do{
		re = read_word(fp,buf);
	}while( re != -1 && !isNumeric(buf) );
	return re;
}


// if v == NULL, fp must specify dimension in the first line and readDimension must be true.
// if v != NULL, fp must contain a vector of the same dimension
int GV_read(FILE* fp, GV*& v, bool readDimension)
{
	char buf[128];
	size_t sz;
	if(readDimension)
	{
		readNumber(fp,buf);
		sz = size_t( atoi(buf) );
		
		if( !v ){
			v = GV_alloc(sz);
		}
		else if ( v->size != sz ){
			return -1; // dimension is different
		}
	}
	else
	{
		sz = v->size;
	}
	
	for(size_t i=0;i<sz;++i)
	{
		if( readNumber(fp,buf) != EOF ) GV_set(v,i, atof(buf) );
		else return -1;
	}
	return 0;
}

int GM_read(FILE* fp, GM*& m, bool readDimension)
{
	char buf[128];
	size_t sz1, sz2;
	if(readDimension){
		readNumber(fp,buf);
		sz1 = size_t( atoi(buf) );
		readNumber(fp,buf);
		sz2 = size_t( atoi(buf) );
				
		if( !m ){
			m = GM_alloc(sz1,sz2);
		}
		else if ( m->size1 != sz1 || m->size2 != sz2 ){
			return -1; // dimension is different
		}
	}
	else{
		sz1 = m->size1;
		sz2 = m->size2;
	}

	size_t i,j;
	for(i=0;i<m->size1;++i)
	for(j=0;j<m->size2;++j)
	{
		if( readNumber(fp,buf) != EOF ) GM_set(m,i,j,atof(buf) );
		else return -1;
	}
	return 0;
}


// fp: FILE pointer. It can be stdout, stderr
// fmat : e.g. "%g","%5.4f"
// name : name of the vector
// if transpose == true, write in row rather than column
// if writeDimension == true, write dimension first
// if flush == true, it flushes the output
void GV_write(GV* v, FILE* fp, const char* fmat, const char* name, bool transpose, bool writeDimension, bool flush)
{
	char fmat_[128];
	if(transpose){
		strcpy( fmat_, fmat );	strcat( fmat_, " ");	// fmat_ = fmat+" "			
	}
	else{
		strcpy( fmat_, fmat );	strcat( fmat_, "\n");	// fmat_ = fmat\n
	}
	
	if (writeDimension) fprintf(fp,"%d\n",v->size);
	if (name) fprintf(fp,"%s=[ ",name);
	for(size_t i=0;i<v->size;++i)	fprintf(fp,fmat_, GV_get(v,i));
	if (name) fprintf(fp," ];");
	fprintf(fp,"\n");
	if(flush) fflush(fp);
}

// fp: FILE pointer. It can be stdout, stderr
// fmat : e.g. "%g","%5.4f"
// name : name of the matrix
// if square == true, write in square rather than column
// if flush == true, it flushes the output
void GM_write( GM* m, FILE* fp, const char* fmat, const char* name, bool square, bool writeDimension, bool flush)
{
	char fmat_[128];
	if(square)
	{ 
		strcpy( fmat_, fmat );	strcat( fmat_, " ");	// fmat_ = fmat+" "            
	}
	else
	{
		strcpy( fmat_, fmat );	strcat( fmat_, "\n");	// fmat_ = fmat\n
	}
	
	if(writeDimension) fprintf(fp,"%d\n%d\n", m->size1, m->size2);
	if(name) fprintf(fp,"%s=[ ", name);
	
	for(size_t i=0;i<m->size1;++i)	
	{
		for(size_t j=0;j<m->size2;++j)
		{ 
			fprintf(fp,fmat_, GM_get(m,i,j));
		}
		if(name) fprintf(fp," ;");
		if(square) fprintf(fp,"\n");
	}
	if(name) fprintf(fp,"];\n");
}



// solve min( |Ax-y| + lambda*|wx| ), x>=0
// => (A'A+lambda*w'w)x = A'y
// Inequality constraint is satisfied by giving high weight value 
// to violating elements.
void solve_Ax_y_pos_x(GV* x, const GM* A, const GV* y, const GV* w_,
					  const gReal lambda)
{
	int signum;
	size_t i,idx;
	gReal u_min,ww;
	size_t cnt = 0;
	bool bFin;
	size_t m = A->size2;

	GM* AA	= GM_alloc(m,m);
	GV* w	= GV_alloc(m);
	GV* b	= GV_alloc(m);
	gsl_permutation* permu = gsl_permutation_alloc(m);
    
	// b = A'y
	GB_gemv (CblasTrans,1,A,y,0,b);	

	// initialize w
	GV_memcpy(w,w_);
		
	do{
		// AA = A'A
		GB_gemm (CblasTrans,CblasNoTrans,1,A,A,0,AA);	
		for(i=0;i<m;++i)
		{
			ww = GV_get(w,i);
			ww *= ww;
			ww *= lambda;
			GM_set(AA,i,i,GM_get(AA,i,i)+ww);	// AA += w'w
		}

		// solve AA*u = b
		gsl_linalg_LU_decomp (AA,permu,&signum);
		//gReal de = gsl_linalg_LU_det (AA,signum);
		gsl_linalg_LU_solve  (AA,permu,b,x);	

		// check negative element
		idx = GV_min_index (x);
		u_min = GV_get(x,idx);

		if( u_min < -1e-2 ){
			GV_set(w,idx,10*GV_get(w,idx) ); // increase weight by ten
			bFin = false;
		}
		else bFin = true;
	}while ( !bFin && cnt++ < 10*m );

	GM_free(AA);
	GV_free(w);
	GV_free(b);
	gsl_permutation_free(permu);
}

//return the index of the most severe inequality violation. -1 if no violation.
//thesh: threshold to determine violation.
// type_c
// 0 : >=
// 1 : ==
// 2 : <=
// other : none
static int findWorstViolation(const GV* x, const GV* x_c, const gsl_vector_int* type_c, gReal thresh)
{
	gReal v;
	gReal maxv = thresh;
	int idx = -1;
	int t;
	for(int i=0; i<x->size;++i)
	{
		t = gsl_vector_int_get(type_c,i);

		if( t == 0 ) // x_c[i] <= x[i]
			v = GV_get(x_c,i) - GV_get(x,i);
		else if( t == 2 ) // x[i] <= x_c[i]
			v = GV_get(x,i) - GV_get(x_c,i);
		else 
			v = 0;
		
		if( v > maxv ){
			maxv = v; 
			idx = i;		
		}
	}
	return idx;
}

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
// - output: 
// residual = || Ax - y ||
//
void solve_Ax_y(GV* x, const GM* A, const GV* y, const GV* w, const GV* x_d, gReal lambda, gReal* residual)
{
	int signum;
	size_t i;
	gReal ww;
	size_t m = A->size2;

	if( A->size1 < A->size2 && lambda < gEPSILON ) return; //fail

	gReal p = 1-lambda;
	
	GM* AA	= GM_alloc(m,m);
	GV* b	= GV_alloc(m);
	gsl_permutation* permu = gsl_permutation_alloc(m);
    
	// b = p*A'*y
	GB_gemv (CblasTrans,p,A,y,0,b);	

	// AA = p*A'*A
	GB_gemm (CblasTrans,CblasNoTrans,p,A,A,0,AA);	

	// AA += lambda*w'w
	// b += lambda*w'w*x_d
	if(lambda > gEPSILON){
		for(i=0;i<m;++i)
		{
			ww = GV_get(w,i);
			ww *= ww;
			ww *= lambda;
			GM_set(AA,i,i,GM_get(AA,i,i)+ww);	

			if(x_d) GV_set(b,i,GV_get(b,i)+ww*GV_get(x_d,i));
		}
	}

	// solve AA*u = b
	gsl_linalg_LU_decomp (AA,permu,&signum);
	gsl_linalg_LU_solve  (AA,permu,b,x);	
	
	if(residual)
	{
		GV* res = GV_alloc(y->size);
		GV_memcpy(res,y);
		GB_gemv(CblasNoTrans,-1,A,x,1,res); //res = y - Ax
		*residual = GB_nrm2(res);
		GV_free(res);
	}

	GM_free(AA);
	GV_free(b);
	gsl_permutation_free(permu);
}


// solve min( w*|A*x-y|^2 + (1-w)*|w_d(x-x_d)|^2 ) s.t. x[i] type_c[i] x_c[i] for all i.
// w weights between the two objective values between 0 and 1.
// Since typically there are infinitely many solutions to satisfy A*x=y, w should be smaller
// than 1.
// type_c
// 0 : x[i] >= x_c[i]
// 1 : x[i] == x_c[i]
// 2 : x[i] <= x_c[i]
// other : none
// this function enforces the inequality constraints iteratively.
// In each iteration, worst violation is found, and it is clamped to the bound.
bool solve_Ax_y_const(GV* x, const GM* A, const GV* y, const GV* w, const GV* x_d, gReal lambda,
					  const GV* x_c, const gsl_vector_int* type_c, gReal thresh, gReal* residual)
{
	int status = 0; // 1: operation success, -1: operation failure, 0: in process
	int n = A->size1;
	int m1, m = A->size2;
	int idx, idx1;
	

	//copy type
	gsl_vector_int* type = gsl_vector_int_alloc(type_c->size);
	gsl_vector_int_memcpy(type,type_c);

	//count num equality conditions
	int numEqCon = 0;
	for(int i=0;i<type->size;++i)
	{
		if(gsl_vector_int_get(type,i)==1) numEqCon++;
	}

	while(status==0)
	{		
		/* column size of new matrix equation. We will treat equality condition as hard constraint.
		I.e., we will solve A1*x1 = y1 where y1 = y - A2*x_c2 where A2 is the portion of the A that corresponds to the equality constraints.
		*/
		m1 = A->size2 - numEqCon; 

		GV* w1 = GV_alloc(m1);		
		GM* A1 = GM_alloc(n,m1); //submatrix of A
		GV* x1 = GV_alloc(m1); //subvector of ddq
		GV* y1 = GV_alloc(n);
		GV* x_d1 = x_d ? GV_calloc(m1) : NULL;

		//make w1,A1,x_d1, y1
		idx1 = 0;
		GV_memcpy(y1,y); //y1 = y
		for(int i=0;i<m;++i)
		{
			if( gsl_vector_int_get(type,i) == 1 ){ // equality condition
				//y1 -= A[:,i]*x_c[i]
				gReal x_eq = GV_get(x_c,i);
				for(int j=0;j<y1->size;++j){
					GV_set( y1, j, GV_get(y1,j) - GM_get(A,j,i) * x_eq );
				}
			}
			else{
				GM_set_col(A1,idx1,&GM_const_column(A,i).vector);
				if(x_d) GV_set(x_d1,idx1,GV_get(x_d,i));
				GV_set(w1,idx1,GV_get(w,i));
				idx1++;
			}
		}

		solve_Ax_y(x1,A1,y1,w1,x_d1,lambda,residual);
		
		//make x
		idx1 = 0;
		for(int i=0;i<m;++i)
		{
			if( gsl_vector_int_get(type,i) == 1 ){ //equality constraint
				GV_set(x,i,GV_get(x_c,i));
			} else {
				GV_set(x,i,GV_get(x1,idx1++));
			}
		}

		//find most severly violated constraint
		idx = findWorstViolation(x,x_c,type,thresh);

		//if no constraint violation, success = true. otherwise, change worst violation to equality constraint (clamping it)
		if ( idx == -1 ) 
			status = 1;
		else{
			gsl_vector_int_set(type,idx,1); //change to equality constraint
			
			//recount the number of eq conditions
			numEqCon = 0;
			for(int i=0;i<type->size;++i)
			{
				if(gsl_vector_int_get(type,i)==1) numEqCon++;
			}

			//check failure
			if( m - numEqCon < n ) status = -1;
		}
		
		GV_free(w1);
		GM_free(A1);
		GV_free(x1);
		if(x_d1) GV_free(x_d1);
		GV_free(y1);
	};

	gsl_vector_int_free(type);

	return status == 1 ? true : false;
}




void GM_randomize(GM* m, gReal low, gReal high)
{
	size_t i,j;
	gReal r;
	gReal coeff = (high-low)/RAND_MAX;

	for(i=0;i<m->size1;++i)
		for(j=0;j<m->size2;++j)
		{
			r = rand()*coeff + low;
			GM_set(m,i,j,r);
		}
}

void GV_randomize(GV* v, gReal low, gReal high)
{
	size_t i;
	gReal r;
	gReal coeff = (high-low)/RAND_MAX;

	for(i=0;i<v->size;++i)
	{
		r = rand()*coeff + low;
		GV_set(v,i,r);
	}
}

void GV_perturb(GV* v,gReal low, gReal high)
{
	size_t i;
	gReal r;
	gReal coeff = (high-low)/RAND_MAX;

	for(i=0;i<v->size;++i)
	{
		r = GV_get(v,i) + rand()*coeff + low;
		GV_set(v,i,r);
	}
}

static gReal random(gReal low, gReal high)
{
	return low + ((high-low) * rand()) / RAND_MAX;
}

void GV_perturb(GV* v,gReal ratio)
{
	size_t i;
	gReal vi,r,noise;
	for(i=0;i<v->size;++i)
	{
		vi = GV_get(v,i);
		r = fabs( vi );
		noise = random( -r*ratio, r*ratio );
		GV_set(v,i,vi + noise);
	}
}



// create right pseudo-inverse
// return pinvM=M'(MM'+epsilon*1)^(-1), so M*pinvM = I
// M should have more columns than rows
// epsilon is small positive number for stability of the inverse. useful when M is rank-defficient.
static GM* compute_right_pseudo_inverse(const GM* M, gReal eps)
{
	size_t m=M->size1, n=M->size2;
	if(m>n) return NULL;

	GM* MMt = GM_calloc(m,m); // square = MM'
	GM* MMti= GM_alloc(m,m);
	
	gsl_permutation* p = gsl_permutation_alloc(m);
	int signum;

	for(int i=0;i<m;++i) GM_set(MMt,i,i,eps); //add eps to diagnoal entries

	gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1, M, M, 1, MMt); // MMt += M*M'
	
	gsl_linalg_LU_decomp (MMt, p, &signum);

	//gReal det = gsl_linalg_LU_det(MMt,signum);

	gsl_linalg_LU_invert (MMt, p, MMti);	// MMti = inv(MMt)
	
	GM* pinvM = GM_alloc(n,m);
	gsl_blas_dgemm (CblasTrans, CblasNoTrans, 1, M, MMti, 0, pinvM); //pinvM = M'*MMti

	GM_free(MMt);
	GM_free(MMti);
	gsl_permutation_free(p);

	return pinvM;
}

//pinvM=(M'M+epsilon*1)^(-1)M'
static GM* compute_left_pseudo_inverse(const GM* M, gReal eps)
{
	size_t m=M->size1, n=M->size2;
	if(m<n) return NULL;

	GM* MtM = GM_calloc(n,n); // square = M'M
	GM* MtMi= GM_alloc(n,n);
	
	gsl_permutation* p = gsl_permutation_alloc(n);
	int signum;

	for(int i=0;i<n;++i) GM_set(MtM,i,i,eps); //add eps to diagnoal entries

	gsl_blas_dgemm (CblasTrans, CblasNoTrans, 1, M, M, 1, MtM); // MtM += M'* M
	
	gsl_linalg_LU_decomp (MtM, p, &signum);

	//gReal det = gsl_linalg_LU_det(MMt,signum);

	gsl_linalg_LU_invert (MtM, p, MtMi);	// MMti = inv(MMt)
	
	GM* pinvM = GM_alloc(n,m);
	gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1, MtMi, M, 0, pinvM); //pinvM = M'*MMti

	GM_free(MtM);
	GM_free(MtMi);
	gsl_permutation_free(p);

	return pinvM;
}


//TODO: THIS PROBLEM CAN BE MORE EASILY SOLVED if formed as a standard least square problem
//and use QR decomposition. DO IT!!
// solve min( w*|A2*x-y2|^2 + (1-w)*|w_d(x-x_d)|^2 ) s.t. A1*x = y1
// i.e., A1*x=y1 is hard constraint, whereas A2*x=y2 and x=x_d are soft constraints
// A1, A2 are flat matrix (ncol >nrow)
// w weights between the two soft constraints
// epsilon is the robustness factor of computing pseudo-inverse of A1.
// output: residual = ||A2*x-y2||
void solve_Ax_y(GV* x, 
				const GM* A1, const GV* y1, 
				const GM* A2, const GV* y2,
				const GV* w_d, const GV* x_d, float w, gReal epsilon, gReal* residual)
{
	//x = x1 + N*z where x1 = pinv(A1)*y1 and N = (1 - pinv(A1)*A1)
	//z should satisfy
	//G*z = h where G = N'*(w*A2'*A2 + (1-w)*w_d*w_d)*N and h = N'*(w*A2'*d + (1-w)*w_d*w_d*(x_d-x1))
	//where d = y2-A2*x1
	//G and N are (large) square matrices.
	//N is idempotent matrix (i.e., N*N=N)
	//G is symmetric matrix and rank(G)==rank(N). non-invertible.

	gReal tmp;
	float w11 = 1.0-w;

	//make x1
	GM* A1_inv = compute_right_pseudo_inverse(A1,epsilon);
	//GV* x1 = GV_alloc(x->size); 
	GV* x1 = x; //we will use x for x1
	GB_gemv(CblasNoTrans,1,A1_inv,y1,0,x1); //x1=pinv(A1)*y1

	//make N
	GM* N = GM_alloc(A1->size2,A1->size2);
	GM_set_identity(N);
	GB_gemm(CblasNoTrans, CblasNoTrans, -1, A1_inv, A1, 1, N); // N -= A1_inv * A1

	//check rank(N)
	//interesting fact: rank(N) = trace(N)
	//gReal r=0;
	//for(int i=0;i<N->size1;++i) r+=GM_get(N,i,i);

	//{make G and h
	GM* G = GM_calloc(N->size1,N->size2); // G = 0
	GV* h = GV_alloc(x->size);
	GV* g = GV_calloc(x->size); // g = 0

	if(w11>0)
	{
		//G
		for(int i=0;i<G->size1;++i){
			tmp = GV_get(w_d,i);
			GM_set(G,i,i,GM_get(G,i,i) + w11*tmp*tmp); // G[i,i] += (1-w)*w_d[i]*w_d[i]
		}

		//h
		GV_memcpy(g,x_d); GV_sub(g,x1); // g = x_d - x1
		for(int i=0;i<g->size;++i){
			tmp = GV_get(w_d,i);
			GV_set(g,i,GV_get(g,i)*tmp*tmp*w11); // g[i] *= (1-w)*w_d[i]*w_d[i]
		}
	}

	if(w>0)
	{
		//G
		GB_gemm(CblasTrans,CblasNoTrans,w,A2,A2,1,G); // G += w*A2'*A2

		//h
		GV* d = GV_calloc(y2->size);
		GV_memcpy(d,y2); GB_gemv(CblasNoTrans,-1,A2,x1,1,d); // d = y2 - A2*x1
		GB_gemv(CblasTrans, w, A2, d, 1, g); // g += w*A2'*d
		GV_free(d);
	}

	//G
	GM* L = GM_alloc(N->size1,N->size2);  
	GB_gemm(CblasTrans,CblasNoTrans,1,N,G,0,L); // L = N'*G
	GB_gemm(CblasNoTrans,CblasNoTrans,1,L,N,0,G);// G = L*N

	//h
	GB_gemv(CblasTrans, 1, N, g, 0, h); // h = N'*g
	//}end of making G and h

	//solve G*z=h using LU decomp
	//note that G is not full rank, so there are many solutions to z
	//Note that G = N'*H*N where N = (1-A1'*A1) (A1's null space, idempotent matrix)
	//and h = N'*g. I'm not sure if LU decomp works in the case..should verify later.
	GV* z = GV_alloc(x->size);
	gsl_permutation* p = gsl_permutation_alloc(G->size1);
	int signum;
	gsl_linalg_LU_decomp (G,p,&signum);
	gsl_linalg_LU_solve  (G,p,h,z);	 // z = inv(G)*h
	gsl_permutation_free(p);

	////solve G*z = h using SVD
	//GV* z = GV_alloc(x->size);
	//GM* V = GM_alloc(G->size1,G->size2);
	//GV* S = GV_alloc(G->size1);	
	//GV* work = GV_alloc(G->size2);
	////gsl_linalg_SV_decomp_jacobi(G,V,S);
	//gsl_linalg_SV_decomp(G,V,S,work);
	////discard near-zero singular values
	//for(int i=0;i<S->size;i++){	if(GV_get(S,i)<1e-6) GV_set(S,i,0);	}
	//gsl_linalg_SV_solve(G,V,S,h,z);
	//GM_free(V);
	//GV_free(S);
	//GV_free(work);

	//x = x1 + N*z
	//GV_memcpy(x,x1); //x = x1 
	GB_gemv(CblasNoTrans,1,N,z,1,x); // x += N*z

	if(residual)
	{
		GV* res = GV_alloc(y2->size);
		GV_memcpy(res,y2);
		GB_gemv(CblasNoTrans,-1,A2,x,1,res); //res = y2 - A2x
		*residual = GB_nrm2(res);
		GV_free(res);
	}

	GV_free(z);	
	GM_free(L);
	GM_free(G);
	GV_free(h);
	GV_free(g);
	//GV_free(x1);
	GM_free(A1_inv);
	GM_free(N);
}



// solve min( w*|A2*x-y2|^2 + (1-w)*|w_d(x-x_d)|^2 ) s.t. A1*x = y1 and x[i] type_c[i] x_c[i] for all i.
// A1, A2 are flat matrix (ncol >nrow)
// w weights between the two objective values between 0 and 1.
// Since typically there are infinitely many solutions to satisfy A1*x=y1 and A2*x=y2, w should be smaller
// than 1.
// type_c
// 0 : x[i] >= x_c[i]
// 1 : x[i] == x_c[i]
// 2 : x[i] <= x_c[i]
// other : none
// this function enforces the inequality constraints iteratively.
// In each iteration, worst violation is found, and it is clamped to the bound.
bool solve_Ax_y_const(GV* x, 
					  const GM* A1, const GV* y1, 
					  const GM* A2, const GV* y2,
					  const GV* w_d, const GV* x_d, float w, gReal epsilon,
					  const GV* x_c, const gsl_vector_int* type_c, gReal thresh, gReal* residual)
{
	int status = 0; // 1: operation success, -1: operation failure, 0: in process
	int n1 = A1->size1;
	int n2 = ( w == 0 ? 0 : A2->size1);
	int m = A1->size2;
	
	int mr;
	int idx, idxr;
	
	//copy type
	gsl_vector_int* type = gsl_vector_int_alloc(type_c->size);
	gsl_vector_int_memcpy(type,type_c);

	//count num equality conditions
	int numEqCon = 0;
	for(int i=0;i<type->size;++i)
	{
		if(gsl_vector_int_get(type,i)==1) numEqCon++;
	}


	while(status==0)
	{		
		/* column size of new matrix equation. We will treat equality condition as hard constraint.
		I.e., we will solve A1*x1 = y1 where y1 = y - A2*x_c2 where A2 is the portion of the A that corresponds to the equality constraints.
		*/
		mr = m - numEqCon; 

		GM* A1r = GM_alloc(n1,mr); //submatrix of A1
		GV* xr = GV_alloc(mr); //subvector of x
		GV* y1r = GV_alloc(n1);
		GV* w_dr = NULL;
		GM* A2r = NULL;
		GV* y2r = NULL;
		GV* x_dr = NULL;

		if(w > 0){
			A2r = GM_alloc(n2,mr);
			y2r = GV_alloc(n2);
		}

		if(w < 1){
			w_dr = GV_alloc(mr);
			x_dr = GV_alloc(mr);
		}

		//make A1r,y1r
		idxr = 0;
		GV_memcpy(y1r,y1); //y1r = y1		
		for(int i=0;i<m;++i)
		{
			if( gsl_vector_int_get(type,i) == 1 ){ // equality condition
				//y1r -= A1[:,i]*x_c[i]
				gReal x_eq = GV_get(x_c,i);
				for(int j=0;j<n1;++j) GV_set( y1r, j, GV_get(y1r,j) - GM_get(A1,j,i) * x_eq );
			}
			else{
				GM_set_col(A1r,idxr,&GM_const_column(A1,i).vector);
				idxr++;
			}
		}

		//make A2r,y2r
		if(w > 0){
			idxr = 0;
			GV_memcpy(y2r,y2); //y2r = y2
			for(int i=0;i<m;++i)
			{
				if( gsl_vector_int_get(type,i) == 1 ){ // equality condition
					gReal x_eq = GV_get(x_c,i);
					for(int j=0;j<n2;++j) GV_set( y2r, j, GV_get(y2r,j) - GM_get(A2,j,i) * x_eq );				
				}
				else{
					GM_set_col(A2r,idxr,&GM_const_column(A2,i).vector);
					idxr++;
				}
			}
		}

		//make w_dr,x_dr
		if(w < 1){
			idxr = 0;
			for(int i=0;i<m;++i)
			{
				if( gsl_vector_int_get(type,i) != 1 ){ // if not equality condition
					GV_set(x_dr,idxr,GV_get(x_d,i));
					GV_set(w_dr,idxr,GV_get(w_d,i));
					idxr++;
				}
			}
		}


		//solve_Ax_y
		solve_Ax_y(xr, A1r, y1r, A2r, y2r, w_dr, x_dr, w, epsilon, residual);
		
		//make x
		idxr = 0;
		for(int i=0;i<m;++i)
		{
			if( gsl_vector_int_get(type,i) == 1 ){ //equality constraint
				GV_set(x,i,GV_get(x_c,i));
			} else {
				GV_set(x,i,GV_get(xr,idxr++));
			}
		}

		//find most severly violated constraint
		idx = findWorstViolation(x,x_c,type,thresh);

		//if no constraint violation, success = true. otherwise, change worst violation to equality constraint (clamping it)
		if ( idx == -1 ) 
			status = 1;
		else{
			gsl_vector_int_set(type,idx,1); //change to equality constraint
			
			//recount the number of eq conditions
			numEqCon = 0;
			for(int i=0;i<type->size;++i)
			{
				if(gsl_vector_int_get(type,i)==1) numEqCon++;
			}

			//check failure
			if( m - numEqCon < n1 ) status = -1;
		}
		
		GM_free(A1r);
		GV_free(xr);
		GV_free(y1r);
		if( w_dr ) GV_free(w_dr);		
		if( A2r ) GM_free(A2r);
		if( y2r ) GV_free(y2r);
		if( x_dr ) GV_free(x_dr);
	};

	gsl_vector_int_free(type);

	return status == 1 ? true : false;
}

//int GM_2_m_file(const char* infile,int row, int col,const char* name)
//{
//	char outfile[128];
//	sprintf(outfile,name);
//	strcat(outfile,".m");
//
//	FILE* fin = fopen(infile,"r");
//	if(!fin) return -1;
//	FILE* fout = fopen(outfile,"w");
//	if(!fout) return -1;
//
//	GM* m = GM_alloc(row,col);
//
//	if( GM_scan(fin,m) ) return -1;
//
//	fprintf(fout,"%s=",name);
//	GM_print(m,"%g",true,fout);
//	fprintf(fout,";");
//
//	GM_free(m);
//	
//	fclose(fout);
//	fclose(fin);
//
//	return 0;
//}

//int GV_swap_ptr(GV* v1, GV* v2)
//{
//	if( v1->size != v2->size ) return -1;
//	if( v1->stride != v2->stride ) return -1;
//	gReal* data = v1->data;
//	v1->data = v2->data; 
//	v2->data = data;
//
//	gsl_block* block = v1->block;
//	v1->block = v2->block;
//	v2->block = block;
//	return 0;
//}

/*
void gauss_seidel(GV* x, const GM* A, const GV* b, int num_iter)
{
	assert(A->size1 == A->size2);
	size_t i,j;
	gReal delta;
	size_t n=A->size1;

	while( (num_iter--) > 0 ){
		for(i=0;i<n;++i)
		{
			delta = 0;
			for(j=0;j<i;++j)
			{
				delta += GM_get(A,i,j)*GV_get(x,j);
			}
			for(j=i+1;j<n;++j)
			{
				delta += GM_get(A,i,j)*GV_get(x,j);
			}
			GV_set(x,i,(GV_get(b,i)-delta)/GM_get(A,i,i));
		}
	}
}

// return || A*x - b ||^2
gReal gauss_seidel_residual(GV* res, const GM* A, const GV* x, const GV* b)
{
	GV_memcpy(res,b);
	GB_gemv(CblasNoTrans,1,A,x,-1,res); // res = A*x - b
	gReal re;
	GB_dot(res,res,&re);
	return re;
}

void projected_gauss_seidel(GV* x, const GM* A, const GV* b, const GV* hi, const GV* lo, int num_iter)
{
	assert(A->size1 == A->size2);
	size_t i,j;
	gReal delta;
	size_t n=A->size1;

	while( (num_iter--) > 0 ){
		for(i=0;i<n;++i)
		{
			delta = 0;
			for(j=0;j<i;++j)
			{
				delta += GM_get(A,i,j)*GV_get(x,j);
			}
			for(j=i+1;j<n;++j)
			{
				delta += GM_get(A,i,j)*GV_get(x,j);
			}
			delta = (GV_get(b,i)-delta)/GM_get(A,i,i);
			
			//projection
			delta = gMin(delta, GV_get(hi,i));
			delta = gMax(delta, GV_get(lo,i));

			GV_set(x,i,delta);
		}
	}
}
*/

int make_matrix_gInertia(GM* M, const gInertia& J)
{
	if(M->size1 != 6 || M->size2 != 6) return -1; // error	

	//	   |						 |
	//     |       I		 mass[c] |
	//  M =|    -mass[c]     mass*E	 |, 
	//	   |						 |

	GM_set_zero(M);

	GM_set(M,0,0, J.rotInertia(gInertia::Ixx));
	GM_set(M,1,0, J.rotInertia(gInertia::Ixy));
	GM_set(M,2,0, J.rotInertia(gInertia::Ixz));

	GM_set(M,0,1, J.rotInertia(gInertia::Iyx));
	GM_set(M,1,1, J.rotInertia(gInertia::Iyy));
	GM_set(M,2,1, J.rotInertia(gInertia::Iyz));

	GM_set(M,0,2, J.rotInertia(gInertia::Izx));
	GM_set(M,1,2, J.rotInertia(gInertia::Izy));
	GM_set(M,2,2, J.rotInertia(gInertia::Izz));

	gReal mx = J.mass() * J.com(gVec3::X);
	gReal my = J.mass() * J.com(gVec3::Y);
	gReal mz = J.mass() * J.com(gVec3::Z);

	GM_set(M,0,4, -mz);
	GM_set(M,0,5,  my);
	GM_set(M,1,3,  mz);
	GM_set(M,1,5, -mx);
	GM_set(M,2,3, -my);
	GM_set(M,2,4,  mx);
	
	GM_set(M,3,1,  mz);
	GM_set(M,3,2, -my);
	GM_set(M,4,0, -mz);
	GM_set(M,4,2,  mx);
	GM_set(M,5,0,  my);
	GM_set(M,5,1, -mx);

	GM_set(M,3,3, J.mass());
	GM_set(M,4,4, J.mass());
	GM_set(M,5,5, J.mass());
	
	return 0;
}

int make_matrix_inv_gInertia(GM* M, gInertia& J)
{
	if(M->size1 != 6 || M->size2 != 6) return -1; // error	

	//			|						 |
	//			|   K		-K[c]		 |
	//inv(J) =	| [c]K   E/mass -[c]K[c] |, where K = R*inv(Ic)*R'
	//			|						 |

	//GM_set_zero(M);

	const gReal* K = J.inverseRotInertia();
	const gReal* C = J.com();

	gReal im = 1/J.mass();
	gReal b00 = C[1]*K[2] - C[2]*K[1];
	gReal b01 = C[2]*K[0] - C[0]*K[2];
	gReal b02 = C[0]*K[1] - C[1]*K[0];
	gReal b10 = C[1]*K[4] - C[2]*K[3];
	gReal b11 = C[2]*K[1] - C[0]*K[4];
	gReal b12 = C[0]*K[3] - C[1]*K[1];
	gReal b20 = C[1]*K[5] - C[2]*K[4];
	gReal b21 = C[2]*K[2] - C[0]*K[5];
	gReal b22 = C[0]*K[4] - C[1]*K[2];
	
	gReal d0 = b20*C[1] - b10*C[2]+im;
	gReal d1 = b21*C[1] - b11*C[2];
	gReal d2 = b22*C[1] - b12*C[2];
	gReal d3 = b01*C[2] - b21*C[0]+im;
	gReal d4 = b02*C[2] - b22*C[0];
	gReal d5 = b12*C[0] - b02*C[1]+im;

	GM_set(M,0,0, K[0]); GM_set(M,1,0, K[1]); GM_set(M,2,0, K[2]);
	GM_set(M,0,1, K[1]); GM_set(M,1,1, K[3]); GM_set(M,2,1, K[4]);
	GM_set(M,0,2, K[2]); GM_set(M,1,2, K[4]); GM_set(M,2,2, K[5]);

	GM_set(M,0,3,b00);	GM_set(M,0,4,b01); GM_set(M,0,5,b02);
	GM_set(M,1,3,b10);	GM_set(M,1,4,b11); GM_set(M,1,5,b12);
	GM_set(M,2,3,b20);	GM_set(M,2,4,b21); GM_set(M,2,5,b22);

	GM_set(M,3,0,b00);	GM_set(M,4,0,b01); GM_set(M,5,0,b02);
	GM_set(M,3,1,b10);	GM_set(M,4,1,b11); GM_set(M,5,1,b12);
	GM_set(M,3,2,b20);	GM_set(M,4,2,b21); GM_set(M,5,2,b22);

	GM_set(M,3,3,d0);	GM_set(M,3,4,d1); GM_set(M,3,5,d2);
	GM_set(M,4,3,d1);	GM_set(M,4,4,d3); GM_set(M,4,5,d4);
	GM_set(M,5,3,d2);	GM_set(M,5,4,d4); GM_set(M,5,5,d5);
	
	return 0;
}

int make_matrix_gRotMat(GM* M, const gRotMat& R)
{
	if(M->size1 != 3 || M->size2 != 3 ) return -1;

	GM_set(M,0,0,R.e(gRotMat::R00));
	GM_set(M,0,1,R.e(gRotMat::R01));
	GM_set(M,0,2,R.e(gRotMat::R02));

	GM_set(M,1,0,R.e(gRotMat::R10));
	GM_set(M,1,1,R.e(gRotMat::R11));
	GM_set(M,1,2,R.e(gRotMat::R12));

	GM_set(M,2,0,R.e(gRotMat::R20));
	GM_set(M,2,1,R.e(gRotMat::R21));
	GM_set(M,2,2,R.e(gRotMat::R22));

	return 0;
}

//set M[r+i,c]=v[i], 0<= i < size
void GM_set_col(GM* M, int r, int c, int size, const gReal* v)
{
	for(int i=0;i<size;++i)
	{
		GM_set(M,r+i,c,v[i]);
	}
}


void GM_set_row(GM* M, int r, int c, int size, const gReal* v)
{
	for(int i=0;i<size;++i)
	{
		GM_set(M,r,c+i,v[i]);
	}
}

//set V[r+i]=v[i], 0 <= i < size
void GV_set(GV* V, int r, int size, const gReal* v)
{
	for(int i=0;i<size;++i)
	{
		GV_set(V,r+i,v[i]);
	}
}



// M = Ad_T
int make_matrix_Ad(GM* M, const gXMat& T)
{
	if(M->size1 != 6 || M->size2 != 6) return -1; // error	

	gVec3 p(T.trn());
	gVec3 Rx(T.e(gXMat::M00), T.e(gXMat::M10), T.e(gXMat::M20) );
	gVec3 Ry(T.e(gXMat::M01), T.e(gXMat::M11), T.e(gXMat::M21) );
	gVec3 Rz(T.e(gXMat::M02), T.e(gXMat::M12), T.e(gXMat::M22) );

	gVec3 px = p % Rx;
	gVec3 py = p % Ry;
	gVec3 pz = p % Rz;

	GM_set_zero(M);
	GM_set(M,0,0,Rx.e(0));	GM_set(M,1,0,Rx.e(1));	GM_set(M,2,0,Rx.e(2));
	GM_set(M,0,1,Ry.e(0));	GM_set(M,1,1,Ry.e(1));	GM_set(M,2,1,Ry.e(2));
	GM_set(M,0,2,Rz.e(0));	GM_set(M,1,2,Rz.e(1));	GM_set(M,2,2,Rz.e(2));

	GM_set(M,3,0,px.e(0));	GM_set(M,4,0,px.e(1));	GM_set(M,5,0,px.e(2));
	GM_set(M,3,1,py.e(0));	GM_set(M,4,1,py.e(1));	GM_set(M,5,1,py.e(2));
	GM_set(M,3,2,pz.e(0));	GM_set(M,4,2,pz.e(1));	GM_set(M,5,2,pz.e(2));

	GM_set(M,3,3,Rx.e(0));	GM_set(M,4,3,Rx.e(1));	GM_set(M,5,3,Rx.e(2));
	GM_set(M,3,4,Ry.e(0));	GM_set(M,4,4,Ry.e(1));	GM_set(M,5,4,Ry.e(2));
	GM_set(M,3,5,Rz.e(0));	GM_set(M,4,5,Rz.e(1));	GM_set(M,5,5,Rz.e(2));

	return 0;
}



int make_matrix_Adi(GM* M, const gXMat& T)
{
	gXMat Ti; 
	Ti.makeInverse(T);
	return make_matrix_Ad(M,Ti);
}



// M = dAd_T 
// it is really just a transpose of Ad
int make_matrix_dAd(GM* M, const gXMat& T)
{
	int re = make_matrix_Ad(M,T);
	if(re==0){
		GM_transpose(M);
		return 0;
	}
	return -1;
}

int make_matrix_dAdi(GM* M, const gXMat& T)
{
	int re = make_matrix_Adi(M,T);
	if(re==0){
		GM_transpose(M);
		return 0;
	}
	return -1;
}

int make_matrix_ad(GM* M, const gTwist& s)
{
	if(M->size1 != 6 || M->size2 != 6) return -1; // error	

	GM_set_zero(M);

	GM_set(M,0,1,-s.e(2));
	GM_set(M,0,2, s.e(1));
	GM_set(M,1,0, s.e(2));
	GM_set(M,1,2,-s.e(0));
	GM_set(M,2,0,-s.e(1));
	GM_set(M,2,1, s.e(0));	

	GM_set(M,3,1,-s.e(5));
	GM_set(M,3,2, s.e(4));
	GM_set(M,4,0, s.e(5));
	GM_set(M,4,2,-s.e(3));
	GM_set(M,5,0,-s.e(4));
	GM_set(M,5,1, s.e(3));	

	GM_set(M,3,4,-s.e(2));
	GM_set(M,3,5, s.e(1));
	GM_set(M,4,3, s.e(2));
	GM_set(M,4,5,-s.e(0));
	GM_set(M,5,3,-s.e(1));
	GM_set(M,5,4, s.e(0));	

	return 0;
}

int make_matrix_dad(GM* M, const gWrench& s)
{
	if(M->size1 != 6 || M->size2 != 6) return -1; // error	

	GM_set_zero(M);

	GM_set(M,1,0,-s.e(2));
	GM_set(M,2,0, s.e(1));
	GM_set(M,0,1, s.e(2));
	GM_set(M,2,1,-s.e(0));
	GM_set(M,0,2,-s.e(1));
	GM_set(M,1,2, s.e(0));	

	GM_set(M,1,3,-s.e(5));
	GM_set(M,2,3, s.e(4));
	GM_set(M,0,4, s.e(5));
	GM_set(M,2,4,-s.e(3));
	GM_set(M,0,5,-s.e(4));
	GM_set(M,1,5, s.e(3));	

	GM_set(M,4,3,-s.e(2));
	GM_set(M,5,3, s.e(1));
	GM_set(M,3,4, s.e(2));
	GM_set(M,5,4,-s.e(0));
	GM_set(M,3,5,-s.e(1));
	GM_set(M,4,5, s.e(0));	
	
	return 0;
}
////ws1 must be 6x6 matrix
////ws2 must be 6 vector
////ws3 must be gsl_eigen_symm_workspace of size 6
////return true if J is positive definite. false otherwise.
//bool check_positive_definiteness(gInertia J, GM* ws1, GV* ws2, gsl_eigen_symm_workspace* ws3)
//{
//	assert( ws1 != NULL && ws2 != NULL && ws3 != NULL );
//	assert( ws1->size1 == 6 && ws1->size2 == 6 && ws2->size == 6 && ws3->size == 6);
//	
//	//clear workspace
//	GM_set_zero(ws1);
//
//	//rotational inertia
//	GM_set(ws1,0,0,J.rotInertia(0));
//	GM_set(ws1,0,1,J.rotInertia(1)); GM_set(ws1,1,0,J.rotInertia(1));
//	GM_set(ws1,0,2,J.rotInertia(2)); GM_set(ws1,2,0,J.rotInertia(2));
//	GM_set(ws1,1,1,J.rotInertia(3)); 
//	GM_set(ws1,1,2,J.rotInertia(4)); GM_set(ws1,2,1,J.rotInertia(4));
//	GM_set(ws1,2,2,J.rotInertia(5));
//
//	//mass
//	GM_set(ws1,3,3,J.mass());
//	GM_set(ws1,4,4,J.mass());
//	GM_set(ws1,5,5,J.mass());
//
//	//upper-right & lower_left block
//	gReal c0 = J.mass()*J.com(0);
//	gReal c1 = J.mass()*J.com(1);
//	gReal c2 = J.mass()*J.com(2);
//
//	GM_set(ws1,0,3+1,-c2);
//	GM_set(ws1,0,3+2, c1);
//	GM_set(ws1,1,3+0, c2);
//	GM_set(ws1,1,3+2,-c0);
//	GM_set(ws1,2,3+0,-c1);
//	GM_set(ws1,2,3+1, c0);
//
//	GM_set(ws1,3+0,1, c2);
//	GM_set(ws1,3+0,2,-c1);
//	GM_set(ws1,3+1,0,-c2);
//	GM_set(ws1,3+1,2, c0);
//	GM_set(ws1,3+2,0, c1);
//	GM_set(ws1,3+2,1,-c0);
//
//	//eigen value
//	gsl_eigen_symm (ws1,ws2,ws3);
//
//	bool re = true;
//	for(int i=0; i<ws2->size; i++)
//	{
//		if(GV_get(ws2,i)<=0) re = false;
//	}
//
//	return re;
//}



bool computeInverseOfPDMatrixCholesky(GM* dst, GM* src)
{
	if( !dst || !src || dst->size1 != dst->size2 || dst->size1 != src->size1 ) return false;
	if( GSL_EDOM == gsl_linalg_cholesky_decomp(src) )	return false;

	GM_set_identity(dst);
	for(int i=0;i<dst->size1;++i)
	{
		gsl_linalg_cholesky_svx(src, &GM_column(dst,i).vector );
	}

	return true;
}

void GM_make_skew_symmetric_matrix(const gVec3& v, GM* M)
{	
	GM_set(M,0,0,0); GM_set(M,0,1,-v.z()); GM_set(M,0,2,v.y());
	GM_set(M,1,0,v.z()); GM_set(M,1,1,0); GM_set(M,1,2,-v.x());
	GM_set(M,2,0,-v.y()); GM_set(M,2,1,v.x()); GM_set(M,2,2,0);
}

bool solveLinAlgLU(GV* x, const GM* A, const GV* y)
{
	assert( x->size == A->size1 && A->size1 == A->size2 && A->size2 == y->size );

	int n = x->size;
	GM* A_ = GM_alloc(n,n);
	gsl_permutation* p = gsl_permutation_alloc(n);
	int signum;

	GM_memcpy(A_,A);	
	gsl_linalg_LU_decomp(A_,p,&signum);
	gReal det = gsl_linalg_LU_det(A_,signum);

	if(fabs(det)<1e-10){
		GM_free(A_);
		gsl_permutation_free(p);
		return false;
	}

	gsl_linalg_LU_solve(A_,p,y,x);
	GM_free(A_);
	gsl_permutation_free(p);
	return true;
}

#endif