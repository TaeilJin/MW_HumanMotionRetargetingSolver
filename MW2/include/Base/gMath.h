//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
//	Name:	gdk_math.h
//	History: 
//	- Latest update: 2011-10-08
//	- 2007/1/4: created
//
//
//	Comment:
//	- gRotMat set-functions don't perform sanity check. 
//	- type is defined in gTwist class for (very little) faster computation	
//  - gVec3, 
//		- "a+=b" is 3-5 times faster than "a=a+b", so is "-=".
//		- "a*=b" is a little faster than "a=a*b", so is "/=".
//	- gTwist,
//		- "a+=b" is 3-5 times faster than "a=a+b", so is "-=".
//		- "a*=b" is a little faster than "a=a*b".
//	- gXMat,
//		- "a*=b" is 2 times faster than "a=a*b".
//	- exp,
//		- exp(s, theta) is 20% faster than exp(s*theta) ( Be cautionus when using exp(s,theta)!! )
//		- Especially, if s.w coincides with x, y or z axis, and s.v is zero, 
//	 		then exp(s, theta) is over 2 times faster than exp(s*theta)
//
// -----------------------------------------------------------------

#ifndef _MBS_MATH_H_
#define _MBS_MATH_H_

#include "gBase.h"

#ifdef MW_USE_GINERTIA_TYPE_INFO
	#define		TYPE_gInertia_Zero_CoM	0x01	// 0000 0001
	#define		TYPE_gInertia_Diag_RI	0x02	// 0000 0010
#endif

#ifdef MW_USE_GINERTIA_TYPE_INFO
	#define		TYPE_gInertia_IS_ZERO_COM(x)			(x&TYPE_gInertia_Zero_CoM)
	#define		TYPE_gInertia_IS_DIAG_ROT_INERTIA(x)	(x&TYPE_gInertia_Diag_RI)
#endif

enum MW_UNIT{
		MW_UNDEFINED = 0,
		MW_METER = 1,
		MW_CENTIMETER = 2,
		MW_INCH = 3,
		MW_FOOT = 4
};

#define	CM2METER	0.01
#define INCH2METER	0.0254
#define FOOT2METER	0.3048
#define METER2CM	100
#define INCH2CM		2.54
#define FOOT2CM		30.48
#define METER2INCH	39.3701
#define CM2INCH		0.393701
#define FOOT2INCH	12
#define METER2FOOT	3.28084
#define CM2FOOT		0.0328084
#define INCH2FOOT	0.0833333

class	gVector;
class	gMatrix;
class	gVec3;
class	gRotMat;
class	gQuat;
class	gXMat;
class	gTwist;
class	gWrench;
class	gInertia;
class	gDInertia;
class	gDXMat;

bool	isRotMat(const gReal* d);	// check if d satisfies gRotMat property
//
///** gVector is a class for arbitrary dimensional vectors
//* 
//*/
//class gVector
//{
//public:
//	/// constructor
//	gVector() { _size=0; _e=NULL; _owner=true; }
//
//	/// create a vector of size n
//	gVector(int n)	{ assert(n>0); _size = n; _owner=true; _e = new gReal[n]; }
//	
//	/// create a vector of size n with inital values being val
//	gVector(int n,gReal val) { assert(n>0); _size = n; _owner=true; _e = new gReal[n]; for(int i=0;i<n;++i) _e[i]=val;}
//
//	/// create a vector of size n from an array ary. If bCopy is true, *this will own a copy of ary. Else, *this will just point to ary.
//	gVector(int n,gReal* ary,bool bCopy=false){ 
//		assert(n>0 && ary!=NULL); 
//		_size=n; 
//		if(bCopy){
//			_owner=true; 
//			_e = new gReal[n];	memcpy(_e,ary,n*sizeof(gReal)); 
//		}else{
//			_owner=false; 
//			_e=ary;
//		} 
//	}
//
//	/// copy constructor. Perform deep copy if src is owner of its array. Otherwise, it executes the shallow copy.
//	gVector(const gVector& src){ 
//		_size = src.size(); _owner = src._owner; 
//		if(_owner){
//			_e = new gReal[_size]; memcpy(_e,src._e,_size*sizeof(gReal)); 
//		}else{
//			_e = src._e;
//		}
//	}
//	
//	/// destructor
//	virtual ~gVector() { if(_owner) delete[] _e; }
//
//	/// return the pointer to the array
//	inline gReal* ptr() { return _e; }
//
//	/// return the pointer to idx-th element. idx must be smaller than the size of the vector.
//	inline gReal* ptr(int idx) { assert(idx<_size); return &_e[idx]; }
//
//	/// return the pointer as const
//	inline const gReal* cptr() const { return _e; }
//
//	/// return the pointer to idx-th element as const. idx must be smaller than the size of the vector.
//	inline const gReal* cptr(int idx) const { assert(idx<_size); return &_e[idx]; }
//
//	/// return the size (dimension) of the vector
//	inline	int size() const { return _size; }
//
//	/// return reference to the idx-th element
//	inline gReal& operator () (int idx) { assert(idx<_size); return _e[idx]; }
//	
//	/// return idx-th element
//	inline gReal  operator () (int idx) const { assert(idx<_size); return _e[idx]; }
//	
//	/// set all elements zero
//	void setZero() { memset(_e,0,_size*sizeof(gReal)); }
//
//	/// set all elements to d
//	void setAll(gReal d) { for(int i=0;i<_size;++i) _e[i]=d; }
//
//	/// set *this the idx-th basis vector. 
//	/** E.g., setBasis(0) makes (1,0,0,...), setBasis(1) makes (0,1,0,0,...)
//	*/
//	void setBasis(int idx) { assert(idx < _size); setZero(); _e[idx]=1; }
//
//	/// copy *this to dst. dst must be the same size as *this
//	void copyTo(gVector& dst) const { assert(dst.size() == _size); for(int i=0;i<_size;++i) dst._e[i] = _e[i]; }
//
//	/// copy n elements of *this starting from srcIdx to dst starting from dstIdx
//	/**  I.e., dst[dstIdx+i]=*this[srcIdx+i] for i=0...n-1
//	*/
//	void copyTo(gVector& dst, int n, int srcIdx, int dstIdx) const
//	{
//		assert(dstIdx+n <= dst.size() && srcIdx+n <=_size);
//		memcpy(dst.ptr(dstIdx),cptr(srcIdx),n*sizeof(gReal));//for(int i=0;i<n;++i) dst._e[dstIdx+i] = _e[srcIdx+i];
//	}
//
//	/// *this += v
//	void add(const gVector& v) {assert(v.size() == _size); for(int i=0;i<_size;++i) _e[i] += v._e[i]; }
//
//	/// add d to every element of the vector
//	void addConstant(gReal d) { for(int i=0;i<_size;++i) _e[i] += d; }
//
//	/// *this -= v
//	void sub(const gVector& v) {assert(v.size() == _size); for(int i=0;i<_size;++i) _e[i] -= v._e[i]; }
//
//	/// *this *= d
//	void scale(gReal d) { for(int i=0;i<_size;++i) _e[i] *= d; }
//
//	/// return the index of the minimum element
//	int minIndex(void) { 
//		int idx=0; gReal v=_e[0]; 
//		for(int i=1;i<_size;++i){ 
//			if( _e[i]<v ){ idx=i; v=_e[i]; } 
//		} 
//		return idx; 
//	}
//
//	/// return the index of the maximum element
//	int maxIndex(void) { 
//		int idx=0; gReal v=_e[0]; 
//		for(int i=1;i<_size;++i){ 
//			if( _e[i]>v ){ idx=i; v=_e[i]; } 
//		} 
//		return idx; 
//	}
//
//	/// return the magnitude squared of the vector
//	gReal magnitudeSquared(void) { gReal re=0; for(int i=0;i<_size;++i) re+=_e[i]*_e[i]; return re; }
//
//	/// return the magnitude of the vector
//	gReal magnitude(void) { return gSqrt(magnitudeSquared()); }
//
//	/// return dot product of *this and v. Sizes of *this and v must be same.
//	gReal dotProd(const gVector& v) { 
//		assert(v.size() == _size); 
//		gReal re=0; 
//		for(int i=0;i<_size;++i) re += v._e[i]*_e[i]; 
//		return re; 
//	}
//	
//private:
//	///pointer to the array
//	gReal* _e;		
//
//	///length of _e
//	int _size;		
//
//	///true if *this is the owner of the array _e
//	bool _owner;	
//
//private:
//	
//	///operator = not to be used publicly.
//	///perform deep copy if rhs is the owner of its array.
//	gVector& operator = (const gVector& rhs) 
//	{ 
//		if(this==&rhs) return *this; 
//		
//		if( _owner && _e ) delete[] _e;
//		
//		_owner = rhs._owner;
//		_size = rhs._size;
//
//		if(_owner){ //deep copy
//			_e = new gReal[_size]; memcpy(_e,rhs._e,sizeof(gReal)*_size);
//		}else{
//			_e = rhs._e;
//		}
//		
//		return *this;
//	}
//};
//
//
///// class gMatrix
///** General matrix class. \n
// * Elements are stored in an array in column-major order 
// * (to be compatible with FORTRAN-based linear algebra routines such as BLAS and LAPACK).
// * For matrix-matrix and matrix-vector multiplications, please use BLAS functions.
//*/
//class gMatrix
//{
//public:
//
//	/// default constructor
//	gMatrix() { _size1=_size2=0; _e=NULL; _owner=true;}
//
//	/// create a matrix of size nrow x ncol. Default values of the matrix not defined.
//	gMatrix(int nrow, int ncol)	{ assert(nrow>0 && ncol>0); _size1=nrow; _size2=ncol; _owner=true; _e = new gReal[_size1*_size2]; }
//	
//	/// create a matrix of size nrow x ncol with inital values being val
//	gMatrix(int nrow, int ncol, gReal val) { 
//		assert(nrow>0 && ncol>0); 
//		_size1=nrow; _size2=ncol; _owner=true;
//		_e = new gReal[_size1*_size2]; 
//		for(int k=0;k<_size1*_size2;++k) _e[k]=val;
//	}
//	
//	/// copy constructor 
//	gMatrix(const gMatrix& src){ 		
//		_size1=src._size1; _size2=src._size2; _owner=src._owner;
//		if(_owner){
//			_e = new gReal[_size1*_size2]; 
//			memcpy(_e,src._e,_size1*_size2*sizeof(gReal));
//		}else{
//			_e = src._e;
//		}
//	}
//
//	///create a matrix from an array ary. If copy is true, *this will own a copy of ary. Else, *this will just point to ary.
//	/**
//	* Note that ary must refer to a column-major order matrix.
//	* Beware that 2D array in C/C++ is row-major order. 
//	* If ary[3][2]={{0,1},{2,3},{4,5}} and you create gMatrix M(3,2,ary,true), M actually denote a matrix [0,3;1,4;2,5].
//	* So be careful when you use 2D array of C/C++.
//	*/
//	gMatrix(int nrow, int ncol, gReal* ary,bool copy=false){ 
//		assert(nrow>0 && ncol>0 && ary!=NULL); 
//		_size1=nrow; _size2=ncol;
//		if(copy){
//			_owner=true; 
//			_e = new gReal[_size1*_size2]; 
//			memcpy(_e,ary,_size1*_size2*sizeof(gReal));
//		}else{
//			_owner=false; 
//			_e=ary;
//		} 
//	}
//
//	/// destructor
//	virtual ~gMatrix() { if(_owner) delete[] _e; }
//
//	/// return the pointer of the element
//	inline gReal* ptr() { return _e; }
//
//	/// return the pointer to r-by-c element
//	inline gReal* ptr(int r, int c) { assert(r<_size1 && c<_size2); return &_e[_size1*c + r]; }
//
//	/// return the pointer of the element as const
//	inline const gReal* cptr() const { return _e; }
//
//	/// return the pointer to the c'th column
//	inline gReal* col(int c) { assert(c<_size2); return &_e[_size1*c]; }
//
//	/// return the number of rows	
//	inline	int sizeRow() const { return _size1; }
//
//	/// return the number of columns
//	inline	int sizeCol() const { return _size2; }
//
//	/// return the reference to r-by-c element
//	inline gReal& operator () (int r, int c) { assert(r<_size1 && c<_size2); return _e[c*_size1+r]; }
//
//	/// return the r-by-c element
//	inline gReal  operator () (int r, int c) const { assert(r<_size1 && c<_size2); return _e[c*_size1+r]; }
//	
//	/// set all elements zero
//	inline void setZero() { memset(_e,0,_size1*_size2*sizeof(gReal)); }
//
//	/// set all elements to d
//	inline void setAll(gReal d) { for(int k=0;k<_size1*_size2;++k) _e[k]=d; }
//
//	/// set c'th column to src. c={0,1,...,sizeColumn()-1}
//	inline void setCol(int c, gReal* src){ assert(c<_size2); memcpy(&_e[c*_size1],src,_size2*sizeof(gReal)); }
//
//	/// set r'th row to src. r={0,1,...sizeRow()-1}
//	void setRow(int r, gReal* src){ assert(r<_size1); for(int i=0;i<_size2;++i) _e[i*_size1+r]=src[i]; }
//
//	/// set diagonal elements one, all others zero. It still works for a non-square matrix.
//	void setIdentity() {
//		setZero(); 
//		int n = ( _size1 < _size2 ? _size1 : _size2 );
//		for(int k=0;k<n;++k) _e[k*(_size1+1)]=gOne;
//	}
//
//	/// copy to a matrix of same size
//	void copyTo(gMatrix& dst) const { 
//		assert(dst._size1 == _size1 && dst._size2 == _size2); 
//		memcpy(dst._e,_e,_size1*_size2*sizeof(gReal)); 
//	}
//
//	/// copy (nr by nc) submatrix of *this starting from (srcRow,srcCol) to dst starting from (dstRow,dstCol)
//	void copyTo(gMatrix& dst, int nr, int nc, int srcRow, int srcCol, int dstRow, int dstCol) const
//	{
//		assert(dstRow+nr <= dst.sizeRow() && dstCol+nc <= dst.sizeCol());
//		assert(srcRow+nr <= sizeRow() && srcCol+nc <= sizeCol());
//		for(int j=0;j<nc;++j) //col
//			for(int i=0;i<nr;++i){ //row
//				dst._e[ dst._size1*(j+dstCol) + i+dstRow ] = _e[ _size1*(j+srcCol) + i+srcRow ];
//			}
//	}
//
//	/// *this += M. Sizes of *this and M must be same.
//	void add(const gMatrix& M) {assert(M._size1 == _size1 && M._size2 == _size2); for(int k=0;k<_size1*_size2;++k) _e[k] += M._e[k]; }
//
//	/// add d to every element of the matrix
//	void addConstant(gReal d) { for(int k=0;k<_size1*_size2;++k) _e[k] += d; }
//
//	/// *this -= M. Sizes of *this and M must be same.
//	void sub(const gMatrix& M) {assert(M._size1 == _size1 && M._size2 == _size2); for(int k=0;k<_size1*_size2;++k) _e[k] -= M._e[k]; }
//
//	/// scale every element of the matrix by d
//	void scale(gReal d) { for(int k=0;k<_size1*_size2;++k) _e[k] *= d; }
//
//private:
//	gReal* _e;	//data array
//	int _size1;	//number of rows
//	int _size2; //number of columns
//	bool _owner;//true if *this is the owner of _e.
//
//private:
//
//	///operator = not to be used publicly.
//	///deep copy if rhs is owner.
//	gMatrix& operator = (const gMatrix& rhs) 
//	{
//		if(this==&rhs) return *this; 
//		
//		if( _owner && _e ) delete[] _e;
//		
//		_owner = rhs._owner;
//		_size1 = rhs._size1; _size2 = rhs._size2;
//
//		if(_owner){ //deep copy
//			_e = new gReal[_size1*_size2]; memcpy(_e,rhs._e,sizeof(gReal)*_size1*_size2);
//		}else{
//			_e = rhs._e;
//		}
//		
//		return *this;
//	}
//
//};

//------------------------------------------------------------------
/// class gVec3: 3 dimensional vector class
//------------------------------------------------------------------
class gVec3 
{
public:
	
	enum Index{X,	/**< enum value 0 */
		Y,			/**< enum value 1 */
		Z,			/**< enum value 2 */
		SIZE_INDEX	/**< enum value 3 */
	};

	/// construct a 3D vector of given initial values
	gVec3 (gReal x=0, gReal y=0, gReal z=0){ _e[0]=x; _e[1]=y; _e[2]=z;}

	/// construct a 3D vector of (v[0],v[1],v[2])
	gVec3 (const gReal *v){gCOPY3(_e,v);}

	/// construct a 3D vector of (v1-v2)
	gVec3 (const gVec3& v1, const gVec3& v2) { gELEMOP33(_e,=,v1._e,-,v2._e); } //this = v1-v2

	/// return the const pointer to the element array
	inline const gReal*	cptr(void) const { return (_e); }
	
	/// return the pointer to the element array
	inline gReal* ptr(void) { return _e; }

	/// return x-element
	inline gReal x(void) const { return _e[0]; }

	/// return y-element
	inline gReal y(void) const { return _e[1]; }

	/// return z-element
	inline gReal z(void) const { return _e[2]; }

	///returns i'th element i={0,1,2}
	inline gReal e(int i) const { return _e[i]; }

	///copy element array to array a. I.e., a[0]=this->x, a[1]=this->y, a[2]=this->z
	inline void	 copyTo(gReal* a) const { a[0] = _e[0]; a[1] = _e[1]; a[2] = _e[2]; }

	///set from a skew symmetric 3x3 matrix R. R must be in the column-major order.
	void		 setFromSkewSymMatrix(const gReal* R) { _e[0] = gREAL(0.5)*(R[5]-R[7]); _e[1] = gREAL(0.5)*(R[6]-R[2]); _e[2] = gREAL(0.5)*(R[1]-R[3]); }

	/// set *this to (v[0],v[1],v[2])
	inline void	 set(const gReal* v) { gCOPY3(_e,v); }

	/// set *this to (x,y,z)
	inline void	 set(gReal x,gReal y,gReal z) { _e[0]=x;_e[1]=y;_e[2]=z; }
	
	/// set x-element to x
	inline void	 setX(gReal x) { _e[0]=x; }

	/// set y-element to y
	inline void	 setY(gReal y) { _e[1]=y; }

	/// set z-element to z
	inline void	 setZ(gReal z) { _e[2]=z; }

	/// set i'th element to d. i={0,1,2}
	inline void	 set(int i, gReal d) { _e[i]=d; }
	
	/// make zero vector
	inline void	 setZero(void){ gZERO3(_e); }

	inline gVec3& operator  = (const gVec3 &v) { gCOPY3(_e,v._e); return (*this);}
	inline gVec3& operator += (const gVec3 &v) { gELEMOP3(_e,+=,v._e); return (*this);}
	inline gVec3& operator -= (const gVec3 &v) { gELEMOP3(_e,-=,v._e); return (*this);}
	inline gVec3& operator *= (gReal val) {gELEMOP1(_e,*=,val);  return (*this);}
	inline gVec3& operator /= (gReal val) {gELEMOP1(_e,*=,gInv(val)); return (*this);}
	inline gVec3 operator  + (const gVec3 &v) const { return gVec3(_e[0]+v._e[0],_e[1]+v._e[1],_e[2]+v._e[2]); }
	inline gVec3 operator  - (const gVec3 &v) const { return gVec3(_e[0]-v._e[0],_e[1]-v._e[1],_e[2]-v._e[2]); }
	inline gVec3 operator  - (void) const { return gVec3(-_e[0],-_e[1],-_e[2]); }
	inline gVec3 operator  * (gReal d) const { return gVec3(_e[0]*d,_e[1]*d,_e[2]*d); }
	friend gVec3 operator  * (gReal d, const gVec3& v) { return v*d; }
	friend gVec3 operator  * (const gVec3& v1, const gVec3& v2) { return gVec3(v1.x()*v2.x(), v1.y()*v2.y(), v1.z()*v2.z()); } //element-wise multiplication of vector
	
	/// return the dot product of *this and v
	inline gReal operator  , (const gVec3 &v) const { return gDOT(_e,v._e);}	// dot product
	
	/// return the cross product ( *this x v )
	inline gVec3 operator  % (const gVec3 &v) const { return gVec3(_e[1]*v._e[2]-_e[2]*v._e[1],_e[2]*v._e[0]-_e[0]*v._e[2] ,_e[0]*v._e[1]-_e[1]*v._e[0]);} // cross product
	
	friend std::istream & operator >> (std::istream &is, gVec3 &op);

	/// return the magnitude 
	inline gReal magnitude (void) const {return gSqrt(gDOT(_e,_e));}

	/// return the magnitde squared
	inline gReal magnitudeSquared(void) const {return gDOT(_e,_e); }

	/// scale each element by x,y,z respectively
	inline void  scale (gReal x, gReal y, gReal z) {_e[0]*=x; _e[1]*=y; _e[2]*=z;}
	
	/// scale each element by v[0], v[1], v[2], respectively
	inline void  scale (const gVec3 v) {gELEMOP3(_e,*=,v._e);}

	/// scale every element by s
	inline void	 scale (gReal s) { _e[0]*=s; _e[1]*=s; _e[2]*=s; }

	/// normalize the vector
	inline void	 normalize (void) { gReal x = magnitude(); if ( x > gEPSILON ) (*this) *= gInv(x); }

	/// return log (exponential coordinates) of a rotation matrix R
	static gVec3 log (const gRotMat& R);

	/// return log (exponential coordinates) of a rotation matrix R that is close to nbor
	static gVec3 log (const gRotMat& R, const gVec3 nbor);

private:

	gReal _e[3];
};


/**
* class gRotMat : rotation matrix of a rigid body  \n
*		R =											\n
*		| e0 e3 e6 |		\n
*		| e1 e4 e7 |		\n
*		| e2 e5 e8 |		\n
*\n
*	Properties of gRotMat: 1) det(R)=1, and 2) R'*R=Eye	\n
*	Matrix is in the column-major order \n
*\n
*/
class gRotMat { 
public:	

	enum Index{R00,R10,R20, R01,R11,R21, R02,R12,R22, SIZE_INDEX };

	/// default constructor. creates the identity matrix by default.
	gRotMat (void);
						
	/// construct from d[9] in column-wise order. No sanity check on d peformed. See rectify().
	gRotMat	(const gReal* d);

	/// construct from d0...d8. No sanity check performed. See rectify()
	gRotMat	(gReal d0,gReal d1,gReal d2,gReal d3,gReal d4,gReal d5,gReal d6,gReal d7,gReal d8); 

	/// construct from Rx,Ry,Rz, which are three column vectors of *this. No sanity check performed. See rectify()
	gRotMat (const gVec3& Rx, const gVec3& Ry, const gVec3& Rz); 
	
	/// return const pointer to the matrix
	inline	const gReal*cptr	(void)  const { return _e; }

	/// return the pointer to the i'th column
	inline	const gReal*col	(int i) const { return (_e+3*i); }	

	/// return the i'th element. i=[0:8]
	inline 	gReal e (int i)	const { return _e[i]; }		
			
	/// return (R00,R10,R20)
	inline	gVec3 x(void) const { return gVec3(_e[0],_e[1],_e[2]); }

	/// return (R01,R11,R21)
	inline	gVec3 y(void) const { return gVec3(_e[3],_e[4],_e[5]); }

	/// return (R02,R12,R22)
	inline	gVec3 z(void) const { return gVec3(_e[6],_e[7],_e[8]); }

	/// set from d[0...8]
	inline 	void  set (const gReal* d) { memcpy(_e,d,9*sizeof(gReal)); }

	/// set from r0...r8
	inline	void  set (gReal r0, gReal r1, gReal r2, gReal r3, gReal r4, gReal r5, gReal r6, gReal r7, gReal r8) { _e[0]=r0; _e[1]=r1; _e[2]=r2; _e[3]=r3; _e[4]=r4; _e[5]=r5; _e[6]=r6; _e[7]=r7; _e[8]=r8; }
	
	/// set from d[0...8]. Check if d is a rotation matrix in debug mode.
	inline	void  setSafe	(const gReal* d) { assert( isRotMat(d) ); set(d); }

	/// set from a rotation matrix
	inline	void  set	(const gRotMat& R) { set(R.cptr()); }
	
	/// set to the Identity matrix
			void  setIdentity	(void);	

	/// set i'th column to v
	inline	void  setColumn (int i, const gVec3& v) { gCOPY3(_e+i*3, v.cptr()); }	
			
	/// this = exp(v)
	void  makeExp	(const gVec3& v);			
	/// this = exp(unitAxis*angle)
	void  makeExp	(const gVec3& unitAxis, gReal angle);
	/// this = exp( gVec3(x,y,z) )
	void  makeExp	(const double x, const double y, const double z);
	/// this = exp(axis*angle). It is same as makeExp(axis,angle)
	void  makeRotate	(const gVec3& axis, gReal angle); 
	
	/// this = exp(gVec3UnitX*x). It makes a rotation matrix that rotates about X by an amount of x rad.
	void  makeRotateX	(gReal x);				

	/// this = exp(gVec3UnitY*y). It makes a rotation matrix that rotates about Y by an amount y rad
	void  makeRotateY	(gReal y);				

	/// this = exp(gVec3UnitZ*z). It makes a rotation matrix that rotates about Z by an amount z rad
	void  makeRotateZ	(gReal z);				

	/// make rotation matrix from Euler angles XYZ. 
	void  makeRotateXYZ (gReal x, gReal y, gReal z); 

	/// make rotation matrix from Euler angles ZYX
	void  makeRotateZYX (gReal z, gReal y, gReal x); 

	/// make rotation matrix from Euler angles ZXY
	void  makeRotateZXY(gReal z, gReal x, gReal y);


	// Modify: 2014-05-12: SukwonLee 
	/// make rotation matrix from Euler angles XZY. 
	void  makeRotateXZY (gReal x, gReal z, gReal y);

	/// make rotation matrix from Euler angles XZY. 
	void  makeRotateYXZ (gReal y, gReal x, gReal z);

	/// make rotation matrix from Euler angles XZY. 
	void  makeRotateYZX (gReal y, gReal z, gReal x);



	/// return a quaternion corresponding to *this
	gQuat  inQuat (void) const;

	/// return the Frobenius norm
	inline	gReal  fnorm (void) const { return (gDOT(_e,_e)+gDOT(_e+3,_e+3)+gDOT(_e+6,_e+6)); }

	/// return the trace (sum of diagonal elements)
	inline	gReal  trace (void) const { return ( _e[0] + _e[4] + _e[8] );}

	/// this = R
   	gRotMat& operator =	(const gRotMat& R);			

	/// this = this*R
	gRotMat& operator *=	(const gRotMat& R);			
	       	
	/// return this*R	
	gRotMat	 operator *	(const gRotMat& R) const;	

	/// return transpose (= inverse)
	gRotMat	 operator ~	(void) const;

	/// return this*v
	gVec3	 operator *	(const gVec3& v) const;

	/// return (inverse of this)*v
	gVec3 invMult	(const gVec3& v) const;	
	
	/// return (inverse of this)*R
	gRotMat	invMult	(const gRotMat& R) const;

	///re-orthogonalize. i=(0,1,2) is the index of basis axis, whose direction doesn't change by rectification.
	void	rectify		(int i);

	/// return exponential of s
	static	gRotMat	exp (const gVec3& s) { gRotMat r; r.makeExp(s); return r; }

	/// return true of e is a rotation matrix
	static	bool isRotMat(const gReal* a);	

	/// return true of the rotation part of T is the rotation matrix
	static	bool isRotMat(const gXMat& T);	

private:

	gReal _e[9];

	inline gReal*		_col		(int i){ return _e+3*i; }
};


/**
* class gQuat : quaternion class  \n
*		Q =	(e0,e1,e2,e3)=(x,y,z,w)	\n
* Relation with angle-axis: w=cos(angle/2), (x,y,z)=sin(angle/2)*axis \n
*\n
*/
class gQuat{
public:
	enum Index{X,Y,Z,W,SIZE_INDEX};

	/// construct a default quaternion (0,0,0,1)
	gQuat(){ _e[0]=_e[1]=_e[2]=0; _e[3]=1; }

	/// construct a quaternion (x,y,z,w)
	gQuat(gReal x, gReal y, gReal z, gReal w) { _e[0]=x; _e[1]=y; _e[2]=z; _e[3]=w; }

	/// construct a quaternion (v[0],v[1],v[2],v[3])
	gQuat(const gReal* v) { _e[0]=v[0]; _e[1]=v[1]; _e[2]=v[2]; _e[3]=v[3]; }

	/// return x-element
	inline	gReal	x(void) const { return _e[0]; }
	
	/// return y-element
	inline	gReal	y(void) const { return _e[1]; }
	
	/// return z-element
	inline	gReal	z(void) const { return _e[2]; }
	
	/// return w-element
	inline	gReal	w(void) const { return _e[3]; }
	
	/// return i-th element (i=[0..3])
	inline	gReal	e(int i) const { return _e[i]; }

	/// return const pointer to the array
	inline const	gReal *cptr(void) const { return (_e); }

	/// set X-element to x
	inline void		setX(gReal x)	{ _e[0] = x; }

	/// set Y-element to y
	inline void		setY(gReal y)	{ _e[1] = y; }

	/// set Z-element to z
	inline void		setZ(gReal z)	{ _e[2] = z; }

	/// set W-element to w
	inline void		setW(gReal w)	{ _e[3] = w; }

	/// set idx-th element to v. idx=[0..3]
	inline void		set(int idx, gReal v) { _e[idx] = v; }

	/// set this = (x,y,z,w)
	inline void		set(gReal x,gReal y,gReal z, gReal w) { _e[0]=x; _e[1]=y; _e[2]=z; _e[3]=w; }
	
	/// set to identity (= (0,0,0,1))
	inline void		setIdentity(void) { _e[0]=_e[1]=_e[2]=0; _e[3]=1; }

	/// make quaternion from exponential coordinates(x,y,z)
	void			makeRotate(gReal x, gReal y, gReal z);
		
	/// make quaternion from exp( axisAngle )
	inline	void	makeRotate(gVec3 axisAngle){	makeRotate(axisAngle.x(), axisAngle.y(), axisAngle.z()) ;	}

	/// make quaternion that rotates vector v1 to vector v2
	void			makeRotate(const gVec3& v1, const gVec3& v2);

	///get axis (x,y,z) and angle from the quaternion
	void			getAxisAngle( gReal& x, gReal& y, gReal& z, gReal& angle ) const;
	
	///get axis and angle from the quaternion
	void			getAxisAngle( gVec3& axis, gReal& angle) const;
	
	///return rotation matrix corresponding to this
	gRotMat			inRotMatrix(void) const;

	/// return the conjugate
	inline	gQuat	conjugate(void) { return gQuat(-_e[0],-_e[1],-_e[2],_e[3]); }	

	/// normalize this
	void			normalize	(void);

	/// return the mangitude
	inline gReal	magnitude(void) { return gSqrt(_e[0]*_e[0]+_e[1]*_e[1]+_e[2]*_e[2]+_e[3]*_e[3]); }
	
	/// this = v
	inline gQuat&	operator  = (const gQuat &v) { _e[0]=v._e[0]; _e[1]=v._e[1]; _e[2]=v._e[2]; _e[3]=v._e[3]; return (*this);}
	
	/// return this * P
	gQuat			operator * (const gQuat P) const;

	/// this = this * P
	inline	gQuat&	operator *= (gQuat P) {	(*this) = (*this)*P; }

	/// return this * v
	gVec3			operator* (const gVec3& v) const; 
	
private:

	gReal _e[4];	//(x,y,z,w)
};


///class gXMat: rigid body transformation matrix (R: rotation matrix, p:translation)
/** \n
* T = \n
* | e0 e4 e8  e12 |	\n
* | e1 e5 e9  e13 | \n
* | e2 e6 e10 e14 |	\n
* | e3 e7 e11 e15 | \n
* = \n
* | R00 R01 R02 p0| \n
* | R10 R11 R12 p1| \n
* | R20 R21 R22 p2| \n
* | 0 0 0 1 |		\n
* , where R is rotation matix (gRotMat), p is translation gVec3 \n
*\n
*/
class gXMat{ 
public:	
	enum Index {M00,M10,M20,M30, M01,M11,M21,M31, M02,M12,M22,M32, M03,M13,M23,M33, SIZE_INDEX };

	/// pointer wrapper / ADD: SWL 2016-07-11
	gXMat			(gReal* p, bool copy_mem=true);
	gXMat			(const gReal* p);
	gXMat			(const gXMat&);

	/// default constructor. Default values are R = I, p = 0
	gXMat			(void);

	/// construct from a rotation matrix R and position v
	gXMat			(const gRotMat& R, const gVec3& v);

	/// construct from a rotation matrix R. Position is set to 0
	gXMat			(const gRotMat& R);						

	/// construct gXMat such that rotation matrix is identity and position is v
	gXMat			(const gVec3& v);						

	/// construct gXMat such that rotation matrix is identity and position is (x,y,z)
	gXMat			(gReal x, gReal y, gReal z);			

	/// construct gXMat such that rotation matrix is (r0...r8) and position is (p0,p1,p2)
	gXMat			(gReal r0, gReal r1, gReal r2, gReal r3, gReal r4, gReal r5, gReal r6, gReal r7, gReal r8, gReal p0, gReal p1, gReal p2);

	///return rotation part
	gRotMat		rot			(void) const { return gRotMat(_e[0],_e[1],_e[2],_e[4],_e[5],_e[6],_e[8],_e[9],_e[10]); }
			
	///return rotation part as quaternion
	gQuat		rotInQuat	(void) const;
			
	///return x-components of rotation matrix. i.e.,(R00,R10,R20)
	gVec3		rotX		(void) const { return gVec3(_e[0],_e[1],_e[2]); }
			
	///return y-components of rotation matrix. i.e.,(R01,R11,R21)
	gVec3		rotY		(void) const { return gVec3(_e[4],_e[5],_e[6]); }
			
	///return z-components of rotation matrix. i.e.,(R02,R12,R22)
	gVec3		rotZ		(void) const { return gVec3(_e[8],_e[9],_e[10]); }
			
	///return translation(position) part
	inline 	gVec3		trn			(void) const { return gVec3(_e[12], _e[13], _e[14]); }

	///return [i,j]th element. i,j=[0..3]
	inline	gReal		e			(int i,int j) const {return _e[i+4*j];}

	///return i-th element. i=[0..15]
	inline	gReal		e			(int i)	const { return _e[i];}
	
	///return const pointer of the element
	inline const gReal*	cptr		(void) const { return _e; }	

	///return pointer to the element
	inline	gReal*		ptr			(void) { return _e; }		

	///return const pointer to rotation part. In fact, it is same as ptr()
	inline const gReal*	ptrRot		(void) const { return _e; }		

	///return const pointer to translation part (i.e.,_e[12])
	inline const gReal* ptrTrn		(void) const { return _e+12; }

	///return const pointer to i_th column. i=[0..3]
	inline const gReal* ptrColumn	(int i) const { return _e+4*i; }	

	///return the trace (sum of diagonal elements)
	inline	gReal		trace		(void) const { return (_e[0]+_e[5]+_e[10]+1); }

	///set from d[0...15]. No sanity check performed.
	inline	void		set			(const gReal* d) { memcpy(_e,d,16*sizeof(gReal)); }

	/// set translation to v
	inline 	void		setTrn		(const gVec3& v) { _e[12]=v.x(); _e[13]=v.y(); _e[14]=v.z(); } 

	/// set translation to (x,y,z)
	inline	void		setTrn		(gReal x, gReal y, gReal z){ _e[12]=x; _e[13]=y; _e[14]=z; }	

	/// set rotation matrix to R
			void		setRot		(const gRotMat& R);						

	/// set to identity matrix
	       	void		setIdentity	(void);									

	/// this = T
	//inline	void		set			(const gXMat& T);
			void		set			(const gXMat& T);

	/// set i-th element to d
	//inline	void		set			(int i, gReal d);
			void		set			(int i, gReal d);

	/// copy translation from T's translation
	inline	void		copyTrn	(const gXMat& T) { _e[12]=T._e[12]; _e[13]=T._e[13]; _e[14]=T._e[14]; }

	/// copy rotation from T's rotation
	inline	void		copyRot	(const gXMat& T) { memcpy(_e,T._e,sizeof(gReal)*12); }

	/// this = inverse(T)
	void		makeInverse	(const gXMat& T);

	/// this = exp(s)
	void		makeExp		(const gTwist& s);

	/// this = exp(s*angle). s.rot() must be normal OR zero vector. this is FASTER than function exp(const gTwist& s).
	void		makeExp		(const gTwist& s, gReal angle);
	
	/// make rotation matrix as makeRotateX(x) with zero translation
	void		makeRotateX	(gReal x);				

	/// make rotation matrix as makeRotateY(y) with zero translation
	void		makeRotateY	(gReal y);				

	/// make rotation matrix as makeRotateZ(z) with zero translation
	void		makeRotateZ	(gReal z);				
			
	/// traslate by v wrt the reference frame		
	inline	void		translateRef(const gVec3& v) { _e[12] += v.x(); _e[13] += v.y(); _e[14] += v.z(); } 

	/// traslate by (x,y,z) wrt the reference frame
	inline	void		translateRef(gReal x, gReal y, gReal z) { _e[12] += x; _e[13] += y; _e[14] += z; }

	/// traslate by v wrt the local (body) frame
	void		translateLocal(const gVec3& v);

	/// tranlate by (x,y,z) wrt the local (body) frame
	void		translateLocal(gReal x, gReal y, gReal z);
			
	/** rotate about axis by angle wrt the reference frame.
		* Axis is defined by its direction (axisDir) and any point on that axis (axisPos), both expressed wrt reference frame.
		* The amount of rotation is angle*||axisDir||
	*/
	void		rotateRef(const gVec3 axisDir, const gVec3 axisPos, gReal angle);
						
	/// rotate by R around pivot. Both R and pivot are expressed wrt the reference frame.
	void		rotateRef(const gRotMat& R, const gVec3& pivot);


	///rotate by R about the frame origin. R is wrt reference frame.
	void		rotateRef(const gRotMat& R); 
			
	/// same as rotateRef, except that axisDir and axisPos are expressed wrt local frame
	void		rotateLocal(const gVec3 axisDir, const gVec3 axisPos, gReal angle);

	/// rotate by R around pivot. Both R and pivot are expressed wrt local frame.
	void		rotateLocal(const gRotMat& R, const gVec3& pivot);		
			
	/// rotate by R about the origin. R is expressed wrt local frame
	void		rotateLocal(const gRotMat& R); 

	/// return exp(s)
	static	gXMat		exp (const gTwist& s) { gXMat r; r.makeExp(s); return r; }

			/** return exp(untiAxis*angle)
			  * unitAngle MUST be unitary. See exp(const gTwist&, gReal)
			  */
	static	gXMat		exp (const gTwist& unitAxis, gReal angle){ gXMat r; r.makeExp(unitAxis,angle); return r; } 

			/// this = T
			gXMat&		operator =	(const gXMat& T);			
			
			/// this = this*T
			gXMat&		operator *=	(const gXMat& T);			
	       	
			/// return this*T
			gXMat		operator *	(const gXMat& T) const;		
			
			/// return inverse
			gXMat		operator ~	(void) const;				

			/// this = T*this			
			void		makePreMult	(const gXMat& T);

			/// return inv(this)*T
			gXMat		invMult		(const gXMat& T) const;		
	       							
			/// (this)*[v;0] = R*v
			gVec3		multVec3	(const gVec3& v) const;		

			/// (this)*[e[0];e[1];e[2];0]
	       	gVec3		multVec3	(const gReal* e) const;		

			/// inv(this)*[v;0] = R'*v
	       	gVec3		invMultVec3 (const gVec3& v) const;		

			/// inv(this)*[e[0];e[1];e[2];0]
			gVec3		invMultVec3 (const gReal* e) const;		
									
			/// (this)*[v;1] = R*v+p
			gVec3		multVec4	(const gVec3& v) const;		

			/// (this)*[e;1] = R*e+p
			gVec3		multVec4	(const gReal* e) const;		

			/// (this*)*[e(0)...e(3)] : general multiplication of 4x4 matrix with 4 vector
			gVec3		multVec4gen	(const gReal *e) const;

			/// inv(this)*[v;1] = R'*(v-p)
			gVec3		invMultVec4 (const gVec3& v) const;		

			///return [R*r,p] (i.e., multiply only the rotation matrix)
			gXMat		multRotMat	(const gRotMat& r) const;	

			///re-orthogonalize rotation part. i=(0,1,2) is the index of basis axis, whose direction doesn't change by rectification.
			void		rectify		(int i);

	friend	std::istream& operator>> ( std::istream& is, gXMat& s );

private:

	//gReal _e[16];
	gReal* _e;
	gReal _ei[16];

	inline gReal*		_rot		(void){ return _e; }
	inline gReal*		_trn		(void){ return _e+12; }
	inline gReal*		_col		(int i){ return _e+4*i; }

};

///class gTwist: se(3) group (w: rotation part, v: translation part)
/** \n
* A twist s is represented as [ e0 e1 e2 e3 e4 e5 ]' = [ w' v' ]'.
*/
class gTwist {
public:		
	enum Index { Rx, Ry, Rz, Tx, Ty, Tz, SIZE_INDEX };

						/// default is zero gVec3					
						gTwist			(void);	

						/// construct a twist (w1,w2,w3,v1,v2,v3)
						gTwist			(gReal w1,gReal w2, gReal w3, gReal v1, gReal v2, gReal v3) ;

						/// construct a twist (w,v)
						gTwist			(const gVec3& w, const gVec3& v);

						/// construct a twist (d[0]...d[5])
						gTwist			(const gReal* d);

	/// return const pointer to the array
	inline const gReal*	cptr		(void) const { return _e; }	

	/// return pointer to the array
	inline gReal*		ptr			(void)		 { return _e; }	

	/// return const pointer to rotation part. It is same as cptr()
	inline const gReal*	ptrRot		(void) const { return _e; }	

	/// return const pointer to translation part.
	inline const gReal* ptrTrn		(void) const { return _e+3; }

	/// return rotation part
	inline  gVec3		rot			(void) const { return gVec3(_e[0],_e[1],_e[2]); }

	/// return translation part
	inline  gVec3		trn			(void) const { return gVec3(_e[3],_e[4],_e[5]); }

	/// return i-th element. i=[0..5]
	inline	gReal		e			(int i) const { return _e[i]; }			

	/// set to (w1,w2,w3,v1,v2,v3)
	inline 	void		set			(gReal w1, gReal w2, gReal w3, gReal v1, gReal v2, gReal v3){_e[0] = w1;_e[1] = w2;_e[2] = w3; _e[3] = v1;_e[4] = v2;_e[5] = v3;}
	
	/// set to (w,v)
	inline	void		set			(const gVec3& w, const gVec3& v){ gCOPY3(_rot(),w.cptr()); gCOPY3(_trn(),v.cptr());}

	/// set to s
	inline	void		set			(const gTwist& s)			{ memcpy(_e,s._e,6*sizeof(gReal));}

	/// set to d[0..5]
	inline	void		set			(const gReal* d)			{ memcpy(_e,d,6*sizeof(gReal)); }

	/// set idx's element to d
	inline	void		set			(int idx,gReal d)			{ _e[idx] = d; }

	/// set to zero
	inline 	void		setZero	(void)							{ _e[5] = _e[4] = _e[3] = _e[2] = _e[1] = _e[0] = 0; }

	/// set translation part to (d[0],d[1],d[2])
	inline	void		setTrn		(const gReal* d)			{ _e[3]=d[0]; _e[4]=d[1]; _e[5]=d[2]; }

	/// set translation part to v
	inline	void		setTrn		(const gVec3& v)			{ _e[3]=v.x(); _e[4]=v.y(); _e[5]=v.z(); }

	/// set translation part to (x,y,z)
	inline	void		setTrn		(gReal x,gReal y,gReal z)	{ _e[3]=x; _e[4]=y;	_e[5]=z;}

	/// set rotation part to (d[0],d[1],d[2])
	inline	void		setRot		(const gReal* d)			{ _e[0]=d[0]; _e[1]=d[1]; _e[2]=d[2]; }

	/// set rotation part to v
	inline	void		setRot		(const gVec3& v)			{ _e[0]=v.x(); _e[1]=v.y(); _e[2]=v.z(); }

	/// set rotation part to (x,y,z)
	inline	void		setRot		(gReal x,gReal y,gReal z)	{ _e[0]=x; _e[1]=y; _e[2]=z;}

	/// set to i-th basis vector. i=[0..5]
	inline	void		setBasis	(int i)						{ memset(_e,0,6*sizeof(gReal)); _e[i]=1; }

	/// copy this to a[0..5]
			void		copyTo		(gReal* a)					{ memcpy(a,_e,6*sizeof(gReal)); }
	
			/// this = s
			gTwist&		operator =	(const gTwist& s);			

			/// this = this + s
	       	gTwist&		operator +=	(const gTwist& s);			

			/// this = this - s
	       	gTwist&		operator -=	(const gTwist& s);			
	       	
			/// this = this * d	
			gTwist&		operator *=	(gReal d);					

			/// return this + s	
	       	gTwist		operator +	(const gTwist& s) const;	

			/// return this - s
	       	gTwist		operator -	(const gTwist& s) const;	
		
			/// return this*(-1)
	inline  gTwist		operator -	(void) const				{ return gTwist(-_e[0],-_e[1],-_e[2],-_e[3],-_e[4],-_e[5]); }

			/// return this * d	
	       	gTwist		operator *	(gReal d) const;			

			/// return d * this
	friend  gTwist		operator *  (gReal d, const gTwist& s) { return s*d; }

			/// return this*p (i.e., cross(w,p) + v )
	       	gVec3		multVec4	(const gVec3& p) const;		

			/// return cross(w,p)
			gVec3		multVec3	(const gVec3& p) const;		

			/// return dot(this,w)
			gReal		operator ,	(const gWrench& w) const;	

	friend std::istream& operator >> (std::istream& is, gTwist& s);

			/// this = Ad(T,s)
			void		 makeAd		(const gXMat& T, const gTwist& s);		
			
			/// this = ad(s1,s2)
			void		 makeLieBracket	(const gTwist& s1, const gTwist& s2);	

			/// this = log(T)
			void		 makeLog	(const gXMat& T);						

			/// this = Ad(T^-1,s)
			void		 makeAdInv	(const gXMat& T, const gTwist& s);		

			/// return coordinate transformation of this. T is the configuration of the current frame wrt the new frame. Equivalent to Ad(T,this). 
			gTwist		 xform		(const gXMat& T) const;			
			
			///same as xform (const gXMat& T) with the rotation matrix of T is identity.
			gTwist		 xform		(const gVec3& p) const; 
			
			/// return coordinate transformation of this. T is the configuration of the new frame wrt the current frame. Equivalent to InvAd(T,this)
			gTwist		 xformInv	(const gXMat& T);						

			/// this = inv(J)*y
			void		 solve		(gInertia& J, const gWrench& y);		

			/// return the magnitude of this
			gReal		 magnitude	(void) const { return sqrt( gSqr(_e[0])+gSqr(_e[1])+gSqr(_e[2])+gSqr(_e[3])+gSqr(_e[4])+gSqr(_e[5]) ); }
			
			/// normalize
			void		 normalize	(void) { gReal x = magnitude(); if ( x > gEPSILON ) (*this) *= gInv(x); }

			/// return log(T) close to nbor
	static	gTwist		log			(const gXMat& T, const gTwist& nbor);

			/// return log(T). log is computed iteratively.
	static	gTwist		logIterative(const gXMat& T, const gTwist& initial, gReal conv=1e-4);

			/// return ad(s1,s2)
	static	gTwist		ad			(const gTwist& s1, const gTwist& s2)	{ gTwist r; r.makeLieBracket(s1,s2); return r; }

			/// return Ad(T,s)
	static	gTwist		Ad			(const gXMat& T, const gTwist& s)		{ gTwist r; r.makeAd(T,s); return r; }

			/// return Ad( inv(T), s )
	static	gTwist		Adi			(const gXMat& T, const gTwist& s)		{ gTwist r; r.makeAdInv(T,s); return r; }

private:
	
	inline gReal*		_rot		(void){ return _e; }
	inline gReal*		_trn		(void){ return _e+3; }

	gReal				_e[6];

};


///class gWrench: wrench group (m: rotation (or torque) part, f: translation (or force) part)
/** \n
* A wrench w is represented as [ e0 e1 e2 e3 e4 e5 ]' = [ m' f' ]'.
*/
class gWrench {
public:	
	enum Index { Rx, Ry, Rz, Tx, Ty, Tz, SIZE_INDEX };

						/// default constructor. this is set to 0 by default.
						gWrench			(void);		

						/// this = (m1,m2,m3,f1,f2,f3)
						gWrench			(gReal m1,gReal m2,gReal m3,gReal f1,gReal f2,gReal f3);

						/// this = (m,f)
						gWrench			(const gVec3& m, const gVec3& f);

						/// this = s
						gWrench			(const gTwist& s);

						/// this = (m[0..2],f[0..2])
						gWrench			(gReal* m, gReal* f){ _e[0]=m[0],_e[1]=m[1],_e[2]=m[2],_e[3]=f[0],_e[4]=f[1],_e[5]=f[2]; }

	/// return pointer to the array
	inline gReal*		ptr				(void)		    { return _e; }	

	/// return const pointer to element
	inline const gReal*	cptr			(void)	const	{ return _e; }

	/// return const pointer to rotation part. It is same as cptr().
	inline const gReal*	ptrRot			(void)	const	{ return _e; }

	/// return const pointer to translation part.
	inline const gReal*	ptrTrn			(void)	const	{ return _e+3; }
    
	/// return rotation part
	inline 	gVec3		rot				(void)	const	{ return gVec3(_e[0],_e[1],_e[2]); }

	/// return translation part
	inline 	gVec3		trn				(void)	const	{ return gVec3(_e[3],_e[4],_e[5]); }

	/// return i'th index element. i=[0..5]
	inline	gReal		e				(int i) const	{ return _e[i]; }

	/// set idx's element to d
	inline	void		set				(int idx,gReal d){ _e[idx] = d; }

	/// set to [m1,m2,m3,f1,f2,f3]
	inline 	void		set				(gReal m1, gReal m2, gReal m3, gReal f1, gReal f2, gReal f3)
										{ _e[0] = m1;_e[1] = m2;_e[2] = m3; _e[3] = f1;_e[4] = f2;_e[5] = f3; }

	/// set to [m,f]
	inline	void		set				(const gVec3& m, const gVec3& f) { gCOPY3(_rot(),m.cptr()); gCOPY3(_trn(),f.cptr()); }
	
	/// set to w
	inline	void		set				(const gWrench& w){ memcpy(_e,w._e,6*sizeof(gReal)); }

	/// set to d[0..5]
	inline	void		set				(const gReal* d){ memcpy(_e,d,6*sizeof(gReal)); } 

	/// set to zero
	inline 	void		setZero			(void)			{ memset(_e,0,6*sizeof(gReal)); }

	/// set to i-th basis vector. i=[0..5]
	inline	void		setBasis		(int i)			{ memset(_e,0,6*sizeof(gReal)); _e[i]=1; }

	/// copy this to a[0..5]
			void		copyTo			(gReal* a)		{ memcpy(a,_e,6*sizeof(gReal)); }

			/// this = wr
	       	gWrench&	operator =		(const gWrench& wr);	

			/// this = this + wr
			gWrench&	operator +=		(const gWrench& wr);		
	       	
			/// this = this - wr
			gWrench&	operator -=		(const gWrench& wr);		
	       	
			/// this = this * d
			gWrench&	operator *=		(gReal d);				
	       	
			/// return this+wr	
			gWrench		operator +		(const gWrench& wr)	const;		
	       	
			/// return this-wr
			gWrench		operator -		(const gWrench& wr)	const;		

			/// return this*(-1)
	inline  gWrench		operator -		(void)				const		{ return gWrench(-_e[0],-_e[1],-_e[2],-_e[3],-_e[4],-_e[5]); }

			/// return this*d	
	       	gWrench		operator *		(gReal d)			const;		

			/// return d*this
	friend  gWrench		operator *		(gReal d, const gWrench& wr) { return wr*d; }

			/// return this*p ( [m]p + f )
	       	gVec3		operator *		(const gVec3& p)	const;		

			/// return dot product (this , s)
	       	gReal		operator ,		(const gTwist& s) const;			

			/// this = dAd(T,wr)
			void		makeDAd			(const gXMat& T, const gWrench& wr);	

			/// this = dAd(T^-1,wr)
			void		makeDAdInv		(const gXMat& T, const gWrench& wr);	

			/// this = dad(s,wr)
			void		makeDLieBracket(const gTwist& s, const gWrench& wr);	

			/// return the magnitude
			gReal		magnitude		(void) const { return sqrt( gSqr(_e[0])+gSqr(_e[1])+gSqr(_e[2])+gSqr(_e[3])+gSqr(_e[4])+gSqr(_e[5]) ); }
			
			/// normalize
			void		normalize		(void) { gReal x = magnitude(); if ( x > gEPSILON ) (*this) *= gInv(x); }

			/// return coordinate transformation of this. T is the configuration of the current frame wrt the new frame. 
			gWrench		xform			(const gXMat& T) const { gWrench r; r.makeDAdInv(T,*this); return r; }

			/// xform(const gXMat& T) with T.R=Identity and T.p=p
			gWrench		xform			(const gVec3& p) const { gVec3 f(trn()); return gWrench(rot()+ (p%f),f); } 

			/// return coordinate transformation of this. T is the configuration of the new frame wrt the current frame.
			gWrench		xformInv		(const gXMat& T) const { gWrench r; r.makeDAd(T,*this); return r; }

	friend std::istream& operator >>	(std::istream& is, gWrench& w);

			/// return dad(s,w) (i.e., ad^T)
	static	gWrench		dad				(const gTwist& s, const gWrench& w)	{ gWrench r; r.makeDLieBracket(s,w); return r; }
	
			/// return dAd(T,w) (i.e., Ad^T)
	static	gWrench		dAd				(const gXMat& T, const gWrench& w)	{ gWrench r; r.makeDAd(T,w); return r; }

			/// return dAd( inv(T), w )
	static	gWrench		dAdInv			(const gXMat& T, const gWrench& w)	{ gWrench r; r.makeDAdInv(T,w); return r; }

private:

	inline gReal*		_rot			(void){ return _e; }
	inline gReal*		_trn			(void){ return _e+3; }

	gReal	_e[6];

};

///Please read description on class gInertia in mbs_math.h 
//------------------------------------------------------------------
// gInertia : generalized inertia is represented as
//     |						 |
//     |       I		 mass[c] |
//  J =|    -mass[c]     mass*E	 |, 
//	   |						 |
// where I = R*Ic*R' - m[c][c], (R,c) is transformation from the body frame
// to Center of Mass frame, and Ic is the 3x3 rotational inertia in CoM frame. (E is identity maxtrix)
// Then the inverse of J is represented as
//          |                        |
//          |   K         -K[c]      |
//inv(J) =	| [c]K   E/mass -[c]K[c] |, where K = R*inv(Ic)*R'
//          |                        |
// I, K are symmetrix, positive definite matrice, which are represented as
//
// I =  | I[0] I[1] I[2] |  ,    K =| K[0] K[1] K[2] | 
//      | I[1] I[3] I[4] |          | K[1] K[3] K[4] |
//      | I[2] I[4] I[5] |          | K[2] K[4] K[5] |
//
//------------------------------------------------------------------
class gInertia{
public:

	enum {INERTIA_REF_BODY, INERTIA_REF_COM};

	///enum for rotational inertia. Ixx=0, Ixy=Iyx=1, Ixz=Izx=2, Iyy=3, Iyz=Izy=4, Izz=5
	enum Index{ 
		Ixx,		//0
		Ixy, Iyx=1, //1
		Ixz, Izx=2, //2
		Iyy,		//3
		Iyz, Izy=4, //4
		Izz			//5
	}; 
	
						 gInertia		(void);

						 /**				
						  * \param[in] mass mass of the body
						  * \param[in] com	center of mass position
						  * \param[in] I	rotation inertia vector I[0..5]
						  * \param[in] referenceFrame = either INERTIA_REF_BODY or INERTIA_REF_COM \n
						  * INERTIA_REF_BODY if I is expressed wrt the body frame \n
						  * INERTIA_REF_COM  if I is expressed wrt a reference frame that is located at CoM and parallel to the body frame. \n
						 */
                         gInertia		(gReal mass, const gVec3& com, const gReal* I, int referenceFrame);	
						 //gInertia		(gReal mass, gReal* Ic, const gXMat& T);		// T: from body frame to mass center frame
					 
	/// return mass
	inline	gReal		 mass		(void) const {return _m;}

	/// return center of mass position
	inline	const gReal* com		(void) const {return _c; }	

	/// return center of mass position as a vector
	inline	const gVec3	 comInVec3	(void) const { return gVec3(_c[0],_c[1],_c[2]); }

	/// return i-th element of CoM (i=0..2 
	inline	gReal		 com		(int i) const {return _c[i]; }	

	/// return const pointer to rotational inertia
	inline	const gReal* rotInertia	(void) const {return _I; }

	/// return i-th element of rotational inertia (i=[0..5])
	inline	gReal		 rotInertia	(int i) const { return _I[i]; }

	/// return inverse of rotational inertia
			const gReal* inverseRotInertia		(void);	

						 /// set gInertia. For parameters, See gInertia	(gReal mass, const gVec3& com, const gReal* I, int referenceFrame) 
	void				 set			(gReal mass, const gVec3& com, const gReal* Ic, int referenceFrame);

	/// set mass, CoM, inertia to zero	
	void				 setZero		(void);

	/// set rotational inertia to I[0..5]
	void				 setRotInertia(gReal* I);

	/// set center of mass to (x,y,z)
	void				 setCOM(gReal x, gReal y, gReal z);

	/// set mass to m
	void				 setMass(gReal m);
	
	/// return this * gTwist 
	gWrench				 operator *		(const gTwist& sc) const;		

	/// return this - G2. It MUST be that this->mass == G2->mass
	gDInertia			 operator -		(const gInertia& G2) const;	
	
	/// this += G2
	gInertia&			 operator +=	(const gInertia& G2);		

	/// this = G2
	gInertia&			 operator =		(const gInertia& G2);


	friend std::istream& operator >> (std::istream& is, gInertia& J);
	
	/// return (rotational inertia) * v
	gVec3				 multRotInertiaWithVec3(const gVec3& v) const; 

	///translate the local frame by (x,y,z) wrt the reference frame
	void				 translateFrame(const gVec3& v);	

	/// return coordinate transformation of this. T is the configuration of the current frame wrt the new frame. I.e., return dAd(inv(T))*(this)*Ad(inv(T)) 
	gInertia			 xform(const gXMat& T) const;	
	
	/// same as xform(const gXMat& T) with T.R=Identity and T.p=p
	gInertia			 xform(const gVec3& p) const;   

	/// return s'*(this)*s
	gReal				 project(const gTwist& s) const;	
	
	///Computes rate of change of GenInertia given body frame T and its body velocity V, expressed in the reference frame(world frame).
	/** T is the transformation matrix of the body frame and V is the body velocity of T \n
	 * If T is the configuration of the body frame, then the function returns the rate of change of this due to V wrt the world frame. \n
	 * If T is set to identity, this function returns the rate of change due to V expressed in the body frame.
	*/
	gDInertia	rateOfChange(const gXMat& T, const gTwist& V);

private:

	/*
	Do not call this function directly. Use rateOfChange function instead.
	BUT NOTE that -1*AdadM(T,V) = rateOfChange(T,V).
	*/
	gDInertia			 AdadM(const gXMat& T, const gTwist& V);

#ifdef MW_USE_GINERTIA_TYPE_INFO
	unsigned char		 type;
	void				 sol_type(void);
#endif
	
	void				 update_K(void);

	gReal				 _I[6], _c[3], _m, _K[6]; 
	bool				 _bK; //true if _K is valid
};

//------------------------------------------------------------------
// gAInertia : articulated body inertia (for Featherstone's method)
//
// Unlike gInertia, we use 6x6 matrix for this group.
// IMPORTANT: Since gAInertia is symmetric, WE ONLY USE UPPER TRIANGULAR PART
// (Then why do we use 6x6 matrix instead of 21 elements? That's because of "solve" function below)
//------------------------------------------------------------------
class gAInertia{
public:
						gAInertia()	{	memset(_e,0,36*sizeof(gReal)); }

	inline gReal		e(int i, int j) const { assert(i<=j); return _e[i][j]; }

	inline void			set(int i, int j, gReal d) { assert(i<=j); _e[i][j] = d; }
	inline void			setZero(void) { memset(_e,0,36*sizeof(gReal));}

	gAInertia&			operator = (const gAInertia& A);
	gAInertia&			operator += (const gAInertia& A);
	gAInertia&			operator -= (const gAInertia& A);
	gAInertia&			operator += (const gInertia& G);
	gAInertia&			operator = (const gInertia& G);
	gWrench				operator * (const gTwist& S) const; // return A*S

	gTwist				solve(const gWrench& w); // compute  inv(this)*w
	// add A. T is the config. of A's frame w.r.t. this' frame
	void				add(const gAInertia& A, const gXMat& T); // (this) += dAd(inv(T))*A*Ad(inv(T))
	void				subtractReciprocalProduct(const gWrench& x, const gWrench& y, gReal scale);/// *this -= x*y'*scale
	
private:

	gReal				_e[6][6]; //use only upper triangle part
};

//------------------------------------------------------------------
//
// class gDXMat = gXMat - gXMat (delta of gXMat)
//
//------------------------------------------------------------------
class gDXMat {

public:
						gDXMat(const gXMat& T1, const gXMat& T2, gReal scale); // dR = (T1.R-T2.R)*scale, dp = (T1.p-T2.p)*scale

	gDXMat&				operator = (const gDXMat& s);
	gDXMat&				operator +=(const gDXMat& s);
	gDXMat&				operator -=(const gDXMat& s);
	gDXMat&				operator *=(gReal scale) ;

	gTwist				preMultInverseOf(const gXMat& T) const;	//gTwist = inv(T) * this

private:

	gReal				_dR[9];
	gReal				_dp[3];
};

//------------------------------------------------------------------
//
// class gDInertia = gDInertia - gDInertia (delta of gDInertia)
//
//	_dI = I1 - I2
//  _dx = mass*(c1 - c2)
//
//------------------------------------------------------------------
class gDInertia {
public:

						gDInertia()	{	_dI[0]=_dI[1]=_dI[2]=_dI[3]=_dI[4]=_dI[5]=0;	_dx[0]=_dx[1]=_dx[2]=0;	}

	inline void			set_dI(int i,gReal d)	{_dI[i]=d;}
	inline void			set_dx(int i,gReal d)	{_dx[i]=d;}
	inline gReal		dI(int i) const { return _dI[i]; }
	inline gReal		dx(int i) const { return _dx[i]; }

	gDInertia&			operator += (const gDInertia& D);
	gDInertia&			operator -= (const gDInertia& D);
	gDInertia&			operator /= (const gReal d);
	gWrench				operator *  (const gTwist& V) const;

	inline	void		setZero(void) { _dI[0]=_dI[1]=_dI[2]=_dI[3]=_dI[4]=_dI[5]=0;_dx[0]=_dx[1]=_dx[2]=0; }
	static gReal		traceMult(const gDInertia& D1, const gDInertia& D2); //trace(D1*D2)

	static gReal		traceMultRotationInertiaOnly(const gDInertia& D1, const gDInertia& D2);

	friend class gInertia;

private:

	gReal				_dI[6];
	gReal				_dx[3];
};

//------------------------------------------------------------------
//global variables
//------------------------------------------------------------------

const gVec3		gVec3Zero(0,0,0);
const gVec3		gVec3UnitX(1,0,0);
const gVec3		gVec3UnitY(0,1,0);
const gVec3		gVec3UnitZ(0,0,1);

const gRotMat	gRotMatOne;
const gRotMat	gRotMatXp90	(1,0,0,0,0,1,0,-1,0);
const gRotMat	gRotMatXm90	(1,0,0,0,0,-1,0,1,0);
const gRotMat	gRotMatYp90	(0,0,-1,0,1,0,1,0,0);
const gRotMat	gRotMatYm90	(0,0,1,0,1,0,-1,0,0);
const gRotMat	gRotMatZp90	(0,1,0,-1,0,0,0,0,1);
const gRotMat	gRotMatZm90	(0,-1,0,1,0,0,0,0,1);

const gXMat		gXMatOne;

const gTwist	gTwistZero(0,0,0,0,0,0);
const gTwist	gTwistRotX(1,0,0,0,0,0);
const gTwist	gTwistRotY(0,1,0,0,0,0);
const gTwist	gTwistRotZ(0,0,1,0,0,0);
const gTwist	gTwistTrnX(0,0,0,1,0,0);
const gTwist	gTwistTrnY(0,0,0,0,1,0);
const gTwist	gTwistTrnZ(0,0,0,0,0,1);

const gWrench	gWrenchZero(0,0,0,0,0,0);

//------------------------------------------------------------------
// glabal functions
//------------------------------------------------------------------

std::ostream & operator << (std::ostream &os, const gVec3 &op);
std::ostream & operator << (std::ostream &os, const gRotMat &op);
std::ostream & operator << (std::ostream &os, const gQuat &op);
std::ostream & operator << (std::ostream &os, const gXMat &op);
std::ostream & operator << (std::ostream &os, const gTwist &op);
std::ostream & operator << (std::ostream &os, const gWrench &op);
std::ostream & operator << (std::ostream &os, const gInertia &op);
std::ostream & operator << (std::ostream &os, const gAInertia &op);

//{ Cholesky solver (from Numerical Recipes)
void choldc(gReal *a, int n, gReal* p);
void cholsl(const gReal *a, int n,const gReal* p,const gReal* b, gReal* x);
//}

//{ Typedef
typedef std::vector<int>					intArray;
typedef std::vector<int>::iterator			intArrayIt;

typedef std::vector<double>					doubleArray;
typedef std::vector<double>::iterator		doubleArrayIt;

typedef std::vector<uint8_t>				boolArray;
typedef std::vector<uint8_t>::iterator		boolArrayIt;

typedef std::vector<gVec3>					gVec3Array;
typedef std::vector<gVec3>::iterator		gVec3ArrayIt;

typedef std::vector<gXMat>					gXMatArray;
typedef std::vector<gXMat>::iterator		gXMatArrayIt;

typedef std::map<int, intArray>				intArrayMap;
typedef std::map<int, intArray>::iterator	intArrayMapIt;

typedef std::map<int, boolArray>			boolArrayMap;
typedef std::map<int, boolArray>::iterator	boolArrayMIt;

typedef std::map<int, gVec3Array>			gVec3ArrayMap;
typedef std::map<int, gVec3Array>::iterator	gVec3ArrayMIt;

typedef std::map<int, gXMatArray>			gXMatArrayMap;
typedef std::map<int, gXMatArray>::iterator	gXMatArrayMIt;

//}

#endif 