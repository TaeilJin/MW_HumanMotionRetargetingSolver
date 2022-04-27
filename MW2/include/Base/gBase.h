//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
//  Basic definitions and macros
//
//	Name:	gdk_general.h
//	Author: Sung-Hee Lee @ GIST
//  LAST UPDATE: 2011/10/16
//	History: 
//  - 2011/10/16: added Doxygen style comments
//	- 2007/1/4: created
//
//	Comment:
//	- source code from OpenDynamicsEngine adopted for floating point(MW_USE_SINGLE_PRECISION/MW_DOUBLE) dependent math functions 
//
// -----------------------------------------------------------------

#ifndef _MBS_GENERAL_H_
#define _MBS_GENERAL_H_

//------------------------------------------------------------------
//common headers
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <list>

//#include <stdio.h>
//#include <stdlib.h>
//#include <iostream>
#include <assert.h>
#include <math.h>
#include <float.h>

#ifdef LINUX
#include <cstring>
#define _stricmp strcasecmp //in Linux, use strcasecmp
#endif


/** double/single precision floating numbers
* !WARNING: Since GSL supports only double types in its linalg library, MW_USE_SINGLE_PRECISION cannot be used.
*/
#ifndef MW_USE_DOUBLE_PRECISION
#ifndef MW_USE_SINGLE_PRECISION
#error MW_USE_DOUBLE_PRECISION or MW_USE_SINGLE_PRECISION must be defined for the default precision of gReal. 
#endif
#endif

typedef double CoordinateType;

/** If defined, gInertia's type info will be used for faster computation. 
* It is recommended to use MW_USE_GINERTIA_TYPE_INFO.
*/
#define	MW_USE_GINERTIA_TYPE_INFO 

/** MW_USE_DIFF_NEWTON_EULER (Expenrimental Stage)
* Define this if you want to use functions regarding the differentiation 
* of the recursive Newton-Euler Inverse Dynamics
*/
#define	MW_USE_DIFF_NEWTON_EULER

/** MW_USE_ARTICULATED_BODY_METHOD
* Define this if you want to use the Articulated Body Method for forward dynamics
*/
#define MW_USE_ARTICULATED_BODY_METHOD

/** Length of string for class' name variable
*/
#define MW_SIZE_NAME	64

// precision dependent scalar math functions 
#if defined(MW_USE_SINGLE_PRECISION)

#define	gReal		float
#define gREAL(x)	(x##f)			/* form a constant */
#define gInv(x)		((1.0f/(x)))		/* inversion */
#define gSqrt(x)	(sqrtf(x))			/* square root */
#define gInvSqrt(x) ((1.0f/sqrtf(x)))	/* reciprocal square root */
#define gSin(x)		(sinf(x))			/* sine */
#define gCos(x)		(cosf(x))			/* cosine */
#define gAsin(x)	(asinf(x))			/* sine */
#define gAcos(x)	(acosf(x))			/* cosine */
#define gFabs(x)	(fabsf(x))			/* absolute value */
#define gAtan2(y,x) (atan2f(y,x))		/* arc tangent with 2 args */

#elif defined(MW_USE_DOUBLE_PRECISION)

#define	gReal		double
#define gREAL(x)	(x)
#define gInv(x)		(1.0/(x))
#define gSqrt(x)	sqrt(x)
#define gInvSqrt(x) (1.0/sqrt(x))
#define gSin(x)		sin(x)
#define gCos(x)		cos(x)
#define gAsin(x)	asin(x)
#define gAcos(x)	acos(x)
#define gFabs(x)	fabs(x)
#define gAtan2(y,x) atan2((y),(x))

#else
#error You must #define MW_USE_SINGLE_PRECISION or MW_USE_DOUBLE_PRECISION
#endif


//------------------------------------------------------------------
// constants & common macro

#define gPI			gREAL(3.1415926535897932384626433832795)

#define gDTR		gREAL(1.74532925199432957692369076848e-2)	//(M_PI/180.0)

#define	gRTD		gREAL(57.295779513082320876798154814105)	//(180.0/M_PI)

#define gEPSILON    gREAL(1e-10)

#define gNEPSILON	-gEPSILON

#define gINFINITY	gREAL(1e20)

#define gOne		gREAL(1.0)

#define gZero		gREAL(0.0)

#define gTwo		gREAL(2.0)

#define gThree		gREAL(3.0)

#define	gOneHalf	gREAL(0.5)

#define gOneThird	gREAL(0.3333333333333333333333333333333)

#define gNA         0 // N/A or Undefined

#define gNEGATIVE   -1

#define gMin(x,y)	((x)<(y)?(x):(y))

#define gMax(x,y)	((x)>(y)?(x):(y))

#define	gSqr(x)		((x)*(x))

//------------------------------------------------------------------
// common functions

/** Dot product with operator
 * 3-way dot product. dDOTpq means that elements of `a' and `b' are spaced
 * p and q indexes apart respectively. dDOT() means dDOT11.
 * in C++ we could use function templates to get all the versions of these
 * functions - but on some compilers this will result in sub-optimal code.
 */
#define gDOTpq(a,b,p,q) ((a)[0]*(b)[0] + (a)[p]*(b)[q] + (a)[2*(p)]*(b)[2*(q)])

inline gReal gDOT   (const gReal *a, const gReal *b) { return gDOTpq(a,b,1,1); }
inline gReal gDOT31 (const gReal *a, const gReal *b) { return gDOTpq(a,b,3,1); }
inline gReal gDOT41 (const gReal *a, const gReal *b) { return gDOTpq(a,b,4,1); }

/** 6D vector dot product 
*/
inline gReal gDOT6  (const gReal *a, const gReal *b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]+a[4]*b[4]+a[5]*b[5]; }

/**cross product with operator
*/
#define gCROSSOP(a,op,b,c) \
do { \
  (a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
  (a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
  (a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]); \
} while(0)

inline void gCROSSeq(gReal* a, const gReal* b, const gReal* c) { gCROSSOP(a,=,b,c);}
inline void gCROSSpe(gReal* a, const gReal* b, const gReal* c) { gCROSSOP(a,+=,b,c);}
inline void gCROSSme(gReal* a, const gReal* b, const gReal* c) { gCROSSOP(a,-=,b,c);}


//matrix multiplications
// A op B*C ( A,C: 3 vector, B:3x3 matrix )
#define gMULTIPLYOP331(A,op,B,C) \
do { \
  (A)[0] op gDOT31((B),(C)); \
  (A)[1] op gDOT31((B+1),(C)); \
  (A)[2] op gDOT31((B+2),(C)); \
} while(0)

inline void gMULTIPLYeq331(gReal* a, const gReal* b, const gReal* c) { gMULTIPLYOP331(a,=,b,c); }
inline void gMULTIPLYpe331(gReal* a, const gReal* b, const gReal* c) { gMULTIPLYOP331(a,+=,b,c); }
inline void gMULTIPLYme331(gReal* a, const gReal* b, const gReal* c) { gMULTIPLYOP331(a,-=,b,c); }


/** A op B'*C ( A,C: 3 vector, B:3x3 matrix )
*/
#define gTMULTIPLYOP331(A,op,B,C) \
do { \
  (A)[0] op gDOT((B),(C)); \
  (A)[1] op gDOT((B+3),(C)); \
  (A)[2] op gDOT((B+6),(C)); \
} while(0)

inline void gTMULTIPLYeq331(gReal* a, const gReal* b, const gReal* c) { gTMULTIPLYOP331(a,=,b,c); }
inline void gTMULTIPLYpe331(gReal* a, const gReal* b, const gReal* c) { gTMULTIPLYOP331(a,+=,b,c); }
inline void gTMULTIPLYme331(gReal* a, const gReal* b, const gReal* c) { gTMULTIPLYOP331(a,-=,b,c); }

/** A op B*C ( A,B,C: 3x3 matrix )
*/
#define gMULTIPLYOP333(A,op,B,C) \
do { \
  (A)[0] op gDOT31((B),(C)); \
  (A)[1] op gDOT31((B+1),(C)); \
  (A)[2] op gDOT31((B+2),(C)); \
  (A)[3] op gDOT31((B),(C+3)); \
  (A)[4] op gDOT31((B+1),(C+3)); \
  (A)[5] op gDOT31((B+2),(C+3)); \
  (A)[6] op gDOT31((B),(C+6)); \
  (A)[7] op gDOT31((B+1),(C+6)); \
  (A)[8] op gDOT31((B+2),(C+6)); \
} while(0)

inline void gMULTIPLYeq333(gReal* a, const gReal* b, const gReal* c) { gMULTIPLYOP333(a,=,b,c); }
inline void gMULTIPLYpe333(gReal* a, const gReal* b, const gReal* c) { gMULTIPLYOP333(a,+=,b,c); }
inline void gMULTIPLYme333(gReal* a, const gReal* b, const gReal* c) { gMULTIPLYOP333(a,-=,b,c); }

// A op B'*C ( A,B,C: 3x3 matrix )
#define gTMULTIPLYOP333(A,op,B,C) \
do { \
  (A)[0] op gDOT((B),(C)); \
  (A)[1] op gDOT((B+3),(C)); \
  (A)[2] op gDOT((B+6),(C)); \
  (A)[3] op gDOT((B),(C+3)); \
  (A)[4] op gDOT((B+3),(C+3)); \
  (A)[5] op gDOT((B+6),(C+3)); \
  (A)[6] op gDOT((B),(C+6)); \
  (A)[7] op gDOT((B+3),(C+6)); \
  (A)[8] op gDOT((B+6),(C+6)); \
} while(0)

inline void gTMULTIPLYeq333(gReal* a, const gReal* b, const gReal* c) { gTMULTIPLYOP333(a,=,b,c); }
inline void gTMULTIPLYpe333(gReal* a, const gReal* b, const gReal* c) { gTMULTIPLYOP333(a,+=,b,c); }
inline void gTMULTIPLYme333(gReal* a, const gReal* b, const gReal* c) { gTMULTIPLYOP333(a,-=,b,c); }


//END OF ODE FUNCTIONS

//element-wise operations
#define gELEMOP1(A,op,B) \
do { \
	(A)[0] op (B); \
	(A)[1] op (B); \
	(A)[2] op (B); \
} while(0)

inline void gCOPY1(gReal* a, gReal b) {gELEMOP1(a,=,b);}
inline void gPE1(gReal* a, gReal b) {gELEMOP1(a,+=,b);}
inline void gME1(gReal* a, gReal b) {gELEMOP1(a,-=,b);}


//element-wise operations
#define gELEMOP3(A,op,B) \
do { \
	(A)[0] op (B)[0]; \
	(A)[1] op (B)[1]; \
	(A)[2] op (B)[2]; \
} while(0)

inline void gCOPY3(gReal* a, const gReal* b) {gELEMOP3(a,=,b);}
inline void gPE3(gReal* a, const gReal* b) {gELEMOP3(a,+=,b);}
inline void gME3(gReal* a, const gReal* b) {gELEMOP3(a,-=,b);}


#define gELEMOP31(A,op1,B,op2,C) \
do { \
	(A)[0] op1 ((B)[0] op2 (C)); \
	(A)[1] op1 ((B)[1] op2 (C)); \
	(A)[2] op1 ((B)[2] op2 (C)); \
} while(0)

//inline void gSCALEeq31(gReal* a,const gReal* b, gReal s) {gELEMOP31(a,=,b,*,s);}
//inline void gSCALEpe31(gReal* a,const gReal* b, gReal s) {gELEMOP31(a,+=,b,*,s);}
//inline void gSCALEme31(gReal* a,const gReal* b, gReal s) {gELEMOP31(a,-=,b,*,s);}
inline void gSCALE3(gReal* a,const gReal* b, gReal s) {gELEMOP31(a,=,b,*,s);}
inline void gSCALE3(gReal* a, gReal s) { gELEMOP31(a,=,a,*,s); }
inline void gPE3(gReal* a,const gReal* b, gReal s) {gELEMOP31(a,+=,b,*,s);}
inline void gME3(gReal* a,const gReal* b, gReal s) {gELEMOP31(a,-=,b,*,s);}


#define gELEMOP33(A,op1,B,op2,C) \
do { \
	(A)[0] op1 ((B)[0] op2 (C)[0]); \
	(A)[1] op1 ((B)[1] op2 (C)[1]); \
	(A)[2] op1 ((B)[2] op2 (C)[2]); \
} while(0)


inline void gSCALEeq33(gReal* a,const gReal* b, const gReal* c) {gELEMOP33(a,=,b,*,c);}
inline void gSCALEpe33(gReal* a,const gReal* b, const gReal* c) {gELEMOP33(a,+=,b,*,c);}
inline void gSCALEme33(gReal* a,const gReal* b, const gReal* c) {gELEMOP33(a,-=,b,*,c);}

inline	void gPLUSeq33(gReal* a, const gReal* b, const gReal* c) {gELEMOP33(a,=,b,+,c);}
inline	void gPLUSpe33(gReal* a, const gReal* b, const gReal* c) {gELEMOP33(a,+=,b,+,c);}
inline	void gPLUSme33(gReal* a, const gReal* b, const gReal* c) {gELEMOP33(a,-=,b,+,c);}

inline	void gMINUSeq33(gReal* a, const gReal* b, const gReal* c) {gELEMOP33(a,=,b,-,c);}
inline	void gMINUSpe33(gReal* a, const gReal* b, const gReal* c) {gELEMOP33(a,+=,b,-,c);}
inline	void gMINUSme33(gReal* a, const gReal* b, const gReal* c) {gELEMOP33(a,-=,b,-,c);}


//ETC
inline void gINV3(gReal* a, const gReal* b)	{a[0]=gInv(b[0]);a[1]=gInv(b[1]);a[2]=gInv(b[2]);}

inline void gZERO3(gReal* a)	{a[0]=a[1]=a[2]=0;}

inline bool IsZero(gReal x)		{ return ( x < gEPSILON && x > gNEPSILON )? true:false; }

#endif //_MBS_GENERAL_H_

