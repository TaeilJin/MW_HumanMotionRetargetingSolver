//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// ----------------------------------------------------------------- 
                                                                     
                                             

#include "Base/gBase.h"
#include "Base/gMath.h"

static inline int gSgn(gReal x)
{
	return (x>=0) ? 1:-1;
} 

static int gRound(gReal x)
{
	return (int)(gSgn(x)*(gFabs(x)+gREAL(0.5)));
}

// A op B*C ( A,C: 3 vector, B:4x4 matrix ) 
#define gMULTIPLYOP431(A,op,B,C) \
do { \
  (A)[0] op gDOT41((B),(C)); \
  (A)[1] op gDOT41((B+1),(C)); \
  (A)[2] op gDOT41((B+2),(C)); \
} while(0)

// A op B'*C ( A,C: 3 vector, B:4x4 matrix ) 
#define gTMULTIPLYOP431(A,op,B,C) \
do { \
  (A)[0] op gDOT((B),(C)); \
  (A)[1] op gDOT((B+4),(C)); \
  (A)[2] op gDOT((B+8),(C)); \
} while(0)

//------------------------------------------------------------------
// inv: inv(m), where m is 3x3 symmetric matrix of
// m =	| m[0] m[1] m[2] |
//		| m[1] m[3] m[4] |
//		| m[2] m[4] m[5] |
static void invSymMat33(gReal* inv, const gReal* m)
//------------------------------------------------------------------
{
	gReal idet = gREAL(1.)/(-m[0]*m[3]*m[5]+m[0]*gSqr(m[4])+gSqr(m[1])*m[5]-2*m[1]*m[2]*m[4]+gSqr(m[2])*m[3]);
	inv[0] = (m[4]*m[4]-m[3]*m[5])*idet;
	inv[1] = (m[1]*m[5]-m[2]*m[4])*idet;
	inv[2] = (m[2]*m[3]-m[1]*m[4])*idet;
	inv[3] = (m[2]*m[2]-m[0]*m[5])*idet;
	inv[4] = (m[0]*m[4]-m[1]*m[2])*idet;
	inv[5] = (m[1]*m[1]-m[0]*m[3])*idet; 
}

//------------------------------------------------------------------
// y = M*x  M[6],x[3],y[3]
static void multSymMat33(gReal* y,const gReal* M,const gReal* x)
//------------------------------------------------------------------
{
	y[0] = M[0]*x[0]+M[1]*x[1]+M[2]*x[2];
	y[1] = M[1]*x[0]+M[3]*x[1]+M[4]*x[2];
	y[2] = M[2]*x[0]+M[4]*x[1]+M[5]*x[2];
};




gRotMat::gRotMat(){  
	memset(_e,0,9*sizeof(gReal));
	_e[0] = 1.; _e[4] = 1.; _e[8] = 1.;
}

gRotMat::gRotMat(const gReal* a){  
	memcpy(_e,a,sizeof(gReal)*9); 
}

gRotMat::gRotMat(gReal a0,gReal a1,gReal a2,gReal a3,gReal a4,gReal a5,gReal a6,gReal a7,gReal a8)
{
	_e[0]=a0;_e[1]=a1;_e[2]=a2;
	_e[3]=a3;_e[4]=a4;_e[5]=a5;
	_e[6]=a6;_e[7]=a7;_e[8]=a8;
}

gRotMat::gRotMat (const gVec3& Rx, const gVec3& Ry, const gVec3& Rz)
{
	_e[0]=Rx.x();_e[1]=Rx.y();_e[2]=Rx.z();
	_e[3]=Ry.x();_e[4]=Ry.y();_e[5]=Ry.z();
	_e[6]=Rz.x();_e[7]=Rz.y();_e[8]=Rz.z();

}

void gRotMat::setIdentity(void){
	_e[0] = _e[4] = _e[8] = 1.;
	_e[1] = _e[2] = _e[3] = _e[5] = _e[6] = _e[7] = 0.;
}

gRotMat& gRotMat::operator *= (const gRotMat &R) {
	gReal r1,r2,r3;
	r1 = _e[0]*R._e[0] + _e[3]*R._e[1] + _e[6]*R._e[2];
	r2 = _e[0]*R._e[3] + _e[3]*R._e[4] + _e[6]*R._e[5];
	r3 = _e[0]*R._e[6] + _e[3]*R._e[7] + _e[6]*R._e[8];
	_e[0] = r1;
	_e[3] = r2;
	_e[6] = r3;

	r1 = _e[1]*R._e[0] + _e[4]*R._e[1] + _e[7]*R._e[2];
	r2 = _e[1]*R._e[3] + _e[4]*R._e[4] + _e[7]*R._e[5];
	r3 = _e[1]*R._e[6] + _e[4]*R._e[7] + _e[7]*R._e[8];
	_e[1] = r1;
	_e[4] = r2;
	_e[7] = r3;

	r1 = _e[2]*R._e[0] + _e[5]*R._e[1] + _e[8]*R._e[2];
	r2 = _e[2]*R._e[3] + _e[5]*R._e[4] + _e[8]*R._e[5];
	r3 = _e[2]*R._e[6] + _e[5]*R._e[7] + _e[8]*R._e[8];
	_e[2] = r1;
	_e[5] = r2;
	_e[8] = r3;
	return *this;
}

gRotMat gRotMat::operator * (const gRotMat &R) const{
	gRotMat re(*this);
	re *= R;
	return re;
}

gRotMat& gRotMat::operator = (const gRotMat &R){				// =
	memcpy(_e,R._e,9*sizeof(gReal));
	return *this;
}

gVec3	 gRotMat::operator *	(const gVec3& v) const
{
	gReal r[3];
	gMULTIPLYOP331(r,=,_e,v.cptr());
	return gVec3(r);
}

gVec3	gRotMat::invMult(const gVec3& v) const
{
	gReal r[3];
	gTMULTIPLYeq331(r,_e,v.cptr());
	return gVec3(r);
}

gRotMat	gRotMat::invMult(const gRotMat& R) const
{
	return gRotMat(
		_e[0]*R._e[0]+ _e[1]*R._e[1]+ _e[2]*R._e[2], 
		_e[3]*R._e[0]+ _e[4]*R._e[1]+ _e[5]*R._e[2], 
		_e[6]*R._e[0]+ _e[7]*R._e[1]+ _e[8]*R._e[2],
		
		_e[0]*R._e[3]+ _e[1]*R._e[4]+ _e[2]*R._e[5], 
		_e[3]*R._e[3]+ _e[4]*R._e[4]+ _e[5]*R._e[5], 
		_e[6]*R._e[3]+ _e[7]*R._e[4]+ _e[8]*R._e[5],
		
		_e[0]*R._e[6]+ _e[1]*R._e[7]+ _e[2]*R._e[8], 
		_e[3]*R._e[6]+ _e[4]*R._e[7]+ _e[5]*R._e[8], 
		_e[6]*R._e[6]+ _e[7]*R._e[7]+ _e[8]*R._e[8]);
}

gRotMat		gRotMat::operator ~(void) const
{
	return gRotMat(_e[0],_e[3],_e[6],_e[1],_e[4],_e[7],_e[2],_e[5],_e[8]); 
}

void gQuat::makeRotate(gReal x, gReal y, gReal z)
{
	gReal angle = gSqrt( x*x + y*y + z*z );
	if ( angle < gREAL(0.0000001) )
	{
		// ~zero magnitude axis, so reset rotation to zero.
		*this = gQuat();
		return;
	}
	gReal normedSinHalfAngle = gSin( gREAL(0.5)*angle )*gInv( angle );
	_e[0] = x * normedSinHalfAngle;
	_e[1] = y * normedSinHalfAngle;
	_e[2] = z * normedSinHalfAngle;
	_e[3] = gCos( gREAL(0.5)*angle );
}

void gQuat::makeRotate( const gVec3& from, const gVec3& to)
{
	// THIS Code has been copied from OSG::Quat

	// This routine takes any vector as argument but normalized 
    // vectors are necessary, if only for computing the dot product.
    // Too bad the API is that generic, it leads to performance loss.
    // Even in the case the 2 vectors are not normalized but same length,
    // the sqrt could be shared, but we have no way to know beforehand
    // at this point, while the caller may know.
    // So, we have to test... in the hope of saving at least a sqrt
    gVec3 sourceVector = from;
    gVec3 targetVector = to;
    
    gReal fromLen2 = from.magnitudeSquared();
    gReal fromLen;
    // normalize only when necessary, epsilon test
    if ((fromLen2 < 1.0-1e-7) || (fromLen2 > 1.0+1e-7)) {
        fromLen = sqrt(fromLen2);
        sourceVector /= fromLen;
    } else fromLen = 1.0;
    
    gReal toLen2 = to.magnitudeSquared();
    // normalize only when necessary, epsilon test
    if ((toLen2 < 1.0-1e-7) || (toLen2 > 1.0+1e-7)) {
        gReal toLen;
        // re-use fromLen for case of mapping 2 vectors of the same length
        if ((toLen2 > fromLen2-1e-7) && (toLen2 < fromLen2+1e-7)) {
            toLen = fromLen;
        } 
        else toLen = sqrt(toLen2);
        targetVector /= toLen;
    }
	    
    // Now let's get into the real stuff
    // Use "dot product plus one" as test as it can be re-used later on
    double dotProdPlus1 = 1.0 + (sourceVector , targetVector);
    
    // Check for degenerate case of full u-turn. Use epsilon for detection
    if (dotProdPlus1 < 1e-7) {
    
        // Get an orthogonal vector of the given vector
        // in a plane with maximum vector coordinates.
        // Then use it as quaternion axis with pi angle
        // Trick is to realize one value at least is >0.6 for a normalized vector.
        if (fabs(sourceVector.x()) < 0.6) {
            const double norm = sqrt(1.0 - sourceVector.x() * sourceVector.x());
            _e[0] = 0.0; 
            _e[1] = sourceVector.z() / norm;
            _e[2] = -sourceVector.y() / norm;
            _e[3] = 0.0;
        } else if (fabs(sourceVector.y()) < 0.6) {
            const double norm = sqrt(1.0 - sourceVector.y() * sourceVector.y());
            _e[0] = -sourceVector.z() / norm;
            _e[1] = 0.0;
            _e[2] = sourceVector.x() / norm;
            _e[3] = 0.0;
        } else {
            const double norm = sqrt(1.0 - sourceVector.z() * sourceVector.z());
            _e[0] = sourceVector.y() / norm;
            _e[1] = -sourceVector.x() / norm;
            _e[2] = 0.0;
            _e[3] = 0.0;
        }
    }    
    else {
        // Find the shortest angle quaternion that transforms normalized vectors
        // into one other. Formula is still valid when vectors are colinear
        const double s = sqrt(0.5 * dotProdPlus1);
        const gVec3 tmp = (sourceVector % targetVector) * gInv(2.0*s);
        _e[0] = tmp.x();
        _e[1] = tmp.y();
        _e[2] = tmp.z();
        _e[3] = s;
    }
}



void gQuat::getAxisAngle( gReal& x, gReal& y, gReal& z, gReal& angle ) const
{
	angle = gREAL(2.0)*gAcos(_e[3]);
	if( IsZero(angle) )
	{
		x=y=z=gReal(0.0);
	}
	else
	{
        gReal invSinHalf = gInv(gSin(gREAL(0.5)*angle));
		x = _e[0]*invSinHalf;
		y = _e[1]*invSinHalf;
		z = _e[2]*invSinHalf;
	}
}

void gQuat::getAxisAngle( gVec3& axis, gReal& angle) const
{
	gReal x,y,z;
	getAxisAngle(x,y,z,angle);
	axis.set(x,y,z);
}

void gQuat::normalize	(void)	
{ 
	gReal ilen = gInv(magnitude()); _e[0]*=ilen; _e[1]*=ilen; _e[2]*=ilen; _e[3]*=ilen; 
}

gQuat gQuat::operator * (const gQuat P) const 
{
	gQuat r;
	r._e[3] = _e[3]*P._e[3] - gDOT(_e,P._e);
	gCROSSeq(r._e,_e,P._e);
	gPE3(r._e,P._e,_e[3]);
	gPE3(r._e,_e,P._e[3]);
	return r;
}


gVec3 gQuat::operator* (const gVec3& x) const {
    gReal a = _e[3]*_e[3] - gDOT(_e,_e);
	gReal b = gREAL(2.0)*_e[3];
	gReal c = gREAL(2.0)*gDOT(x.cptr(),_e);
	
	gReal r[3];
	gCROSSeq(r,_e,x.cptr());	//r = cross(q,x)

	return x*a + gVec3(r)*b + gVec3(_e)*c;                
}
	
gRotMat	gQuat::inRotMatrix(void) const
{
	gReal x2 = _e[0]*_e[0];
	gReal y2 = _e[1]*_e[1];
	gReal z2 = _e[2]*_e[2];
	gReal xy = _e[0]*_e[1];
	gReal yz = _e[1]*_e[2];
	gReal xz = _e[2]*_e[0];
	gReal wx = _e[3]*_e[0];
	gReal wy = _e[3]*_e[1];
	gReal wz = _e[3]*_e[2];

	return gRotMat(
		1 - 2*(y2 + z2),  
		2*(xy + wz),
		2*(xz - wy),		
		2*(xy - wz), 
		1 - 2*(x2 + z2), 
		2*(yz + wx),		
		2*(xz + wy), 
		2*(yz - wx), 
		1 - 2*(x2 + y2) 		);		
}

gXMat::gXMat(gReal* p, bool copy_mem){
	if (copy_mem)
	{
		memcpy(_ei, p, SIZE_INDEX * sizeof(gReal));
		_e = _ei;
	}
	else
		_e = p;
}
gXMat::gXMat(const gReal* p){
	memcpy(_ei, p, SIZE_INDEX*sizeof(gReal));
	_e = _ei;
}
gXMat::gXMat (const gXMat& rhs){
	if( rhs._e == rhs._ei )
	{
		memcpy(_ei, rhs._ei, SIZE_INDEX*sizeof(gReal));
		_e = _ei;
	} else {
		_e = rhs._e;
	}
}

gXMat::gXMat(void){
	_e = _ei;
	memset(_e,0,16*sizeof(gReal));
	_e[0] = _e[5] = _e[10] = _e[15] = 1.;
}

gXMat::gXMat(const gRotMat& R, const gVec3& v){
	_e = _ei;

	_e[3] = _e[7] = _e[11] = 0.; _e[15] = 1.;

	_e[0] = R.e(0); _e[1] = R.e(1); _e[2] = R.e(2);
	_e[4] = R.e(3); _e[5] = R.e(4); _e[6] = R.e(5);
	_e[8] = R.e(6); _e[9] = R.e(7); _e[10] = R.e(8);
	_e[12] = v.x(); _e[13] = v.y(); _e[14] = v.z(); 
}

gXMat::gXMat(const gRotMat& R){
	_e = _ei;

	_e[3] = _e[7] = _e[11] = 0.; _e[15] = 1.;
	_e[0] = R.e(0); _e[1] = R.e(1); _e[2] = R.e(2);
	_e[4] = R.e(3); _e[5] = R.e(4); _e[6] = R.e(5);
	_e[8] = R.e(6); _e[9] = R.e(7); _e[10] = R.e(8);
	_e[12] = _e[13] = _e[14] = 0; 
}

gXMat::gXMat(gReal r0, gReal r1, gReal r2, gReal r3, gReal r4, gReal r5, gReal r6, gReal r7, gReal r8, gReal p0, gReal p1, gReal p2)
{
	_e = _ei;

	_e[3] = _e[7] = _e[11] = 0.; _e[15] = 1.;
	_e[0] = r0; _e[1] = r1; _e[2] = r2;
	_e[4] = r3; _e[5] = r4; _e[6] = r5;
	_e[8] = r6; _e[9] = r7; _e[10] = r8;
	_e[12] = p0; _e[13] = p1; _e[14] = p2; 
}

gXMat::gXMat(const gVec3& v){
	_e = _ei;

	_e[0] = _e[5] = _e[10] = _e[15] = 1.;
	_e[1] = _e[2] = _e[3] = _e[4] = _e[6] = _e[7] = _e[8] = _e[9] = _e[11] = 0.;
	_e[12] = v.x(); _e[13] = v.y(); _e[14] = v.z();	
}

gXMat::gXMat(gReal x, gReal y, gReal z)
{
	_e = _ei;

	_e[0] = _e[5] = _e[10] = _e[15] = 1.;
	_e[1] = _e[2] = _e[3] = _e[4] = _e[6] = _e[7] = _e[8] = _e[9] = _e[11] = 0.;
	_e[12] = x; _e[13] = y; _e[14] = z;	
}

void gXMat::translateLocal(gReal x, gReal y, gReal z)
{
	translateLocal(gVec3(x,y,z));
}

void gXMat::translateLocal(const gVec3& v) { 
	_e[12] += ( _e[0]*v.x() + _e[4]*v.y() + _e[8]*v.z() );
	_e[13] += ( _e[1]*v.x() + _e[5]*v.y() + _e[9]*v.z() );
	_e[14] += ( _e[2]*v.x() + _e[6]*v.y() + _e[10]*v.z() );
}

void gXMat::rotateRef(const gRotMat& R) //rotate wrt reference frame
{
	makePreMult(gXMat(R));
}

void gXMat::rotateLocal(const gRotMat& R) //rotate wrt itself
{
	*this *= gXMat(R);
}

//( Exp, axisPos-Exp*axisPos )*(*this) where Exp = exp( axisDir*angle )
void gXMat::rotateRef(const gVec3 axisDir, const gVec3 axisPos, gReal angle)
{
	gRotMat R = gRotMat::exp(axisDir*angle);
	gXMat T; T.setRot(R); T.setTrn(axisPos - R*axisPos);
	makePreMult(T);	
}


	
//(*this)*( Exp, axisPos-Exp*axisPos ) where Exp = exp( axisDir*angle )
void gXMat::rotateLocal(const gVec3 axisDir, const gVec3 axisPos, gReal angle)
{
	gRotMat R = gRotMat::exp(axisDir*angle);
	gXMat T; T.setRot(R); T.setTrn(axisPos - R*axisPos);
	(*this) *= T;	
}

void gXMat::rotateRef(const gRotMat& R, const gVec3& pivot)
{
	gXMat T; T.setRot(R); T.setTrn(pivot - R*pivot);
	makePreMult(T);	
}

void gXMat::rotateLocal(const gRotMat& R, const gVec3& pivot)
{
	gXMat T; T.setRot(R); T.setTrn(pivot - R*pivot);
	(*this) *= T;	
}

void gXMat::setRot(const gRotMat& R){
	_e[0] = R.e(0); _e[1] = R.e(1); _e[2] = R.e(2);
	_e[4] = R.e(3); _e[5] = R.e(4); _e[6] = R.e(5);
	_e[8] = R.e(6); _e[9] = R.e(7); _e[10] = R.e(8);
}

gXMat gXMat::operator ~	(void) const 
{
	gXMat r;
	r.makeInverse(*this);
	return r;
}

//R = T.R'
//p = -T.R'*T.p
void gXMat::makeInverse(const gXMat& T){
	// R= T.R'
    _e[0] = T._e[0]; _e[4] = T._e[1]; _e[8] = T._e[2]; 
	_e[1] = T._e[4]; _e[5] = T._e[5]; _e[9] = T._e[6]; 
	_e[2] = T._e[8]; _e[6] = T._e[9]; _e[10] = T._e[10]; 
	// p = -R*T.p
	_e[12] = -T._e[0]*T._e[12]-T._e[1]*T._e[13]-T._e[2]*T._e[14];
	_e[13] = -T._e[4]*T._e[12]-T._e[5]*T._e[13]-T._e[6]*T._e[14];
	_e[14] = -T._e[8]*T._e[12]-T._e[9]*T._e[13]-T._e[10]*T._e[14];
}

gXMat gXMat::invMult	(const gXMat& T)	const			// return inv(this)*T
{
	gXMat r;
	r._e[0]= _e[0]*T._e[0]+_e[1]*T._e[1]+_e[2]*T._e[2];
	r._e[1]= _e[4]*T._e[0]+_e[5]*T._e[1]+_e[6]*T._e[2];
	r._e[2]= _e[8]*T._e[0]+_e[9]*T._e[1]+_e[10]*T._e[2];
	r._e[4]= _e[0]*T._e[4]+_e[1]*T._e[5]+_e[2]*T._e[6];
	r._e[5]= _e[4]*T._e[4]+_e[5]*T._e[5]+_e[6]*T._e[6];
	r._e[6]= _e[8]*T._e[4]+_e[9]*T._e[5]+_e[10]*T._e[6];
	r._e[8]= _e[0]*T._e[8]+_e[1]*T._e[9]+_e[2]*T._e[10];
	r._e[9]= _e[4]*T._e[8]+_e[5]*T._e[9]+_e[6]*T._e[10];
	r._e[10]= _e[8]*T._e[8]+_e[9]*T._e[9]+_e[10]*T._e[10];
	r._e[12]= _e[0]*(T._e[12]-_e[12])+_e[1]*(T._e[13]-_e[13])+_e[2]*(T._e[14]-_e[14]);
	r._e[13]= _e[4]*(T._e[12]-_e[12])+_e[5]*(T._e[13]-_e[13])+_e[6]*(T._e[14]-_e[14]);
	r._e[14]= _e[8]*(T._e[12]-_e[12])+_e[9]*(T._e[13]-_e[13])+_e[10]*(T._e[14]-_e[14]);
	return r;
}

void gXMat::setIdentity(void)
{
	_e[0] = _e[5] = _e[10] = 1.0;
	_e[1] = _e[2] = _e[4] = _e[6] = _e[8] = _e[9] = _e[12] = _e[13] = _e[14] = 0.0;	
}

void gXMat::set(const gXMat& T)
{ 
	// ??
	if( _e != _ei )
	{
		_e = _ei;
	}

	memcpy(_e,T._e,16*sizeof(gReal)); 
}

void gXMat::set(int i, gReal d)
{
	// ??
	if( _e != _ei )
	{
		memcpy( _ei, _e, SIZE_INDEX*sizeof(gReal) ); 
		_e = _ei;
	}
	_e[i] = d; 
}

gXMat gXMat::operator *(const gXMat& s) const
{
	gXMat ss(*this);
	ss *= s;
	return ss;
};

gVec3 gXMat::multVec3(const gReal* a) const
{	
	return gVec3(
		_e[0]*a[0] + _e[4]*a[1] + _e[8]*a[2], 
		_e[1]*a[0] + _e[5]*a[1] + _e[9]*a[2], 
		_e[2]*a[0] + _e[6]*a[1] + _e[10]*a[2]);
}

gVec3 gXMat::multVec3(const gVec3& v) const
{	
	return multVec3(v.cptr());
}


gVec3 gXMat::invMultVec3(const gReal* a)	const
{
	return gVec3(
		_e[0]*a[0] + _e[1]*a[1] + _e[2]*a[2] , 
		_e[4]*a[0] + _e[5]*a[1] + _e[6]*a[2], 
		_e[8]*a[0] + _e[9]*a[1] + _e[10]*a[2] );
}

gVec3 gXMat::invMultVec3(const gVec3& v)	const
{
	return invMultVec3(v.cptr());
}

gVec3 gXMat::multVec4(const gVec3& v) const
{	
	return multVec4(v.cptr());
}

gVec3 gXMat::multVec4(const gReal* a) const
{	
	return gVec3(
		_e[0]*a[0] + _e[4]*a[1] + _e[8]*a[2] + _e[12],
		_e[1]*a[0] + _e[5]*a[1] + _e[9]*a[2] + _e[13],
		_e[2]*a[0] + _e[6]*a[1] + _e[10]*a[2] + _e[14]);
}

gVec3 gXMat::multVec4gen(const gReal* a) const
{	
	return gVec3(
		_e[0]*a[0] + _e[4]*a[1] + _e[8]*a[2] + _e[12]*a[3],
		_e[1]*a[0] + _e[5]*a[1] + _e[9]*a[2] + _e[13]*a[3],
		_e[2]*a[0] + _e[6]*a[1] + _e[10]*a[2] + _e[14]*a[3]);
}

gXMat	gXMat::multRotMat(const gRotMat& r) const		// [R*r,p]
{
	return gXMat(
	_e[0]*r.e(0) + _e[4]*r.e(1) + _e[8]*r.e(2),
	_e[1]*r.e(0) + _e[5]*r.e(1) + _e[9]*r.e(2),
	_e[2]*r.e(0) + _e[6]*r.e(1) + _e[10]*r.e(2),
	_e[0]*r.e(3) + _e[4]*r.e(4) + _e[8]*r.e(5),
	_e[1]*r.e(3) + _e[5]*r.e(4) + _e[9]*r.e(5),
	_e[2]*r.e(3) + _e[6]*r.e(4) + _e[10]*r.e(5),
	_e[0]*r.e(6) + _e[4]*r.e(7) + _e[8]*r.e(8),
	_e[1]*r.e(6) + _e[5]*r.e(7) + _e[9]*r.e(8),
	_e[2]*r.e(6) + _e[6]*r.e(7) + _e[10]*r.e(8),
	_e[12],_e[13],_e[14]);
}

// return (R'*(v - p))
gVec3 gXMat::invMultVec4(const gVec3& v) const
{
	gReal r[3];
	gELEMOP33(r,=,v.cptr(),-,_e+12);	//r=v-p
	return gVec3( gDOT(_e,r), gDOT(_e+4,r), gDOT(_e+8,r) );
}

//i=0,1,2
void gRotMat::rectify(int i)
{
	int j=(i+1)%3;
	int k=(i+2)%3;	
	gReal len=gSqrt(gDOT(_col(i),_col(i)));	
	gELEMOP1(_col(i),*=,gInv(len));			// x.normalize
	gReal p = gDOT(_col(i),_col(j));		// p = x'y	
	gELEMOP31(_col(j),-=,_col(i),*,p);		// y = y-x*p : now x and y are orthogonal
	len = gSqrt(gDOT(_col(j),_col(j)));	
	gELEMOP1(_col(j),*=,gInv(len));			// y.normalize
	gCROSSOP(_col(k),=,_col(i),_col(j));
}


//i=0,1,2
void gXMat::rectify(int i)
{
	int j=(i+1)%3;
	int k=(i+2)%3;	
	gReal len=gSqrt(gDOT(_col(i),_col(i)));	
	gELEMOP1(_col(i),*=,gInv(len));			// x.normalize
	gReal p = gDOT(_col(i),_col(j));		// p = x'y	
	gELEMOP31(_col(j),-=,_col(i),*,p);		// y = y-x*p : now x and y are orthogonal
	len = gSqrt(gDOT(_col(j),_col(j)));	
	gELEMOP1(_col(j),*=,gInv(len));			// y.normalize
	gCROSSOP(_col(k),=,_col(i),_col(j));
}

void gXMat::makePreMult(const gXMat& T) 
{
	gReal r1, r2, r3;
	
	r1 = T._e[0]*_e[0] + T._e[4]*_e[1] + T._e[8]*_e[2];
	r2 = T._e[1]*_e[0] + T._e[5]*_e[1] + T._e[9]*_e[2];
	r3 = T._e[2]*_e[0] + T._e[6]*_e[1] + T._e[10]*_e[2];
	_e[0] = r1;
	_e[1] = r2;
	_e[2] = r3;

	r1 = T._e[0]*_e[4] + T._e[4]*_e[5] + T._e[8]*_e[6];
	r2 = T._e[1]*_e[4] + T._e[5]*_e[5] + T._e[9]*_e[6];
	r3 = T._e[2]*_e[4] + T._e[6]*_e[5] + T._e[10]*_e[6];
	_e[4] = r1;	
	_e[5] = r2;
	_e[6] = r3;

	r1 = T._e[0]*_e[8] + T._e[4]*_e[9] + T._e[8]*_e[10];
	r2 = T._e[1]*_e[8] + T._e[5]*_e[9] + T._e[9]*_e[10];
	r3 = T._e[2]*_e[8] + T._e[6]*_e[9] + T._e[10]*_e[10];
	_e[8] = r1;
	_e[9] = r2;
	_e[10] = r3;

	r1 = T._e[12] + T._e[0]*_e[12] + T._e[4]*_e[13] + T._e[8]*_e[14];
	r2 = T._e[13] + T._e[1]*_e[12] + T._e[5]*_e[13] + T._e[9]*_e[14];
	r3 = T._e[14] + T._e[2]*_e[12] + T._e[6]*_e[13] + T._e[10]*_e[14];
	
	_e[12] = r1;
	_e[13] = r2;
	_e[14] = r3;

}

gXMat& gXMat::operator *=(const gXMat& T)
{
	gReal r1, r2, r3;
	
	_e[12] += _e[0]*T._e[12] + _e[4]*T._e[13] + _e[8]*T._e[14];
	_e[13] += _e[1]*T._e[12] + _e[5]*T._e[13] + _e[9]*T._e[14];
	_e[14] += _e[2]*T._e[12] + _e[6]*T._e[13] + _e[10]*T._e[14];
	
	r1 = _e[0]*T._e[0] + _e[4]*T._e[1] + _e[8]*T._e[2];
	r2 = _e[0]*T._e[4] + _e[4]*T._e[5] + _e[8]*T._e[6];
	r3 = _e[0]*T._e[8] + _e[4]*T._e[9] + _e[8]*T._e[10];
	_e[0] = r1; _e[4] = r2; _e[8] = r3;
	r1 = _e[1]*T._e[0] + _e[5]*T._e[1] + _e[9]*T._e[2];
	r2 = _e[1]*T._e[4] + _e[5]*T._e[5] + _e[9]*T._e[6];
	r3 = _e[1]*T._e[8] + _e[5]*T._e[9] + _e[9]*T._e[10];
	_e[1] = r1; _e[5] =r2; _e[9] = r3;
	r1 = _e[2]*T._e[0] + _e[6]*T._e[1] + _e[10]*T._e[2];
	r2 = _e[2]*T._e[4] + _e[6]*T._e[5] + _e[10]*T._e[6];
	r3 = _e[2]*T._e[8] + _e[6]*T._e[9] + _e[10]*T._e[10];
	_e[2] = r1; _e[6] = r2; _e[10] = r3;
	
	return *this;
}

gXMat& gXMat::operator = (const gXMat& f)
{
	memcpy(_e,f._e,16*sizeof(gReal)); 
	return *this;
}

gTwist::gTwist(void){
	_e[0] = _e[1] = _e[2] = _e[3] = _e[4] = _e[5] = 0;
}

gTwist::gTwist(gReal w1,gReal w2,gReal w3,gReal f1,gReal f2,gReal f3) {
	_e[0] = w1; _e[1] = w2; _e[2] = w3; 
	_e[3] = f1; _e[4] = f2; _e[5] = f3; 
}

gTwist::gTwist(const gReal* d)
{
	 memcpy(_e,d,6*sizeof(gReal)); 
}

gTwist::gTwist(const gVec3& w, const gVec3& v){
	gCOPY3(_rot(),w.cptr());
	gCOPY3(_trn(),v.cptr());
}

gTwist& gTwist::operator = (const gTwist& s)			// F1 = F2
{
	memcpy(_rot(),s._e,6*sizeof(gReal));
	return *this;
}

gTwist gTwist::operator + (const gTwist& s) const		// F1+F2	
{
	gTwist re(*this);
	re += s;
	return re;
}

gTwist gTwist::operator - (const gTwist& s) const		// F1-F2
{
	gTwist re(*this);
	re -= s;
	return re;
}

gTwist& gTwist::operator += (const gTwist& s)	// F1+=F2
{
	_e[0] += s._e[0]; _e[1] += s._e[1]; _e[2] += s._e[2];
	_e[3] += s._e[3]; _e[4] += s._e[4]; _e[5] += s._e[5];
	return *this;
}

gTwist& gTwist::operator -= (const gTwist& s) // F1-=F2
{
	_e[0] -= s._e[0]; _e[1] -= s._e[1]; _e[2] -= s._e[2];
	_e[3] -= s._e[3]; _e[4] -= s._e[4]; _e[5] -= s._e[5];
	return *this;
}

gTwist gTwist::operator * (gReal d) const
{
	gTwist s(*this);
	s *= d;
	return s;
}

gTwist& gTwist::operator *= (gReal d)   // F*=d	
{
	_e[0]*=d; _e[1]*=d; _e[2]*=d; _e[3]*=d; _e[4]*=d; _e[5]*=d; 
	return *this;
}


gVec3 gTwist::multVec4 (const gVec3& p) const	// [w]*p+v
{
	return gVec3(_e[1]*p.z()-p.y()*_e[2] + _e[3], _e[2]*p.x()-_e[0]*p.z() + _e[4], _e[0]*p.y()-p.x()*_e[1] + _e[5]);
}

gVec3 gTwist::multVec3 (const gVec3& p) const	// [w]*p
{
	return gVec3(_e[1]*p.z()-p.y()*_e[2], _e[2]*p.x()-_e[0]*p.z(), _e[0]*p.y()-p.x()*_e[1]);
}

gReal gTwist::operator , (const gWrench& w)	const		// return this'w
{
	return _e[0]*w.e(0)+_e[1]*w.e(1)+_e[2]*w.e(2)+_e[3]*w.e(3)+_e[4]*w.e(4)+_e[5]*w.e(5);
}

gWrench::gWrench(void){
	_e[0] = _e[1] = _e[2] = _e[3] = _e[4] = _e[5] = 0;
}

gWrench::gWrench(gReal m1,gReal m2,gReal m3,gReal t1,gReal t2,gReal t3) {
	_e[0] = m1; _e[1] = m2; _e[2] = m3; 
	_e[3] = t1; _e[4] = t2; _e[5] = t3; 
}

gWrench::gWrench(const gVec3& m, const gVec3& t){
	_e[0] = m.x(); _e[1] = m.y(); _e[2] = m.z(); 
	_e[3] = t.x(); _e[4] = t.y(); _e[5] = t.z(); 
}

gWrench::gWrench(const gTwist& s){
	memcpy(_e,s.cptr(),6*sizeof(gReal));
}

gWrench& gWrench::operator = (const gWrench& t)			// F1 = F2
{
	memcpy(_e,t._e,6*sizeof(gReal));
	return *this;
}

gWrench gWrench::operator + (const gWrench& t) const		// F1+F2	
{
	return gWrench(_e[0]+t._e[0], _e[1]+t._e[1], _e[2]+t._e[2], 
			_e[3]+t._e[3], _e[4]+t._e[4], _e[5]+t._e[5] );
}

gWrench gWrench::operator - (const gWrench& t) const		// F1-F2
{
	return gWrench(_e[0]-t._e[0], _e[1]-t._e[1], _e[2]-t._e[2], 
			_e[3]-t._e[3], _e[4]-t._e[4], _e[5]-t._e[5] );
}

gWrench& gWrench::operator += (const gWrench& t)	// F1+=F2
{
	_e[0] += t._e[0]; _e[1] += t._e[1]; _e[2] += t._e[2];
	_e[3] += t._e[3]; _e[4] += t._e[4]; _e[5] += t._e[5];
	return *this;
}

gWrench& gWrench::operator -= (const gWrench& t) // F1-=F2
{
	_e[0] -= t._e[0]; _e[1] -= t._e[1]; _e[2] -= t._e[2];
	_e[3] -= t._e[3]; _e[4] -= t._e[4]; _e[5] -= t._e[5];
	return *this;
}

gWrench gWrench::operator * (gReal d) const
{
	return gWrench(_e[0]*d,_e[1]*d,_e[2]*d,_e[3]*d,_e[4]*d,_e[5]*d);
}

gVec3 gWrench::operator * (const gVec3& p) const	// [m]*p+f
{
	return gVec3(_e[1]*p.z()-p.y()*_e[2] + _e[3], _e[2]*p.x()-_e[0]*p.z() + _e[4], _e[0]*p.y()-p.x()*_e[1] + _e[5]);	
}

gWrench& gWrench::operator *= (gReal d)   // F*=d	
{
	_e[0]*=d; _e[1]*=d; _e[2]*=d; _e[3]*=d; _e[4]*=d; _e[5]*=d; 
	return *this;
}

gReal gWrench::operator , (const gTwist& s) const
{
	return (_e[0]*s.e(0)+_e[1]*s.e(1)+_e[2]*s.e(2)+_e[3]*s.e(3)+_e[4]*s.e(4)+_e[5]*s.e(5));
}

#ifdef MW_USE_GINERTIA_TYPE_INFO
void gInertia::sol_type(void)
{
	type = 0x00;
	if(gFabs(_I[1])+gFabs(_I[2])+gFabs(_I[4])<gEPSILON) type = TYPE_gInertia_Diag_RI;
	if(gFabs(_c[0])+gFabs(_c[1])+gFabs(_c[2])<gEPSILON) type |= TYPE_gInertia_Zero_CoM;
}
#endif

void gInertia::setRotInertia(gReal* e) 
{ 
	for(int i=0; i<6; i++) _I[i] = e[i]; 
#ifdef MW_USE_GINERTIA_TYPE_INFO
	sol_type();
#endif
}

void gInertia::setCOM(gReal x, gReal y, gReal z) 
{ 
	_c[0] = x, _c[1] = y, _c[2] = z; 
#ifdef MW_USE_GINERTIA_TYPE_INFO
	sol_type();
#endif
}

void gInertia::setMass(gReal m) 
{ 
	_m = m; 
}


void gInertia::update_K()
{
	//K = inv(_I + mass [c][c])
	
	gReal mat[6]; 
	gReal x = _m*_c[0]*_c[0];
	gReal y = _m*_c[1]*_c[1];
	gReal z = _m*_c[2]*_c[2];
	memcpy(mat,_I,6*sizeof(gReal));
	mat[0] -= (z+y);
	mat[1] +=  _m*_c[1]*_c[0];
	mat[2] +=  _m*_c[2]*_c[0];
	mat[3] -= (z+x);
	mat[4] +=  _m*_c[2]*_c[1];
	mat[5] -= (y+x);

	invSymMat33(_K,mat);

	_bK=true;
}

gInertia::gInertia()
{
	_m  = 0.;
	_I[0] = _I[3] = _I[5] = 0.f;
	_I[1] = _I[2] = _I[4] = 0.f;
	_c[0] = _c[1] = _c[2] = 0.f;
	_bK = false;

#ifdef MW_USE_GINERTIA_TYPE_INFO
	type = 0;
#endif	
}

void gInertia::setZero(void)
{
	_I[0]=_I[1]=_I[2]=_I[3]=_I[4]=_I[5]=0;
	_c[0]=_c[1]=_c[2]=0;
	_m=0;
	//K is garbaged
	_bK = false;
#ifdef MW_USE_GINERTIA_TYPE_INFO
	type = 0;
#endif	
}

gInertia::gInertia(gReal mass_, const gVec3& com, const gReal* I, int referenceFrame)
{	// com is gVec3 from zero to cm
	_m  = mass_;

	_c[0] = com.x();
	_c[1] = com.y();
	_c[2] = com.z();

	if(referenceFrame==INERTIA_REF_COM) // reference frame is at COM
	{
		_I[0] = I[0]+_m*(gSqr(_c[2])+gSqr(_c[1]));
		_I[1] = I[1]-_m*_c[1]*_c[0];
		_I[2] = I[2]-_m*_c[2]*_c[0];
		_I[3] = I[3]+_m*(gSqr(_c[2])+gSqr(_c[0]));
		_I[4] = I[4]-_m*_c[2]*_c[1];
		_I[5] = I[5]+_m*(gSqr(_c[1])+gSqr(_c[0]));	 	
	}
	else // reference frame is the body frame
	{
		_I[0] = I[0]; _I[1] = I[1]; _I[2] = I[2];
		_I[3] = I[3]; _I[4] = I[4]; _I[5] = I[5];
	}

	invSymMat33(_K,_I);
	_bK=true;

#ifdef MW_USE_GINERTIA_TYPE_INFO
	sol_type();
#endif	
}

/*
gInertia::gInertia(gReal* Ic, gReal mass_, const gXMat& T)
{
	_m = mass_;

	_c[0] = T.e(12);
	_c[1] = T.e(13);
	_c[2] = T.e(14);

	_I[0] = (T.e(0)*Ic[0]+T.e(4)*Ic[1]+T.e(8)*Ic[2]) *T.e(0)+(T.e(0)*Ic[1]+T.e(4)*Ic[3]+T.e(8)*Ic[4])*T.e(4) +(T.e(0)*Ic[2]+T.e(4)*Ic[4]+T.e(8)*Ic[5])*T.e(8);
	_I[1] = (T.e(1)*Ic[0]+T.e(5)*Ic[1]+T.e(9)*Ic[2]) *T.e(0)+(T.e(1)*Ic[1]+T.e(5)*Ic[3]+T.e(9)*Ic[4])*T.e(4) +(T.e(1)*Ic[2]+T.e(5)*Ic[4]+T.e(9)*Ic[5])*T.e(8);
	_I[2] = (T.e(2)*Ic[0]+T.e(6)*Ic[1]+T.e(10)*Ic[2])*T.e(0)+(T.e(2)*Ic[1]+T.e(6)*Ic[3]+T.e(10)*Ic[4])*T.e(4)+(T.e(2)*Ic[2]+T.e(6)*Ic[4]+T.e(10)*Ic[5])*T.e(8);
	_I[3] = (T.e(1)*Ic[0]+T.e(5)*Ic[1]+T.e(9)*Ic[2]) *T.e(1)+(T.e(1)*Ic[1]+T.e(5)*Ic[3]+T.e(9)*Ic[4])*T.e(5) +(T.e(1)*Ic[2]+T.e(5)*Ic[4]+T.e(9)*Ic[5])*T.e(9);
	_I[4] = (T.e(2)*Ic[0]+T.e(6)*Ic[1]+T.e(10)*Ic[2])*T.e(1)+(T.e(2)*Ic[1]+T.e(6)*Ic[3]+T.e(10)*Ic[4])*T.e(5)+(T.e(2)*Ic[2]+T.e(6)*Ic[4]+T.e(10)*Ic[5])*T.e(9);
	_I[5] = (T.e(2)*Ic[0]+T.e(6)*Ic[1]+T.e(10)*Ic[2])*T.e(2)+(T.e(2)*Ic[1]+T.e(6)*Ic[3]+T.e(10)*Ic[4])*T.e(6)+(T.e(2)*Ic[2]+T.e(6)*Ic[4]+T.e(10)*Ic[5])*T.e(10);

	invSymMat33(_K,_I);
	_bK=true;

	_I[0] += _m*(gSqr(T.e(14))+gSqr(T.e(13)));
	_I[1] -= _m*T.e(13)*T.e(12);
	_I[2] -= _m*T.e(14)*T.e(12);
	_I[3] += _m*(gSqr(T.e(14))+gSqr(T.e(12)));
	_I[4] -= _m*T.e(14)*T.e(13);
	_I[5] += _m*(gSqr(T.e(13))+gSqr(T.e(12)));	

#ifdef MW_USE_GINERTIA_TYPE_INFO
	sol_type();
#endif	
}
*/

void gInertia::set(gReal mass_, const gVec3& com, const gReal* I, int referenceFrame)
{
	gInertia r(mass_,com, I, referenceFrame);
	*this = r;	
}

// J*s = (_I*s.w + m*cross(c,s.v), m*s.v - m*cross(c,w))
gWrench gInertia::operator * (const gTwist& s) const 
{
	gReal m0,m1,m2,f0,f1,f2;

#ifdef MW_USE_GINERTIA_TYPE_INFO

	if(TYPE_gInertia_IS_DIAG_ROT_INERTIA(type))
	{
		if(TYPE_gInertia_IS_ZERO_COM(type))
		{
			m0 = _I[0]*s.e(0);
			m1 = _I[3]*s.e(1);
			m2 = _I[5]*s.e(2);
			f0 = _m*s.e(3);
			f1 = _m*s.e(4);
			f2 = _m*s.e(5);
		}
		else
		{
			m0 = _I[0]*s.e(0)+_m*(_c[1]*s.e(5)-_c[2]*s.e(4));
			m1 = _I[3]*s.e(1)+_m*(_c[2]*s.e(3)-_c[0]*s.e(5));
			m2 = _I[5]*s.e(2)+_m*(_c[0]*s.e(4)-_c[1]*s.e(3));
			f0 = _m*(s.e(3)-_c[1]*s.e(2)+_c[2]*s.e(1));
			f1 = _m*(s.e(4)-_c[2]*s.e(0)+_c[0]*s.e(2));
			f2 = _m*(s.e(5)-_c[0]*s.e(1)+_c[1]*s.e(0));
		}
	}
	else
	{
		if(TYPE_gInertia_IS_ZERO_COM(type))
		{
			m0 = _I[0]*s.e(0)+_I[1]*s.e(1)+_I[2]*s.e(2);
			m1 = _I[1]*s.e(0)+_I[3]*s.e(1)+_I[4]*s.e(2);
			m2 = _I[2]*s.e(0)+_I[4]*s.e(1)+_I[5]*s.e(2);
			f0 = _m*s.e(3);
			f1 = _m*s.e(4);
			f2 = _m*s.e(5);
		}
		else
		{
#endif
			m0 = _I[0]*s.e(0)+_I[1]*s.e(1)+_I[2]*s.e(2)+_m*(_c[1]*s.e(5)-_c[2]*s.e(4));
			m1 = _I[1]*s.e(0)+_I[3]*s.e(1)+_I[4]*s.e(2)+_m*(_c[2]*s.e(3)-_c[0]*s.e(5));
			m2 = _I[2]*s.e(0)+_I[4]*s.e(1)+_I[5]*s.e(2)+_m*(_c[0]*s.e(4)-_c[1]*s.e(3));
			f0 = _m*(s.e(3)-_c[1]*s.e(2)+_c[2]*s.e(1));
			f1 = _m*(s.e(4)-_c[2]*s.e(0)+_c[0]*s.e(2));
			f2 = _m*(s.e(5)-_c[0]*s.e(1)+_c[1]*s.e(0));
#ifdef MW_USE_GINERTIA_TYPE_INFO
		}
	}
#endif
	return gWrench(m0,m1,m2,f0,f1,f2);
}

//
gDInertia gInertia::operator -	(const gInertia& gI2) const
{
	assert( IsZero(_m - gI2._m) );

	gDInertia D;
	D.set_dI(0, _I[0]-gI2._I[0]);
	D.set_dI(1, _I[1]-gI2._I[1]);
	D.set_dI(2, _I[2]-gI2._I[2]);
	D.set_dI(3, _I[3]-gI2._I[3]);
	D.set_dI(4, _I[4]-gI2._I[4]);
	D.set_dI(5, _I[5]-gI2._I[5]);
	D.set_dx(0, _m*(_c[0]-gI2._c[0]));
	D.set_dx(1, _m*(_c[1]-gI2._c[1]));
	D.set_dx(2, _m*(_c[2]-gI2._c[2]));
	return D;
}

gInertia& gInertia::operator +=	(const gInertia& G2)		// this += G2
{
	gReal scale = gInv(_m+G2._m);

	_I[0]+=G2._I[0];
	_I[1]+=G2._I[1];
	_I[2]+=G2._I[2];
	_I[3]+=G2._I[3];
	_I[4]+=G2._I[4];
	_I[5]+=G2._I[5];

	_c[0] = (_m*_c[0]+G2._m*G2._c[0])*scale;
	_c[1] = (_m*_c[1]+G2._m*G2._c[1])*scale;
	_c[2] = (_m*_c[2]+G2._m*G2._c[2])*scale;

	_m += G2._m;

	_bK = false;

	return *this;
}

gInertia& gInertia::operator = (const gInertia& G2)
{
	memcpy(_I,G2._I,sizeof(gReal)*6);
	if(G2._bK)	memcpy(_K,G2._K,sizeof(gReal)*6);
	gCOPY3(_c,G2._c);
	_m = G2._m;
	_bK = G2._bK;

#ifdef MW_USE_GINERTIA_TYPE_INFO
	type = G2.type;
#endif
	return *this;
}

const gReal* gInertia::inverseRotInertia(void)
{
	if(!_bK) update_K();
	return _K;
}

/*
gDInertia gInertia::ad(const gTwist& s) const	// return (dad(s)*(this) + (this)*ad(s))
{
	gReal w0 = s.e(0);
	gReal w1 = s.e(1);
	gReal w2 = s.e(2);
	gReal v0 = s.e(3);
	gReal v1 = s.e(4);
	gReal v2 = s.e(5);

	gDInertia D;

	D.dI(0,2*(_I[1]*w2-_I[2]*w1-_m*(_c[2]*v2+_c[1]*v1)));
	D.dI(1,-_I[0]*w2+_I[2]*w0+_I[3]*w2-_I[4]*w1+_m*(_c[0]*v1+_c[1]*v0));
	D.dI(2,_I[0]*w1-_I[1]*w0+_I[4]*w2-_I[5]*w1+_m*(_c[0]*v2+_c[2]*v0));
    D.dI(3,2*(-_I[1]*w2+_I[4]*w0-_m*(_c[2]*v2+_c[0]*v0)));  
	D.dI(4,_I[1]*w1-_I[3]*w0-_I[2]*w2+_I[5]*w0+_m*(_c[1]*v2+_c[2]*v1));
	D.dI(5,2*(_I[2]*w1-_I[4]*w0-_m*(_c[1]*v1+_c[0]*v0)));
	D.set_dx(0,_m*(_c[1]*w2-v0-_c[2]*w1));
	D.set_dx(1,_m*(_c[2]*w0-v1-_c[0]*w2));
	D.set_dx(2,_m*(_c[0]*w1-v2-_c[1]*w0));

	return D;
}
*/

gReal gInertia::project(const gTwist& s) const	// return S'*(this)*S
{
	//return S'*(this)*S = w'*_I*w + _m*(v,v-cross(_c,w))
	gReal r,x[3];
	multSymMat33(x,_I,s.ptrRot());//x=_I*w
	r = gDOT(s.ptrRot(),x);		//r= w'x
	gCOPY3(x,s.ptrTrn());			//x=v
	gCROSSOP(x,-=,_c,s.ptrRot());	//x-=croos(_c,w);
	r += _m*gDOT(s.ptrTrn(),x);	//r+=_m*(v,x)
	return r;
}

gDInertia& gDInertia::operator += (const gDInertia& D)
{
	for(int i=0;i<6;++i) _dI[i] += D._dI[i];
	_dx[0] += D._dx[0];
	_dx[1] += D._dx[1];
	_dx[2] += D._dx[2];
	return *this;
}

gDInertia& gDInertia::operator -= (const gDInertia& D)
{
	for(int i=0;i<6;++i) _dI[i] -= D._dI[i];
	_dx[0] -= D._dx[0];
	_dx[1] -= D._dx[1];
	_dx[2] -= D._dx[2];
	return *this;
}

gDInertia& gDInertia::operator /= (const gReal d)
{
	for(int i=0;i<6;++i) _dI[i] /= d;
	_dx[0] /= d;
	_dx[1] /= d;
	_dx[2] /= d;
	return *this;
}

gWrench gDInertia::operator *  (const gTwist& s) const
{
	gReal m0,m1,m2,f0,f1,f2;
	m0 = _dI[0]*s.e(0)+_dI[1]*s.e(1)+_dI[2]*s.e(2)+(_dx[1]*s.e(5)-_dx[2]*s.e(4));
	m1 = _dI[1]*s.e(0)+_dI[3]*s.e(1)+_dI[4]*s.e(2)+(_dx[2]*s.e(3)-_dx[0]*s.e(5));
	m2 = _dI[2]*s.e(0)+_dI[4]*s.e(1)+_dI[5]*s.e(2)+(_dx[0]*s.e(4)-_dx[1]*s.e(3));
	f0 = -_dx[1]*s.e(2)+_dx[2]*s.e(1);
	f1 = -_dx[2]*s.e(0)+_dx[0]*s.e(2);
	f2 = -_dx[0]*s.e(1)+_dx[1]*s.e(0);
	return gWrench(m0,m1,m2,f0,f1,f2);
}


//trace(D1*D2)
gReal gDInertia::traceMult(const gDInertia& D1, const gDInertia& D2)
{
	return 
		D1.dI(0)*D2.dI(0)+D1.dI(3)*D2.dI(3)+D1.dI(5)*D2.dI(5)+
		2*(D1.dI(1)*D2.dI(1)+D1.dI(2)*D2.dI(2)+D1.dI(4)*D2.dI(4))+		
		4*(D1.dx(2)*D2.dx(2)+D1.dx(1)*D2.dx(1)+D1.dx(0)*D2.dx(0));
}

//trace(D1*D2)
gReal gDInertia::traceMultRotationInertiaOnly(const gDInertia& D1, const gDInertia& D2)
{
	//return D1.M[3]*D2.M[3]+D1.M[5]*D2.M[5];
	return 
		D1._dI[0]*D2._dI[0]+D1._dI[3]*D2._dI[3]+D1._dI[5]*D2._dI[5]+
		2*(D1._dI[1]*D2._dI[1]+D1._dI[2]*D2._dI[2]+D1._dI[4]*D2._dI[4]);
}

gVec3 gInertia::multRotInertiaWithVec3(const gVec3& v) const
{
	return gVec3(
		_I[0]*v.x()+_I[1]*v.y()+_I[2]*v.z(),
		_I[1]*v.x()+_I[3]*v.y()+_I[4]*v.z(),
		_I[2]*v.x()+_I[4]*v.y()+_I[5]*v.z());
}

//translate the reference frame by (x,y,z) wrt the current reference frame
void gInertia::translateFrame(const gVec3& v)
{
	//R*Ic*R' = cur_I + m[c][c] 
	//new_I = R*Ic*R' - m[c-p][c-p] = cur_I + m[c][c]-m[c-p][c-p]
	// = cur_I + m ( -[p][p] + [c][p] + [p][c] )
	_I[0] += _m*(-2*(v.z()*_c[2]+v.y()*_c[1])+v.z()*v.z()+v.y()*v.y());
	_I[1] += _m*(v.y()*_c[0]+v.x()*_c[1]-v.y()*v.x());
	_I[2] += _m*(v.z()*_c[0]+v.x()*_c[2]-v.z()*v.x());
	_I[3] += _m*(-2*(v.z()*_c[2]+v.x()*_c[0])+v.z()*v.z()+v.x()*v.x());
	_I[4] += _m*(v.z()*_c[1]+v.y()*_c[2]-v.z()*v.y());
	_I[5] += _m*(-2*(v.y()*_c[1]+v.x()*_c[0])+v.y()*v.y()+v.x()*v.x());
	_c[0]-=v.x(); _c[1]-=v.y(); _c[2]-=v.z();
	_bK=false;
}

gInertia gInertia::xform(const gVec3& p) const
{
	gInertia M(*this);
	M.translateFrame(-p);
	return M;
}

// J = dAd(inv(T))*L*Ad(inv(T))
// J._I = R(_I+mass[c][c])R' - mass[Rc+p][Rc+p]
// J._c = Rc+p
// if zero_com, then J._I = R*_I*R' - mass[p][p] and J.c = p
gInertia gInertia::xform(const gXMat& T) const
{
	gInertia J;
	J._bK = false;

	gReal a,b,c,d,e,f,h,j,q,u,v,w,x,y,z;
	gReal m = _m;

	//set mass
	J._m = m;

#ifdef MW_USE_GINERTIA_TYPE_INFO
	if(TYPE_gInertia_IS_ZERO_COM(type))
	{
		a = _I[0];
		b = _I[1];
		c = _I[2];
		d = _I[3];
		e = _I[4];
		f = _I[5];
		q = T.e(12);
		j = T.e(13);
		h = T.e(14);
	}
	else
	{
#endif
		a = _I[0]-m*_c[2]*_c[2]-m*_c[1]*_c[1];
		b = _I[1]+m*_c[1]*_c[0];
		c = _I[2]+m*_c[2]*_c[0];
		d = _I[3]-m*_c[2]*_c[2]-m*_c[0]*_c[0];
		e = _I[4]+m*_c[2]*_c[1];
		f = _I[5]-m*_c[1]*_c[1]-m*_c[0]*_c[0];
		q = T.e(0)*_c[0]+T.e(4)*_c[1]+T.e(8)*_c[2]+T.e(12);
		j = T.e(1)*_c[0]+T.e(5)*_c[1]+T.e(9)*_c[2]+T.e(13);
		h = T.e(2)*_c[0]+T.e(6)*_c[1]+T.e(10)*_c[2]+T.e(14);		
#ifdef MW_USE_GINERTIA_TYPE_INFO
	}
#endif

	u = T.e(0)*a+T.e(4)*b+T.e(8)*c;
	v = T.e(1)*a+T.e(5)*b+T.e(9)*c;
	w = T.e(0)*b+T.e(4)*d+T.e(8)*e;
	x = T.e(0)*c+T.e(4)*e+T.e(8)*f;
	y = T.e(1)*b+T.e(5)*d+T.e(9)*e;
	z = T.e(1)*c+T.e(5)*e+T.e(9)*f;

	// set _c[]
	J._c[0] = q;
	J._c[1] = j;
	J._c[2] = h;

	// set _I[]
	J._I[0] =  u*T.e(0)+w*T.e(4)+x*T.e(8)+m*(h*h+j*j);
	J._I[1] =  u*T.e(1)+w*T.e(5)+x*T.e(9)-m*j*q;
	J._I[2] =  u*T.e(2)+w*T.e(6)+x*T.e(10)-m*h*q;
	J._I[3] =  v*T.e(1)+y*T.e(5)+z*T.e(9)+m*(h*h+q*q);
	J._I[4] =  v*T.e(2)+y*T.e(6)+z*T.e(10)-m*h*j;
	J._I[5] =  (T.e(2)*a+T.e(6)*b+T.e(10)*c)*T.e(2)+(T.e(2)*b+T.e(6)*d+T.e(10)*e)*T.e(6)+(T.e(2)*c+T.e(6)*e+T.e(10)*f)*T.e(10)+m*(j*j+q*q);
 
#ifdef MW_USE_GINERTIA_TYPE_INFO
	// determine type
	J.sol_type();
#endif
 
	return J;
}


// solve *this = inv(J)*y
// this->w = _K*(m - cross(c,f))
// this->v = cross(c,this->w) + f/mass
void gTwist::solve(gInertia& J, const gWrench& y)
{
	gReal x[3];
	gCROSSOP(x,=,J.com(),y.ptrTrn());	// x = [c]y.f
	gELEMOP33(x,=,y.ptrRot(),-,x);	// x = y.m-x
	multSymMat33(_rot(),J.inverseRotInertia(),x);	// this.w = K*x;

	gELEMOP31(_trn(),=,y.ptrTrn(),/,J.mass()); // v=f/mass
    gCROSSOP(_trn(),+=,J.com(),ptrRot());	// v += [c]this.w
}

void gWrench::makeDAd(const gXMat& T, const gWrench& t)
{
//	tw.m = ~(T.R)*( t.m - cross(T.p,t.f) );
//	tw.f = ~(T.R)*t.f;
	gReal r[3];
	gCOPY3(r,t.ptrRot());
	gCROSSOP(r,-=,T.ptrTrn(),t.ptrTrn()); // r=m-cross(T.p,t.f)
	gTMULTIPLYOP431(_rot(),=,T.ptrRot(),r);
	gTMULTIPLYOP431(_trn(),=,T.ptrRot(),t.ptrTrn());
}
 
void gWrench::makeDLieBracket(const gTwist& s, const gWrench& t)
{
//	m = cross(t.m,w)+cross(t.f,v);
//	f = cross(t.f,w)
	gCROSSOP(_rot(),=,t.ptrRot(),s.ptrRot());
	gCROSSOP(_rot(),+=,t.ptrTrn(),s.ptrTrn());
	gCROSSOP(_trn(),=,t.ptrTrn(),s.ptrRot());
}

void gTwist::makeAd(const gXMat& f, const gTwist& s)
{
	//  Symbolic operation is as follows:
	//w = f.R * s.w;
	//v = cross(f.p , f.R * s.w)  + f.R * s.v = cross(f.p,w) + f.R*s.v

	gMULTIPLYOP431(_rot(),=,f.ptrRot(),s.ptrRot());	//w = f.R*s.w
	gMULTIPLYOP431(_trn(),=,f.ptrRot(),s.ptrTrn());	//v = f.R*s.v
	gCROSSOP(_trn(),+=,f.ptrTrn(),_e);		//v+= cross(f.p,w)
}

gTwist gTwist::xform(const gXMat& T) const
{
	gTwist r;
	r.makeAd(T,*this);
	return r;
}

gTwist gTwist::xform(const gVec3& p) const
{
	gVec3 w(rot());
	return gTwist(w,trn()+(p%w));
}

void gTwist::makeAdInv(const gXMat& f, const gTwist& s)
{
	// Ad^{-1}(f,s) is equal to Ad(f.inv(),s)
	// w  = R'*s.w
	// v  = R'*(s.v - cross(f.p,s.w))

	gReal r[3];
	gCOPY3(r,s.ptrTrn());
	gCROSSOP(r,-=,f.ptrTrn(),s.ptrRot());	//r=s.v - cross(f.p,s.w)
	gTMULTIPLYOP431(_rot(),=,f.ptrRot(),s.ptrRot());
    gTMULTIPLYOP431(_trn(),=,f.ptrRot(),r);
}

gTwist gTwist::xformInv(const gXMat& f)
{
	gTwist r;
	r.makeAdInv(f,*this);
	return r;
}

void gWrench::makeDAdInv(const gXMat& f, const gWrench& t)
{
	//f'=f.R*t.f
	//m'=f.R*t.m + cross(f.p,f')
	gMULTIPLYOP431(_trn(),=,f.ptrRot(),t.ptrTrn());
	gMULTIPLYOP431(_rot(),=,f.ptrRot(),t.ptrRot());
	gCROSSOP(_rot(),+=,f.ptrTrn(),_e+3);
}

void gTwist::makeLieBracket (const gTwist& s1, const gTwist& s2)
{	
//	w = skew(s1.w)*s2.w ;
//	v = skew(s1.v)*s2.w + skew(s1.w)*s2.v;
	gCROSSOP(_rot(),=,s1.ptrRot(),s2.ptrRot());
	gCROSSOP(_trn(),=,s1.ptrTrn(),s2.ptrRot());
	gCROSSOP(_trn(),+=,s1.ptrRot(),s2.ptrTrn());
}

gVec3 gVec3::log(const gRotMat& R)
{
	gReal x = gMin(1 , gREAL(0.5)*R.trace()-gREAL(0.5));
	x = gMax(-1, x);
	gReal angle = acos( x );	// 0 to pi
	gVec3 re(0,0,0);
	if ( angle > gEPSILON ) 
	{
		re.setFromSkewSymMatrix(R.cptr());
		re *= (angle/gSin(angle));
	}
	return re;
}

// choose w s.t. w=log(R) and close to neighbor
gVec3 gVec3::log(const gRotMat& R, const gVec3 nbor)
{
	gReal x = gMin(1 , gREAL(0.5)*R.trace()-gREAL(0.5));
	x = gMax(-1, x);
	gReal angle = acos( x );	// 0 to pi
	
	if( angle < gEPSILON )//if R=eye
	{
		if( IsZero(nbor.x()) && IsZero(nbor.y()) && IsZero(nbor.z()) )
		{
			return gVec3(0,0,0);
		}
		else
		{
			gReal len = nbor.magnitude();
			gVec3 w(nbor); w/=len;
			return w*(2*gPI*gRound(len/(2*gPI)));
		}
	}
	else
	{
		gReal s = gSin(angle);
		gVec3 w;
		w.setFromSkewSymMatrix(R.cptr());
		w*=gInv(s);

		int n = gRound( ( (nbor,w)/(w,w) - angle ) / (2*gPI) );
		gVec3 w1 = w * (2*gPI*n+angle);

		int n2 = gRound( ( (nbor,w)/(w,w) + angle ) / (2*gPI) );
		gVec3 w2 = w * (2*gPI*(n2+1)-angle);

		gVec3 wr;
		if( (nbor-w1).magnitudeSquared() > (nbor-w2).magnitudeSquared() )
		{
			wr = w2;
		} else {
			wr = w1;
		}

		return wr;
	}
}



void gRotMat::makeExp(const gVec3& s)
{
	gReal th = s.magnitude();
	if	(IsZero(th)) setIdentity();
	else
	{
		gReal ith = gInv(th);
		gReal x = s.x()*ith;
		gReal y = s.y()*ith;
		gReal z = s.z()*ith;
		gReal a = gSin(th);
		gReal b = 1-gCos(th);
		gReal c = a*z;
		gReal d = a*y;
		gReal f = a*x;
		// this = eye(3)+a*[w]+b*[w]^2
		_e[0] = 1+b*(gSqr(x)-1);
		_e[1] = b*x*y+c;
		_e[2] = b*x*z-d;
		_e[3] = _e[1]-2*c;
		_e[4] = 1+b*(gSqr(y)-1);
		_e[5] = b*z*y+f;
		_e[6] = _e[2]+2*d;
		_e[7] = _e[5]-2*f;
		_e[8] = 1+b*(gSqr(z)-1);		
	}

}

// u must be normal vector
void gRotMat::makeExp(const gVec3& u, gReal angle)
{
	if(IsZero(angle))
	{
		setIdentity();
	}
	else
	{
		gReal a = gSin(angle);
		gReal b = 1-gCos(angle);
		gReal c = a*u.z();
		gReal d = a*u.y();
		gReal f = a*u.x();
		// this = eye(3)+a*[w]+b*[w]^2
		_e[0] = 1+b*(gSqr(u.x())-1);
		_e[1] = b*u.x()*u.y()+c;
		_e[2] = b*u.x()*u.z()-d;
		_e[3] = _e[1]-2*c;
		_e[4] = 1+b*(gSqr(u.y())-1);
		_e[5] = b*u.z()*u.y()+f;
		_e[6] = _e[2]+2*d;
		_e[7] = _e[5]-2*f;
		_e[8] = 1+b*(gSqr(u.z())-1);		
	}
}

void gRotMat::makeExp(const double _x, const double _y, const double _z)
{
	gReal th = sqrt(_x*_x + _y*_y + _z*_z);

	if	(IsZero(th)) setIdentity();
	else
	{
		gReal ith = gInv(th);
		gReal x = _x*ith;
		gReal y = _y*ith;
		gReal z = _z*ith;
		gReal a = gSin(th);
		gReal b = 1-gCos(th);
		gReal c = a*z;
		gReal d = a*y;
		gReal f = a*x;
		// this = eye(3)+a*[w]+b*[w]^2
		_e[0] = 1+b*(gSqr(x)-1);
		_e[1] = b*x*y+c;
		_e[2] = b*x*z-d;
		_e[3] = _e[1]-2*c;
		_e[4] = 1+b*(gSqr(y)-1);
		_e[5] = b*z*y+f;
		_e[6] = _e[2]+2*d;
		_e[7] = _e[5]-2*f;
		_e[8] = 1+b*(gSqr(z)-1);		
	}

}

void gXMat::makeRotateX	(gReal x)				// this = exp(gVec3UnitX*x)
{
	gReal s = gSin(x);
	gReal c = gCos(x);
	_e[0]=1; _e[1]=0; _e[2]=0;
	_e[4]=0; _e[5]=c; _e[6]=s;
	_e[8]=0; _e[9]=-s;_e[10]=c;
	_e[12]=_e[13]=_e[14]=0;
}

void gXMat::makeRotateY	(gReal y)				// this = exp(gVec3UnitY*y)
{
	gReal s = gSin(y);
	gReal c = gCos(y);
	_e[0]=c; _e[1]=0; _e[2]=-s;
	_e[4]=0; _e[5]=1; _e[6]=0;
	_e[8]=s; _e[9]=0; _e[10]=c;
	_e[12]=_e[13]=_e[14]=0;

}

void gXMat::makeRotateZ	(gReal z)				// this = exp(gVec3UnitZ*z)
{
	gReal s = gSin(z);
	gReal c = gCos(z);
	_e[0]=c; _e[1]=s; _e[2]=0;
	_e[4]=-s;_e[5]=c; _e[6]=0;
	_e[8]=0; _e[9]=0; _e[10]=1;
	_e[12]=_e[13]=_e[14]=0;
}



//axis: normal vector
//angle
void gRotMat::makeRotate (const gVec3& axis, gReal angle) //this = exp(axis*angle)
{
	gReal ilen = gInv(axis.magnitude());
	makeExp(axis*ilen,angle);
}

void gRotMat::makeRotateX	(gReal x)		// this = exp(gVec3UnitX*x)
{
	gReal s = gSin(x);
	gReal c = gCos(x);
	_e[0]=1; _e[1]=0; _e[2]=0;
	_e[3]=0; _e[4]=c; _e[5]=s;
	_e[6]=0; _e[7]=-s;_e[8]=c;
}

void gRotMat::makeRotateY(gReal y)		// this = exp(gVec3UnitY*y)
{
	gReal s = gSin(y);
	gReal c = gCos(y);
	_e[0]=c; _e[1]=0; _e[2]=-s;
	_e[3]=0; _e[4]=1; _e[5]=0;
	_e[6]=s; _e[7]=0; _e[8]=c;
}
void gRotMat::makeRotateZ(gReal z)		// this = exp(gVec3UnitZ*z)
{
	gReal s = gSin(z);
	gReal c = gCos(z);
	_e[0]=c; _e[1]=s; _e[2]=0;
	_e[3]=-s;_e[4]=c; _e[5]=0;
	_e[6]=0; _e[7]=0; _e[8]=1;
}

void gTwist::makeLog(const  gXMat &T )
{
	//this code is from Jinwook's vphysics
	gReal theta, cof, x, y, z, sth;

	x = gMin(1, gREAL(0.5) * ( T.e(0) + T.e(5) + T.e(10) - 1 ) );
	x = gMax(-1, x);	// ensure x=[-1:1]
	
	theta = acos( x );  // 0 to pi
	
	if ( theta < gEPSILON ) 
	{	
		// w = 0, v = T.p
		_e[0]=_e[1]=_e[2]=0;
		_e[3]=T.e(12);
		_e[4]=T.e(13);
		_e[5]=T.e(14);
	}
	else
	{
		sth = gSin(theta); 
		cof = theta / ( 2 * sth );
		x = cof * ( T.e(6) - T.e(9) );	// w[0] <- non-normalized w
		y = cof * ( T.e(8) - T.e(2) );	// w[1]
		z = cof * ( T.e(1) - T.e(4) );	// w[2]
		//theta = gSqrt( x * x + y * y + z * z );		//necessary?
		cof = ( 2 * sth - theta * ( 1 + gCos( theta ) ) ) / ( 2 * theta * theta * sth );

		_e[0] = x;	
		_e[1] = y;	
		_e[2] = z;		
		_e[3] = ( 1 - cof * ( y * y + z * z ) ) * T.e(12) + ( gREAL(0.5) * z + cof * x * y ) * T.e(13) + ( cof * x * z - gREAL(0.5) * y ) * T.e(14);
		_e[4] = ( 1 - cof * ( x * x + z * z ) ) * T.e(13) + ( gREAL(0.5) * x + cof * z * y ) * T.e(14) + ( cof * x * y - gREAL(0.5) * z ) * T.e(12);
		_e[5] = ( 1 - cof * ( y * y + x * x ) ) * T.e(14) + ( gREAL(0.5) * y + cof * x * z ) * T.e(12) + ( cof * y * z - gREAL(0.5) * x ) * T.e(13);
	}
}

gTwist log (const gXMat& T, const gTwist& nbor)
{
	gVec3 w = gVec3::log(T.rot(),nbor.rot());

	if( IsZero(w.x()) && IsZero(w.y()) && IsZero(w.z()) )
	{
		return gTwist(w,T.trn());
	}
	else
	{
		gReal len = w.magnitude();
		gReal s = gSin(len);
		gReal a = (2*s-len*(1+gCos(len)))/(2*len*len*s);
		gVec3 p(T.trn());
		gVec3 v = p*(1-a*len*len) - (w%p)*(0.5) + w*(a*(w,p));
		return gTwist(w,v);
	}
}

void gXMat::makeExp(const gTwist& s)
{
	gReal th = gSqrt(gSqr(s.e(0))+gSqr(s.e(1))+gSqr(s.e(2)));
	if(IsZero(th)){
		_e[0] = _e[5] = _e[10] = 1;
		_e[1] = _e[2] = _e[4] = _e[6] = _e[8] = _e[9] = 0;
		_e[12] = s.e(3);
		_e[13] = s.e(4);
		_e[14] = s.e(5);
	}
	else{
		gReal ith = gInv(th);
		gReal w0 = s.e(0)*ith;
		gReal w1 = s.e(1)*ith;
		gReal w2 = s.e(2)*ith;
		gReal v0 = s.e(3)*ith;
		gReal v1 = s.e(4)*ith;
		gReal v2 = s.e(5)*ith;
		gReal a = gSin(th);
		gReal b = 1-gCos(th);		
		gReal d = th*(w0*v0+w1*v1+w2*v2);
		gReal f = w2*v1-w1*v2;
		gReal g = w0*v2-w2*v0;
		gReal h = w1*v0-w0*v1;
		gReal i = a*w2;
		gReal j = a*w1;
		gReal k = a*w0;
		_e[0] = b*(gSqr(w0)-1);
		_e[1] = b*w0*w1+i;
		_e[2] = b*w0*w2-j;
		_e[4] = _e[1]-2*i;
		_e[5] = b*(gSqr(w1)-1);
		_e[6] = b*w2*w1+k;
		_e[8] = _e[2]+2*j;
		_e[9] = _e[6]-2*k;
		_e[10] = b*(gSqr(w2)-1);
		_e[12] = _e[0]*f+_e[4]*g+_e[8]*h+d*w0;
		_e[13] = _e[1]*f+_e[5]*g+_e[9]*h+d*w1;
		_e[14] = _e[2]*f+_e[6]*g+_e[10]*h+d*w2;
		_e[0]+=1;
		_e[5]+=1;
		_e[10]+=1;
	}

}


//TODO: separate this function to pure (normal) rotation and pure translation case
void gXMat::makeExp(const gTwist& s, gReal th) 
{
	//s.rot() must be normal or zero vector
	if(IsZero(th))
	{
		setIdentity();
	}
	else if ( IsZero(s.e(0)) && IsZero(s.e(1)) && IsZero(s.e(2)) ) //pure translation
	{
		_e[0] = _e[5] = _e[10] = 1;
		_e[1] = _e[2] = _e[4] = _e[6] = _e[8] = _e[9] = 0;
		_e[12] = s.e(3)*th;
		_e[13] = s.e(4)*th;
		_e[14] = s.e(5)*th;
	}
	else
	{
		gReal a = gSin(th);
		gReal b = 1-gCos(th);		
		gReal d = th*gDOT(s.ptrRot(),s.ptrTrn());
		gReal f = s.e(2)*s.e(4)-s.e(1)*s.e(5);
		gReal g = s.e(0)*s.e(5)-s.e(2)*s.e(3);
		gReal h = s.e(1)*s.e(3)-s.e(0)*s.e(4);
		_e[0] = b*(gSqr(s.e(0))-1);
		_e[1] = b*s.e(0)*s.e(1)+a*s.e(2);
		_e[2] = b*s.e(0)*s.e(2)-a*s.e(1);
		_e[4] = _e[1]-2*a*s.e(2);
		_e[5] = b*(gSqr(s.e(1))-1);
		_e[6] = b*s.e(2)*s.e(1)+a*s.e(0);
		_e[8] = _e[2]+2*a*s.e(1);
		_e[9] = _e[6]-2*a*s.e(0);
		_e[10] = b*(gSqr(s.e(2))-1);
		_e[12] = _e[0]*f+_e[4]*g+_e[8]*h+d*s.e(0);
		_e[13] = _e[1]*f+_e[5]*g+_e[9]*h+d*s.e(1);
		_e[14] = _e[2]*f+_e[6]*g+_e[10]*h+d*s.e(2);
		_e[0]+=1;
		_e[5]+=1;
		_e[10]+=1;
	}
}



//resulting matrix = exp(gVec3UnitX*x)*exp(gVec3UnitY*y)*exp(gVec3UnitZ*z)
void gRotMat::makeRotateXYZ(gReal x, gReal y, gReal z)
{
	gReal sa=gSin(x);
	gReal ca=gCos(x);
	gReal sb=gSin(y);
	gReal cb=gCos(y);
	gReal sc=gSin(z);
	gReal cc=gCos(z);

	_e[0]=cb*cc;
	_e[1]=sa*sb*cc+ca*sc;
	_e[2]=-ca*sb*cc+sa*sc;
	_e[3]=-cb*sc;
	_e[4]=-sa*sb*sc+ca*cc;
	_e[5]=ca*sb*sc+sa*cc;
	_e[6]=sb;
	_e[7]=-sa*cb;
	_e[8]=ca*cb;
}


//resulting matrix = exp(gVec3UnitZ*z)*exp(gVec3UnitY*y)*exp(gVec3UnitX*x)
void gRotMat::makeRotateZYX(gReal z, gReal y, gReal x)
{
	gReal sc=gSin(z);
	gReal cc=gCos(z);
	gReal sb=gSin(y);
	gReal cb=gCos(y);
	gReal sa=gSin(x);
	gReal ca=gCos(x);
	_e[0]=cb*cc;
	_e[1]=cb*sc;
	_e[2]=-sb;
	_e[3]=-ca*sc+sa*sb*cc;
	_e[4]=ca*cc+sa*sb*sc;
	_e[5]=sa*cb;
	_e[6]=sa*sc+ca*sb*cc;
	_e[7]=-sa*cc+ca*sb*sc;
	_e[8]=ca*cb;
}

void gRotMat::makeRotateZXY(gReal z, gReal x, gReal y)
{
 gReal sc=gSin(z);
 gReal cc=gCos(z);
 gReal sa=gSin(x);
 gReal ca=gCos(x);
 gReal sb=gSin(y);
 gReal cb=gCos(y);
 
 _e[0] = cb * cc - sa * sb * sc;
 _e[1] = cb * sc + sa * sb * cc;
 _e[2] = -ca * sb;
 _e[3] = -ca * sc;
 _e[4] = ca * cc;
 _e[5] = sa;
 _e[6] = sb * cc + sa * cb * sc;
 _e[7] = sb * sc - sa * cb * cc;
 _e[8] = ca * cb;
}


// Modify: 2014-05-12: SukwonLee 
void  gRotMat::makeRotateXZY (gReal x, gReal z, gReal y)
{
 gReal sc=gSin(z);
 gReal cc=gCos(z);
 gReal sa=gSin(x);
 gReal ca=gCos(x);
 gReal sb=gSin(y);
 gReal cb=gCos(y);
 
 _e[0] = cb * cc;
 _e[1] = sa * sb + ca * cb * cc;
 _e[2] = -ca * sb + cb * sa * sc;
 _e[3] = -sc;
 _e[4] = ca * cc;
 _e[5] = cc * sa;
 _e[6] = cc * sb;
 _e[7] = -cb * sa + ca * sb * sc;
 _e[8] = ca * cb + sa * sb * sc;
}

void  gRotMat::makeRotateYXZ(gReal y, gReal x, gReal z)
{
 gReal sc=gSin(z);
 gReal cc=gCos(z);
 gReal sa=gSin(x);
 gReal ca=gCos(x);
 gReal sb=gSin(y);
 gReal cb=gCos(y);
 
 _e[0] = cb * cc + sa * sb * sc;
 _e[1] = ca * sc;
 _e[2] = -ca * sb + cb * sa * sc;
 _e[3] = cc * sa * sb - cb * sc;
 _e[4] = ca * cc;
 _e[5] = cb * cc * sa + sb * sc;
 _e[6] = ca * sb;
 _e[7] = - sa;
 _e[8] = ca * cb;
}

void  gRotMat::makeRotateYZX (gReal y, gReal z, gReal x)
{
 gReal sc=gSin(z);
 gReal cc=gCos(z);
 gReal sa=gSin(x);
 gReal ca=gCos(x);
 gReal sb=gSin(y);
 gReal cb=gCos(y);
 
 _e[0] = cb * cc;
 _e[1] = sc;
 _e[2] = -cc * sb;
 _e[3] = sa * sb - ca * cb * sc;
 _e[4] = ca * cc;
 _e[5] = cb * sa + ca * sb * sc;
 _e[6] = ca * sb + cb * sa * sc;
 _e[7] = -cc * sa;
 _e[8] = ca * cb - sa * sb * sc;
}

gQuat gXMat::rotInQuat(void) const
{
	/* from Shoemake's tutorial on quaternions...
	* This algorithm avoids near-zero divides by looking for a large component
	* first w, then x, y, or z. When the trace is greater than zero,
	* |w| is greater than 1/2, which is as small as a largest component can be.
	* Otherwise, the largest diagonal entry corresponds to the largest of |x|,
	* |y|, or |z|, one of which must be larger than |w|, and at least 1/2. */
	gReal tr, s, x, y, z, w;
	tr =  _e[0] + _e[5] + _e[10]; 
	if (tr >= gREAL(0.0)) {
		s = gSqrt(tr + 1);
		w = s*gREAL(0.5);
		s = 0.5 / s;
		x = (_e[6] - _e[9]) * s;
		y = (_e[8] - _e[2]) * s;
		z = (_e[1] - _e[4]) * s;
	} 
	else 
	{
		// find largest in x,y,z
		int h = 0; // X
		if (_e[5] > _e[h]) h = 4; // Y
		if (_e[10] > _e[h]) h = 8; // Z
		switch(h)
		{
		case 0:	// X major
			s = gSqrt( 1 + _e[0] - _e[5] - _e[10] );
			x = s*gREAL(0.5);
			s = gREAL(0.5) / s;
			y = (_e[4] + _e[1]) * s;
			z = (_e[2] + _e[8]) * s;
			w = (_e[6] - _e[9]) * s;
			break;
		case 4: // Y major
			s = gSqrt( 1 - _e[0] + _e[5] - _e[10] );
			y = s*gREAL(0.5);
			s = gREAL(0.5) / s;
			x = (_e[4] + _e[1]) * s;
			z = (_e[6] + _e[9]) * s;
			w = (_e[8] - _e[2]) * s;
			break;
		case 8: // Z major
			s = gSqrt( 1 - _e[0] - _e[5] + _e[10] );
			z = s*gREAL(0.5);
			s = gREAL(0.5) / s;
			x = (_e[2] + _e[8]) * s;
			y = (_e[6] + _e[9]) * s;
			w = (_e[1] - _e[4]) * s;
			break;
		}
	}
	return gQuat(x,y,z,w);
}

gQuat gRotMat::inQuat	(void) const 
{ 
	/* from Shoemake's tutorial on quaternions...
	* This algorithm avoids near-zero divides by looking for a large component
	* first w, then x, y, or z. When the trace is greater than zero,
	* |w| is greater than 1/2, which is as small as a largest component can be.
	* Otherwise, the largest diagonal entry corresponds to the largest of |x|,
	* |y|, or |z|, one of which must be larger than |w|, and at least 1/2. */
	gReal tr, s, x, y, z, w;
	tr =  _e[0] + _e[4] + _e[8]; 
	if (tr >= gREAL(0.0)) {
		s = gSqrt(tr + 1);
		w = s*gREAL(0.5);
		s = 0.5 / s;
		x = (_e[5] - _e[7]) * s;
		y = (_e[6] - _e[2]) * s;
		z = (_e[1] - _e[3]) * s;
	} 
	else 
	{
		// find largest in x,y,z
		int h = 0; // X
		if (_e[4] > _e[h]) h = 4; // Y
		if (_e[8] > _e[h]) h = 8; // Z
		switch(h)
		{
		case 0:	// X major
			s = gSqrt( 1 + _e[0] - _e[4] - _e[8] );
			x = s*gREAL(0.5);
			s = gREAL(0.5) / s;
			y = (_e[3] + _e[1]) * s;
			z = (_e[2] + _e[6]) * s;
			w = (_e[5] - _e[7]) * s;
			break;
		case 4: // Y major
			s = gSqrt( 1 - _e[0] + _e[4] - _e[8] );
			y = s*gREAL(0.5);
			s = gREAL(0.5) / s;
			x = (_e[3] + _e[1]) * s;
			z = (_e[5] + _e[7]) * s;
			w = (_e[6] - _e[2]) * s;
			break;
		case 8: // Z major
			s = gSqrt( 1 - _e[0] - _e[4] + _e[8] );
			z = s*gREAL(0.5);
			s = gREAL(0.5) / s;
			x = (_e[2] + _e[6]) * s;
			y = (_e[5] + _e[7]) * s;
			w = (_e[1] - _e[3]) * s;
			break;
		}
	}
	return gQuat(x,y,z,w);
}

gDXMat& gDXMat::operator = (const gDXMat& s)
{
	memcpy(_dR,s._dR,9*sizeof(gReal));
	_dp[0]=s._dp[0];_dp[1]=s._dp[1];_dp[2]=s._dp[2];
	return (*this);
}

gDXMat& gDXMat::operator +=(const gDXMat& s)
{
	_dR[0]+=s._dR[0];_dR[1]+=s._dR[1];_dR[2]+=s._dR[2];
	_dR[3]+=s._dR[3];_dR[4]+=s._dR[4];_dR[5]+=s._dR[5];
	_dR[6]+=s._dR[6];_dR[7]+=s._dR[7];_dR[8]+=s._dR[8];
	_dp[0]+=s._dp[0];_dp[1]+=s._dp[1];_dp[2]+=s._dp[2];
	return (*this);
}

gDXMat& gDXMat::operator -=(const gDXMat& s)
{
	_dR[0]-=s._dR[0];_dR[1]-=s._dR[1];_dR[2]-=s._dR[2];
	_dR[3]-=s._dR[3];_dR[4]-=s._dR[4];_dR[5]-=s._dR[5];
	_dR[6]-=s._dR[6];_dR[7]-=s._dR[7];_dR[8]-=s._dR[8];
	_dp[0]-=s._dp[0];_dp[1]-=s._dp[1];_dp[2]-=s._dp[2];
	return (*this);
}

gDXMat& gDXMat::operator *=(gReal scale) 
{
	_dR[0]*=scale;_dR[1]*=scale;_dR[2]*=scale;
	_dR[3]*=scale;_dR[4]*=scale;_dR[5]*=scale;
	_dR[6]*=scale;_dR[7]*=scale;_dR[8]*=scale;
	_dp[0]*=scale;_dp[1]*=scale;_dp[2]*=scale;	
	return (*this);
}

//gTwist = inv(T) * this
//(w',v')' = (R',-R'p)*(_dR,dp)
//so, v=R'dp, [w]=R'_dR
gTwist gDXMat::preMultInverseOf(const gXMat& T)	const 
{
	gReal w0,w1,w2,v0,v1,v2;
	w0=gREAL(0.5)*((T.e(8)*_dR[3]+T.e(9)*_dR[4]+T.e(10)*_dR[5]) - (T.e(4)*_dR[6]+T.e(5)*_dR[7]+T.e(6)*_dR[8]));
	w1=gREAL(0.5)*((T.e(0)*_dR[6]+T.e(1)*_dR[7]+T.e(2)*_dR[8]) - (T.e(8)*_dR[0]+T.e(9)*_dR[1]+T.e(10)*_dR[2]));
	w2=gREAL(0.5)*((T.e(4)*_dR[0]+T.e(5)*_dR[1]+T.e(6)*_dR[2]) - (T.e(0)*_dR[3]+T.e(1)*_dR[4]+T.e(2)*_dR[5]));
	v0= T.e(0)*_dp[0]+T.e(1)*_dp[1]+T.e(2)*_dp[2];
	v1= T.e(4)*_dp[0]+T.e(5)*_dp[1]+T.e(6)*_dp[2];
	v2= T.e(8)*_dp[0]+T.e(9)*_dp[1]+T.e(10)*_dp[2];
	return gTwist(w0,w1,w2,v0,v1,v2);
}

gDXMat::gDXMat(const gXMat& T1, const gXMat& T2, gReal scale)
{
	_dR[0]=(T1.e(0) -T2.e(0))*scale;
	_dR[1]=(T1.e(1) -T2.e(1))*scale;
	_dR[2]=(T1.e(2) -T2.e(2))*scale;
	_dR[3]=(T1.e(4) -T2.e(4))*scale;
	_dR[4]=(T1.e(5) -T2.e(5))*scale;
	_dR[5]=(T1.e(6) -T2.e(6))*scale;
	_dR[6]=(T1.e(8) -T2.e(8))*scale;
	_dR[7]=(T1.e(9) -T2.e(9))*scale;
	_dR[8]=(T1.e(10)-T2.e(10))*scale;
	_dp[0]=(T1.e(12)-T2.e(12))*scale;
	_dp[1]=(T1.e(13)-T2.e(13))*scale;
	_dp[2]=(T1.e(14)-T2.e(14))*scale;
}

bool gRotMat::isRotMat(const gXMat& T)	// check if T.rot() is gRotMat
{
	if (  // A'*A == I   
		 IsZero( T.e(0)*T.e(0)+T.e(1)*T.e(1)+T.e(2)*T.e(2)-gREAL(1.0) )
		 &&
		 IsZero( T.e(4)*T.e(4)+T.e(5)*T.e(5)+T.e(6)*T.e(6)-gREAL(1.0) )
		 &&
		 IsZero( T.e(8)*T.e(8)+T.e(9)*T.e(9)+T.e(10)*T.e(10)-gREAL(1.0) )
		 &&
		 IsZero( T.e(0)*T.e(4)+T.e(1)*T.e(5)+T.e(2)*T.e(6) )
		 &&
		 IsZero( T.e(0)*T.e(8)+T.e(1)*T.e(9)+T.e(2)*T.e(10) )
		 &&
		 IsZero( T.e(4)*T.e(8)+T.e(5)*T.e(9)+T.e(6)*T.e(10) )
		 &&	// determinant == 1
		 IsZero( T.e(0)*( T.e(5)*T.e(10) - T.e(9)*T.e(6) ) - T.e(1)*( T.e(4)*T.e(10) - T.e(8)*T.e(6) ) + T.e(2)*( T.e(4)*T.e(9) - T.e(8)*T.e(5) ) - gREAL(1.0) )  
		 )
		 return true;
	else 
		return false;
}

bool gRotMat::isRotMat(const gReal* a)
{
	if (  // A'*A == I   
		 IsZero( a[0]*a[0]+a[1]*a[1]+a[2]*a[2]-gREAL(1.0) )
		 &&
		 IsZero( a[3]*a[3]+a[4]*a[4]+a[5]*a[5]-gREAL(1.0) )
		 &&
		 IsZero( a[6]*a[6]+a[7]*a[7]+a[8]*a[8]-gREAL(1.0) )
		 &&
		 IsZero( a[0]*a[3]+a[1]*a[4]+a[2]*a[5] )
		 &&
		 IsZero( a[0]*a[6]+a[1]*a[7]+a[2]*a[8] )
		 &&
		 IsZero( a[3]*a[6]+a[4]*a[7]+a[5]*a[8] )
		 &&	// determinant == 1
		 IsZero( a[0]*( a[4]*a[8] - a[7]*a[5] ) - a[1]*( a[3]*a[8] - a[6]*a[5] ) + a[2]*( a[3]*a[7] - a[6]*a[4] ) - gREAL(1.0) )  
		 )
		 return true;
	else 
		return false;
}

std::istream & operator >> (std::istream &is, gVec3 &op)
{
	is >> op._e[0] >> op._e[1] >> op._e[2];
	return (is);
}

std::ostream & operator << (std::ostream &os, const gVec3 &op)
{
	os << op.e(0) << " " << op.e(1) << " " << op.e(2);
	return (os);
}

std::ostream& operator<< ( std::ostream& os, const gRotMat& R )
{
	os << R.e(0) << " " << R.e(1) << " " << R.e(2) << " ";
	os << R.e(3) <<" "  << R.e(4) << " " << R.e(5) << " ";
	os << R.e(6) << " " << R.e(7) << " " << R.e(8) ;
	return os;
}

std::ostream & operator << (std::ostream &os, const gQuat &op)
{
	os << op.x() << " " << op.y() << " " << op.z() << " " << op.w() ;
	return os;
}

std::ostream& operator<< ( std::ostream& os, const gXMat& s )
{
	os <<s.e(0)<<" "<<s.e(1)<<" "<<s.e(2)<<" "
		<<s.e(4)<<" "<<s.e(5)<<" "<<s.e(6)<<" "
		<<s.e(8)<<" "<<s.e(9)<<" "<<s.e(10)<<" "
		<<s.e(12)<<" "<<s.e(13)<<" "<<s.e(14);
	return os;
};

std::istream& operator>> ( std::istream& is, gXMat& s )
{
	is >>s._e[0]>>s._e[1]>>s._e[2];
	is >>s._e[4]>>s._e[5]>>s._e[6];
	is >>s._e[8]>>s._e[9]>>s._e[10];
	is >>s._e[12]>>s._e[13]>>s._e[14];	
	return is;
};

std::ostream& operator << (std::ostream& os, const gTwist& s){
	os  << s.e(0) <<" " << s.e(1) <<" " << s.e(2) <<" "
		<< s.e(3) <<" " << s.e(4) <<" " << s.e(5);
	return os;
}

std::istream& operator >> (std::istream& is, gTwist& s)
{
	is  >> s._e[0] >> s._e[1] >> s._e[2] >> s._e[3] >> s._e[4] >> s._e[5];
	return is;
}

std::ostream& operator << (std::ostream& os, const gWrench& w){
	os  << w.e(0) <<" " << w.e(1) <<" " << w.e(2) <<" "
		<< w.e(3) <<" " << w.e(4) <<" " << w.e(5);
	return os;
}

std::istream& operator >> (std::istream& is, gWrench& w)
{
	is  >> w._e[0] >> w._e[1] >> w._e[2] >> w._e[3] >> w._e[4] >> w._e[5];
	return is;
}

std::ostream& operator << (std::ostream& os, const gInertia& J){
	os	<< J.mass() <<" "
		<< J.rotInertia(0) << " "<< J.rotInertia(1) << " "<< J.rotInertia(2) << " "
		<< J.rotInertia(3) << " "<< J.rotInertia(4) << " "<< J.rotInertia(5) << " "
		<< J.com(0)<< " "<< J.com(1)<< " "<< J.com(2);
	return os;
}

std::istream& operator >> (std::istream& is, gInertia& J)
{
	is  >> J._m  
		>> J._I[0] >> J._I[1] >> J._I[2] >> J._I[3] >> J._I[4] >> J._I[5] 
		>> J._c[0] >> J._c[1] >> J._c[2];
	return is;
}

std::ostream & operator << (std::ostream &os, const gAInertia &op)
{
	os <<op.e(0,0)<<" "<<op.e(0,1)<<" "<<op.e(0,2)<<" "<<op.e(0,3)<<" "<<op.e(0,4)<<" "<<op.e(0,5)<< " ";
	os <<op.e(1,1)<<" "<<op.e(1,2)<<" "<<op.e(1,3)<<" "<<op.e(1,4)<<" "<<op.e(1,5)<< " ";
	os <<op.e(2,2)<<" "<<op.e(2,3)<<" "<<op.e(2,4)<<" "<<op.e(2,5)<< " ";
	os <<op.e(3,3)<<" "<<op.e(3,4)<<" "<<op.e(3,5)<<" ";
	os <<op.e(4,4)<<" "<<op.e(4,5)<< " ";
	os <<op.e(5,5)<<" ";
	return os;
}

gAInertia& gAInertia::operator = (const gAInertia& A)
{
	int i,j;
	for(i=0;i<6;++i){
		for(j=i;j<6;++j) _e[i][j] = A._e[i][j];
	}
	return *this;
}

gAInertia& gAInertia::operator += (const gAInertia& A)
{
	int i,j;
	for(i=0;i<6;++i){
		for(j=i;j<6;++j) _e[i][j] += A._e[i][j];
	}
	return *this;
}

gAInertia& gAInertia::operator -= (const gAInertia& A)
{
	int i,j;
	for(i=0;i<6;++i){
		for(j=i;j<6;++j) _e[i][j] -= A._e[i][j];
	}
	return *this;
}

gAInertia& gAInertia::operator += (const gInertia& G)
{
	gReal r0,r1,r2;
	r0=G.mass()*G.com(0);
	r1=G.mass()*G.com(1);
	r2=G.mass()*G.com(2);

	_e[0][0] += G.rotInertia(0);
	_e[0][1] += G.rotInertia(1);
	_e[0][2] += G.rotInertia(2);
	_e[0][4] -= r2;
	_e[0][5] += r1;
	_e[1][1] += G.rotInertia(3);
	_e[1][2] += G.rotInertia(4);
	_e[1][3] += r2;
	_e[1][5] -= r0;
    _e[2][2] += G.rotInertia(5);
	_e[2][3] -= r1;
	_e[2][4] += r0;
	_e[3][3] += G.mass();
	_e[4][4] += G.mass();
	_e[5][5] += G.mass();
	return *this;
}

gAInertia& gAInertia::operator = (const gInertia& G)
{
	gReal r0,r1,r2;
	r0=G.mass()*G.com(0);
	r1=G.mass()*G.com(1);
	r2=G.mass()*G.com(2);

	_e[0][0] = G.rotInertia(0);
	_e[0][1] = G.rotInertia(1);
	_e[0][2] = G.rotInertia(2);
	_e[0][3] = 0;
	_e[0][4] = -r2;
	_e[0][5] = r1;
	_e[1][1] = G.rotInertia(3);
	_e[1][2] = G.rotInertia(4);
	_e[1][3] = r2;
	_e[1][4] = 0;
	_e[1][5] = -r0;
	_e[2][2] = G.rotInertia(5);
	_e[2][3] = -r1;
	_e[2][4] = r0;
	_e[2][5] = 0;
	_e[3][3] = G.mass();
	_e[3][4] = 0;
	_e[3][5] = 0;
	_e[4][4] = G.mass();
	_e[4][5] = 0;		
	_e[5][5] = G.mass();
	return *this;
}

void gAInertia::add(const gAInertia& A, const gXMat& T) // (this) += dAd(inv(T))*A*Ad(inv(T))
{	

	/*
	// Let A=[ B C ; C' D ], where B=B', D=D'
	// Let P = dAd(inv(T))*A*Ad(inv(T)) = [H I ; I' J]
	//
	// H = RBR'+[p]RC'R'-RCR'[p]-[p]RDR'[p]
	// I = RCR'+[p]RDR'
	// J = RDR'
	//
	// Let  X = RDR', Y = RCR', Z = RBR'. And X=X', Z=Z'
	// Then
	// J = X 
	// I = Y + [p]J
	// H = Z + [p]Y - I[p]

	J00 = (r0*a33+r3*a43+r6*a53)*r0+(r0*a43+r3*a44+r6*a54)*r3+(r0*a53+r3*a54+r6*a55)*r6;
	J01 = (r0*a33+r3*a43+r6*a53)*r1+(r0*a43+r3*a44+r6*a54)*r4+(r0*a53+r3*a54+r6*a55)*r7;
	J02 = (r0*a33+r3*a43+r6*a53)*r2+(r0*a43+r3*a44+r6*a54)*r5+(r0*a53+r3*a54+r6*a55)*r8;
	J11 = (r1*a33+r4*a43+r7*a53)*r1+(r1*a43+r4*a44+r7*a54)*r4+(r1*a53+r4*a54+r7*a55)*r7;
	J12 = (r1*a33+r4*a43+r7*a53)*r2+(r1*a43+r4*a44+r7*a54)*r5+(r1*a53+r4*a54+r7*a55)*r8;
	J22 = (r2*a33+r5*a43+r8*a53)*r2+(r2*a43+r5*a44+r8*a54)*r5+(r2*a53+r5*a54+r8*a55)*r8;

	Y00 = (r0*a30+r3*a31+r6*a32)*r0+(r0*a40+r3*a41+r6*a42)*r3+(r0*a50+r3*a51+r6*a52)*r6;
	Y01 = (r0*a30+r3*a31+r6*a32)*r1+(r0*a40+r3*a41+r6*a42)*r4+(r0*a50+r3*a51+r6*a52)*r7;
	Y02 = (r0*a30+r3*a31+r6*a32)*r2+(r0*a40+r3*a41+r6*a42)*r5+(r0*a50+r3*a51+r6*a52)*r8;
	Y10 = (r1*a30+r4*a31+r7*a32)*r0+(r1*a40+r4*a41+r7*a42)*r3+(r1*a50+r4*a51+r7*a52)*r6;
	Y11 = (r1*a30+r4*a31+r7*a32)*r1+(r1*a40+r4*a41+r7*a42)*r4+(r1*a50+r4*a51+r7*a52)*r7;
	Y12 = (r1*a30+r4*a31+r7*a32)*r2+(r1*a40+r4*a41+r7*a42)*r5+(r1*a50+r4*a51+r7*a52)*r8;
	Y20 = (r2*a30+r5*a31+r8*a32)*r0+(r2*a40+r5*a41+r8*a42)*r3+(r2*a50+r5*a51+r8*a52)*r6;
	Y21 = (r2*a30+r5*a31+r8*a32)*r1+(r2*a40+r5*a41+r8*a42)*r4+(r2*a50+r5*a51+r8*a52)*r7;
	Y22 = (r2*a30+r5*a31+r8*a32)*r2+(r2*a40+r5*a41+r8*a42)*r5+(r2*a50+r5*a51+r8*a52)*r8;

	Z00 = (r0*a00+r3*a10+r6*a20)*r0+(r0*a10+r3*a11+r6*a21)*r3+(r0*a20+r3*a21+r6*a22)*r6;
	Z01 = (r0*a00+r3*a10+r6*a20)*r1+(r0*a10+r3*a11+r6*a21)*r4+(r0*a20+r3*a21+r6*a22)*r7;
	Z02 = (r0*a00+r3*a10+r6*a20)*r2+(r0*a10+r3*a11+r6*a21)*r5+(r0*a20+r3*a21+r6*a22)*r8;
	Z11 = (r1*a00+r4*a10+r7*a20)*r1+(r1*a10+r4*a11+r7*a21)*r4+(r1*a20+r4*a21+r7*a22)*r7;
	Z12 = (r1*a00+r4*a10+r7*a20)*r2+(r1*a10+r4*a11+r7*a21)*r5+(r1*a20+r4*a21+r7*a22)*r8;
	Z22 = (r2*a00+r5*a10+r8*a20)*r2+(r2*a10+r5*a11+r8*a21)*r5+(r2*a20+r5*a21+r8*a22)*r8;

	I00 = y00-p2*j01+p1*j02;
	I01 = y01-p2*j11+p1*j12;
	I02 = y02-p2*j12+p1*j22;
	I10 = y10+p2*j00-p0*j02;
	I11 = y11+p2*j01-p0*j12;
	I12 = y12+p2*j02-p0*j22;
	I20 = y20-p1*j00+p0*j01;
	I21 = y21-p1*j01+p0*j11;
	I22 = y22-p1*j02+p0*j12;

	H00 = z00-p2*y10+p1*y20-i01*p2+i02*p1;
	H01 = z01-p2*y11+p1*y21+i00*p2-i02*p0;
	H02 = z02-p2*y12+p1*y22-i00*p1+i01*p0;
	H11 = z11+p2*y01-p0*y21+i10*p2-i12*p0;
	H12 = z12+p2*y02-p0*y22-i10*p1+i11*p0;
	H22 = z22-p1*y02+p0*y12-i20*p1+i21*p0;

	_e[0][0] += H00;
	_e[0][1] += H01;
	_e[0][2] += H02;
	_e[0][3] += I00;
	_e[0][4] += I01;
	_e[0][5] += I02;
	_e[1][1] += H11;
	_e[1][2] += H12;
	_e[1][3] += I10;
	_e[1][4] += I11;
	_e[1][5] += I12;
	_e[2][2] += H22;
	_e[2][3] += I20;
	_e[2][4] += I21;
	_e[2][5] += I22;
	_e[3][3] += J00;
	_e[3][4] += J01;
	_e[3][5] += J02;
	_e[4][4] += J11;
	_e[4][5] += J12;
	_e[5][5] += J22;
	*/


	gReal d0=(T.e(0)*A.e(0,3)+T.e(4)*A.e(0,4)+T.e(8)*A.e(0,5));
	gReal d1=(T.e(0)*A.e(3,3)+T.e(4)*A.e(3,4)+T.e(8)*A.e(3,5));
	gReal d2=(T.e(1)*A.e(0,3)+T.e(5)*A.e(0,4)+T.e(9)*A.e(0,5));
	gReal d3=(T.e(1)*A.e(3,3)+T.e(5)*A.e(3,4)+T.e(9)*A.e(3,5));
	gReal d4=(T.e(2)*A.e(0,3)+T.e(6)*A.e(0,4)+T.e(10)*A.e(0,5));
	gReal d5=(T.e(2)*A.e(3,3)+T.e(6)*A.e(3,4)+T.e(10)*A.e(3,5));
	gReal d6=(T.e(0)*A.e(1,3)+T.e(4)*A.e(1,4)+T.e(8)*A.e(1,5));
	gReal d7=(T.e(0)*A.e(3,4)+T.e(4)*A.e(4,4)+T.e(8)*A.e(4,5));
	gReal d8=(T.e(1)*A.e(1,3)+T.e(5)*A.e(1,4)+T.e(9)*A.e(1,5));
	gReal d9=(T.e(1)*A.e(3,4)+T.e(5)*A.e(4,4)+T.e(9)*A.e(4,5));
	gReal f0=(T.e(0)*A.e(2,3)+T.e(4)*A.e(2,4)+T.e(8)*A.e(2,5));
	gReal f1=(T.e(0)*A.e(3,5)+T.e(4)*A.e(4,5)+T.e(8)*A.e(5,5));
	gReal f2=(T.e(1)*A.e(2,3)+T.e(5)*A.e(2,4)+T.e(9)*A.e(2,5));
	gReal f3=(T.e(1)*A.e(3,5)+T.e(5)*A.e(4,5)+T.e(9)*A.e(5,5));
	gReal f4=(T.e(2)*A.e(1,3)+T.e(6)*A.e(1,4)+T.e(10)*A.e(1,5));
	gReal f5=(T.e(2)*A.e(2,3)+T.e(6)*A.e(2,4)+T.e(10)*A.e(2,5));
	gReal f6=(T.e(2)*A.e(3,4)+T.e(6)*A.e(4,4)+T.e(10)*A.e(4,5));
	gReal f7=(T.e(2)*A.e(3,5)+T.e(6)*A.e(4,5)+T.e(10)*A.e(5,5));
	gReal x0=(-T.e(14)*T.e(1)+T.e(13)*T.e(2));
	gReal x1=(-T.e(14)*T.e(5)+T.e(13)*T.e(6));
	gReal x2=(-T.e(14)*T.e(9)+T.e(13)*T.e(10));
	gReal x3=(T.e(14)*T.e(0)-T.e(12)*T.e(2));
	gReal x4=(T.e(14)*T.e(4)-T.e(12)*T.e(6));
	gReal x5=(-T.e(13)*T.e(0)+T.e(12)*T.e(1));
	gReal x6=(-T.e(13)*T.e(4)+T.e(12)*T.e(5));
	gReal x7=(T.e(14)*T.e(8)-T.e(12)*T.e(10));
	gReal x8=(-T.e(13)*T.e(8)+T.e(12)*T.e(9));
	gReal y0=(T.e(1)*A.e(0,0) + T.e(5)*A.e(0,1) + T.e(9)*A.e(0,2) + x3*A.e(0,3) + x4*A.e(0,4) + x7*A.e(0,5));
	gReal y1=(T.e(1)*A.e(0,1) + T.e(5)*A.e(1,1) + T.e(9)*A.e(1,2) + x3*A.e(1,3) + x4*A.e(1,4) + x7*A.e(1,5));
	gReal y2=(T.e(1)*A.e(0,2) + T.e(5)*A.e(1,2) + T.e(9)*A.e(2,2) + x3*A.e(2,3) + x4*A.e(2,4) + x7*A.e(2,5));
	gReal y3=(T.e(1)*A.e(0,3) + T.e(5)*A.e(1,3) + T.e(9)*A.e(2,3) + x3*A.e(3,3) + x4*A.e(3,4) + x7*A.e(3,5));
	gReal y4=(T.e(1)*A.e(0,4) + T.e(5)*A.e(1,4) + T.e(9)*A.e(2,4) + x3*A.e(3,4) + x4*A.e(4,4) + x7*A.e(4,5));
	gReal y5=(T.e(1)*A.e(0,5) + T.e(5)*A.e(1,5) + T.e(9)*A.e(2,5) + x3*A.e(3,5) + x4*A.e(4,5) + x7*A.e(5,5));
	gReal y6=(T.e(2)*A.e(0,0) + T.e(6)*A.e(0,1) + T.e(10)*A.e(0,2) + x5*A.e(0,3) + x6*A.e(0,4) + x8*A.e(0,5));
	gReal y7=(T.e(2)*A.e(0,1) + T.e(6)*A.e(1,1) + T.e(10)*A.e(1,2) + x5*A.e(1,3) + x6*A.e(1,4) + x8*A.e(1,5));
	gReal y8=(T.e(2)*A.e(0,2) + T.e(6)*A.e(1,2) + T.e(10)*A.e(2,2) + x5*A.e(2,3) + x6*A.e(2,4) + x8*A.e(2,5));
	gReal y9=(T.e(2)*A.e(0,3) + T.e(6)*A.e(1,3) + T.e(10)*A.e(2,3) + x5*A.e(3,3) + x6*A.e(3,4) + x8*A.e(3,5));
	gReal z0=(T.e(2)*A.e(0,4) + T.e(6)*A.e(1,4) + T.e(10)*A.e(2,4) + x5*A.e(3,4) + x6*A.e(4,4) + x8*A.e(4,5));
	gReal z1=(T.e(2)*A.e(0,5) + T.e(6)*A.e(1,5) + T.e(10)*A.e(2,5) + x5*A.e(3,5) + x6*A.e(4,5) + x8*A.e(5,5));
	gReal z2=(T.e(0)*A.e(0,0) + T.e(4)*A.e(0,1) + T.e(8)*A.e(0,2) + x0*A.e(0,3) + x1*A.e(0,4) + x2*A.e(0,5));
	gReal z3=(T.e(0)*A.e(0,1) + T.e(4)*A.e(1,1) + T.e(8)*A.e(1,2) + x0*A.e(1,3) + x1*A.e(1,4) + x2*A.e(1,5));
	gReal z4=(T.e(0)*A.e(0,2) + T.e(4)*A.e(1,2) + T.e(8)*A.e(2,2) + x0*A.e(2,3) + x1*A.e(2,4) + x2*A.e(2,5));
	gReal z5=(T.e(0)*A.e(0,3) + T.e(4)*A.e(1,3) + T.e(8)*A.e(2,3) + x0*A.e(3,3) + x1*A.e(3,4) + x2*A.e(3,5));
	gReal z6=(T.e(0)*A.e(0,4) + T.e(4)*A.e(1,4) + T.e(8)*A.e(2,4) + x0*A.e(3,4) + x1*A.e(4,4) + x2*A.e(4,5));
	gReal z7=(T.e(0)*A.e(0,5) + T.e(4)*A.e(1,5) + T.e(8)*A.e(2,5) + x0*A.e(3,5) + x1*A.e(4,5) + x2*A.e(5,5));

	_e[0][0] +=	z2*T.e(0) + z3*T.e(4) + z4*T.e(8) + z5*x0 + z6*x1 + z7*x2;
	_e[0][1] +=	y0*T.e(0) + y1*T.e(4) + y2*T.e(8) + y3*x0 + y4*x1 + y5*x2;
	_e[0][2] += y6*T.e(0) + y7*T.e(4) + y8*T.e(8) + y9*x0 + z0*x1 + z1*x2;
	_e[0][3] += d0*T.e(0) + d6*T.e(4) + f0*T.e(8) + d1*x0 + d7*x1 + f1*x2;
	_e[0][4] += d2*T.e(0) + d8*T.e(4) + f2*T.e(8) + d3*x0 + d9*x1 + f3*x2;
	_e[0][5] += d4*T.e(0) + f4*T.e(4) + f5*T.e(8) + d5*x0 + f6*x1 + f7*x2;
	_e[1][1] += y0*T.e(1) + y1*T.e(5) + y2*T.e(9) + y3*x3 + y4*x4 + y5*x7;
	_e[1][2] += y6*T.e(1) + y7*T.e(5) + y8*T.e(9) + y9*x3 + z0*x4 + z1*x7;
	_e[1][3] += d0*T.e(1) + d6*T.e(5) + f0*T.e(9) + d1*x3 + d7*x4 + f1*x7;
	_e[1][4] += d2*T.e(1) + d8*T.e(5) + f2*T.e(9) + d3*x3 + d9*x4 + f3*x7;
	_e[1][5] += d4*T.e(1) + f4*T.e(5) + f5*T.e(9) + d5*x3 + f6*x4 + f7*x7;
	_e[2][2] += y6*T.e(2) + y7*T.e(6) + y8*T.e(10) + y9*x5 + z0*x6 + z1*x8;
	_e[2][3] += d0*T.e(2) + d6*T.e(6) + f0*T.e(10) + d1*x5 + d7*x6 + f1*x8;
	_e[2][4] += d2*T.e(2) + d8*T.e(6) + f2*T.e(10) + d3*x5 + d9*x6 + f3*x8;
	_e[2][5] += d4*T.e(2) + f4*T.e(6) + f5*T.e(10) + d5*x5 + f6*x6 + f7*x8;
	_e[3][3] += d1*T.e(0) + d7*T.e(4) + f1*T.e(8);
	_e[3][4] += d3*T.e(0) + d9*T.e(4) + f3*T.e(8);
	_e[3][5] += d5*T.e(0) + f6*T.e(4) + f7*T.e(8);
	_e[4][4] += d3*T.e(1) + d9*T.e(5) + f3*T.e(9);
	_e[4][5] += d5*T.e(1) + f6*T.e(5) + f7*T.e(9);
	_e[5][5] += d5*T.e(2) + f6*T.e(6) + f7*T.e(10);

	//return *this;

}

// *this -= x*y'*scale
void gAInertia::subtractReciprocalProduct(const gWrench& x, const gWrench& y, gReal scale)
{
	_e[0][0] -= (x.e(0)*y.e(0)*scale);		
	_e[0][1] -= (x.e(0)*y.e(1)*scale);
	_e[0][2] -= (x.e(0)*y.e(2)*scale);
	_e[0][3] -= (x.e(0)*y.e(3)*scale);
	_e[0][4] -= (x.e(0)*y.e(4)*scale);
	_e[0][5] -= (x.e(0)*y.e(5)*scale);
	_e[1][1] -= (x.e(1)*y.e(1)*scale);
	_e[1][2] -= (x.e(1)*y.e(2)*scale);
	_e[1][3] -= (x.e(1)*y.e(3)*scale);
	_e[1][4] -= (x.e(1)*y.e(4)*scale);
	_e[1][5] -= (x.e(1)*y.e(5)*scale);
	_e[2][2] -= (x.e(2)*y.e(2)*scale);
	_e[2][3] -= (x.e(2)*y.e(3)*scale);
	_e[2][4] -= (x.e(2)*y.e(4)*scale);
	_e[2][5] -= (x.e(2)*y.e(5)*scale);
	_e[3][3] -= (x.e(3)*y.e(3)*scale);
	_e[3][4] -= (x.e(3)*y.e(4)*scale);
	_e[3][5] -= (x.e(3)*y.e(5)*scale);
	_e[4][4] -= (x.e(4)*y.e(4)*scale);
	_e[4][5] -= (x.e(4)*y.e(5)*scale);
	_e[5][5] -= (x.e(5)*y.e(5)*scale);
}

// return A*S
gWrench gAInertia::operator * (const gTwist& S) const
{
	gWrench r;
	r.set(0,(S.e(0)*_e[0][0]+S.e(1)*_e[0][1]+S.e(2)*_e[0][2]+S.e(3)*_e[0][3]+S.e(4)*_e[0][4]+S.e(5)*_e[0][5]));
	r.set(1,(S.e(0)*_e[0][1]+S.e(1)*_e[1][1]+S.e(2)*_e[1][2]+S.e(3)*_e[1][3]+S.e(4)*_e[1][4]+S.e(5)*_e[1][5]));
	r.set(2,(S.e(0)*_e[0][2]+S.e(1)*_e[1][2]+S.e(2)*_e[2][2]+S.e(3)*_e[2][3]+S.e(4)*_e[2][4]+S.e(5)*_e[2][5]));
	r.set(3,(S.e(0)*_e[0][3]+S.e(1)*_e[1][3]+S.e(2)*_e[2][3]+S.e(3)*_e[3][3]+S.e(4)*_e[3][4]+S.e(5)*_e[3][5]));
	r.set(4,(S.e(0)*_e[0][4]+S.e(1)*_e[1][4]+S.e(2)*_e[2][4]+S.e(3)*_e[3][4]+S.e(4)*_e[4][4]+S.e(5)*_e[4][5]));
	r.set(5,(S.e(0)*_e[0][5]+S.e(1)*_e[1][5]+S.e(2)*_e[2][5]+S.e(3)*_e[3][5]+S.e(4)*_e[4][5]+S.e(5)*_e[5][5]));
	
	return r;
}

gTwist gAInertia::solve(const gWrench& w)  // compute  inv(this)*w
{
	gReal p[6];
	gReal x[6];
	choldc((gReal*)_e,6,p);
	cholsl((const gReal*)_e,6,p,w.cptr(),x);
	return gTwist(x);
}

//FROM NUMERICAL RECIPES
void choldc(gReal *a, int n, gReal* p)
//Givne a PD symmetric matrix a[n][n], this routine constructs
//Cholesky decomposition, A=L*L'. On input, only the upper triangle
//of a need be given; it is not modified. The cholesky factor L
//is returned in the lower triangle of a, except for its diagonal 
//elements which are returned in p[n]
{
	int i,j,k;
	gReal sum;

	for (i=0;i<n;i++){
		for(j=i;j<n;j++){
			for (sum=a[n*i+j],k=i-1;k>=0;k--) sum -= a[n*i+k]*a[n*j+k];
			if (i==j){
				if (sum <= 0) {//error
					std::cout << "choldc error" << std::endl;
					return;
				}
				p[i] = gSqrt(sum);
			} else a[n*j+i] = sum/p[i];
		}
	}
}

void cholsl(const gReal *a, int n,const gReal* p,const gReal* b, gReal* x)
//solves the set of n linear equations A*x = b, where a is a PD matrix
//a[n][n] and p[n] are inputs as the output of choldc.
//Only the lower subdiagonal portion of a is accessed. b[1..n] is input
//as the right hand side vector. The solution vector is returned in 
//x[1..n]. a,n and p are not modified and can be left in place for
//successive calls with different right hand sides b.
//b is not modified unless you identify b and x in the calling sequence,
//which is allowed.
{
	int i,k;
	gReal sum;
	for(i=0;i<n;i++) { //Solve L*y = b, storing y in x
		for (sum=b[i],k=i-1;k>=0;k--) sum -= a[n*i+k]*x[k];
		x[i]=sum/p[i];
	}
	for(i=n-1;i>=0;i--) { //Solve L'*x = y
		for (sum=x[i],k=i+1;k<n;k++) sum -= a[n*k+i]*x[k];
		x[i]=sum/p[i];
	}
}


// update rule
// [delta_s]=T-T_cur
// s += delta_s
// si: initial value
// conv: convergence criteria
gTwist		gTwist::logIterative(const gXMat& T, const gTwist& si, gReal conv)
{
	gTwist s(si);
	gXMat T_cur, T_err;
	gTwist d_s; //delta_s
	int cnt = 0;
	gReal err, err1;
	do{
		T_cur.makeExp(s);	// current T

		T_err = T_cur.invMult(T);
		d_s.makeLog(T_err);
		d_s.makeAd(T_cur,d_s);
		s += d_s;

		/*d_s.set(0, gREAL(0.5)*( ( T.e(6) - T_cur.e(6) ) - ( T.e(9) - T_cur.e(9) ) ) );
		d_s.set(1, gREAL(0.5)*( ( T.e(8) - T_cur.e(8) ) - ( T.e(2) - T_cur.e(2) ) ) );
		d_s.set(2, gREAL(0.5)*( ( T.e(1) - T_cur.e(1) ) - ( T.e(4) - T_cur.e(4) ) ) );		
		d_s.set(3, T.e(12) -T_cur.e(12) );
		d_s.set(4, T.e(13) -T_cur.e(13) );
		d_s.set(5, T.e(14) -T_cur.e(14) );
		s += d_s;*/

		cnt++;
		err = d_s.magnitude();
	}
	while( err > conv );

	return s;
}




gDInertia	gInertia::rateOfChange(const gXMat& T, const gTwist& V)
{
	return this->AdadM(T,V*(-1));
}

//M[6] represent symmetric matrix MM
//| MM  [x] | = Ad(inv(T))'* ( ad(V)'*(this) + (this)*ad(V) )*Ad(inv(T))
//|-[x]  0  |
gDInertia gInertia::AdadM(const gXMat& T, const gTwist& V)
{
	gDInertia D;
	
	gXMat TT(T);
	gTwist VV(V);
	double* pT = TT.ptr();
	double* pV = VV.ptr();

	double m=_m;
	double a=(-pV[1]*_c[2]-pV[3]+pV[2]*_c[1]);
	double b=(-pV[2]*_c[0]-pV[4]+pV[0]*_c[2]);
	double c=(-pV[0]*_c[1]-pV[5]+pV[1]*_c[0]);
	double d=(2*pV[2]*_I[1]-2*pV[1]*_I[2]+m*(-2*pV[5]*_c[2]-2*pV[4]*_c[1]));
	double e=(-pV[2]*_I[0]+pV[0]*_I[2]+pV[2]*_I[3]-pV[1]*_I[4]+m*(pV[4]*_c[0]+pV[3]*_c[1]));
	double f=(pV[1]*_I[0]-pV[0]*_I[1]+pV[2]*_I[4]-pV[1]*_I[5]+m*(pV[5]*_c[0]+pV[3]*_c[2]));
	double g=(-2*pV[2]*_I[1]+2*pV[0]*_I[4]+m*(-2*pV[5]*_c[2]-2*pV[3]*_c[0]));
	double h=(pV[1]*_I[1]-pV[0]*_I[3]-pV[2]*_I[2]+pV[0]*_I[5]+m*(pV[5]*_c[1]+pV[4]*_c[2]));
	double i=(2*pV[1]*_I[2]-2*pV[0]*_I[4]+m*(-2*pV[4]*_c[1]-2*pV[3]*_c[0]));
	double j=(pT[0]*d+pT[4]*e+pT[8]*f);
	double k=(pT[0]*e+pT[4]*g+pT[8]*h);
	double l=(pT[0]*f+pT[4]*h+pT[8]*i);
	double n=(pT[1]*d+pT[5]*e+pT[9]*f);
	double o=(pT[1]*e+pT[5]*g+pT[9]*h);
	double q=(pT[1]*f+pT[5]*h+pT[9]*i);


	D._dx[0]=m*(pT[0]*a+pT[4]*b+pT[8]*c);
	D._dx[1]=m*(pT[1]*a+pT[5]*b+pT[9]*c);
	D._dx[2]=m*(pT[2]*a+pT[6]*b+pT[10]*c);
	D._dI[0]=j*pT[0]+k*pT[4]+l*pT[8]+2*pT[14]*D._dx[2]+2*pT[13]*D._dx[1];
	D._dI[1]=j*pT[1]+k*pT[5]+l*pT[9]-pT[13]*D._dx[0]-pT[12]*D._dx[1];
	D._dI[2]=j*pT[2]+k*pT[6]+l*pT[10]-pT[14]*D._dx[0]-pT[12]*D._dx[2];
	D._dI[3]=n*pT[1]+o*pT[5]+q*pT[9]+2*pT[14]*D._dx[2]+2*pT[12]*D._dx[0];
	D._dI[4]=n*pT[2]+o*pT[6]+q*pT[10]-pT[14]*D._dx[1]-pT[13]*D._dx[2];
	D._dI[5]=(pT[2]*d+pT[6]*e+pT[10]*f)*pT[2]+(pT[2]*e+pT[6]*g+pT[10]*h)*pT[6]+(pT[2]*f+pT[6]*h+pT[10]*i)*pT[10]+2*pT[13]*D._dx[1]+2*pT[12]*D._dx[0];


	return D;
}
