//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
#include "mbs/gMbsUtil.h"


int make_matrix_Ad(arma::mat& M, const gXMat& T)
{	
	if(M.n_rows!= 6 || M.n_cols!= 6) return -1; // error	

	gVec3 p(T.trn());
	gVec3 Rx(T.e(gXMat::M00), T.e(gXMat::M10), T.e(gXMat::M20) );
	gVec3 Ry(T.e(gXMat::M01), T.e(gXMat::M11), T.e(gXMat::M21) );
	gVec3 Rz(T.e(gXMat::M02), T.e(gXMat::M12), T.e(gXMat::M22) );

	gVec3 px = p % Rx;
	gVec3 py = p % Ry;
	gVec3 pz = p % Rz;

	M.fill(0.0);
	
	M(0,0)=Rx.e(0);	M(1,0)=Rx.e(1);	M(2,0)=Rx.e(2);
	M(0,1)=Ry.e(0);	M(1,1)=Ry.e(1);	M(2,1)=Ry.e(2);
	M(0,2)=Rz.e(0);	M(1,2)=Rz.e(1);	M(2,2)=Rz.e(2);

	M(3,0)=px.e(0);	M(4,0)=px.e(1);	M(5,0)=px.e(2);
	M(3,1)=py.e(0);	M(4,1)=py.e(1);	M(5,1)=py.e(2);
	M(3,2)=pz.e(0);	M(4,2)=pz.e(1);	M(5,2)=pz.e(2);

	M(3,3)=Rx.e(0);	M(4,3)=Rx.e(1);	M(5,3)=Rx.e(2);
	M(3,4)=Ry.e(0);	M(4,4)=Ry.e(1);	M(5,4)=Ry.e(2);
	M(3,5)=Rz.e(0);	M(4,5)=Rz.e(1);	M(5,5)=Rz.e(2);

	return 0;
}
int make_matrix_Adi(arma::mat& M, const gXMat& T)
{
	gXMat Ti;
	Ti.makeInverse(T);
	return make_matrix_Ad(M,Ti);
}

bool check_positive_definiteness(gInertia J)
{
	arma::mat M(6,6,arma::fill::zeros);

	//rotational inertia
	M(0,0)=J.rotInertia(0);
	M(0,1)=J.rotInertia(1); M(1,0)=J.rotInertia(1);
	M(0,2)=J.rotInertia(2); M(2,0)=J.rotInertia(2);
	M(1,1)=J.rotInertia(3); 
	M(1,2)=J.rotInertia(4); M(2,1)=J.rotInertia(4);
	M(2,2)=J.rotInertia(5);

	//mass
	M(3,3)=J.mass();
	M(4,4)=J.mass();
	M(5,5)=J.mass();

	//upper-right & lower_left block
	gReal c0 = J.mass()*J.com(0);
	gReal c1 = J.mass()*J.com(1);
	gReal c2 = J.mass()*J.com(2);

	M(0,3+1)=-c2;
	M(0,3+2)=c1;
	M(1,3+0)=c2;
	M(1,3+2)=-c0;
	M(2,3+0)=-c1;
	M(2,3+1)=c0;

	M(3+0,1)=c2;
	M(3+0,2)=-c1;
	M(3+1,0)=-c2;
	M(3+1,2)=c0;
	M(3+2,0)=c1;
	M(3+2,1)=-c0;

	//M.print();
	//eigen value
	arma::vec eigval = eig_sym(M);
	bool re = true;
	
	for(int i=0; i<eigval.n_rows; i++)
	{
		if(eigval(i)<=0.0){	
			re = false;
			break;
		}
	}

	return re;
}

void computePrincipalAxis(const gInertia& M, gRotMat& Paxes, gVec3& Pmoments)
{
	gInertia M1(M);
	M1.translateFrame(M.comInVec3()); //now M1 is at CoM

	//construct 3x3 rotational matrix
	arma::mat R(3,3);
	R(0,0)=M1.rotInertia(gInertia::Ixx);
	R(1,1)=M1.rotInertia(gInertia::Iyy);
	R(2,2)=M1.rotInertia(gInertia::Izz);	
	R(0,1)=M1.rotInertia(gInertia::Ixy);
	R(1,0)=M1.rotInertia(gInertia::Ixy);	
	R(0,2)=M1.rotInertia(gInertia::Ixz);
	R(2,0)=M1.rotInertia(gInertia::Ixz);
	R(1,2)=M1.rotInertia(gInertia::Iyz);
	R(2,1)=M1.rotInertia(gInertia::Iyz);
	
	//gsl_eigen_symmv_workspace* w = gsl_eigen_symmv_alloc(3);
	arma::vec eval(3); //eigen values
	arma::mat evec(3,3); //eigne vectors

	arma::eig_sym(eval,evec,R);
		
	gVec3 R0(evec(0,0),evec(1,0),evec(2,0));
	gVec3 R1(evec(0,1),evec(1,1),evec(2,1));
	gVec3 R2(evec(0,2),evec(1,2),evec(2,2));

	//make evec right-handed
	//Let evec=[R0,R1,R2], if dot( cross(R0,R1), R2 ) < 0, then set R2 = -R2
	
	gReal test = ( (R0%R1) , R2 );
	if(test<0) R2 *= -1;
		
	Paxes.setColumn(0,R0);
	Paxes.setColumn(1,R1);
	Paxes.setColumn(2,R2);

	Pmoments.set( eval(0), eval(1), eval(2) );

}



gVec3	multMatrix33WithVec3(const gReal* M, const gVec3& v) //return M*v where M[9] is 3x3 matrix (column major order)
{
	return gVec3(
		M[0]*v.x()+M[3]*v.y()+M[6]*v.z(),
		M[1]*v.x()+M[4]*v.y()+M[7]*v.z(),
		M[2]*v.x()+M[5]*v.y()+M[8]*v.z() );
}

//------------------------------------------------------------------
// solve K=inv(I), I=3by3 matrix
void invMatrix33(gReal* K,const gReal* I)
//------------------------------------------------------------------
{
	gReal a,b,c,det,invDet;
	a = I[4]*I[8]-I[7]*I[5];
	b = I[7]*I[2]-I[1]*I[8];
	c = I[1]*I[5]-I[4]*I[2];
	det =I[0]*a+I[3]*b+I[6]*c ;
	// check if det != 0
	invDet = gInv(det);
	K[0]=invDet*a;						// 4 8 - 7 5
	K[1]=invDet*b;						// 7 2 - 1 8
	K[2]=invDet*c;						// 1 5 - 4 2
	
	K[3]=invDet*(I[6]*I[5]-I[3]*I[8]);	// 6 5 - 3 8
	K[4]=invDet*(I[0]*I[8]-I[6]*I[2]);	// 0 8 - 6 2
	K[5]=invDet*(I[3]*I[2]-I[0]*I[5]);	// 3 2 - 0 5
		
	K[6]=invDet*(I[3]*I[7]-I[6]*I[4]);	// 3 7 - 6 4
	K[7]=invDet*(I[6]*I[1]-I[0]*I[7]);	// 6 1 - 0 7
	K[8]=invDet*(I[0]*I[4]-I[3]*I[1]);	// 0 4 - 3 1
};

gVec3 proj(const gVec3& x, const gVec3& u, const gVec3& v )
{
	//gVec3 n = Cross( u, v ).normalize();	// normal gVec3 to the plane
	gVec3 n = (u%v);
	n.normalize();
	//return x - Inner(x,n)*n;
	return ( x - n*(x,n));
}

gReal distancePoint2Line( const gVec3& x, const gVec3& start, const gVec3& end )
{
	gVec3 v1 = end - start;
	gVec3 v2 = x - start;
	gReal norm = v2.magnitude();
	return norm*gSin( acos( (v1,v2)/( v1.magnitude()*norm ) ) );
}

gReal	distanceGeodesic(const gRotMat& R1, const gRotMat& R2)
{
	gRotMat e = ~R1*R2;				// e = R1^-1 R2
	return gVec3::log(e).magnitude();
}


void computeProjectionOfVectorToPlane(
	 gReal* c1,
	 gReal* c2,
	 const gVec3& v,
	 const gVec3& point1,
	 const gVec3& point2
	 )
{
	/* let v = c1*p1 + c2*p2 + n
	where p1 = point1, p2 = point2, ci: coeffs,
	n = normal component of v perpendicular to the plane
	then from n'p1 = 0 and n'p2 = 0,
	|			  ||	|	|		 |
	| p1'p1 p1'p2 || a	| = |	p1'v |
	| p1'p2 p2'p2 || b	|	|	p2'v |
	|			  ||	|	|		 |
	then proj = c1*p1 + c2*p2
	*/

	gReal A= point1.magnitudeSquared();
	gReal B= (point1,point2);
	gReal D= point2.magnitudeSquared();
	gReal y1=(point1,v);
	gReal y2=(point2,v);
	gReal det=A*D-B*B;
	assert(!IsZero(det));
	*c1 = (D*y1-B*y2)/det;
	*c2 = (A*y2-B*y1)/det;
}


void computeTangentialPointsFromTwoPointsToSphere(
	gVec3& tan1, 
	gVec3& tan2, 
	const gVec3& point1, 
	const gVec3& point2, 
	const gVec3& center, 
	gReal radius, 
	const gVec3& normal)
{
	gVec3 l(point1,center);	//l = point1-center
	gVec3 r(point2,center);	//r = point2-center
	gReal a,b;
	computeProjectionOfVectorToPlane(&a,&b,normal,l,r); // n is the projection of normal to plane
	gVec3 n=l*a+r*b;
	n.normalize();
	//tan1 = x1*l + x2*n + center
	//tan2 = x3*r + x4*n + center
	gReal l2 = l.magnitudeSquared();
	gReal r2 = r.magnitudeSquared();
	gReal ln = (l,n);
	gReal rn = (r,n);
	gReal d2 = gSqr(radius);
    gReal x2 = radius*gSqrt((l2-d2)/(l2-gSqr(ln)));
	gReal x1 = (d2 - x2*ln)/l2;
	gReal x4 = radius*gSqrt((r2-d2)/(r2-gSqr(rn)));
	gReal x3 = (d2 - x4*rn)/r2;
	tan1 = l*x1 + n*x2 + center;
	tan2 = r*x3 + n*x4 + center;
}

//return true if a circle and line contact, false or else.
//con: contact point if contacted
//center: center of circle
//dir: direction of circle
//point1,2: points in line
//Between the two intersection points between a circle and a line,
//con is chosen that center-con and dir form acute angle
//let con = a*(point1-center)+b*(point2-center)
//if a+b > 1 then no-contact
//else contact
//WARNING: point1,2,and center must not be collinear.
bool	contact_directionalCircleAndLine(
		gVec3& con, 
		const gVec3& center, 
		gReal radius,
		const gVec3& dir, 
		const gVec3& point1, 
		const gVec3& point2
		)
{
	gVec3 p1(point1,center);
	gVec3 p2(point2,center);
	gReal c1,c2;
	computeProjectionOfVectorToPlane(&c1,&c2,dir,p1,p2);
    con = p1*c1 + p2*c2; //projection vector
	gReal s = radius/con.magnitude();
	gReal t = s*(c1+c2);
	con = con*s + center;
	if(t<1 && t>0) 
		return false;
	else	
		return true;
}