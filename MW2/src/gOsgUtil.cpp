//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
//#include "Base/gBase.h"
#include "Visualizer/gOsgUtil.h"
//#include "osg/ShapeDrawable"

//	quat.x = log(R).x
//	quat.y = log(R).y
//	quat.z = log(R).z
//  quat.w = 1 - sqrt(gSqr(quat.x)+gSqr(quat.y)+gSqr(quat.z))
using namespace osg;

void gRotMatToOsgQuat(osg::Quat& quat, const gRotMat& R)
{
	gXMat T(R);
	osg::Matrix mat(T.ptr());
	quat.set(mat);	
}

void osgQuatTogRotMat(gRotMat& R, const osg::Quat& quat)
{
	osg::Matrix mat(quat);
	gXMat T;
	T.set(mat.ptr());
	R = T.rot();
}

#if defined(MW_USE_SINGLE_PRECISION)
osg::Vec3f& convertToOsgVec3(const gVec3& in)
{
	return osg::Vec3f(in.x(), in.y(), in.z());
}
osg::Matrixd& convertToOsgMat(const gXMat& in)
{
	//return 0;
}
#else
osg::Vec3d convertToOsgVec3(const gVec3& in)
{
	return osg::Vec3d(in.x(), in.y(), in.z());
}

osg::Matrixd convertToOsgMat(const gXMat& in)
{
	return osg::Matrixd(in.cptr());
}

#endif

class MeshMomentOfInertia
{
public:
	MeshMomentOfInertia()
	{
		initialize();
	}

	void initialize()
	{
		_m = 0;                              
		_Cx = _Cy = _Cz = 0;                 
		_xx = _yy = _zz = _yx = _zx = _zy = 0;
	}

/**********************************************************************
Add the contribution of a triangle to the mass properties.
Call this method for each one of the mesh's triangles.  
The order of vertices must be such that their determinant is positive
when the origin lies on the inner side of this facet's plane.
Author: Michael.Kallay@microsoft.com
**********************************************************************/
    void AddTriangleContribution(
        double x1, double y1, double z1,    // Triangle's vertex 1
        double x2, double y2, double z2,    // Triangle's vertex 2
        double x3, double y3, double z3)    // Triangle's vertex 3
    {
        // Signed volume of this tetrahedron.
        double v = x1*y2*z3 + y1*z2*x3 + x2*y3*z1 -
                  (x3*y2*z1 + x2*y1*z3 + y3*z2*x1);
        
        // Contribution to the mass
        _m += v;

        // Contribution to the centroid
        double x4 = x1 + x2 + x3;           _Cx += (v * x4);
        double y4 = y1 + y2 + y3;           _Cy += (v * y4);
        double z4 = z1 + z2 + z3;           _Cz += (v * z4);

        // Contribution to moment of inertia monomials
        _xx += v * (x1*x1 + x2*x2 + x3*x3 + x4*x4);
        _yy += v * (y1*y1 + y2*y2 + y3*y3 + y4*y4);
        _zz += v * (z1*z1 + z2*z2 + z3*z3 + z4*z4);
        _yx += v * (y1*x1 + y2*x2 + y3*x3 + y4*x4);
        _zx += v * (z1*x1 + z2*x2 + z3*x3 + z4*x4);
        _zy += v * (z1*y1 + z2*y2 + z3*y3 + z4*y4);        
    }

 
/**********************************************************************
This method is called to obtain the results.
This call modifies the internal data; calling it again will return 
incorrect results.
Origianl Author: Michael.Kallay@microsoft.com
Modified by: Sung-Hee
**********************************************************************/
    void GetResults(
		gReal density,							  // Density
        double & m,                               // Total mass
        double & Cx,  double & Cy,  double & Cz,  // Centroid
        double & Ixx, double & Iyy, double & Izz, // Moment of inertia       
                                                  // diagonal entries
        double & Iyx, double & Izx, double & Izy) // Moment of inertia 
                                                  // mixed entries
    {
		// The case _m = 0 needs to be addressed here.
        if(IsZero(_m))
		{
			m=0;
			Cx=Cy=Cz=0;
			Ixx=Iyy=Izz=Iyx=Izx=Izy=0;
			return;
		}

        // Centroid.  
        double r = 1.0 / (4 * _m);
        Cx = _Cx * r;
        Cy = _Cy * r;
        Cz = _Cz * r;

        // Mass
        m = _m / 6;

        // Moment of inertia about the centroid.
        r = 1.0 / 120;
        Iyx = _yx * r - m * Cy*Cx;
        Izx = _zx * r - m * Cz*Cx;
        Izy = _zy * r - m * Cz*Cy;

        _xx = _xx * r - m * Cx*Cx;
        _yy = _yy * r - m * Cy*Cy;
        _zz = _zz * r - m * Cz*Cz;

        Ixx = _yy + _zz;
        Iyy = _zz + _xx;
        Izz = _xx + _yy;

		// 		
		m *= density;
		Ixx *= density;
		Iyy *= density;
		Izz *= density;
		Iyx *= density;
		Izx *= density;
		Izy *= density;
    }
    
    // Data members
private:
    double _m;                              // Mass
    double _Cx, _Cy, _Cz;                   // Centroid
    double _xx, _yy, _zz, _yx, _zx, _zy;    // Moment of inertia tensor
};

//#include "osg/Geometry"
//#include "osg/PrimitiveSet"


unsigned int addTrianglesFromPrimitiveSet(MeshMomentOfInertia& MMOI,const osg::Vec3Array* vertex_array, osg::PrimitiveSet* primSet,unsigned int numIndices)
{
	using namespace osg;

	unsigned int i,numPolygons;
	
	Vec3 v1,v2,v3;

	switch(primSet->getMode())
	{
	case PrimitiveSet::TRIANGLES:
		numPolygons = numIndices/3;
		for(i=0;i<numPolygons;++i)
		{			
			v1 = (*vertex_array)[primSet->index(3*i)];
			v2 = (*vertex_array)[primSet->index(3*i+1)];
			v3 = (*vertex_array)[primSet->index(3*i+2)];

			MMOI.AddTriangleContribution(v1[0],v1[1],v1[2],v2[0],v2[1],v2[2],v3[0],v3[1],v3[2]);
		}
		
		break;
	
	case PrimitiveSet::QUADS:
		numPolygons = numIndices/4;
		for(i=0;i<numPolygons;++i)
		{
			v1 = (*vertex_array)[primSet->index(4*i)];
			v2 = (*vertex_array)[primSet->index(4*i+1)];
			v3 = (*vertex_array)[primSet->index(4*i+2)];
			MMOI.AddTriangleContribution(v1[0],v1[1],v1[2],v2[0],v2[1],v2[2],v3[0],v3[1],v3[2]);

			v1 = (*vertex_array)[primSet->index(4*i+2)];
			v2 = (*vertex_array)[primSet->index(4*i+3)];
			v3 = (*vertex_array)[primSet->index(4*i)];
			MMOI.AddTriangleContribution(v1[0],v1[1],v1[2],v2[0],v2[1],v2[2],v3[0],v3[1],v3[2]);
		}
		break;

	case PrimitiveSet::TRIANGLE_STRIP:
		numPolygons = numIndices-2;
		for(i=0;i<numPolygons;++i)
		{			
			v1 = (*vertex_array)[primSet->index(i)];
			if(i%2){ 
				v2 = (*vertex_array)[primSet->index(i+2)];
				v3 = (*vertex_array)[primSet->index(i+1)];
			}else{
				v2 = (*vertex_array)[primSet->index(i+1)];
				v3 = (*vertex_array)[primSet->index(i+2)];
			}
			MMOI.AddTriangleContribution(v1[0],v1[1],v1[2],v2[0],v2[1],v2[2],v3[0],v3[1],v3[2]);
		}
		break;

	case PrimitiveSet::TRIANGLE_FAN:
		numPolygons = numIndices-2;
		for(i=0;i<numPolygons;++i)
		{
			v1 = (*vertex_array)[primSet->index(0)];
			v2 = (*vertex_array)[primSet->index(i+2)];
			v3 = (*vertex_array)[primSet->index(i+1)];
			MMOI.AddTriangleContribution(v1[0],v1[1],v1[2],v2[0],v2[1],v2[2],v3[0],v3[1],v3[2]);
		}
        break;

	case PrimitiveSet::QUAD_STRIP:
		numPolygons = numIndices/2-1;
		for (i=0; i<numPolygons ; ++i)
		{
			v1 = (*vertex_array)[primSet->index(4*i)];
			v2 = (*vertex_array)[primSet->index(4*i+1)];
			v3 = (*vertex_array)[primSet->index(4*i+3)];
			MMOI.AddTriangleContribution(v1[0],v1[1],v1[2],v2[0],v2[1],v2[2],v3[0],v3[1],v3[2]);

			v1 = (*vertex_array)[primSet->index(4*i+3)];
			v2 = (*vertex_array)[primSet->index(4*i+2)];
			v3 = (*vertex_array)[primSet->index(4*i)];
			MMOI.AddTriangleContribution(v1[0],v1[1],v1[2],v2[0],v2[1],v2[2],v3[0],v3[1],v3[2]);
		}
		break;
	
	default:
		break;
	}

	return numPolygons;
}

gInertia computeInertiaOfGeode(osg::Geode* gd, gReal density)
{
	using namespace osg;

	MeshMomentOfInertia MMOI;

	for(unsigned int k=0;k<gd->getNumDrawables();++k)
	{
		osg::Drawable* d = gd->getDrawable(k);
		osg::Geometry* geom = d->asGeometry();

		if(geom!=NULL)
		{
			// make points
			const Vec3Array* vertex_array	 = dynamic_cast<const Vec3Array*>(geom->getVertexArray());
			osg::DrawArrayLengths*	dal;
			unsigned int nps = geom->getNumPrimitiveSets();
			for(unsigned int i=0; i<nps; ++i)
			{
				PrimitiveSet* primSet = geom->getPrimitiveSet(i);
				PrimitiveSet::Type primSetType = primSet->getType();
				unsigned int numIndices = primSet->getNumIndices();
				unsigned int numPrims	= primSet->getNumPrimitives();
				switch(primSetType)
				{
				case PrimitiveSet::DrawArrayLengthsPrimitiveType:
					dal = dynamic_cast<DrawArrayLengths*>(primSet);
					for(unsigned int j=0;j<numPrims;++j)
					{						
						addTrianglesFromPrimitiveSet(MMOI,vertex_array,dal,(*dal)[j]);
						
						dal->offsetIndices((*dal)[j]);
					}
					break;

				default:
					
					addTrianglesFromPrimitiveSet(MMOI,vertex_array,primSet,numIndices);
					break;
				}
			}

		}
		else
		{
			//don't support osg::ShapeDrawables
		}
	}
	double m,cx,cy,cz,Ixx,Iyy,Izz,Iyx,Izx,Izy;
	MMOI.GetResults(density,m,cx,cy,cz,Ixx,Iyy,Izz,Iyx,Izx,Izy);
	gInertia J;
	gReal Ic[6],com[3];
	Ic[0] = Ixx; Ic[1] = Iyx; Ic[2] = Izx; Ic[3] = Iyy; Ic[4] = Izy; Ic[5] = Izz;
	com[0] = cx; com[1] = cy; com[2] = cz;
	J.set(m,gVec3(com),Ic,1);
	return J;
}

osg::BoundingBox unionBoundingBox(osg::BoundingBox& b1, osg::BoundingBox& b2)
{
	osg::BoundingBox b;
	b._min[0] = gMin(b1._min[0],b2._min[0]);
	b._min[1] = gMin(b1._min[1],b2._min[1]);
	b._min[2] = gMin(b1._min[2],b2._min[2]);
	b._max[0] = gMax(b1._max[0],b2._max[0]);
	b._max[1] = gMax(b1._max[1],b2._max[1]);
	b._max[2] = gMax(b1._max[2],b2._max[2]);
	return b;
}

osg::Geode* createBox(const gVec3& center, gReal lenX, gReal lenY, gReal lenZ, gReal r, gReal g, gReal b, gReal a)
{
	Geode* geode = new Geode();
	StateSet* stateset = new StateSet();
	ShapeDrawable* sd = new osg::ShapeDrawable(new osg::Box(osg::Vec3(center.x(),center.y(),center.z()),lenX,lenY,lenZ)); 
	sd->setColor(Vec4(r,g,b,a));
	geode->setStateSet( stateset );
	geode->addDrawable(sd);
	return geode;
}

osg::Geode* createCylinder(const gVec3& bottom, const gVec3& top, int sides, gReal radius, gReal r,gReal g,gReal b,gReal a)
{
	int i;
	// make cap
	Vec3* cap = new Vec3[sides];
	Vec3*	capNormal = new Vec3[sides];
	gReal angle = 2*gPI/sides;
	for(i=0;i<sides;++i)
	{
		capNormal[i].set(gCos(i*angle),gSin(i*angle),0);
		cap[i] = capNormal[i]*radius;		
	}

	Geode* geode = new Geode;
	Geometry* geom = new Geometry;
	geom->setUseDisplayList(false);
	int numVerts = 2*(sides+1);
	Vec3Array* vertices = new Vec3Array(numVerts);
	geom->setVertexArray(vertices);

	//set colors
	Vec4Array* colors = new Vec4Array;
	colors->push_back(Vec4(r,g,b,a));
	geom->setColorArray(colors);
	geom->setColorBinding(Geometry::BIND_OVERALL);

	//set normals
	Vec3Array*	normals = new Vec3Array(numVerts);
	geom->setNormalArray(normals);
	geom->setNormalBinding(Geometry::BIND_PER_VERTEX);

	//binding
	geom->addPrimitiveSet(new DrawArrays(PrimitiveSet::QUAD_STRIP,0,numVerts));
	geode->addDrawable(geom);

	//
	Vec3 left,right;
	Vec3 v, n;
	gVec3ToOsgVec3(left,bottom);
	gVec3ToOsgVec3(right,top);
	Quat q; q.makeRotate(Z_AXIS,right-left);

	osg::Vec3Array::iterator vitr = vertices->begin();
	osg::Vec3Array::iterator nitr = normals->begin();
	for(i=0;i<sides;++i)
	{
		v = q*cap[i];
		n = q*capNormal[i];
		(vitr++)->set( v+left );
		(vitr++)->set( v+right);
		(nitr++)->set( n );
		(nitr++)->set( n );		
	}

	//for last vertices/normals
	v = q*cap[0];
	n = q*capNormal[0];
	(vitr++)->set( v+left );
	vitr->set( v+right);
	(nitr++)->set( n );
	nitr->set( n );	

	return geode;
}

osg::Geode* createCapsule(const gVec3& center, gReal radius, gReal height, gReal r, gReal g, gReal b, gReal a)
{
	Geode* geode = new Geode();
	StateSet* stateset = new StateSet();
	ShapeDrawable* sd = new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(center.x(),center.y(),center.z()),radius,height)); 
	sd->setColor(Vec4(r,g,b,a));
	geode->setStateSet( stateset );
	geode->addDrawable(sd);
	return geode;
}