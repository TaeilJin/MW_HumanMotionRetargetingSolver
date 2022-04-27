//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

// -----------------------------------------------------------------
// author: Sung-Hee Lee (leesunghee@gmail.com)
// -----------------------------------------------------------------
#ifndef _MBS_OSG_UTIL_H_
#define _MBS_OSG_UTIL_H_

#include "Base/gBase.h"
#include "Base/gMath.h"
//#include "Visualizer/gVisHeader.h"

//#include <Windows.h>

#if defined(WIN32) || defined(WIN64)
#include <Windows.h>
#endif

#include <osg/Quat>
#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/BoundingBox>
#include <osg/NodeVisitor>
#include <osg/MatrixTransform>
#include <osg/Geode>


#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osg/ShapeDrawable>


//#include "btBulletCollisionCommon.h"

//----------------------------------------------------------
// Conversion between osg::Matrix and gXMat
//----------------------------------------------------------
// Basically the order of elements are same.
// Note that gXMat stores matrix in single array in column-major order
// osg::Matrix stores matrix in double array (hence row-major), with transposed form of the homogeneous matrix representation of SE(3), i.e., [R' 0; p' 0]
// As a result, the order of matrix element of the two classes are effectively same.
// e.g., gXMat.e(1) = osg::Matrix(0,1), gMat.e(4) = osg::Matrix(1,0), etc.
//
// Be warned that X1*X2 where X is osg::Matrix actually means M1' * M2' = (M2*M1)'
// where M is the corresponding SE(3) of X.

#if defined(MW_USE_SINGLE_PRECISION)
	inline void gXMatToOsgMatrix(osg::Matrixf& to, const gXMat& from)		{ to.set( from.cptr() ); }
	inline void osgMatrixTogXMat(gXMat& to,const osg::Matrixf& from)		{ to.set( from.ptr() ); }
#else //defined(MW_USE_DOUBLE_PRECISION)
	inline void gXMatToOsgMatrix(osg::Matrixd& to,const gXMat& from)		{ to.set( from.cptr() ); }
	inline void osgMatrixTogXMat(gXMat& to,const osg::Matrixd& from)		{ to.set( from.ptr() ); }
#endif

inline void gVec3ToOsgVec3(osg::Vec3d& to,const gVec3& from)		{ to.set(from.e(0),from.e(1),from.e(2)); }
inline void gVec3ToOsgVec3(osg::Vec3f& to,const gVec3& from)		{ to.set(from.e(0),from.e(1),from.e(2)); }

inline void osgVec3TogVec3(gVec3& to,const osg::Vec3d& from)		{ to.setX(from.x()),to.setY(from.y()),to.setZ(from.z()); }
inline void osgVec3TogVec3(gVec3& to,const osg::Vec3f& from)		{ to.setX(from.x()),to.setY(from.y()),to.setZ(from.z()); }

void	gRotMatToOsgQuat(osg::Quat& quat, const gRotMat& R);
void	osgQuatTogRotMat(gRotMat& R, const osg::Quat& quat);


inline osg::Vec3 g2o(const gVec3& from) { return osg::Vec3(from.e(0),from.e(1),from.e(2)); }
inline gVec3 o2g(const osg::Vec3& from) { return gVec3(from.x(),from.y(), from.z()); }

// SWL
#if defined(MW_USE_SINGLE_PRECISION)
osg::Vec3f convertToOsgVec3(const gVec3& in);
osg::Matrixf convertToOsgMat(const gXMat& in);
#else
osg::Vec3d convertToOsgVec3(const gVec3& in);  //SHL: removed & (2014.12.31)
osg::Matrixd convertToOsgMat(const gXMat& in); 
//osg::Vec3d& convertToOsgVec3(const gVec3& in);
//osg::Matrixd& convertToOsgMat(const gXMat& in);
#endif


osg::BoundingBox unionBoundingBox(osg::BoundingBox& b1, osg::BoundingBox& b2);

gInertia computeInertiaOfGeode(osg::Geode* gd, gReal density);

class NodeFinder:public osg::NodeVisitor
{
public:
	NodeFinder():osg::NodeVisitor(TRAVERSE_ALL_CHILDREN){ _found = false; _node = 0; }

	//virtual void apply(osg::Node& node)
	//{
	//	if(_found) return;
	//	if(_name == node.getName()) 
	//	{
	//		_node = &node;
	//		_found = true;
	//	}
	//}

	//virtual void apply(osg::Group& node)
	//{
	//	if(_found) return;
	//	if(_name == node.getName()) 
	//	{
	//		_node = &node;
	//		_found = true;
	//	}
	//}

	//virtual void apply(osg::MatrixTransform& node)
	//{
	//	if(_found) return;
	//	if(_name == node.getName()) 
	//	{
	//		_node = &node;
	//		_found = true;
	//	}
	//}

	virtual void apply(osg::Geode& node)
	{
		if(_found) return;
		if(_name == node.getName()) 
		{
			_node = &node;
			_found = true;
		}
	}

	virtual void reset()
	{
		_node = 0;
		_found = false;
	}

	void setTargetName(const char* name)
	{
		reset();
		strcpy(_name,name);		
	}

	osg::ref_ptr<osg::Node> _node; 

private:
	virtual ~NodeFinder(){};

	char _name[128];
	bool _found;
};


//Bounding Box Computer
class BBoxComputer:public osg::NodeVisitor
{
public:
	osg::BoundingBox bbox;

	//set maxDepth to -1 if you don't want to set maximum depth
	BBoxComputer(unsigned int maxDepth=-1):osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
	{
		_currentDepth = 0;
		mat.identity();
		if(maxDepth<0) _depthCheck = false;
		else {
			_depthCheck = true;
			_maxDepth = maxDepth;
		}        
	}

	virtual void apply(osg::MatrixTransform &transform)
	{
		if(_depthCheck && _currentDepth >= _maxDepth) return;
		osg::Matrix curMatrix = mat; //store
		mat.preMult(transform.getMatrix());
		_currentDepth++;
		traverse(transform);
		_currentDepth--;
		mat = curMatrix;
	}

	virtual void apply(osg::Geode& geode)
	{
		if(_depthCheck && _currentDepth >= _maxDepth) return;

		osg::BoundingBox box = geode.getBoundingBox();
		//postMult? or preMultiply??
		osg::Vec4 min_ = mat.preMult(osg::Vec4(box._min,1));
		osg::Vec4 max_ = mat.preMult(osg::Vec4(box._max,1)); 

		if(bbox._min[0]>min_[0]) bbox._min[0] = min_[0];
		if(bbox._min[1]>min_[1]) bbox._min[1] = min_[1];
		if(bbox._min[2]>min_[2]) bbox._min[2] = min_[2];

		if(bbox._max[0]<max_[0]) bbox._max[0] = max_[0];
		if(bbox._max[1]<max_[1]) bbox._max[1] = max_[1];
		if(bbox._max[2]<max_[2]) bbox._max[2] = max_[2];
	}

	virtual void reset()
	{
		bbox.init();
		mat.identity();
		_currentDepth = 0;
	}


protected:
	virtual ~BBoxComputer(){};

	unsigned int _currentDepth, _maxDepth;
	bool _depthCheck;
	osg::Matrix mat;
};


//gInertia Computer
//See compute_rotationalInertia() for the usage of this class
class gInertiaComputer:public osg::NodeVisitor
{
public:
	
	//set maxDepth to -1 if you don't want to set maximum depth
	gInertiaComputer(gReal density, unsigned int maxDepth=-1):osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
	{
		_density = density;
		_currentDepth = 0;
		_mat.identity();
		if(maxDepth<0) _depthCheck = false;
		else {
			_depthCheck = true;
			_maxDepth = maxDepth;
		}        
	}

	virtual void apply(osg::MatrixTransform &transform)
	{
		if(_depthCheck && _currentDepth >= _maxDepth) return;
		osg::Matrix curMatrix = _mat; //store
		_mat.preMult(transform.getMatrix());
		_currentDepth++;
		traverse(transform);
		_currentDepth--;
		_mat = curMatrix;
	}

	virtual void apply(osg::Geode& geode)
	{
		if(_depthCheck && _currentDepth >= _maxDepth) return;
		gInertia j = computeInertiaOfGeode(&geode, _density);		
		gXMat T;	osgMatrixTogXMat(T,_mat);		
		_J += j.xform(T);	// coordinate transform to reference frame;
	}

	virtual void reset(gReal density)
	{
		_mat.identity();
		_currentDepth = 0;
		_J.setZero();
		_density = density;
	}

	inline gInertia& inertia() { return _J;	}

protected:
	virtual ~gInertiaComputer(){};

	unsigned int _currentDepth, _maxDepth;
	bool _depthCheck;
	osg::Matrix _mat;
	gInertia _J;
	gReal _density;
};

osg::Geode* createCylinder(const gVec3& bottom, const gVec3& top, int sides, gReal radius, gReal r,gReal g,gReal b,gReal a);
osg::Geode* createBox(const gVec3& center, gReal lenX, gReal lenY, gReal lenZ,gReal r,gReal g,gReal b,gReal a);
osg::Geode* createCapsule(const gVec3& center, gReal radius, gReal height, gReal r, gReal g, gReal b, gReal a);

#endif