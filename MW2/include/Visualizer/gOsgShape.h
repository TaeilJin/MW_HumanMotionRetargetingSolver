//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _MBS_BD_OSG_STATIC_SHAPE_H_
#define _MBS_BD_OSG_STATIC_SHAPE_H_

#if defined(WIN32) || defined(WIN64)
	#include <Windows.h>
#endif

#include <osg/MatrixTransform>
//#include <btBulletDynamicsCommon.h>
#include <osg/PrimitiveSet>
#include <osg/Geode>

#include "Base/gMath.h"

class gOSGShape
{
public:
	enum UPAXIS {
		X = 0,
		Y,
		Z
	};

public:
	gOSGShape() {};
	~gOSGShape() {};
public:
	static osg::ref_ptr<osg::Geode>	createSphereShape(const double radius, const osg::Vec3& pos);
	static osg::ref_ptr<osg::Geode>	createBoxShape(const osg::Vec3& lengths, const osg::Vec3& pos);
	static osg::ref_ptr<osg::Geode> createLineShape( const osg::Vec3& pos, const osg::Vec3& dir, const float length, const float width);
	static osg::ref_ptr<osg::Geode> createLineShape( const osg::Vec3& start, const osg::Vec3& end, const float width);
	static osg::ref_ptr<osg::Geode>	createCapsuleShape(const osg::Vec3& center, double radius, double height, const int upAxis=0);
	static osg::ref_ptr<osg::Geode>	createAxis(const double width, const double size);
	static osg::ref_ptr<osg::Geode>	createFrameAxis(const osg::Vec3& pos, const osg::Vec3& rotX, const osg::Vec3& rotY, const osg::Vec3& rotZ, const double width, const double size);

	static osg::ref_ptr<osg::Geode>	createArc(const osg::Vec3& center, const osg::Vec3& normal, const osg::Vec3& axis, double radiusA, double radiusB, double minAngle, double maxAngle, bool drawSect, double stepDegrees = 10);
	static osg::ref_ptr<osg::Geode> createVertexShape( const osg::ref_ptr<osg::Vec3Array> arrays, const osg::PrimitiveSet::Mode mode = osg::PrimitiveSet::POINTS);

	static osg::ref_ptr<osg::Geode> createTextSprite( const std::string& text, osg::Vec3& pos, const float size);
	static osg::ref_ptr<osg::Geode> createConeShape(const osg::Vec3& pos, const double radius, const double height);

	//static osg::ref_ptr<osg::MatrixTransform>	createAxis(const double width, const double size, const osg::Matrix trans);
	//static osg::ref_ptr<osg::Geode> createText(const std::string text, const float size = 10.f, const osg::Vec4 color = osg::Vec4(0,0,0,1) );
	
	static void setColor(osg::Vec4 _color) { color = _color; };


	// setter
	static osg::ref_ptr<osg::Geode> createPoint( osg::Vec3 point, float size = 1 );
	//osg::ref_ptr<osg::Geode> createDynamicPoint( osg::Vec3 point, float size = 1 );
	static osg::ref_ptr<osg::Geode> createDynamicLine( osg::ref_ptr<osg::Vec3Array> &points, float width=0.5f, osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array() );
	static osg::ref_ptr<osg::Geode> createDynamicLines( osg::ref_ptr<osg::Vec3Array> &points, float width=0.5f, osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array() );
	static osg::ref_ptr<osg::Geode> createDynamicsPoints( osg::ref_ptr<osg::Vec3Array> points, float size = 1, osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array() );

	static osg::ref_ptr<osg::Geode> createPoints( osg::ref_ptr<osg::Vec3Array> points, float size = 1, osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array() );
	static osg::ref_ptr<osg::Geode> createPoints( std::vector<gVec3>& points, float size = 1, osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array() );
	//osg::ref_ptr<osg::Geode> createPoints( std::vector<gVec3*> *points, , float size = 1 );
	//osg::ref_ptr<osg::Geode> createVoxels( std::vector<gVec3> *points, gVec3 size );
	static osg::ref_ptr<osg::Geode> createVoxels( osg::ref_ptr<osg::Vec3Array> points, gVec3 size );
	static osg::ref_ptr<osg::Geode> createPlane( osg::Vec3 normal, osg::Vec3 center, float width, float height, osg::Vec3 refPoint = osg::Vec3(0.0f,0.0f,1.0f));
	static osg::ref_ptr<osg::Geode> createPlane(osg::Vec3 pos00, osg::Vec3 pos01, osg::Vec3 pos10, osg::Vec3 pos11);
	//static osg::ref_ptr<osg::Geode> createPlane( gVec3 normal, gVec3 center, float width, float height, gVec3 refPoint = gVec3(0.0f,0.0f,1.0f));
	//static osg::ref_ptr<osg::Geode> createLine( gVec3 pos, gVec3 dir, float length = 1.0f, float width=0.5f);
	static osg::ref_ptr<osg::Geode> createLine( osg::ref_ptr<osg::Vec3Array>& points, float width=0.5f, osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array());
	static osg::ref_ptr<osg::Geode> createDynamicTriangles( osg::ref_ptr<osg::Vec3Array>& points);
	static osg::ref_ptr<osg::Geode> createTriangles( osg::ref_ptr<osg::Vec3Array>& points);
	//osg::ref_ptr<osg::Geode> createTraingles( std::vector<gVec3> *points, float size = 1 );
	static osg::ref_ptr<osg::Geode> createText(std::string text, osg::Vec3 pos=osg::Vec3(0,0,0), float size = 10.f);
	static osg::ref_ptr<osg::Geode> createDynamicText(std::string& text, osg::Vec3 pos=osg::Vec3(0,0,0), float size = 10.f);

	static osg::ref_ptr<osg::MatrixTransform> createArrow(const osg::Vec3& pos, const osg::Vec3& dir, const float length, const float radius);

//private:
	//static osg::Vec4 color = osg::Vec4(1,1,0,0.5);
	static osg::Vec4 color;
	static double _width;

	// dynaimc update callback
	class dynUpdateCallback : public osg::NodeCallback 
	{
		void operator()(osg::Node* node, osg::NodeVisitor* nv)
		{
			node->dirtyBound();
		}
	};
};

#endif