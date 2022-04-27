//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

//#include "Visualizer/gVisHeader.h"
#include "Visualizer/gOSGShape.h"
//#include "Visualizer/gBDOSGUtil.h"

#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>
#include <osg/Point>
#include <osgText/Text>
#include <osg/PolygonMode>


#define SIMD_PI           3.1415926535897932384626433832795029
#define SIMD_2_PI         (2.0 * SIMD_PI)
#define SIMD_RADS_PER_DEG (SIMD_2_PI / 360.0)

osg::Vec4 gOSGShape::color = osg::Vec4(1,1,0,0.5);
double gOSGShape::_width = 3.0;

osg::ref_ptr<osg::Geode> gOSGShape::createSphereShape(const double radius, const osg::Vec3& pos)
{
	// create _transMat & geode	
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->setDataVariance(osg::Object::STATIC);
	osg::ref_ptr<osg::ShapeDrawable> shape;

	shape = new osg::ShapeDrawable(new osg::Sphere(pos, radius));
	shape->setColor(color);
	geode->addDrawable(shape.get());
	
	return geode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createBoxShape(const osg::Vec3& lengths, const osg::Vec3& pos)
{
	// create _transMat & geode	
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::ShapeDrawable> shape;

	// test
	geode->setCullingActive(false);

	shape = new osg::ShapeDrawable(new osg::Box(pos,lengths.x(),lengths.y(),lengths.z()));
	shape->setColor(color);
	geode->addDrawable(shape.get());
	
	return geode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createConeShape(const osg::Vec3& pos, const double radius, const double height)
{
	// create _transMat & geode	
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::ShapeDrawable> shape;

	// test
	//geode->setCullingActive(false);

	//shape = new osg::ShapeDrawable(new osg::Box(pos, lengths.x(), lengths.y(), lengths.z()));
	shape = new osg::ShapeDrawable(new osg::Cone(pos, radius, height));

	shape->setColor(color);
	geode->addDrawable(shape.get());

	return geode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createLineShape(const osg::Vec3& pos, const osg::Vec3& dir, const float length, const float width)
{
	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> LineGeode = new osg::Geode;
	LineGeode->addDrawable(lineGeometry);

	// state
	osg::ref_ptr<osg::StateSet> lineState = new osg::StateSet();
	lineState->setMode(GL_LIGHTING, false);
	lineState->setAttribute( new osg::LineWidth( width ) );
	LineGeode->setStateSet(lineState);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( color );

	osg::ref_ptr<osg::Vec3Array> vertex = new osg::Vec3Array;
	vertex->push_back( pos );
	vertex->push_back( pos + dir*length );
	
	lineGeometry->setVertexArray(vertex);
	lineGeometry->setColorArray(colors);
	lineGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	lineGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertex->size()) );

	return LineGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createLineShape(const osg::Vec3& start, const osg::Vec3& end, const float width)
{
	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> LineGeode = new osg::Geode;
	LineGeode->addDrawable(lineGeometry);

	// state
	osg::ref_ptr<osg::StateSet> lineState = new osg::StateSet();
	lineState->setMode(GL_LIGHTING, false);
	lineState->setAttribute( new osg::LineWidth( width ) );
	LineGeode->setStateSet(lineState);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( color );

	osg::ref_ptr<osg::Vec3Array> vertex = new osg::Vec3Array;
	vertex->push_back( start );
	vertex->push_back( end );
	
	lineGeometry->setVertexArray(vertex);
	lineGeometry->setColorArray(colors);
	lineGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	lineGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertex->size()) );

	return LineGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createCapsuleShape(const osg::Vec3& center, double radius, double height, const int upAxis)
{
	// create _transMat & geode	
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Capsule> capsule = new osg::Capsule(center, radius, height);

	osg::Quat rot;
	switch(upAxis)
	{
	case X: //X
		rot.makeRotate( osg::DegreesToRadians(90.0), osg::Vec3(0, 1, 0) );
		break;
	case Y: // Y
		rot.makeRotate( osg::DegreesToRadians(-90.0), osg::Vec3(1, 0, 0) );
		break;
	case Z:
		break;
	}
	capsule->setRotation(rot);

	osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(capsule);
	shape->setColor(color);
	geode->addDrawable(shape);
	
	return geode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createAxis(const double width, const double size)
{
	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> LineGeode = new osg::Geode;
	LineGeode->addDrawable(lineGeometry);

	// state
	osg::ref_ptr<osg::StateSet> lineState = new osg::StateSet();
	lineState->setMode(GL_LIGHTING, false);
	lineState->setAttribute( new osg::LineWidth( width ) );
	LineGeode->setStateSet(lineState);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( osg::Vec4(1,0,0,1) );
	colors->push_back( osg::Vec4(1,0,0,1) );
	colors->push_back( osg::Vec4(0,1,0,1) );
	colors->push_back( osg::Vec4(0,1,0,1) );
	colors->push_back( osg::Vec4(0,0,1,1) );
	colors->push_back( osg::Vec4(0,0,1,1) );

	osg::ref_ptr<osg::Vec3Array> vertex = new osg::Vec3Array;
	vertex->push_back( osg::Vec3(0, 0, 0) );
	vertex->push_back( osg::Vec3(1*size, 0, 0) );
	vertex->push_back( osg::Vec3(0, 0, 0) );
	vertex->push_back( osg::Vec3(0, 1*size, 0) );
	vertex->push_back( osg::Vec3(0, 0, 0) );
	vertex->push_back( osg::Vec3(0, 0, 1*size) );
	
	lineGeometry->setVertexArray(vertex);
	lineGeometry->setColorArray(colors);
	lineGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	lineGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertex->size()) );

	return (LineGeode);
}
osg::ref_ptr<osg::Geode> gOSGShape::createFrameAxis(const osg::Vec3& pos, const osg::Vec3& rotX, const osg::Vec3& rotY, const osg::Vec3& rotZ, const double width, const double size)
{
	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> LineGeode = new osg::Geode;
	LineGeode->addDrawable(lineGeometry);

	// state
	osg::ref_ptr<osg::StateSet> lineState = new osg::StateSet();
	lineState->setMode(GL_LIGHTING, false);
	lineState->setAttribute(new osg::LineWidth(width));
	LineGeode->setStateSet(lineState);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1, 0, 0, 1));
	colors->push_back(osg::Vec4(1, 0, 0, 1));
	colors->push_back(osg::Vec4(0, 1, 0, 1));
	colors->push_back(osg::Vec4(0, 1, 0, 1));
	colors->push_back(osg::Vec4(0, 0, 1, 1));
	colors->push_back(osg::Vec4(0, 0, 1, 1));

	osg::ref_ptr<osg::Vec3Array> vertex = new osg::Vec3Array;
	vertex->push_back(pos);
	vertex->push_back(rotX *size);
	vertex->push_back(pos);
	vertex->push_back(rotY*size);
	vertex->push_back(pos);
	vertex->push_back(rotZ*size);

	lineGeometry->setVertexArray(vertex);
	lineGeometry->setColorArray(colors);
	lineGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	lineGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertex->size()));

	return (LineGeode);
}
osg::ref_ptr<osg::Geode> gOSGShape::createArc(const osg::Vec3& center, const osg::Vec3& normal, const osg::Vec3& axis, double radiusA, double radiusB, double minAngle, double maxAngle, bool drawSect, double stepDegrees)
{
	const osg::Vec3& vx = axis;
	osg::Vec3 vy = normal ^ axis;
	double step = stepDegrees * SIMD_RADS_PER_DEG;
	int nSteps = (int)((maxAngle - minAngle) / step);
	if(!nSteps) nSteps = 1;

	//osg::Vec3 prev = center + (vx * radiusA * btCos(minAngle)) + (vy * radiusB * btSin(minAngle));

	osg::ref_ptr<osg::Vec3Array> vertex = new osg::Vec3Array;

	if(drawSect)
	{
		vertex->push_back( center );
		//vertex->push_back( prev );
	}

	osg::Vec3 next;
	//for(int i = 1; i <= nSteps; i++)
	for(int i = 0; i <= nSteps; i++)
	{
		double angle = minAngle + (maxAngle - minAngle) * double(i) / double(nSteps);
		next = center + vx *radiusA * cos(angle) + vy * radiusB * sin(angle);

		//vertex->push_back( prev );
		vertex->push_back( next );
		//prev = next;
	}

	if(drawSect)
	{
		vertex->push_back( center );
		//vertex->push_back( prev );
	}
		
	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry;
	osg::ref_ptr<osg::Geode> LineGeode = new osg::Geode;
	LineGeode->addDrawable(lineGeometry);

	// state
	osg::ref_ptr<osg::StateSet> lineState = new osg::StateSet();
	lineState->setMode(GL_LIGHTING, false);
	lineState->setAttribute( new osg::LineWidth( _width ) );
	LineGeode->setStateSet(lineState);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( color );

	lineGeometry->setVertexArray(vertex);
	lineGeometry->setColorArray(colors);
	lineGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	lineGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertex->size()) );

	return LineGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createVertexShape( const osg::ref_ptr<osg::Vec3Array> arrays, const osg::PrimitiveSet::Mode mode)
{
	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry;
	osg::ref_ptr<osg::Geode> LineGeode = new osg::Geode;
	LineGeode->addDrawable(lineGeometry);

	// state
	osg::ref_ptr<osg::StateSet> lineState = new osg::StateSet();
	lineState->setMode(GL_LIGHTING, false);
	lineState->setAttribute( new osg::LineWidth( _width ) );
	
	osg::ref_ptr<osg::Point> ptState = new osg::Point;
	ptState->setSize(_width);
	lineState->setAttribute(ptState);

	LineGeode->setStateSet(lineState);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( color );

	osg::ref_ptr<osg::Vec3Array> vertex = new osg::Vec3Array();

	lineGeometry->setVertexArray(arrays);
	lineGeometry->setColorArray(colors);
	lineGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	lineGeometry->addPrimitiveSet( new osg::DrawArrays(mode, 0, arrays->size()) );

	return LineGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createTextSprite( const std::string& text, osg::Vec3& pos, const float size)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osgText::Text> osgText = new osgText::Text;

	geode->addDrawable(osgText);
	//osg::ref_ptr<osgText::Font> hudFont = osgText::readFontFile("C:/WINDOWS/Fonts/verdena.ttf");
	//osgText->setFont(hudFont);
	osgText->setCharacterSize(size);
	osgText->setMaximumHeight(10.0);
	osgText->setAlignment(osgText::Text::LEFT_TOP);
	//osgText->setAlignment(osgText::Text::CENTER_TOP);
	//osgText->setDataVariance(osg::Object::DYNAMIC);
	osgText->setColor( color );
	osgText->setText( text );

	//osgText->setAxisAlignment( osgText::Text::REVERSED_XZ_PLANE );
	osgText->setAutoRotateToScreen(true);
	osgText->setPosition( pos );

	return geode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createPoint( osg::Vec3 point, float size)
{
	osg::ref_ptr<osg::Geometry> pointGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode;
	pointGeode->setCullingActive(false);

	//StateSet
	osg::ref_ptr<osg::Point> ptState = new osg::Point;
	ptState->setSize(size);

	osg::ref_ptr<osg::StateSet> state = new osg::StateSet;
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	state->setAttribute(ptState);
	pointGeode->setStateSet(state);
	pointGeode->addDrawable(pointGeometry);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( color );

	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	points->push_back(point);

	pointGeometry->setVertexArray(points);
	pointGeometry->setColorArray(colors);
	pointGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	pointGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points->size()) );

	return pointGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createDynamicsPoints( osg::ref_ptr<osg::Vec3Array> points, float size, osg::ref_ptr<osg::Vec4Array> colors )
{
	osg::ref_ptr<osg::Geometry> pointGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode;
	pointGeode->setCullingActive(false);

	//StateSet
	osg::ref_ptr<osg::Point> ptState = new osg::Point;
	ptState->setSize(size);

	osg::ref_ptr<osg::StateSet> state = new osg::StateSet;
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	state->setAttribute(ptState);
	pointGeode->setStateSet(state);
	pointGeode->addDrawable(pointGeometry);

	pointGeometry->setSupportsDisplayList(false);
	pointGeometry->setUseDisplayList(false);
	pointGeometry->setDataVariance(osg::Object::DYNAMIC);
	//pointGeometry->setUseVertexBufferObjects(true);

	// Init geometry
	//osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	pointGeometry->setVertexArray(points);

	if( colors->size() == 0 )
	{
		colors->push_back( color );
		pointGeometry->setColorArray(colors);
		pointGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	} else {
		pointGeometry->setColorArray(colors);
		pointGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	}

	pointGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points->size()) );

	return pointGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createPoints( osg::ref_ptr<osg::Vec3Array> points, float size, osg::ref_ptr<osg::Vec4Array> colors )
{
	osg::ref_ptr<osg::Geometry> pointGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode;
	pointGeode->setCullingActive(false);

	//StateSet
	osg::ref_ptr<osg::Point> ptState = new osg::Point;
	ptState->setSize(size);

	osg::ref_ptr<osg::StateSet> state = new osg::StateSet;
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	state->setAttribute(ptState);
	pointGeode->setStateSet(state);
	pointGeode->addDrawable(pointGeometry);

	// Init geometry
	pointGeometry->setVertexArray(points);

	if( colors->size() == 0 )
	{
		colors->push_back( color );
		pointGeometry->setColorArray(colors);
		pointGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	} else {
		pointGeometry->setColorArray(colors);
		pointGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	}

	pointGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points->size()) );

	return pointGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createPoints( std::vector<gVec3>& points, float size, osg::ref_ptr<osg::Vec4Array> colors )
{
	osg::ref_ptr<osg::Vec3Array> osgPoints = new osg::Vec3Array;
	
	osgPoints->resize( points.size() );
	std::vector<gVec3>::iterator it;
	for( int i=0; i<points.size(); ++i )
	{
		osgPoints->at(i).set(it->x(), it->y(), it->z());
	}

	return createPoints(osgPoints, size);
}

//osg::ref_ptr<osg::Geode> gOSGShape::createPoints( std::vector<gVec3*> *points, float size )
//{
//	osg::ref_ptr<osg::Vec3Array> osgPoints = new osg::Vec3Array;
//	
//	std::vector<gVec3*>::iterator it;
//	for( it=points->begin(); it != points->end(); it++ )
//	{
//		osgPoints->push_back( osg::Vec3( (*it)->x(), (*it)->y(), (*it)->z()) );
//	}
//
//	addPoints(osgPoints, color, size);
//}
//
//osg::ref_ptr<osg::Geode> gOSGShape::createVoxels( std::vector<gVec3> *points, gVec3 size )
//{
//	osg::ref_ptr<osg::Group> group = getNewGroup();
//
//	osg::ref_ptr<osg::Geode> voxelGeode = new osg::Geode;
//	group->addChild(voxelGeode);
//
//	// voxels
//	osg::ref_ptr<osg::StateSet> vxState = new osg::StateSet();
//	vxState->setAttribute( new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE) );
//	vxState->setMode(GL_LIGHTING, false);
//	vxState->setAttribute( new osg::LineWidth( 0.5f ) );
//	voxelGeode->setStateSet(vxState);
//
//	// box
//	osg::ref_ptr<osg::Box> voxelBox;
//	osg::ref_ptr<osg::ShapeDrawable> shape;
//
//	std::vector<gVec3>::iterator it;
//	for( it=points->begin(); it != points->end(); it++ )
//	{
//		voxelBox = new osg::Box( osg::Vec3(it->x(), it->y(), it->z()),
//								size.x(), size.y(), size.z() );		
//		
//		shape = new osg::ShapeDrawable( voxelBox );
//		shape->setColor(color);
//		voxelGeode->addDrawable( shape );
//	}
//}

osg::ref_ptr<osg::Geode> gOSGShape::createVoxels( osg::ref_ptr<osg::Vec3Array> points, gVec3 size )
{
	osg::ref_ptr<osg::Geode> voxelGeode = new osg::Geode;
	
	// voxels
	osg::ref_ptr<osg::StateSet> vxState = new osg::StateSet();
	vxState->setAttribute( new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE) );
	vxState->setMode(GL_LIGHTING, false);
	vxState->setAttribute( new osg::LineWidth( 0.5f ) );
	voxelGeode->setStateSet(vxState);

	// box
	osg::ref_ptr<osg::Box> voxelBox;
	osg::ref_ptr<osg::ShapeDrawable> shape;

	osg::Vec3Array::iterator it;
	for( it=points->begin(); it != points->end(); it++ )
	{
		voxelBox = new osg::Box( *it,
								size.x(), size.y(), size.z() );
		
		shape = new osg::ShapeDrawable( voxelBox );
		shape->setColor(color);
		voxelGeode->addDrawable( shape );
	}

	return voxelGeode;
}
osg::ref_ptr<osg::Geode> gOSGShape::createPlane(osg::Vec3 pos00, osg::Vec3 pos01, osg::Vec3 pos10, osg::Vec3 pos11 )
{
	
	// Draw Plane
	osg::ref_ptr<osg::Geometry> planeGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> planeGeode = new osg::Geode;
	planeGeode->addDrawable(planeGeometry);

	//StateSet
	osg::ref_ptr<osg::StateSet> state = planeGeode->getOrCreateStateSet();
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	//colors->push_back( color );
	colors->push_back(color);

	osg::ref_ptr<osg::Vec3Array> vertex = new osg::Vec3Array;
	vertex->push_back(pos00);
	vertex->push_back(pos01);
	vertex->push_back(pos10);
	vertex->push_back(pos11);


	planeGeometry->setVertexArray(vertex);
	planeGeometry->setColorArray(colors);
	planeGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	planeGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP, 0, vertex->size()));

	return planeGeode;
}
osg::ref_ptr<osg::Geode> gOSGShape::createPlane( osg::Vec3 normal, osg::Vec3 center, float width, float height, osg::Vec3 refPoint)
{
	osg::Vec3 RC, RCW, RCH;

	normal.normalize();
	RC = refPoint - center;
	RC.normalize();

	RCW = RC ^ normal;
	RCW.normalize();

	RCH = RCW ^ normal;
	RCH.normalize();

	RCW *= width/2;
	RCH *= height/2;

	// Draw Plane
	osg::ref_ptr<osg::Geometry> planeGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> planeGeode = new osg::Geode;
	planeGeode->addDrawable(planeGeometry);

	//StateSet
	osg::ref_ptr<osg::StateSet> state = planeGeode->getOrCreateStateSet();
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	//colors->push_back( color );
	colors->push_back( color );

	osg::ref_ptr<osg::Vec3Array> vertex = new osg::Vec3Array;
	vertex->push_back( center+RCW+RCH );
	vertex->push_back( center-RCW+RCH );
	vertex->push_back( center+RCW-RCH );
	vertex->push_back( center-RCW-RCH );


	planeGeometry->setVertexArray(vertex);
	planeGeometry->setColorArray(colors);
	planeGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	planeGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP, 0, vertex->size()) );

	return planeGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createLine( osg::ref_ptr<osg::Vec3Array> &points, float width, osg::ref_ptr<osg::Vec4Array> colors)
{
	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> LineGeode = new osg::Geode;
	LineGeode->addDrawable(lineGeometry);

	// state
	osg::ref_ptr<osg::StateSet> lineState = new osg::StateSet();
	lineState->setMode(GL_LIGHTING, false);
	lineState->setAttribute( new osg::LineWidth( width ) );
	LineGeode->setStateSet(lineState);

	// Init geometry
	if( colors->size() == 0 )
	{
		colors->push_back( color );
		lineGeometry->setColorArray(colors);
		lineGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	} else {
		lineGeometry->setColorArray(colors);
		lineGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	}
	
	lineGeometry->setVertexArray(points);
	//lineGeometry->setColorArray(colors);
	//lineGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	lineGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, points->size()) );
	lineGeometry->setUseDisplayList(false);

	return LineGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createTriangles( osg::ref_ptr<osg::Vec3Array>& points)
{
	osg::ref_ptr<osg::Geode> triGeode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> triGeometry = new osg::Geometry;

	osg::ref_ptr<osg::StateSet> state = new osg::StateSet;
	state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	//state->setAttribute(new osg::PolygonMode( osg::PolygonMode::FRONT, osg::PolygonMode::FILL));
	state->setMode( GL_BLEND, osg::StateAttribute::ON);
	state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 

	triGeode->setStateSet(state);
	triGeode->addDrawable(triGeometry);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( gOSGShape::color );
	triGeometry->setVertexArray(points);
	triGeometry->setColorArray(colors);
	triGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

	osg::Vec3Array*	normals = new osg::Vec3Array(points->size());
	triGeometry->setNormalArray(normals);
	triGeometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

	triGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, points->size()) );
	//triGeometry->setUseDisplayList(false);

	return triGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createDynamicTriangles( osg::ref_ptr<osg::Vec3Array> &points)
{
	osg::ref_ptr<osg::Geode> triGeode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> triGeometry = new osg::Geometry;
	//triGeometry->setCullingActive(false);

	osg::ref_ptr<osg::StateSet> state = new osg::StateSet;
	state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	//state->setAttribute(new osg::PolygonMode( osg::PolygonMode::FRONT, osg::PolygonMode::FILL));
	state->setMode( GL_BLEND, osg::StateAttribute::ON);
	state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 

	triGeode->setStateSet(state);
	triGeode->addDrawable(triGeometry);

	// Init geometry
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( gOSGShape::color );
	triGeometry->setVertexArray(points);
	triGeometry->setColorArray(colors);
	triGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

	osg::Vec3Array*	normals = new osg::Vec3Array(points->size());
	triGeometry->setNormalArray(normals);
	triGeometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

	triGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, points->size()) );
		
	triGeometry->setSupportsDisplayList(false);
	triGeometry->setUseDisplayList(false);
	triGeometry->setDataVariance(osg::Object::DYNAMIC);


	//osg::ref_ptr<gOSGShape::dynUpdateCallback> update = new gOSGShape::dynUpdateCallback;
	//triGeode->setUpdateCallback(update);

	return triGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createDynamicLine( osg::ref_ptr<osg::Vec3Array> &points, float width, osg::ref_ptr<osg::Vec4Array> colors)
{
	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> LineGeode = new osg::Geode;
	LineGeode->setCullingActive(false);
	LineGeode->addDrawable(lineGeometry);

	// state
	osg::ref_ptr<osg::StateSet> lineState = new osg::StateSet();
	lineState->setMode(GL_LIGHTING, false);
	lineState->setAttribute( new osg::LineWidth( width ) );
	LineGeode->setStateSet(lineState);

	// Init geometry
	lineGeometry->setVertexArray(points);

	if( colors->size() == 0 )
	{
		colors->push_back( color );
		lineGeometry->setColorArray(colors);
		lineGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	} else {
		lineGeometry->setColorArray(colors);
		lineGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	}
	lineGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, points->size()) );
	lineGeometry->setUseDisplayList(false);
	lineGeometry->setSupportsDisplayList(false);
	lineGeometry->setDataVariance(osg::Object::DYNAMIC);
	//osg::ref_ptr<gOSGShape::dynUpdateCallback> update = new gOSGShape::dynUpdateCallback;
	//LineGeode->setUpdateCallback(update);

	return LineGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createDynamicLines( osg::ref_ptr<osg::Vec3Array> &points, float width, osg::ref_ptr<osg::Vec4Array> colors)
{
	osg::ref_ptr<osg::Geometry> lineGeometry = new osg::Geometry();
	osg::ref_ptr<osg::Geode> LineGeode = new osg::Geode;
	LineGeode->setCullingActive(false);
	LineGeode->addDrawable(lineGeometry);

	// state
	osg::ref_ptr<osg::StateSet> lineState = new osg::StateSet();
	lineState->setMode(GL_LIGHTING, false);
	lineState->setAttribute( new osg::LineWidth( width ) );
	LineGeode->setStateSet(lineState);

	// Init geometry
	lineGeometry->setVertexArray(points);

	if( colors->size() == 0 )
	{
		colors->push_back( color );
		lineGeometry->setColorArray(colors);
		lineGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	} else {
		lineGeometry->setColorArray(colors);
		lineGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	}

	lineGeometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, points->size()) );
	lineGeometry->setUseDisplayList(false);
	lineGeometry->setSupportsDisplayList(false);
	lineGeometry->setDataVariance(osg::Object::DYNAMIC);
	//osg::ref_ptr<gOSGShape::dynUpdateCallback> update = new gOSGShape::dynUpdateCallback;
	//LineGeode->setUpdateCallback(update);

	return LineGeode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createText(std::string text, osg::Vec3 pos, float size)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osgText::Text> osgText = new osgText::Text;

	geode->addDrawable(osgText);
	osg::ref_ptr<osgText::Font> hudFont = osgText::readFontFile("C:/WINDOWS/Fonts/verdena.ttf");
	osgText->setFont(hudFont);
	osgText->setCharacterSize(size);
	osgText->setAlignment(osgText::Text::LEFT_TOP);
	//osgText->setAlignment(osgText::Text::CENTER_TOP);
	//osgText->setDataVariance(osg::Object::DYNAMIC);
	osgText->setColor( color );
	osgText->setText( text );

	//osgText->setAxisAlignment( osgText::Text::REVERSED_XZ_PLANE );
	osgText->setAutoRotateToScreen(true);
	osgText->setPosition( pos );

	return geode;
}

osg::ref_ptr<osg::Geode> gOSGShape::createDynamicText(std::string& text, osg::Vec3 pos, float size)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->setCullingActive(false);

	osg::ref_ptr<osgText::Text> osgText = new osgText::Text;
	geode->addDrawable(osgText);

	osg::ref_ptr<osg::StateSet> state = new osg::StateSet();
	state->setMode(GL_LIGHTING, false);
	//state->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	//state->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
	//state->setRenderBinDetails( 11, "RenderBin");
	
	geode->setStateSet(state);

	osg::ref_ptr<osgText::Font> hudFont = osgText::readFontFile("C:/WINDOWS/Fonts/verdena.ttf");
	osgText->setFont(hudFont);
	osgText->setCharacterSize(size);
	//osgText->setAlignment(osgText::Text::RIGHT_TOP);
	osgText->setDataVariance(osg::Object::DYNAMIC);
	osgText->setColor( color );
	osgText->setText( text );

	//osgText->setAxisAlignment( osgText::Text::SCREEN );
	osgText->setAutoRotateToScreen(true);
	osgText->setPosition( pos );

	return geode;
}

osg::ref_ptr<osg::MatrixTransform> gOSGShape::createArrow(const osg::Vec3& pos, const osg::Vec3& dir, const float length, const float radius)
{
	const double coneRadius = radius * 1.5;
	const double coneHeight = coneRadius * 2.;
	const double cylinderLength = length-coneHeight-radius/2.;

	if( length < coneHeight )
	{
		return new osg::MatrixTransform;
	}

	osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;

	osg::ref_ptr<osg::Geode> cylinderGeode = new osg::Geode;
	osg::ref_ptr<osg::Cylinder> cylinder = new osg::Cylinder(osg::Vec3(0, 0, cylinderLength/2.), radius, cylinderLength);
	osg::ref_ptr<osg::ShapeDrawable> cylinderShape = new osg::ShapeDrawable(cylinder);
	cylinderShape->setColor(color);
	cylinderGeode->addDrawable(cylinderShape);
	trans->addChild( cylinderGeode );

	// state
	osg::ref_ptr<osg::StateSet> capState = new osg::StateSet();
	capState->setMode(GL_LIGHTING, false);
	cylinderGeode->setStateSet(capState);

	// cone
	osg::ref_ptr<osg::Geode> coneGeode = new osg::Geode;
	osg::ref_ptr<osg::Cone> cone = new osg::Cone(osg::Vec3(0,0,0), coneRadius, coneHeight);

	osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(cone);
	shape->setColor(color);
	coneGeode->addDrawable(shape);

	osg::ref_ptr<osg::MatrixTransform> coneTrans = new osg::MatrixTransform;
	osg::Matrixd coneMat;
	coneMat.setTrans(osg::Vec3(0, 0, length - coneHeight));
	coneTrans->setMatrix(coneMat);

	coneTrans->addChild(coneGeode);
	trans->addChild(coneTrans);


	osg::Quat rot;
	rot.makeRotate(osg::Vec3(0,0,1), dir / dir.length());

	osg::Matrixd mat(rot);
	mat.setTrans(pos);
	
	trans->setMatrix(mat);

	return trans;
}
