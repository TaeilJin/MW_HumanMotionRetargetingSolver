//#####################################################################
// Copyright 2010-2015, Hynchul Choi, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#include "Character/bShapeSet.h"

/*
btCollisionShape* ShapeInfo::getCollisionShape()
{
	assert( shape > 0 );

	if( shape == bShapeSet::Box ) {
		return new btBoxShape(length);
	}
	else if( shape == bShapeSet::CapsuleX) {
		return new btCapsuleShapeX(length.x(), length.y());
	}
	else if( shape == bShapeSet::CapsuleY) {
		return new btCapsuleShape(length.x(), length.y());
	}
	else if( shape == bShapeSet::CapsuleZ) {
		return new btCapsuleShapeZ(length.x(), length.y());
	}
	else if( shape == bShapeSet::Sphere ) {
		return new btSphereShape(length.x());
	}
	
	return NULL;
}

char* ShapeInfo::getShapeName()
{
	assert( shape > 0 );

	if( shape == bShapeSet::Box ) {
		return "BOX";
	}
	else if( shape == bShapeSet::CapsuleX ) {
		return "CAPSULEX";
	}
	else if( shape == bShapeSet::CapsuleY ) {
		return "CAPSULEY";
	}
	else if( shape == bShapeSet::CapsuleZ ) {
		return "CAPSULEZ";
	}
	else if( shape == bShapeSet::Sphere ) {
		return "SPHERE";
	}

	return NULL;
}*/

bShapeSet::~bShapeSet()
{
	/*
	ShapeMIt it;
	for( it=m_shapes.begin(); it!=m_shapes.end(); it++ )
	{
		delete it->second;
	}*/
}

void bShapeSet::setShape(std::string name, btCollisionShape *shape)
{
	if( m_shapes.find(name) == m_shapes.end() )
	{
		m_shapes[name] = shape;
	} else {
		printf("Duplicated assigned link.\n");
	}
}

void bShapeSet::setShapeXform(std::string name, gXMat& mat)
{
	if( m_shapeXforms.find(name) == m_shapeXforms.end() )
	{
		m_shapeXforms[name] = mat;
	} else {
		printf("Duplicated assigned link.\n");
	}
}

void bShapeSet::setVisShape(std::string name, std::string file_path)
{
	if( m_visShape.find(name) == m_visShape.end() )
	{
		m_visShape[name] = file_path;
	} else {
		printf("Duplicated assigned link.(setVisShape)\n");
	}
}

btCollisionShape* bShapeSet::getShape(std::string name)
{
	ShapeMIt it =  m_shapes.find(name);
	if( it == m_shapes.end() ) return NULL;
	return it->second;
}

gXMat& bShapeSet::getShapeXform(std::string name)
{
	ShapeXformMIt it =  m_shapeXforms.find(name);
	if( it == m_shapeXforms.end() ) return gXMat(-1e10,-1e10,-1e10);
	return it->second;
}

std::string* bShapeSet::getVisShapePath(std::string name)
{
	VisShapeMIt it = m_visShape.find(name);
	if( it == m_visShape.end() ) return NULL;
	return &it->second;
}
