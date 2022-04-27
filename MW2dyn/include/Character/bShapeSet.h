//#####################################################################
// Copyright 2010-2015, Hynchul Choi, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _BULLET_SHAPE_SET_IN_MBS_H_
#define _BULLET_SHAPE_SET_IN_MBS_H_

#include "Base/gMath.h"
#include <BulletCollision/CollisionShapes/btCollisionShape.h>

/**
* MBS-BD Visual System Shape Definition
**/
class bShapeSet {
public:
	enum Shape { Box=1, CapsuleX, CapsuleY, CapsuleZ, Sphere };

	bShapeSet() {};
	~bShapeSet();

	// setting the collision shpae
	void setShape(std::string name, btCollisionShape* shape);
	void setVisShape(std::string name, std::string file_path);
	void setShapeXform(std::string name, gXMat& mat);

	btCollisionShape* getShape(std::string name);
	std::string* getVisShapePath(std::string name);
	gXMat& getShapeXform(std::string name);
	std::size_t size() { return m_shapes.size(); }

protected:
	typedef std::map<std::string, btCollisionShape*>			ShapeMap;
	typedef std::map<std::string, btCollisionShape*>::iterator	ShapeMIt;
	typedef std::map<std::string, std::string>					VisShapeMap;
	typedef std::map<std::string, std::string>::iterator		VisShapeMIt;
	typedef std::map<std::string, gXMat>						ShapeXformMap; 
	typedef std::map<std::string, gXMat>::iterator				ShapeXformMIt;

	ShapeMap m_shapes;
	ShapeXformMap m_shapeXforms; //2014-5-29 SHL: xform matrix for the shape wrt body frame
	VisShapeMap m_visShape;	
};




#endif