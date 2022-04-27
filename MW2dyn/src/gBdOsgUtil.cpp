//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#include "Visualizer/gBDOSGUtil.h"

#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>

osg::Matrix asOsgMatrix( const btTransform& t )
{
    btScalar ogl[ 16 ];
    t.getOpenGLMatrix( ogl );
    osg::Matrix m( ogl );
    return m;
}

btTransform sBtTransform( const osg::Matrix& m )
{
    const osg::Matrix::value_type* oPtr = m.ptr();
    btScalar bPtr[ 16 ];
    int idx;
    for (idx=0; idx<16; idx++)
        bPtr[ idx ] = oPtr[ idx ];
    btTransform t;
    t.setFromOpenGLMatrix( bPtr );
    return t;
}

osg::Matrix asOsgMatrix( const btMatrix3x3& m )
{
    btScalar f[ 9 ];
    m.getOpenGLSubMatrix( f );
    return( osg::Matrix(
        f[0], f[1], f[2], 0.,
        f[3], f[4], f[5], 0.,
        f[6], f[7], f[8], 0.,
        0., 0., 0., 1. ) );
}

btMatrix3x3 asBtMatrix3x3( const osg::Matrix& m )
{
    return( btMatrix3x3(
        m(0,0), m(0,1), m(0,2),
        m(1,0), m(1,1), m(1,2),
        m(2,0), m(2,1), m(2,2) ) );
}

osg::Vec3 asOsgVec3( const btVector3& v )
{
    return osg::Vec3( v.x(), v.y(), v.z() );
}

btVector3 asBtVector3( const osg::Vec3& v )
{
    return btVector3( v.x(), v.y(), v.z() );
}

osg::Vec4 asOsgVec4( const btVector3& v, const double w )
{
    return osg::Vec4( v.x(), v.y(), v.z(), w );
}

osg::Vec4 asOsgVec4( const btVector4& v )
{
    return osg::Vec4( v.x(), v.y(), v.z(), v.w() );
}

btVector4 asBtVector4( const osg::Vec4& v )
{
    return btVector4( v.x(), v.y(), v.z(), v.w() );
}


// ===================================================================================================================
// BT BODY -> OSG
// ===================================================================================================================

osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btCollisionShape* btShape, const btTransform& trans )
{
    if( btShape->getShapeType() == BOX_SHAPE_PROXYTYPE )
    {
		const btBoxShape* btBox = static_cast< const btBoxShape* >( btShape );
        return( osgNodeFromBtCollisionShape( btBox, trans ) );
    }
    else if( btShape->getShapeType() == SPHERE_SHAPE_PROXYTYPE )
    {
        const btSphereShape* btSphere = static_cast< const btSphereShape* >( btShape );
        return( osgNodeFromBtCollisionShape( btSphere, trans ) );
    }
    else if( btShape->getShapeType() == CYLINDER_SHAPE_PROXYTYPE )
    {
        const btCylinderShape* btCylinder = static_cast< const btCylinderShape* >( btShape );
        return( osgNodeFromBtCollisionShape( btCylinder, trans ) );
    }
    else if( btShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE )
    {
        const btBvhTriangleMeshShape* btTriMesh = static_cast< const btBvhTriangleMeshShape* >( btShape );
        // Do NOT pass in a transform. Unlike cylinder, sphere, and box,
        // tri meshes are always in absolute space.
        return( osgNodeFromBtCollisionShape( btTriMesh, trans ) );
    }
	else if( btShape->getShapeType() == CAPSULE_SHAPE_PROXYTYPE )
	{
        const btCapsuleShape* btCapsule = static_cast< const btCapsuleShape* >( btShape );
        return( osgNodeFromBtCollisionShape( btCapsule, trans ) );
	}
	/*
    else if( btShape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE )
    {
        const btConvexTriangleMeshShape* btConvexTriMesh = static_cast< const btConvexTriangleMeshShape* >( btShape );
        // Do NOT pass in a transform. Unlike cylinder, sphere, and box,
        // tri meshes are always in absolute space.
        return( osgNodeFromBtCollisionShape( btConvexTriMesh ) );
    }
    else if( btShape->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE )
    {
        const btConvexHullShape* convexHull = static_cast< const btConvexHullShape* >( btShape );
        // Do NOT pass in a transform. Unlike cylinder, sphere, and box,
        // tri meshes are always in absolute space.
        return( osgNodeFromBtCollisionShape( convexHull ) );
    }
    else if( btShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE )
    {
        const btCompoundShape* masterShape = static_cast< const btCompoundShape* >( btShape );
        osg::Group* grp = new osg::Group;
        int idx;
        for (idx=0; idx< masterShape->getNumChildShapes(); idx++)
        {
            const btCollisionShape* s = masterShape->getChildShape( idx );
            const btTransform t = masterShape->getChildTransform( idx );
            const btTransform accumTrans = trans * t;
            grp->addChild( osgNodeFromBtCollisionShape( s, accumTrans ) );
        }
        return( grp );
    }*/
    else
    {
#ifdef _DEBUG
		printf("osgNodeFromBtCollisionShape: Unsupported shape type: %d\n",  btShape->getShapeType() );
#endif
		/*
        osg::notify( osg::WARN ) << "osgNodeFromBtCollisionShape: Unsupported shape type: " <<
        btShape->getShapeType() << std::endl;*/
        return( NULL );
    }
}

osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btBoxShape* btBox, const btTransform& trans )
{
	btVector3 length = btBox->getHalfExtentsWithMargin() * 2;

	osg::ref_ptr<osg::Geode> geode = gOSGShape::createBoxShape( asOsgVec3(length), osg::Vec3(0,0,0) );

    osg::Matrix m = asOsgMatrix( trans );
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->setMatrix( m );
    mt->addChild( geode );

    return mt;
}

osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btSphereShape* btSphere, const btTransform& trans )
{
	osg::Geode* geode = gOSGShape::createSphereShape( btSphere->getRadius(), osg::Vec3(0,0,0) );

    osg::Matrix m = asOsgMatrix( trans );
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    mt->setMatrix( m );
    mt->addChild( geode );

    return mt;
}

osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btCylinderShape * btCylinder, const btTransform& trans )
{
    osg::Cylinder* cylinder = new osg::Cylinder();
    cylinder->setRadius( btCylinder->getRadius() );

    switch( btCylinder->getUpAxis() )
    {
        case gOSGShape::X:
            cylinder->setHeight( 2 * btCylinder->getHalfExtentsWithMargin().getX() );
            cylinder->setRotation( osg::Quat( osg::PI_2, osg::Vec3( 0, 1, 0 ) ) );
            break;
        case gOSGShape::Y:
            cylinder->setHeight( 2 * btCylinder->getHalfExtentsWithMargin().getY() );
            cylinder->setRotation( osg::Quat( osg::PI_2, osg::Vec3( 1, 0, 0 ) ) );
            break;
        case gOSGShape::Z:
            cylinder->setHeight( 2 * btCylinder->getHalfExtentsWithMargin().getZ() );
    }

    osg::TessellationHints* hints = new osg::TessellationHints();
    hints->setDetailRatio( 1.0f );

    osg::ShapeDrawable* shape = new osg::ShapeDrawable( cylinder, hints );
    shape->setColor( gOSGShape::color );
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( shape );

    osg::Matrix m = asOsgMatrix( trans );
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    mt->setMatrix( m );
    mt->addChild( geode );
    return mt;
}

// Add 2012-10-09
osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btTriangleMeshShape* btTriMesh, const btTransform& trans )
{
    const btTriangleMesh* mesh = dynamic_cast< const btTriangleMesh* >( btTriMesh->getMeshInterface() );
    if( !mesh )
    {
        osg::notify( osg::FATAL ) << "osgNodeFromBtCollisionShape: No triangle mesh." << std::endl;
        return( NULL );
    }

    btVector3* verts;
    int* indices;
    int numVerts;
    int numFaces;
    PHY_ScalarType vt, ft;
    int vs, fs;

    mesh->getLockedReadOnlyVertexIndexBase( ( const unsigned char** )&verts, numVerts, vt, vs, ( const unsigned char** )&indices, fs, numFaces, ft );

	osg::ref_ptr<osg::Vec3Array> vec = new osg::Vec3Array();
    vec->resize( numVerts );
    int idx;
    for( idx = 0; idx < numVerts; idx++ )
    {
        const btVector3& bulletVert = verts[ idx ];
        ( *vec )[ idx ].set( bulletVert.getX(), bulletVert.getY(), bulletVert.getZ() );
    }

	osg::ref_ptr<osg::DrawElementsUInt> deui = new osg::DrawElementsUInt( GL_TRIANGLES );
    for( idx = 0; idx < numFaces * 3; idx++ )
        deui->push_back( indices[ idx ] );

    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array();
    color->push_back( osg::Vec4( 1., 1., 1., 1. ) );

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray( vec );
    geom->setColorArray( color );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    geom->addPrimitiveSet( deui );

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable( geom );

    osg::Matrix m = asOsgMatrix( trans );
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->setMatrix(m);
	mt->addChild( geode );

	return mt;
}

osg::ref_ptr<osg::MatrixTransform> osgNodeFromBtCollisionShape( const btCapsuleShape* btCapsule, const btTransform& trans )
{
	float radius = btCapsule->getRadius();
	float height = btCapsule->getHalfHeight() * 2;
	
	osg::ref_ptr<osg::MatrixTransform> _transMat = new osg::MatrixTransform;
	osg::ref_ptr<osg::Geode> geode = gOSGShape::createCapsuleShape(osg::Vec3(0,0,0), radius, height, btCapsule->getUpAxis());
	//geode->setDataVariance(osg::Object::STATIC);
	
	osg::Matrix m = asOsgMatrix( trans );
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    mt->setMatrix( m );
    mt->addChild( geode );
    return mt;
}


// ===================================================================================================================
// BT CONSTRAINT -> OSG
// ===================================================================================================================
int osgNodeFromBtConstraint( btTypedConstraint* constraint, osg::ref_ptr<osg::MatrixTransform> transA, osg::ref_ptr<osg::MatrixTransform> transB )
{
	osg::Vec4 oldColor = gOSGShape::color;
	gOSGShape::color.set(0,0,0,1);


	/*bool drawFrames = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraints) != 0;
	bool drawLimits = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraintLimits) != 0;
*/

	// TODO
	bool drawFrames = true;
	bool drawLimits = true;
	double _width = 3.0;


	osg::Matrix trOsg;
	btScalar dbgDrawSize = constraint->getDbgDrawSize();
	if( !transA.valid() ) transA = new osg::MatrixTransform;
	if( !transB.valid() ) transB = new osg::MatrixTransform;

	if(constraint->getConstraintType() == POINT2POINT_CONSTRAINT_TYPE)
    {
		btPoint2PointConstraint* p2pC = static_cast<btPoint2PointConstraint*>(constraint);
	
		btVector3 pivot = p2pC->getPivotInA();
		trOsg.identity();
		trOsg.setTrans( pivot.x(), pivot.y(), pivot.z() );
		transA->setMatrix(trOsg);
		transA->addChild( gOSGShape::createAxis( gOSGShape::_width, dbgDrawSize ) );

		// that ideally should draw the same frame	
		pivot = p2pC->getPivotInB();
		trOsg.setTrans( pivot.x(), pivot.y(), pivot.z() );
		transB->setMatrix(trOsg);
		transB->addChild( gOSGShape::createAxis( gOSGShape::_width, dbgDrawSize ) );
	} else if(constraint->getConstraintType() == HINGE_CONSTRAINT_TYPE)
    {
		btHingeConstraint* pHinge = static_cast<btHingeConstraint*>(constraint);

		pHinge->getAFrame().getOpenGLMatrix(trOsg.ptr());
		transA->setMatrix(trOsg);
		//transA->addChild( createAxis( _width, dbgDrawSize ) );
		

		pHinge->getBFrame().getOpenGLMatrix(trOsg.ptr());
		transB->setMatrix(trOsg);
		//transB->addChild( createAxis( _width, dbgDrawSize ) );
		gOSGShape::setColor( osg::Vec4(0,0,0,1) );
		transB->addChild( gOSGShape::createLineShape(osg::Vec3(0,0,0), osg::Vec3(1,0,0), dbgDrawSize, gOSGShape::_width ) );

		btScalar minAng = pHinge->getLowerLimit();
		btScalar maxAng = pHinge->getUpperLimit();
		if(minAng == maxAng)
		{
			return 0;
		}
		bool drawSect = true;
		if(minAng > maxAng)
		{
			minAng = btScalar(0.f);
			maxAng = SIMD_2_PI;
			drawSect = false;
		}
		transA->addChild( gOSGShape::createArc(osg::Vec3(0,0,0), osg::Vec3(0,0,1), osg::Vec3(1,0,0), dbgDrawSize, dbgDrawSize, minAng, maxAng, drawSect) );

	} else if(constraint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE)
    {
		btConeTwistConstraint* pCT = static_cast<btConeTwistConstraint*>(constraint);
		pCT->getAFrame().getOpenGLMatrix(trOsg.ptr());
		transA->setMatrix(trOsg);
		//transA->addChild( createAxis( _width, dbgDrawSize ) );
		gOSGShape::setColor( osg::Vec4(0,0,0,1) );
		transA->addChild( gOSGShape::createLineShape(osg::Vec3(0,0,0), osg::Vec3(1,0,0), dbgDrawSize, gOSGShape::_width ) );

		pCT->getBFrame().getOpenGLMatrix(trOsg.ptr());
		transB->setMatrix(trOsg);
		//transB->addChild( createAxis( _width, dbgDrawSize ) );
		
		const btScalar length = dbgDrawSize;
		static int nSegments = 8*4;
		btScalar fAngleInRadians = btScalar(2.*3.1415926) * (btScalar)(nSegments-1)/btScalar(nSegments);
		btVector3 pPrev = pCT->GetPointForAngle(fAngleInRadians, length);
		
		osg::ref_ptr<osg::Vec3Array> vertex = new osg::Vec3Array;
		vertex->push_back( osg::Vec3(pPrev.x(), pPrev.y(), pPrev.z()) );
		for (int i=0; i<nSegments; i++)
		{
			fAngleInRadians = btScalar(2.*3.1415926) * btScalar(i)/btScalar(nSegments);
			
			btVector3 pCur = pCT->GetPointForAngle(fAngleInRadians, length);
			vertex->push_back( osg::Vec3(pCur.x(), pCur.y(), pCur.z()) );

			if (i%(nSegments/8) == 0)
			{
				vertex->push_back( osg::Vec3(0, 0, 0) );
				vertex->push_back( osg::Vec3(pCur.x(), pCur.y(), pCur.z()) );
			}
		}
		transB->addChild( gOSGShape::createVertexShape(vertex, osg::PrimitiveSet::LINE_STRIP) );

		btScalar tws = pCT->getTwistSpan();
		btScalar twa = pCT->getTwistAngle();
		bool useFrameB = (pCT->getRigidBodyB().getInvMass() > btScalar(0.f));
		/*
		if(useFrameB)
		{
			transB->addChild(createArc(osg::Vec3(0,0,0), osg::Vec3(1,0,0), osg::Vec3(0,1,0), dbgDrawSize, dbgDrawSize, -twa-tws, -twa+tws, true));
		}
		else
		{
			transA->addChild(createArc(osg::Vec3(0,0,0), osg::Vec3(1,0,0), osg::Vec3(0,1,0), dbgDrawSize, dbgDrawSize, -twa-tws, -twa+tws, true));
		}
		*/
	} else if(constraint->getConstraintType() == D6_CONSTRAINT_TYPE || constraint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE)
    {
		btGeneric6DofConstraint* p6DOF = (btGeneric6DofConstraint*)constraint;
		p6DOF->getFrameOffsetA().getOpenGLMatrix(trOsg.ptr());
		//p6DOF->getCalculatedTransformA().getOpenGLMatrix(trOsg.ptr());
		transA->setMatrix(trOsg);
		if(drawFrames) transA->addChild( gOSGShape::createAxis( _width, dbgDrawSize ) );

		p6DOF->getFrameOffsetB().getOpenGLMatrix(trOsg.ptr());
		transB->setMatrix(trOsg);
		if(drawFrames) transB->addChild( gOSGShape::createAxis( _width, dbgDrawSize ) );

		/*if(drawLimits) 
		{
			tr = p6DOF->getCalculatedTransformA();
			const btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
			btVector3 up = tr.getBasis().getColumn(2);
			btVector3 axis = tr.getBasis().getColumn(0);
			btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
			btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
			btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
			btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
			getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btScalar(.9f), minTh, maxTh, minPs, maxPs, btVector3(0,0,0));
			axis = tr.getBasis().getColumn(1);
			btScalar ay = p6DOF->getAngle(1);
			btScalar az = p6DOF->getAngle(2);
			btScalar cy = btCos(ay);
			btScalar sy = btSin(ay);
			btScalar cz = btCos(az);
			btScalar sz = btSin(az);
			btVector3 ref;
			ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
			ref[1] = -sz*axis[0] + cz*axis[1];
			ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
			tr = p6DOF->getCalculatedTransformB();
			btVector3 normal = -tr.getBasis().getColumn(0);
			btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
			btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
			if(minFi > maxFi)
			{
				getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0,0,0), false);
			}
			else if(minFi < maxFi)
			{
				getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0,0,0), true);
			}
			tr = p6DOF->getCalculatedTransformA();
			btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
			btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
			getDebugDrawer()->drawBox(bbMin, bbMax, tr, btVector3(0,0,0));
		}*/

	} else if(constraint->getConstraintType() == SLIDER_CONSTRAINT_TYPE)
    {
		printf("not supported constraint.\n");
	} else {
		printf("ERROR : can not detect the type of constraint.\n");
		return -1;
	}
	
	gOSGShape::color = oldColor;

	return 0;
}

/*
void btDiscreteDynamicsWorld::debugDrawConstraint(btTypedConstraint* constraint)
{
	bool drawFrames = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraints) != 0;
	bool drawLimits = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraintLimits) != 0;
	btScalar dbgDrawSize = constraint->getDbgDrawSize();

	switch(constraint->getConstraintType())
	{
		case D6_SPRING_CONSTRAINT_TYPE:
		case D6_CONSTRAINT_TYPE:
			{
				btGeneric6DofConstraint* p6DOF = (btGeneric6DofConstraint*)constraint;
				btTransform tr = p6DOF->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = p6DOF->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits) 
				{
					tr = p6DOF->getCalculatedTransformA();
					const btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
					btVector3 up = tr.getBasis().getColumn(2);
					btVector3 axis = tr.getBasis().getColumn(0);
					btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
					btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
					btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
					btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
					getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btScalar(.9f), minTh, maxTh, minPs, maxPs, btVector3(0,0,0));
					axis = tr.getBasis().getColumn(1);
					btScalar ay = p6DOF->getAngle(1);
					btScalar az = p6DOF->getAngle(2);
					btScalar cy = btCos(ay);
					btScalar sy = btSin(ay);
					btScalar cz = btCos(az);
					btScalar sz = btSin(az);
					btVector3 ref;
					ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
					ref[1] = -sz*axis[0] + cz*axis[1];
					ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
					tr = p6DOF->getCalculatedTransformB();
					btVector3 normal = -tr.getBasis().getColumn(0);
					btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
					btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
					if(minFi > maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0,0,0), false);
					}
					else if(minFi < maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0,0,0), true);
					}
					tr = p6DOF->getCalculatedTransformA();
					btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
					btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
					getDebugDrawer()->drawBox(bbMin, bbMax, tr, btVector3(0,0,0));
				}
			}
			break;
		case SLIDER_CONSTRAINT_TYPE:
			{
				btSliderConstraint* pSlider = (btSliderConstraint*)constraint;
				btTransform tr = pSlider->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pSlider->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					btTransform tr = pSlider->getUseLinearReferenceFrameA() ? pSlider->getCalculatedTransformA() : pSlider->getCalculatedTransformB();
					btVector3 li_min = tr * btVector3(pSlider->getLowerLinLimit(), 0.f, 0.f);
					btVector3 li_max = tr * btVector3(pSlider->getUpperLinLimit(), 0.f, 0.f);
					getDebugDrawer()->drawLine(li_min, li_max, btVector3(0, 0, 0));
					btVector3 normal = tr.getBasis().getColumn(0);
					btVector3 axis = tr.getBasis().getColumn(1);
					btScalar a_min = pSlider->getLowerAngLimit();
					btScalar a_max = pSlider->getUpperAngLimit();
					const btVector3& center = pSlider->getCalculatedTransformB().getOrigin();
					getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, a_min, a_max, btVector3(0,0,0), true);
				}
			}
			break;
		default : 
			break;
	}
	return;
}*/