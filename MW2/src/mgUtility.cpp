//#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#include "MocapProcessor/mgUtility.h"
#include <bitset>
#include <sstream>

//using namespace std;

int mgUtility::getAngleFromRotMat(gVec3 Axis, const gRotMat &R, double& outAngle)
{
	Axis.normalize();
	gVec3 x = gVec3::log(R);
	outAngle = (Axis, x);
	return 0;
}

gVec3 mgUtility::getZXYFromRotation(const gRotMat &rotMat)
{
	double x, y, z;

	if( rotMat.e(gRotMat::R21) < 1.0 )
	{
		if( rotMat.e(gRotMat::R21) > -1.0 )
		{
			x = asin( rotMat.e(gRotMat::R21) );
			z = atan2( -rotMat.e(gRotMat::R01), rotMat.e(gRotMat::R11) );
			y = atan2( -rotMat.e(gRotMat::R20), rotMat.e(gRotMat::R22) );
		} else {
			x = -gPI/2.0;
			z = -atan2( rotMat.e(gRotMat::R02), rotMat.e(gRotMat::R00) );
			y = 0;
		}
	} else {
		x = gPI/2.0;
		z = atan2( rotMat.e(gRotMat::R02), rotMat.e(gRotMat::R00) );
		y = 0;
	}
	return gVec3(x, y, z);
}
gVec3 mgUtility::getXYZFromRotation(const gRotMat &rotMat)
{
	double x, y, z;

	if( rotMat.e(gRotMat::R02) < 1.0 )
	{
		if( rotMat.e(gRotMat::R02) > -1.0 )
		{
			x = atan2( -rotMat.e(gRotMat::R12), rotMat.e(gRotMat::R22) );
			y = asin( rotMat.e(gRotMat::R02));
			z = atan2( -rotMat.e(gRotMat::R01), rotMat.e(gRotMat::R00) );
		} else {
			x = -atan2( rotMat.e(gRotMat::R10), rotMat.e(gRotMat::R11) );
			y = -gPI/2.0;
			z = 0;
		}
	} else {
		x = atan2( rotMat.e(gRotMat::R10), rotMat.e(gRotMat::R11) );
		y = gPI/2.0;
		z = 0;
	}
	return gVec3(x, y, z);
}
gVec3 mgUtility::getXZYFromRotation(const gRotMat &rotMat)
{
	double x, y, z;

	if( rotMat.e(gRotMat::R01) < 1.0 )
	{
		if( rotMat.e(gRotMat::R01) > -1.0 )
		{
			x = atan2( rotMat.e(gRotMat::R21), rotMat.e(gRotMat::R11) );
			y = atan2( rotMat.e(gRotMat::R02), rotMat.e(gRotMat::R00) );
			z = asin( -rotMat.e(gRotMat::R01) );
		} else {
			x = atan2( -rotMat.e(gRotMat::R20), rotMat.e(gRotMat::R22) );
			y = 0;
			z = gPI/2.0;
		}
	} else {
		x = atan2( -rotMat.e(gRotMat::R20), rotMat.e(gRotMat::R22) );
		y = 0;
		z = -gPI/2.0;
	}
	return gVec3(x, y, z);
}
gVec3 mgUtility::getYXZFromRotation(const gRotMat &rotMat)
{
	double x, y, z;

	if (rotMat.e(gRotMat::R12) < 1.0)
	{
		if (rotMat.e(gRotMat::R12) > -1.0)
		{
			x = asin(-rotMat.e(gRotMat::R12));
			y = atan2(rotMat.e(gRotMat::R02), rotMat.e(gRotMat::R22));
			z = atan2(rotMat.e(gRotMat::R10), rotMat.e(gRotMat::R11));
		}
		else {
			x = gPI / 2.0;
			y = -atan2(-rotMat.e(gRotMat::R01), rotMat.e(gRotMat::R00));
			z = 0.;
		}
	}
	else {
		x = -gPI / 2.0;
		y = atan2(-rotMat.e(gRotMat::R01), rotMat.e(gRotMat::R00));
		z = 0.0;
	}
	return gVec3(x, y, z);
}
gVec3 mgUtility::getYZXFromRotation(const gRotMat &rotMat)
{
	double x, y, z;

	if( rotMat.e(gRotMat::R10) < 1.0 )
	{
		if( rotMat.e(gRotMat::R10) > -1.0 )
		{
			x = atan2( -rotMat.e(gRotMat::R12), rotMat.e(gRotMat::R11) );
			y = atan2( -rotMat.e(gRotMat::R20), rotMat.e(gRotMat::R00) );
			z = asin( rotMat.e(gRotMat::R10));
		} else {
			x = 0;
			y = -atan2( rotMat.e(gRotMat::R21), rotMat.e(gRotMat::R22) );
			z = -gPI/2.0;
		}
	} else {
		x = 0;
		y = atan2( rotMat.e(gRotMat::R21), rotMat.e(gRotMat::R22) );
		z = gPI/2.0;
	}
	return gVec3(x, y, z);
}
gVec3 mgUtility::getZYXFromRotation(const gRotMat &rotMat)
{
	double x, y, z;

	if( rotMat.e(gRotMat::R20) < 1.0 )
	{
		if( rotMat.e(gRotMat::R20) > -1.0 )
		{
			x = atan2( rotMat.e(gRotMat::R21), rotMat.e(gRotMat::R22) );
			y = asin( -rotMat.e(gRotMat::R20));
			z = atan2( rotMat.e(gRotMat::R10), rotMat.e(gRotMat::R00) );
		} else {
			x = 0;
			y = gPI/2.0;
			z = -atan2( -rotMat.e(gRotMat::R12), rotMat.e(gRotMat::R11) );
		}
	} else {
		x = 0;
		y = -gPI/2.0;
		z = atan2( -rotMat.e(gRotMat::R12), rotMat.e(gRotMat::R11) );
	}
	return gVec3(x, y, z);
}


int mgUtility::getTransAndRotMatFromData(const CoordinateType* data, mgBone* bone, gVec3& transVec, gRotMat& rotMat, double scale)
{
	gVec3 rotVec;
	getTransAndRotVecFromData(data, bone, transVec, rotVec, scale);
	getRotMatFromRotVec(bone, rotVec, rotMat);

	return 0;
}

int mgUtility::setTransAndRotMatToData(CoordinateType* data, const mgBone* bone, const gVec3& transVec, const gRotMat& rotMat, double scale)
{
	gVec3 rotVec;
	setRotMatToRotVec(bone, rotVec, rotMat);
	setTransAndRotVecToData(data, bone, transVec, rotVec, scale);
	return 0;
}

int mgUtility::getTransAndRotVecFromData(const CoordinateType* data, mgBone* bone, gVec3& transVec, gVec3& rotVec, double scale)
{
	transVec.setZero();
	rotVec.setZero();

	int pos = bone->skeleton->dataPos[bone->id];
	if ( bone->channel & mgBone::translateChannel )  // Position keyframe
	{
		if( bone->channel & mgBone::Xposition ) transVec.setX( data[ pos++ ] * scale );
		if( bone->channel & mgBone::Yposition ) transVec.setY( data[ pos++ ] * scale );
		if( bone->channel & mgBone::Zposition ) transVec.setZ( data[ pos++ ] * scale );

		//transMat.setTrn(transVec);
	}
	if ( bone->channel & mgBone::rotationChannel )
	{
		// TODO: channel
		switch( bone->order )
		{
		case mgBone::XYZ:
			{
				rotVec.setX(data[ pos++ ] * gDTR);
				rotVec.setY(data[ pos++ ] * gDTR);
				rotVec.setZ(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::ZXY:
			{
				rotVec.setZ(data[ pos++ ] * gDTR);
				rotVec.setX(data[ pos++ ] * gDTR);
				rotVec.setY(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::YXZ:
			{
				rotVec.setY(data[ pos++ ] * gDTR);
				rotVec.setX(data[ pos++ ] * gDTR);
				rotVec.setZ(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::YZX:
			{
				rotVec.setY(data[ pos++ ] * gDTR);
				rotVec.setZ(data[ pos++ ] * gDTR);
				rotVec.setX(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::XZY:
			{
				rotVec.setX(data[ pos++ ] * gDTR);
				rotVec.setZ(data[ pos++ ] * gDTR);
				rotVec.setY(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::ZYX:
			{
				rotVec.setZ(data[ pos++ ] * gDTR);
				rotVec.setY(data[ pos++ ] * gDTR);
				rotVec.setX(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::XY:
			{
				rotVec.setX(data[ pos++ ] * gDTR);
				rotVec.setY(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::ZX:
			{
				rotVec.setZ(data[ pos++ ] * gDTR);
				rotVec.setX(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::YX:
			{
				rotVec.setY(data[ pos++ ] * gDTR);
				rotVec.setX(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::YZ:
			{
				rotVec.setY(data[ pos++ ] * gDTR);
				rotVec.setZ(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::XZ:
			{
				rotVec.setX(data[ pos++ ] * gDTR);
				rotVec.setZ(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::ZY:
			{
				rotVec.setZ(data[ pos++ ] * gDTR);
				rotVec.setY(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::X:
			{
				rotVec.setX(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::Y:
			{
				rotVec.setY(data[ pos++ ] * gDTR);
			}
			break;
		case mgBone::Z:
			{
				rotVec.setZ(data[ pos++ ] * gDTR);
			}
			break;
		default:
			{
				printf("Error: mgUtility::getTransAndRotMatFromData. %s \n", bone->name.c_str());
			}
			break;
		}

	} else if ( bone->channel & mgBone::quaternionChannel )
	{
		//tRot.makeRotateZXY(data[ pos ] * gDTR, data[ pos + 1 ] * gDTR, data[ pos + 2 ] * gDTR);
		gQuat quat;
		quat.set( data[pos+0], data[pos+1], data[pos+2], data[pos+3] );
		//rotMat = quat.inRotMatrix();
	} else if ( bone->channel & mgBone::expChannel )
	{
		rotVec.set( data[pos+0], data[pos+1], data[pos+2] );
	}

	return 0;
}

int mgUtility::setTransAndRotVecToData(CoordinateType* data, const mgBone* bone, const gVec3& transVec, const gVec3& rotVec, double scale)
{
	int pos = bone->skeleton->dataPos[bone->id];
	if ( bone->channel & mgBone::translateChannel )  // Position keyframe
	{
		if( bone->channel & mgBone::Xposition ) data[ pos++ ] = transVec.x();
		if( bone->channel & mgBone::Yposition ) data[ pos++ ] = transVec.y();
		if( bone->channel & mgBone::Zposition ) data[ pos++ ] = transVec.z();
	}
	if ( bone->channel & mgBone::rotationChannel )
	{
		// TODO: channel
		switch( bone->order )
		{
		case mgBone::XYZ:
			{
				data[ pos++ ] = rotVec.x() * gRTD;
				data[ pos++ ] = rotVec.y() * gRTD;
				data[ pos++ ] = rotVec.z() * gRTD;
			}
			break;
		case mgBone::ZXY:
			{
				data[ pos++ ] = rotVec.z() * gRTD;
				data[ pos++ ] = rotVec.x() * gRTD;
				data[ pos++ ] = rotVec.y() * gRTD;
			}
			break;
		case mgBone::YXZ:
			{
				data[ pos++ ] = rotVec.y() * gRTD;
				data[ pos++ ] = rotVec.x() * gRTD;
				data[ pos++ ] = rotVec.z() * gRTD;
			}
			break;
		case mgBone::YZX:
			{
				data[ pos++ ] = rotVec.y() * gRTD;
				data[ pos++ ] = rotVec.z() * gRTD;
				data[ pos++ ] = rotVec.x() * gRTD;
			}
			break;
		case mgBone::XZY:
			{
				data[ pos++ ] = rotVec.x() * gRTD;
				data[ pos++ ] = rotVec.z() * gRTD;
				data[ pos++ ] = rotVec.y() * gRTD;
			}
			break;
		case mgBone::ZYX:
			{
				data[ pos++ ] = rotVec.z() * gRTD;
				data[ pos++ ] = rotVec.y() * gRTD;
				data[ pos++ ] = rotVec.x() * gRTD;
			}
			break;
		case mgBone::XY:
			{
				data[ pos++ ] = rotVec.x() * gRTD;
				data[ pos++ ] = rotVec.y() * gRTD;
			}
			break;
		case mgBone::ZX:
			{
				data[ pos++ ] = rotVec.z() * gRTD;
				data[ pos++ ] = rotVec.x() * gRTD;
			}
			break;
		case mgBone::YX:
			{
				data[ pos++ ] = rotVec.y() * gRTD;
				data[ pos++ ] = rotVec.x() * gRTD;
			}
			break;
		case mgBone::YZ:
			{
				data[ pos++ ] = rotVec.y() * gRTD;
				data[ pos++ ] = rotVec.z() * gRTD;
			}
			break;
		case mgBone::XZ:
			{
				data[ pos++ ] = rotVec.x() * gRTD;
				data[ pos++ ] = rotVec.z() * gRTD;
			}
			break;
		case mgBone::ZY:
			{
				data[ pos++ ] = rotVec.z() * gRTD;
				data[ pos++ ] = rotVec.y() * gRTD;
			}
			break;
		case mgBone::X:
			{
				data[ pos++ ] = rotVec.x() * gRTD;
			}
			break;
		case mgBone::Y:
			{
				data[ pos++ ] = rotVec.y() * gRTD;
			}
			break;
		case mgBone::Z:
			{
				data[ pos++ ] = rotVec.z() * gRTD;
			}
			break;
		default:
			{
				printf("Error: mgUtility::getTransAndRotMatFromData.\n");
			}
			break;
		}

	} else if ( bone->channel & mgBone::quaternionChannel )
	{
		printf("Error: mgUtility::getTransAndRotMatFromData. at quaternionChannel (not implemented yet)\n");
	} else if ( bone->channel & mgBone::expChannel )
	{
		data[ pos++ ] = rotVec.x();
		data[ pos++ ] = rotVec.y();
		data[ pos++ ] = rotVec.z();
	}

	return 0;
}

int mgUtility::getRotMatFromRotVec(const mgBone* bone, const gVec3& rotVec, gRotMat& rotMat)
{	
	if( bone->channel & mgBone::rotationChannel )
	{
		switch( bone->order )
		{
		case mgBone::XYZ:
			if(bone->eulerConv==mgBone::INTRINSIC)
				rotMat.makeRotateXYZ(rotVec.x(), rotVec.y(), rotVec.z());
			else 
				rotMat.makeRotateZYX(rotVec.z(), rotVec.y(), rotVec.x());
			break;
		case mgBone::XZY:
			if(bone->eulerConv==mgBone::INTRINSIC)
				//rotMat = gRotMat::exp(gVec3UnitX * rotVec.x()) * gRotMat::exp(gVec3UnitZ * rotVec.z()) * gRotMat::exp(gVec3UnitY * rotVec.y());
				rotMat.makeRotateXZY(rotVec.x(), rotVec.z(), rotVec.y());
			else
				//rotMat = gRotMat::exp(gVec3UnitY * rotVec.y()) * gRotMat::exp(gVec3UnitZ * rotVec.z()) * gRotMat::exp(gVec3UnitX * rotVec.x());
				rotMat.makeRotateYZX(rotVec.y(), rotVec.z(), rotVec.x());
			break;

		case mgBone::YXZ:
			if(bone->eulerConv==mgBone::INTRINSIC)
				//rotMat = gRotMat::exp(gVec3UnitY * rotVec.y()) * gRotMat::exp(gVec3UnitX * rotVec.x()) * gRotMat::exp(gVec3UnitZ * rotVec.z());
				rotMat.makeRotateYXZ(rotVec.y(), rotVec.x(), rotVec.z());
			else
				rotMat.makeRotateZXY(rotVec.z(), rotVec.x(), rotVec.y());
			break;
		case mgBone::YZX:
			if(bone->eulerConv==mgBone::INTRINSIC)
				//rotMat = gRotMat::exp(gVec3UnitY * rotVec.y()) * gRotMat::exp(gVec3UnitZ * rotVec.z()) * gRotMat::exp(gVec3UnitX * rotVec.x());
				rotMat.makeRotateYZX(rotVec.y(), rotVec.z(), rotVec.x());
			else
				//rotMat = gRotMat::exp(gVec3UnitX * rotVec.x()) * gRotMat::exp(gVec3UnitZ * rotVec.z()) * gRotMat::exp(gVec3UnitY * rotVec.y());
				rotMat.makeRotateXZY(rotVec.x(), rotVec.z(), rotVec.y());
			break;

		case mgBone::ZXY:
			if(bone->eulerConv==mgBone::INTRINSIC)
				rotMat.makeRotateZXY(rotVec.z(), rotVec.x(), rotVec.y());
			else
				//rotMat = gRotMat::exp(gVec3UnitY * rotVec.y()) * gRotMat::exp(gVec3UnitX * rotVec.x()) * gRotMat::exp(gVec3UnitZ * rotVec.z());
				rotMat.makeRotateYXZ(rotVec.y(), rotVec.x(), rotVec.z());
			break;
		case mgBone::ZYX:
			if(bone->eulerConv==mgBone::INTRINSIC)
				rotMat.makeRotateZYX(rotVec.z(), rotVec.y(), rotVec.x());
			else
				rotMat.makeRotateXYZ(rotVec.x(), rotVec.y(), rotVec.z());
			break;

		case mgBone::XY:
			rotMat = gRotMat::exp(gVec3UnitX * rotVec.x()) * gRotMat::exp(gVec3UnitY * rotVec.y());
			break;
		case mgBone::XZ:
			rotMat = gRotMat::exp(gVec3UnitX * rotVec.x()) * gRotMat::exp(gVec3UnitZ * rotVec.z());
			break;

		case mgBone::YX:
			rotMat = gRotMat::exp(gVec3UnitY * rotVec.y()) * gRotMat::exp(gVec3UnitX * rotVec.x());
			break;
		case mgBone::YZ:
			rotMat = gRotMat::exp(gVec3UnitY * rotVec.y()) * gRotMat::exp(gVec3UnitZ * rotVec.z());
			break;

		case mgBone::ZX:
			rotMat = gRotMat::exp(gVec3UnitZ * rotVec.z()) * gRotMat::exp(gVec3UnitX * rotVec.x());
			break;
		case mgBone::ZY:
			rotMat = gRotMat::exp(gVec3UnitZ * rotVec.z()) * gRotMat::exp(gVec3UnitY * rotVec.y());
			break;

		case mgBone::X:
			rotMat.makeRotateX(rotVec.x());
			break;
		case mgBone::Y:
			rotMat.makeRotateY(rotVec.y());
			break;
		case mgBone::Z:
			rotMat.makeRotateZ(rotVec.z());
			break;

		default:
			printf("Error: mgUtility::getTransAndRotMatFromData.\n");
			break;
		}
	} else if( bone->channel & mgBone::expChannel )
	{
		rotMat = gRotMat::exp(rotVec);
	}
	//rotMat.makeRotateZYX(rotVec.z(), rotVec.y(), rotVec.x());
	return 0;
}

int mgUtility::setRotMatToRotVec(const mgBone* bone, gVec3& rotVec, const gRotMat& rotMat)
{
	if( bone->channel & mgBone::rotationChannel )
	{
		double outRot;
		rotVec.setZero();

		switch( bone->order )
		{
		case mgBone::XYZ:
			if(bone->eulerConv==mgBone::INTRINSIC)
				rotVec = mgUtility::getXYZFromRotation( rotMat );
			else 
				rotVec = mgUtility::getZYXFromRotation( rotMat );
			break;
		case mgBone::XZY:
			if(bone->eulerConv==mgBone::INTRINSIC)
				rotVec = mgUtility::getXZYFromRotation( rotMat );
			else
				rotVec = mgUtility::getYZXFromRotation( rotMat );
			break;

		case mgBone::YXZ:
			if(bone->eulerConv==mgBone::INTRINSIC)
				rotVec = mgUtility::getYXZFromRotation( rotMat );
			else
				rotVec = mgUtility::getZXYFromRotation( rotMat );
			break;
		case mgBone::YZX:
			if(bone->eulerConv==mgBone::INTRINSIC)
				rotVec = mgUtility::getYZXFromRotation( rotMat );
			else
				rotVec = mgUtility::getXZYFromRotation( rotMat );
			break;

		case mgBone::ZXY:
			if(bone->eulerConv==mgBone::INTRINSIC)
				rotVec = mgUtility::getZXYFromRotation( rotMat );
			else
				rotVec = mgUtility::getYXZFromRotation( rotMat );
			break;
		case mgBone::ZYX:
			if(bone->eulerConv==mgBone::INTRINSIC)
				rotVec = mgUtility::getZYXFromRotation( rotMat );
			else
				rotVec = mgUtility::getXYZFromRotation( rotMat );
			break;

		case mgBone::XY:
			rotVec = mgUtility::getXYZFromRotation( rotMat );
			rotVec.setZ(0);
			break;
		case mgBone::XZ:
			rotVec = mgUtility::getXZYFromRotation( rotMat );
			rotVec.setY(0);
			break;

		case mgBone::YX:
			rotVec = mgUtility::getYXZFromRotation( rotMat );
			rotVec.setZ(0);
			break;
		case mgBone::YZ:
			rotVec = mgUtility::getYZXFromRotation( rotMat );
			rotVec.setX(0);
			break;

		case mgBone::ZX:
			rotVec = mgUtility::getZXYFromRotation( rotMat );
			rotVec.setY(0);
			break;
		case mgBone::ZY:
			rotVec = mgUtility::getZYXFromRotation( rotMat );
			rotVec.setX(0);
			break;

		case mgBone::X:
			getAngleFromRotMat( gVec3UnitX, rotMat, outRot);
			rotVec.setX(outRot);
			break;
		case mgBone::Y:
			getAngleFromRotMat( gVec3UnitY, rotMat, outRot);
			rotVec.setY(outRot);
			break;
		case mgBone::Z:
			getAngleFromRotMat( gVec3UnitZ, rotMat, outRot);
			rotVec.setZ(outRot);
			break;

		default:
			printf("Error: mgUtility::getTransAndRotMatFromData.\n");
			break;
		}
	} else if( bone->channel & mgBone::expChannel )
	{
		rotVec = gVec3::log( rotMat );
	}

	return 0;
}

int mgUtility::getRotMatFromRotVec(const char* angleOrder, mgBone::_EULERANGLECONV eulerConv, const gVec3& angles, gRotMat& rotMat)
{
	gRotMat R;
	if( eulerConv == mgBone::INTRINSIC )
	{
		for(int i=0;i<3;++i)
		{		
			if(angleOrder[i]=='x') R *= gRotMat::exp( gVec3(angles.x(),0,0) );
			else if (angleOrder[i]=='y') R *= gRotMat::exp( gVec3(0,angles.y(),0) );
			else R *= gRotMat::exp( gVec3(0,0,angles.z()) );
		}
	}else
	{
		for(int i=0;i<3;++i)
		{		
			if(angleOrder[i]=='x') R = gRotMat::exp( gVec3(angles.x(),0,0) )*R;
			else if (angleOrder[i]=='y') R = gRotMat::exp( gVec3(0,angles.y(),0) )*R;
			else R = gRotMat::exp( gVec3(0,0,angles.z()) )*R;
		}
	}
	rotMat = R;
	return 0;
	
}

int mgUtility::getRotVecFromRotMat(const mgBone* bone, double* data, const gRotMat& rotMat)
{
	gVec3 rotVec;

	if( bone->eulerConv == mgBone::INTRINSIC )
	{
		switch( bone->order )
		{
		case mgBone::XYZ:
			rotVec = getXYZFromRotation(rotMat);
			data[0] = rotVec.x();
			data[1] = rotVec.y();
			data[2] = rotVec.z();
			break;
		case mgBone::XZY:
			rotVec = getXZYFromRotation(rotMat);
			data[0] = rotVec.x();
			data[1] = rotVec.z();
			data[2] = rotVec.y();
			break;
		case mgBone::YXZ:
			rotVec = getYXZFromRotation(rotMat);
			data[0] = rotVec.y();
			data[1] = rotVec.x();
			data[2] = rotVec.z();
			break;
		case mgBone::YZX:
			rotVec = getYZXFromRotation(rotMat);
			data[0] = rotVec.y();
			data[1] = rotVec.z();
			data[2] = rotVec.x();
			break;
		case mgBone::ZXY:
			rotVec = getZXYFromRotation(rotMat);
			data[0] = rotVec.z();
			data[1] = rotVec.x();
			data[2] = rotVec.y();
			break;
		case mgBone::ZYX:
			rotVec = getZYXFromRotation(rotMat);
			data[0] = rotVec.z();
			data[1] = rotVec.y();
			data[2] = rotVec.x();
			break;		
		}
	} else {
		switch( bone->order )
		{
		case mgBone::XYZ:
			rotVec = getZYXFromRotation(rotMat);
			data[0] = rotVec.x();
			data[1] = rotVec.y();
			data[2] = rotVec.z();
			break;
		case mgBone::XZY:
			rotVec = getYZXFromRotation(rotMat);
			data[0] = rotVec.x();
			data[1] = rotVec.z();
			data[2] = rotVec.y();
			break;
		case mgBone::YXZ:
			rotVec = getZXYFromRotation(rotMat);
			data[0] = rotVec.y();
			data[1] = rotVec.x();
			data[2] = rotVec.z();
			break;
		case mgBone::YZX:
			rotVec = getXZYFromRotation(rotMat);
			data[0] = rotVec.y();
			data[1] = rotVec.z();
			data[2] = rotVec.x();
			break;
		case mgBone::ZXY:
			rotVec = getYXZFromRotation(rotMat);
			data[0] = rotVec.z();
			data[1] = rotVec.x();
			data[2] = rotVec.y();
			break;
		case mgBone::ZYX:
			rotVec = getXYZFromRotation(rotMat);
			data[0] = rotVec.z();
			data[1] = rotVec.y();
			data[2] = rotVec.x();
			break;		
		}
	}
	return 0;
}

int mgUtility::getStringFromMat(std::string& buf, gXMat& mat, const char* delim)
{
	std::ostringstream stream;

	for( int i=0; i<4; i++ )
	{
		for( int j=0; j<4; j++ )
		{
			stream << mat.e(i, j);
			if( j < 3 ) stream << delim;
		}
		stream << std::endl;
	}

	buf = stream.str();
	return 0;
}

int mgUtility::getStringFromVec(std::string& buf, gVec3& vec, const char* delim)
{
	std::ostringstream stream;
	stream << vec.x() << delim << vec.y() << delim << vec.z();
	buf = stream.str();
	return 0;
}

int mgUtility::saveMotionToAMCFile(const char* filename, const mgSkeleton* skeleton, const mgData* motion)
{
	std::fstream f;
	f.open(filename, std::fstream::out);

	f << ":FULLY-SPECIFIED" << std::endl << ":DEGREES" << std::endl;

	for( int i=0; i<motion->nMotion; ++i )
	{
		f << i+1 << std::endl;

		int bonePos = 0;
		for( int j=0; j<skeleton->bones.size(); ++j )
		{
			mgBone *bone = skeleton->bones[j];

			if( bone->nChannel < 1 ) continue;

			f << bone->name;

			for( int c=0; c<bone->nChannel; ++c )
			{
				f << " " << motion->motions[i][bonePos];
				bonePos++;
			}

			f << std::endl;
		}

	}

	f.close();

	return 0;
}

int mgUtility::saveMotionToAMCFile(const char* filename, const mgSkeleton* skeleton, const arma::mat& motion)
{
	std::fstream f;
	f.open(filename, std::fstream::out);

	f << ":FULLY-SPECIFIED" << std::endl << ":DEGREES" << std::endl;

	for (int i = 0; i<motion.n_cols; ++i)
	{
		f << i + 1 << std::endl;

		int bonePos = 0;
		for (int j = 0; j<skeleton->bones.size(); ++j)
		{
			mgBone *bone = skeleton->bones[j];

			if (bone->nChannel < 1) continue;

			f << bone->name;

			for (int c = 0; c<bone->nChannel; ++c)
			{
				f << " " << motion(bonePos, i);//motion->motions[i][bonePos];
				bonePos++;
			}

			f << std::endl;
		}

	}

	f.close();

	return 0;
}

int mgUtility::convertQuaternionPose(double* quatPose, const double* inPose, const mgSkeleton* skeleton)
{
	int jPos  = 0;
	for( int i=0; i < skeleton->bones.size(); i++ )
	{
		gVec3 trans;
		gRotMat rotMat;
		mgBone* bone = skeleton->bones[i];

		getTransAndRotMatFromData(inPose, bone, trans, rotMat);

		if( i==0 )
		{
			quatPose[jPos++] = trans.x();
			quatPose[jPos++] = trans.y();
			quatPose[jPos++] = trans.z();
		}


	}

	return 0;
}


/*

int getConstraint(std::string constName, std::vector<bool> *contactFlag)
{
	int boneId = bvh.getBoneIdFromName(constName);
	//std::vector<bool> *contactFlag = &motion->contactsFlagMap[boneId];

	// parameter
	double velocityThreshold =  _constVelThres;
	unsigned int sizeofHalfL1 = _sizeHalfWindow; // window size

	// get constraint 3d position array from all motions
	std::vector<gVec3> allPositions;
	allPositions.reserve(model.nMotion);
	gXMat worldMatrix;

	// apply motion
	for( unsigned int frame=0; frame<model.nMotion; frame++)
	{
		worldMatrix.setIdentity();
		bvh.getWMatrixAt(constName,model.motions[frame], worldMatrix);
		allPositions.push_back( worldMatrix.trn() );
	}

	// detect contact.
	std::vector<gVec3>::iterator posIt = allPositions.begin();
	gVec3 prevPos;
	//contact flag
	contactFlag->clear();
	contactFlag->reserve(model.nMotion);
	contactFlag->push_back(0);

	// get the frame Id where the constraint position is steady in sequence.
	prevPos = *posIt;
	posIt++;
	while( posIt != allPositions.end() )
	{
		if( (*posIt - prevPos).magnitude() < velocityThreshold ) contactFlag->push_back(1);
		else contactFlag->push_back(0);
		prevPos = *posIt;
		posIt++;
	}

	// convolute. filtering noise.
	std::deque<bool> convolute;
	unsigned int sum = 0, pos;

	for( pos=0; pos<contactFlag->size(); pos++ )
	{
		convolute.push_back(contactFlag->at(pos));
		sum += contactFlag->at(pos);
		
		if( convolute.size() < sizeofHalfL1 + 1 ) continue;
		else if( convolute.size() > (sizeofHalfL1 * 2 + 1) ) {
			sum -= convolute.front();
			convolute.pop_front();
		}

		if( sum * 2 >= convolute.size() ) contactFlag->at(pos - sizeofHalfL1) = 1;
		else contactFlag->at(pos - sizeofHalfL1) = 0;
	}

	while( pos < sizeofHalfL1 + contactFlag->size())
	{
		sum -= convolute.front();
		convolute.pop_front();

		if( sum * 2 >= convolute.size() ) contactFlag->at(pos - sizeofHalfL1) = 1;
		else contactFlag->at(pos - sizeofHalfL1) = 0;
		pos++;
	}

	return 0;
}

void filteringData(mgUtility::CoordinateType **data, int filterSize, int numOfApply)
{
	std::list<mgUtility::CoordinateType>				queueData;
	std::list<mgUtility::CoordinateType>::iterator	queueIt;
	std::vector<double> gaussian(filterSize);

	double sigma = 1; // (double)filterSize / 8.0;
	double mu = (double)filterSize / 2.0;
	//double filtered;
	// gaussian

	double filterSum=0;
	for( int i=0; i<filterSize; i++ )
	{
		filterSum += gaussian[i] = exp(-1 * ( (i-mu)*(i-mu) / (2.0*sigma*sigma) ) ) / (sigma * sqrt( 2.0 * gPI ));
	}
	// normalize
	for( int i=0; i<filterSize; i++ )
	{
		gaussian[i] = gaussian[i] / filterSum;
	}

	int numR = bvh.getNumOfRotElement();
	int dataPos;

	std::list< gQuat >				_rotQueue;
	std::list< gQuat >::iterator	_rotQueueIt;

	std::list< gVec3 >				_trnQueue;
	std::list< gVec3 >::iterator	_trnQueueIt;

	gRotMat R;
	gQuat quat, sumQuat;
	gVec3 rotData, sumTrn;
	//mgUtility::Skeleton* bvhSkeleton = bvh.getLoadedSkeleton();
	int p=0;

	
	for( int a=0; a<numOfApply; a++ )
	{

		for( int bondId=0; bondId<bvhSkeleton->bones.size(); bondId++ )
		{
			_rotQueue.clear();
			_trnQueue.clear();

			for( int i=0; i<nFrames; i++ )
			{
				if( bvhSkeleton->bones[bondId]->channel & 0x07 )
				{
					_trnQueue.push_back( mgUtility::getRootPositionFromPose(data[i]) );
				
					if( _trnQueue.size() > filterSize ) {
						_trnQueue.pop_front();

						p=0;
						sumTrn.set(0,0,0);
						for( _trnQueueIt=_trnQueue.begin(); _trnQueueIt!=_trnQueue.end(); _trnQueueIt++ )
						{
							sumTrn += (*_trnQueueIt) * gaussian[p];
							p++;
						}
						data[(int)(i-mu)][0] = sumTrn.x();
						data[(int)(i-mu)][1] = sumTrn.y();
						data[(int)(i-mu)][2] = sumTrn.z();
					}

				}
				if( bvhSkeleton->bones[bondId]->channel & 0x38 ) {
					dataPos = bvhSkeleton->dataPos[bondId];
					if( dataPos == 0 ) dataPos = 3;

					R.makeRotateZXY( data[i][dataPos] * gDTR, data[i][dataPos+1] * gDTR, data[i][dataPos+2] * gDTR );
					quat = R.inQuat();
					//quat.normalize();
					if( quat.w() < 0 ) quat.set( -1 * quat.x(), -1 * quat.y(), -1 * quat.z(), -1 * quat.w() );

					_rotQueue.push_back( quat );

					if( _rotQueue.size() > filterSize ) {
						_rotQueue.pop_front();

						p=0;
						sumQuat.set(0,0,0,0);
						for( _rotQueueIt=_rotQueue.begin(); _rotQueueIt!=_rotQueue.end(); _rotQueueIt++ )
						{
							sumQuat.set( sumQuat.x() + (*_rotQueueIt).x() * gaussian[p],
								sumQuat.y() + (*_rotQueueIt).y() * gaussian[p],
								sumQuat.z() + (*_rotQueueIt).z() * gaussian[p],
								sumQuat.w() + (*_rotQueueIt).w() * gaussian[p] );

							p++;
						}
						sumQuat.normalize();
						rotData = bvh.getZXYFromRotation(sumQuat.inRotMatrix());
						data[(int)(i-mu)][dataPos] = rotData.z() * gRTD;
						data[(int)(i-mu)][dataPos+1] = rotData.x() * gRTD;
						data[(int)(i-mu)][dataPos+2] = rotData.y() * gRTD;
					}
				} else if( bvhSkeleton->bones[bondId]->channel & 0x3C0 ) {
					dataPos = bvhSkeleton->dataPos[bondId];
					if( dataPos == 0 ) dataPos = 3;

					//quat.set(data[i][dataPos+3], data[i][dataPos], data[i][dataPos+1], data[i][dataPos+2]);
					quat.set(data[i][dataPos+0], data[i][dataPos+1], data[i][dataPos+2], data[i][dataPos+3]);
					//if( quat.w() < 0 ) quat.set( -1 * quat.x(), -1 * quat.y(), -1 * quat.z(), -1 * quat.w() );

					_rotQueue.push_back( quat );

					if( _rotQueue.size() > filterSize ) {
						_rotQueue.pop_front();

						p=0;
						sumQuat.set(0,0,0,0);
						for( _rotQueueIt=_rotQueue.begin(); _rotQueueIt!=_rotQueue.end(); _rotQueueIt++ )
						{
							sumQuat.set( sumQuat.x() + (*_rotQueueIt).x() * gaussian[p],
								sumQuat.y() + (*_rotQueueIt).y() * gaussian[p],
								sumQuat.z() + (*_rotQueueIt).z() * gaussian[p],
								sumQuat.w() + (*_rotQueueIt).w() * gaussian[p] );

							p++;
						}
						sumQuat.normalize();

						data[(int)(i-mu)][dataPos] = sumQuat.x();
						data[(int)(i-mu)][dataPos+1] = sumQuat.y();
						data[(int)(i-mu)][dataPos+2] = sumQuat.z();
						data[(int)(i-mu)][dataPos+3] = sumQuat.w();
					}
				}

			}
		}
	}
}

void interpolator(CoordinateType* src, CoordinateType* tgtPose, int size)
{
	int frame = nFrames - size;
	double alpha=0;
	double step = 1.0/(double)size;

	while( frame < nFrames )
	{
		for( int col=0; col<nChannel; col++ )
		{
			src[frame * nChannel + col] = ( 1.0 - alpha) * src[frame * nChannel + col] + alpha * tgtPose[col];
		}
		
		alpha += step;
		frame++;
	}
}

void ikApply(CoordinateType** pData, boolArray &contactFlag, int eeId, int baseId)
{
	double minDist = DBL_MAX;
	double foot2RootDist;
	gXMat tempMat, minMat;
	gXMatArray contactFootMat;
	int idx=0;
	bool fPrevContact;
	int contactIdx;

	//double orgDist, ikDist;
	double maxDist = 0;
	gVec3 tgt2RootVec, minVec, maxVec;
	CoordinateType *Tpose = new CoordinateType[nChannel];
	memset( Tpose, 0, sizeof(CoordinateType) * nChannel );
	gXMatList poseMat;
	gXMat baseMat;
	bvh.getWMatrixFromToAt(eeId, baseId, Tpose, poseMat);
	//mg.getLoadedBVHData()->getWMatrixFromToAt(eeId, baseId, Tpose, poseMat);
	double foot2RootMaxDist = (poseMat[baseId].trn() - poseMat[eeId].trn()).magnitude();

	for( boolArrayIt contactIt = contactFlag.begin(); 
		contactIt != contactFlag.end(); 
		contactIt++ )
	{
		if( *contactIt ) 
		{
			bvh.getWMatrixFromToAt(eeId, baseId, pData[idx], poseMat);
			tgt2RootVec = (poseMat[baseId].trn() - poseMat[eeId].trn());
			foot2RootDist = tgt2RootVec.magnitude();
			tempMat = poseMat[eeId];

			if( minDist > foot2RootDist )
			{
				minDist = foot2RootDist;
				minMat = tempMat;
				minVec = tgt2RootVec;
			}
			

		} else {
			if( minDist != DBL_MAX )
			{
				minDist = DBL_MAX;
				contactFootMat.push_back(minMat);
			}
		}
		idx++;
	}

	if( minDist != DBL_MAX )
		contactFootMat.push_back(minMat);

	// plant the foot!
	osg::ref_ptr<osg::Vec3Array> rootPts;

	idx=0;
	fPrevContact = false;
	contactIdx = 0;

	// TEST
	printf("# of Contact : %d\n", contactFootMat.size() );
	
	mgUtility::CoordinateType *ikTempData = new mgUtility::CoordinateType[bvhSkeleton->nTotalChannel];

	for( boolArrayIt contactIt = contactFlag.begin(); 
		contactIt != contactFlag.end(); 
		contactIt++ )
	{
		if( *contactIt ) 
		{
			if( !fPrevContact )
			{
				// Test
				float R = (float)(rand() % 254)/(float)254;
				float G = (float)(rand() % 254)/(float)254;
				float B = (float)(rand() % 254)/(float)254;
				createRefGeom(contactFootMat[contactIdx], osg::Vec4(R,G,B,1));

				fPrevContact = true;
				// init Data
				//memcpy(ikTempData, pData[idx], sizeof(mgUtility::CoordinateType) * bvhSkeleton->nTotalChannel);
			}
			//posecpyFromTo( pData[idx], ikTempData, eeId, baseId);
			skeleton->ik_with_jacobian(eeId, baseId, contactFootMat[contactIdx], pData[idx]);
			//skeleton->ik_with_baseJacobian(eeId, contactFootMat[contactIdx], pData[idx]);
			//posecpyFromTo( ikTempData, pData[idx], eeId, baseId);
		} else {
			if( fPrevContact )
			{
				contactIdx++;
			}
			fPrevContact = false;
		}
		idx++;
	}

	delete [] ikTempData;
}
*/