//#####################################################################
// Copyright 2010-2015, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#include "Loader/BVHLoader.h"
#include <algorithm>
//using namespace std;

int BVHLoader::loadMotion(const char* filename, mgSkeleton *skeleton, mgData *motion)
{
	assert( skeleton && motion );

	if(open_file(filename)==0) return (int)RT_OPENERR;
	
	const int max_buffer_size = 128;
	char buf[max_buffer_size];
	memset(buf, 0, max_buffer_size);

	do {
		read_word(buf);
	} while( strncmp(buf, "MOTION", max_buffer_size) );

	if( !strncmp(buf, "MOTION", max_buffer_size) )
	{
		readMotion(skeleton, motion);
	} else {
		// fail to load
		printf("BVHLoader: fail to load motion. \n");
	}

	close_file();

	return (int)RT_OK;
}

int BVHLoader::loadSkeleton(const char* filename, mgSkeleton *skeleton)
{
	if(open_file(filename)==0) return (int)RT_OPENERR;
	
	const int max_buffer_size = 128;
	char buf[max_buffer_size];
	memset(buf, 0, max_buffer_size);

	do {
		read_word(buf);
	} while( strncmp(buf, "HIERARCHY", max_buffer_size) );

	if( !strncmp(buf, "HIERARCHY", max_buffer_size) )
	{
		tDataPos = 0;
		readSkeleton(skeleton, NULL);
	} else {
		// fail to load
		printf("BVHLoader: fail to load skeleton. \n");
	}

	close_file();

	return (int)RT_OK;
}

int BVHLoader::readSkeleton(mgSkeleton *skeleton, mgBone* bone)
{
	bool isRecognized = false;
	//mgBone *bone = NULL;
	//char orderBuf[7] = "";
	std::string orderBuf;
	//mgBone::_AXISORDER order;

	// string buffer
	const int max_buffer_size = 128;
	char buf[max_buffer_size];
	memset(buf, 0, max_buffer_size);

    if ( !skeleton ) return (int)RT_NULLP;

	read_word_s(buf, max_buffer_size);
	
	while( !feof(_file) && strncmp(buf, "}", max_buffer_size) != 0 )
	{
		if ( !strncmp(buf, "OFFSET", max_buffer_size) )
		{
			isRecognized = true;

			gVec3 offset = readVec3();
			bone->H.setTrn(offset * _transScale);

			// setting Local End point
			if( bone->parent )
			{
				bone->parent->localEndPoints.push_back(bone->H.trn());
			}

			read_word_s(buf, max_buffer_size);
		}

		if ( !strncmp(buf, "CHANNELS", max_buffer_size) )
		{
			isRecognized = true;
			int noChannels = readInteger(buf);

			// joint channel
			//nChannels += noChannels;
			skeleton->nTotalChannel += noChannels;

			orderBuf.clear();
			for ( int i=0; i<noChannels; ++i )
			{
				read_word(buf);
				//alterChannel( buf, max_buffer_size, bone->channel );
				alterChannel( buf, bone->channel, orderBuf);
			}

			// order
			if( orderBuf == "xyz" ) bone->order = mgBone::XYZ;
			else if( orderBuf == "xzy" ) bone->order = mgBone::XZY;
			else if( orderBuf == "yxz" ) bone->order = mgBone::YXZ;
			else if( orderBuf == "yzx" ) bone->order = mgBone::YZX;
			else if( orderBuf == "zxy" ) bone->order = mgBone::ZXY;
			else if( orderBuf == "zyx" ) bone->order = mgBone::ZYX;
			else if( orderBuf == "xy" ) bone->order = mgBone::XY;
			else if( orderBuf == "xz" ) bone->order = mgBone::XZ;
			else if( orderBuf == "yx" ) bone->order = mgBone::YX;
			else if( orderBuf == "yz" ) bone->order = mgBone::YZ;
			else if( orderBuf == "zx" ) bone->order = mgBone::ZX;
			else if( orderBuf == "zy" ) bone->order = mgBone::ZY;
			else if( orderBuf == "x" ) bone->order = mgBone::X;
			else if( orderBuf == "y" ) bone->order = mgBone::Y;
			else if( orderBuf == "z" ) bone->order = mgBone::Z;
			else printf("does not support %s order.\n", orderBuf.c_str());

			// update dataPos
			skeleton->dataPos.push_back(tDataPos);

			//int tDataPos = 0;
			if ( bone->channel&0x07 ) 
			{
				bone->nChannel += 3;
				tDataPos += 3;
				skeleton->numT++;
			}
			if ( bone->channel&0x38 ) 
			{
				bone->nChannel += 3;
				tDataPos += 3;
				skeleton->numR++;
			}
			if ( bone->channel&0x3C0 ) 
			{
				bone->nChannel += 4;
				tDataPos += 4;
				skeleton->numR++;
			}

			read_word_s(buf, max_buffer_size);
		} 

		if ( !strncmp(buf, "End", max_buffer_size) )
		{
			isRecognized = true;
			read_word(buf);
			read_word(buf);
			read_word(buf);

			if ( !strncmp(buf, "OFFSET", max_buffer_size) )
			{
				gVec3 offsetEndSite = readVec3() * _transScale;
				bone->localEndPoints.push_back(offsetEndSite);
			}

			read_word_s(buf, max_buffer_size);
			read_word_s(buf, max_buffer_size);
		}

		if ( !strncmp(buf, "JOINT", max_buffer_size) )
		{
			isRecognized = true;

			// name
			read_word(buf);

			mgBone *joint = NULL;
			static int nNoNamedBone = 0;
			if( buf[0] == '{' )
			{
				// warning.
				printf("warning: no named bone\n");
				sprintf(buf, "noname_%d", nNoNamedBone);
				nNoNamedBone++;

				joint = skeleton->createBone(buf);

			} else {

				joint = skeleton->createBone(buf);

				//{
				read_word(buf);
			}

			//mgBone *joint = skeleton->createBone(buf);
			skeleton->addChild(bone, joint);

			readSkeleton(skeleton, joint);
			//}
			read_word(buf);
		}

		if ( !strncmp(buf, "ROOT", max_buffer_size) )
		{
			isRecognized = true;

			// name
			read_word(buf);
			
			mgBone *root = skeleton->createBone(buf);
			skeleton->boneRoot = root;

			//{
			read_word(buf);

			readSkeleton(skeleton, root);

			break;
		}

		if ( !isRecognized )
		{
			printf("BVH Reader: Unrecognized symbol %s. Ignore current field or block.\n", buf);
			read_word(buf);
			//while( !strncmp(buf, "}", max_buffer_size) || eof) read_word(buf);
		}
	}

	return (int)RT_OK;
}

int BVHLoader::readMotion(mgSkeleton *skeleton, mgData *motion)
{
	unsigned int i=0;
	const int max_buffer_size = 128;
	char buf[max_buffer_size];

	read_word(buf);
    if ( !strncmp(buf, "Frames:", max_buffer_size) )
    {
		motion->nMotion = readInteger(buf);
	}
	read_word(buf);
	if ( !strncmp(buf, "Frame", max_buffer_size) )
    {
		read_word(buf);
		motion->frameTime = readReal(buf);
	}

	/* allocate space for objects[][] and read all objects */
	//int len = skeleton->nTotalChannel * motion->nMotion;
	//motion->pMotions = new CoordinateType[len];
	//
	//motion->motions = new CoordinateType*[motion->nMotion];
	//for( unsigned int row=0; row < motion->nMotion; row++ )
	//{
	//	motion->motions[row] = &( motion->pMotions[ row * skeleton->nTotalChannel ] );
	//}
	motion->nChannel = skeleton->nTotalChannel;
	motion->allocMemory();

	int coordIdx;
	CoordinateType motionData;
	while( !feof(_file) && i < motion->nMotion )
	{
		coordIdx=0;
		for( unsigned int n=0; n<skeleton->bones.size(); n++ )
		{
			if ( skeleton->bones[n]->channel&0x07 ) {
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData * _transScale;
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData * _transScale;
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData * _transScale;
			}
			if ( skeleton->bones[n]->channel&0x38 )
			{
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData;
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData;
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData;
			} else if ( skeleton->bones[n]->channel&0x3C0 )
			{
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData;
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData;
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData;
				motionData = readReal(buf);
				motion->motions[i][coordIdx++] = motionData;
			}
		}
		i++;
	}

	if( coordIdx != skeleton->nTotalChannel )
	{
		printf("BVH Load Error! : not matched dimension.\n");
	}

	return true;
}

void BVHLoader::alterChannel( std::string name, unsigned int& value, std::string& orderTxt )
{
	std::transform(name.begin(), name.end(), name.begin(), ::tolower);

    if      ( !name.compare("xposition") ) value |= mgBone::Xposition;
    else if ( !name.compare("yposition") ) value |= mgBone::Yposition;
    else if ( !name.compare("zposition") ) value |= mgBone::Zposition;
    else if ( !name.compare("zrotation") ) {
		value |= mgBone::Zrotation;
		orderTxt += "z";
	} else if ( !name.compare("xrotation") ) {
		value |= mgBone::Xrotation;
		orderTxt += "x";
	} else if ( !name.compare("yrotation") ) {
		value |= mgBone::Yrotation;
		orderTxt += "y";
	}
	// Add quat
	else if ( !name.compare("xquat") ) value |= mgBone::Xquat;
	else if ( !name.compare("yquat") ) value |= mgBone::Yquat;
	else if ( !name.compare("zquat") ) value |= mgBone::Zquat;
	else if ( !name.compare("wquat") ) value |= mgBone::Wquat;
}
