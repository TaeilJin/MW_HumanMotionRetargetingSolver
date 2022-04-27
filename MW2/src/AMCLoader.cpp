//#####################################################################
// Copyright 2010-2015, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#include "Loader/AMCLoader.h"
#include "MocapProcessor/mgUtility.h"
#include <algorithm>
#include <functional> 
#include <cctype>
//#include <locale>


int AMCLoader::loadMotion(const char* filename, mgSkeleton *skeleton, mgData *motion)
{
	assert( skeleton && motion );

	if(open_file( filename )==0) return (int)RT_OPENERR;

	readMotion(skeleton, motion);

	close_file();

	return 0;
}

int AMCLoader::loadSkeleton(const char* filename, mgSkeleton *skeleton)
{
	//get ASF file
	std::string path(filename);
	std::string::size_type sIdx = path.rfind('.');
	path = path.substr(0, sIdx+1) + "asf";

	//if(open_file(filename)==0) return (int)RT_OPENERR;
	if(open_file( path.c_str() )==0) return (int)RT_OPENERR;
	
	const int max_buffer_size = 128;
	char buf[max_buffer_size];
	memset(buf, 0, max_buffer_size);

	// get Root information
	if( go_after(":root") == -1 )
	{
		printf("Error: cannot find :root section.\n");
		return -1;
	}
	skeleton->boneRoot = skeleton->createBone("root");
	skeleton->dataPos.push_back(0);

	bool isRecognized = false;
	gVec3 posVec, rotVec;
	std::string axis;
	std::string orderBuf = "";

	read_word_s(buf, max_buffer_size);
	while( !feof(_file) && strncmp(buf, ":bonedata", max_buffer_size) != 0 )
	{
		isRecognized = false;
		if ( !strncmp(buf, "order", max_buffer_size) )
		{
			isRecognized = true;
			read_line_into_string(buf);
			
			std::string order = buf;
			
			order.erase(std::find_if(order.rbegin(), order.rend(),
				std::not1(std::ptr_fun<int, int>(std::isspace))).base(), order.end());

			int pos, prevPos=0;
			orderBuf.clear();
			while( (pos = order.find(' ', prevPos)) != std::string::npos )
			{
				alterChannel(order.substr(prevPos, pos-prevPos), skeleton->boneRoot->channel, orderBuf);
				prevPos = pos + 1;
				skeleton->boneRoot->nChannel++;
			}

			alterChannel(order.substr(prevPos), skeleton->boneRoot->channel, orderBuf);
			skeleton->boneRoot->nChannel++;
			
			skeleton->boneRoot->eulerConv = mgBone::EXTRINSIC; //SHL added to set "extrinsic" convention of Euler angles in AMC format
			
			if( orderBuf == "xyz" ) order = mgBone::XYZ;
			else if( orderBuf == "xzy" ) order = mgBone::XZY;
			else if( orderBuf == "yxz" ) order = mgBone::YXZ;
			else if( orderBuf == "yzx" ) order = mgBone::YZX;
			else if( orderBuf == "zxy" ) order = mgBone::ZXY;
			else if( orderBuf == "zyx" ) order = mgBone::ZYX;
			else if( orderBuf == "xy" ) order = mgBone::XY;
			else if( orderBuf == "xz" ) order = mgBone::XZ;
			else if( orderBuf == "yx" ) order = mgBone::YX;
			else if( orderBuf == "yz" ) order = mgBone::YZ;
			else if( orderBuf == "zx" ) order = mgBone::ZX;
			else if( orderBuf == "zy" ) order = mgBone::ZY;
			else if( orderBuf == "x" ) order = mgBone::X;
			else if( orderBuf == "y" ) order = mgBone::Y;
			else if( orderBuf == "z" ) order = mgBone::Z;
			else printf("does not support %s order.\n", orderBuf.c_str());
		}

		if ( !strncmp(buf, "position", max_buffer_size) )
		{
			isRecognized = true;
			posVec = readVec3();
		}

		if ( !strncmp(buf, "orientation", max_buffer_size) )
		{
			isRecognized = true;
			rotVec = readVec3();
		}

		if ( !strncmp(buf, "axis", max_buffer_size) )
		{
			isRecognized = true;
			read_word_s(buf, max_buffer_size);
			axis = buf;
		}

		if ( !isRecognized )
		{
			printf("BVH Reader: Unrecognized symbol %s. Ignore current field or block.\n", buf);
			//read_word(buf);
		}

		read_word_s(buf, max_buffer_size);
	}
	
	gRotMat rotMat;
	rotVec = gDTR * rotVec;

	std::transform(axis.begin(), axis.end(), axis.begin(), ::tolower);
	mgUtility::getRotMatFromRotVec( axis.c_str(), mgBone::EXTRINSIC, rotVec, rotMat);
	
	
	skeleton->boneRoot->H.setTrn(posVec);
	skeleton->boneRoot->H.setRot(rotMat);
	skeleton->boneRoot->localEndPoints.push_back(posVec);

	skeleton->nTotalChannel += skeleton->boneRoot->nChannel;

	if( skeleton->boneRoot->channel & mgBone::translateChannel )
		skeleton->numT++;
	if( skeleton->boneRoot->channel & mgBone::rotationChannel || 
	skeleton->boneRoot->channel & mgBone::expChannel || 
	skeleton->boneRoot->channel & mgBone::quaternionChannel)
		skeleton->numR++;

	readASFfile(skeleton);

	close_file();

	return 0;
}

int AMCLoader::readASFfile(mgSkeleton *skeleton)//, mgBone* root)
{
	const int max_buffer_size = 128;
	char buf[max_buffer_size];
	memset(buf, 0, max_buffer_size);

	bool isRecognized = false;
	mgBone* joint = NULL;
	std::string name;
	int id;
	gVec3 axis;
	unsigned int channel, noChannel;
	double limLow, limHigh;
	std::vector<std::pair<double,double>> limit;
	
	gVec3 direction_w;
	double length;

	// AXIS Storage
	// Warning: Assume that root matrix is identity.
	std::vector<gRotMat> axisMats;
	axisMats.push_back(gRotMat());

	
	int tPos = 6;
	mgBone::_AXISORDER order;
	
	if ( !skeleton ) return (int)RT_NULLP;

	read_word_s(buf, max_buffer_size);

	while( !feof(_file) && strncmp(buf, ":hierarchy", max_buffer_size) != 0 )
	{
		isRecognized = false;
		if ( !strncmp(buf, "id", max_buffer_size) )
		{
			isRecognized = true;

			//read_word_s(buf, max_buffer_size);
			id = readInteger(buf);
		}

		if ( !strncmp(buf, "name", max_buffer_size) )
		{
			isRecognized = true;

			read_word_s(buf, max_buffer_size);
			name = buf;
		}

		if ( !strncmp(buf, "direction", max_buffer_size) )
		{
			isRecognized = true;
			direction_w = readVec3();
		} 

		if ( !strncmp(buf, "length", max_buffer_size) )
		{
			isRecognized = true;
			length = readReal(buf);
		} 

		if ( !strncmp(buf, "axis", max_buffer_size) )
		{
			isRecognized = true;

			axis = readVec3();
			read_word_s(buf, max_buffer_size);

			std::string strAxis = buf;
			std::transform(strAxis.begin(), strAxis.end(), strAxis.begin(), ::tolower);
			
			axisMats.push_back(gRotMat());
			axis = gDTR * axis;
			mgUtility::getRotMatFromRotVec(strAxis.c_str(), mgBone::EXTRINSIC, axis, axisMats.back());
		}
		
		// read DOF & limit
		if ( !strncmp(buf, "dof", max_buffer_size) )
		{
			isRecognized = true;
			std::string orderBuf;

			read_word_s(buf, max_buffer_size);
			while( strncmp(buf, "limits", max_buffer_size) )
			{
				alterChannel(buf, channel, orderBuf);
				noChannel++;

				read_word_s(buf, max_buffer_size);
			}

			if( orderBuf == "xyz" ) order = mgBone::XYZ;
			else if( orderBuf == "xzy" ) order = mgBone::XZY;
			else if( orderBuf == "yxz" ) order = mgBone::YXZ;
			else if( orderBuf == "yzx" ) order = mgBone::YZX;
			else if( orderBuf == "zxy" ) order = mgBone::ZXY;
			else if( orderBuf == "zyx" ) order = mgBone::ZYX;
			else if( orderBuf == "xy" ) order = mgBone::XY;
			else if( orderBuf == "xz" ) order = mgBone::XZ;
			else if( orderBuf == "yx" ) order = mgBone::YX;
			else if( orderBuf == "yz" ) order = mgBone::YZ;
			else if( orderBuf == "zx" ) order = mgBone::ZX;
			else if( orderBuf == "zy" ) order = mgBone::ZY;
			else if( orderBuf == "x" ) order = mgBone::X;
			else if( orderBuf == "y" ) order = mgBone::Y;
			else if( orderBuf == "z" ) order = mgBone::Z;
			else printf("does not support %s order.\n", orderBuf.c_str());
			// read limit
			for( int i=0; i<noChannel; i++ )
			{
				read_word_s(buf, max_buffer_size);
				limLow = gReal(atof(buf+1));
				limHigh = readReal(buf);

				limit.push_back( std::pair<double, double>(limLow, limHigh) );
			}
		}

		if ( !strncmp(buf, "begin", max_buffer_size) )
		{
			isRecognized = true;
			noChannel = channel = 0;
			limit.clear();
			order = mgBone::XYZ;
			direction_w.setZero();
			length = 0.;
		}

		if ( !strncmp(buf, "end", max_buffer_size) )
		{
			isRecognized = true;

			joint = skeleton->createBone(name);
			skeleton->dataPos.push_back(tPos);
			tPos += noChannel;
			
			if( joint->id != id )
			{
				printf("warning: find a disparity between auto ID and real ID.\n");
			}

			joint->channel = channel;
			joint->nChannel = noChannel;
			skeleton->nTotalChannel += noChannel;
			joint->order = order;
			
			joint->eulerConv = mgBone::EXTRINSIC; //SHL added to set "extrinsic" convention of Euler angles in AMC format
			joint->limit.assign(limit.begin(), limit.end());
			//std::copy(limit.begin(), limit.end(), joint->limit.begin());

			if( joint->channel & mgBone::translateChannel )
				skeleton->numT++;
			if( joint->channel & mgBone::rotationChannel || 
				joint->channel & mgBone::expChannel || 
				joint->channel & mgBone::quaternionChannel)
				skeleton->numR++;

			joint->direction_w = direction_w;
			joint->length = length;
		}

		if ( !isRecognized )
		{
			printf("AMC Reader: Unrecognized symbol %s. Ignore current field or block.\n", buf);
		}

		read_word_s(buf, max_buffer_size);
	}

	// hierarchy
	// begin
	read_word_s(buf, max_buffer_size);

	mgSkeleton::BONEMIt it;
	read_word_s(buf, max_buffer_size);
	gXMat parentWMat, currentWMat;
	gRotMat currentRot;

	while( !feof(_file) && strncmp(buf, "end", max_buffer_size) != 0 )
	{
		//read_word_s(buf, max_buffer_size);

		it = skeleton->boneMap.find(buf);
		if( it == skeleton->boneMap.end() )
		{
			printf("error: there is no %s bone.\n", buf);
			break;
		}
		
		joint = skeleton->bones[it->second];

		read_line_into_string(buf);

		std::string names = buf;
		std::string name;
		int pos, prevPos=0;
		mgBone* child;
		gXMat trans;
		
		skeleton->getWHMatrixAt(joint->id, parentWMat);

		while( (pos = names.find(' ', prevPos)) != std::string::npos )
		{
			name = names.substr(prevPos, pos-prevPos);
			it = skeleton->boneMap.find(name);
			if( it == skeleton->boneMap.end() )
			{
				printf("error: there is no %s bone.\n", buf);
				break;
			}

			child = skeleton->bones[it->second];
			joint->children.push_back(child);
			child->parent = joint;

			// set H matrix
			currentRot = parentWMat.rot().invMult(axisMats[child->id]);
			child->localEndPoints.push_back(axisMats[child->id].invMult( child->direction_w ) * child->length);
			child->H.setTrn(joint->localEndPoints[0]);
			child->H.setRot(currentRot);

			prevPos = pos+1;
		}

		name = names.substr(prevPos);
		it = skeleton->boneMap.find(name);
		if( it == skeleton->boneMap.end() )
		{
			printf("error: there is no bone name is %s.\n", buf);
			break;
		}

		child = skeleton->bones[it->second];
		joint->children.push_back(child);
		child->parent = joint;

		// set H matrix
		currentRot = parentWMat.rot().invMult(axisMats[child->id]);
		child->localEndPoints.push_back(axisMats[child->id].invMult( child->direction_w ) * child->length);
		
		child->H.setTrn(joint->localEndPoints[0]);
		child->H.setRot(currentRot);

		read_word_s(buf, max_buffer_size);
	}
	// end

	// setting end effector info
	for(int i=0; i<skeleton->bones.size(); i++)
	{
		joint = skeleton->bones[i];
		if( joint->children.empty() )
		{
			skeleton->getWHMatrixAt(joint->id, parentWMat);
			currentRot = parentWMat.rot().invMult(axisMats[joint->id]);
		}
	}
	
	return 0;
}

int	AMCLoader::readMotion(mgSkeleton *skeleton, mgData *motion)
{
	const int max_buffer_size = 128;
	char buf[max_buffer_size];
	memset(buf, 0, max_buffer_size);

	mgSkeleton::BONEMIt it;

	/*
	while(1) {
		read_word_s(buf, max_buffer_size);
	
		if( buf[0] != '#' && buf[0] != ':' )
		{
		}
	}
	*/
	
	std::vector<double *> pMotion;
	int frame = 0;
	int pos = 0;
	int i_ch = 0;
	mgBone *bone;

	go_after("1");

	read_word_s(buf, max_buffer_size);
	while( !feof(_file) )
	{
		pMotion.push_back( new double[skeleton->nTotalChannel] );
		memset(pMotion.back(), 0, sizeof(double) * skeleton->nTotalChannel );

		i_ch = 0;
		while( !feof(_file) && !isdigit(buf[0]) )
		{
			//read_word_s(buf, max_buffer_size);

			it = skeleton->boneMap.find(buf);
			if( it == skeleton->boneMap.end() )
			{
				printf("error: there is no bone name is %s.\n", buf);
				break;
			}

			pos = skeleton->dataPos[it->second];
			bone = skeleton->bones[it->second];
			
			for( int i=0; i< bone->nChannel; i++ )
			{
				pMotion[frame][pos+i] = readReal(buf);
			}
			//printf("%d\n", isdigit(buf[0]));
			read_word_s(buf, max_buffer_size);

			i_ch++;
		}
		frame++;
		read_word_s(buf, max_buffer_size);
	}

	//testtest
	motion->frameTime = 1./120.;
	motion->nMotion = pMotion.size();
	motion->nChannel = skeleton->nTotalChannel;
	motion->allocMemory();

	//TEST
	/*FILE* fp = fopen("Motion.txt", "w");
	for( int i=0; i<skeleton->bones.size(); i++ )
	{
		bone = skeleton->bones[i];
		for( int j=0; j<bone->nChannel; j++ )
		{
			fprintf(fp, "%s\t", bone->name.c_str());
		}
	}
	fprintf(fp, "\n");*/

	for( int i=0; i<pMotion.size(); i++ )
	{
		memcpy_s(motion->motions[i], skeleton->nTotalChannel * sizeof(double), pMotion[i], skeleton->nTotalChannel * sizeof(double));

		/*for( int j=0; j<skeleton->nTotalChannel; j++ )
		{
			fprintf(fp, "%f\t", motion->motions[i][j]);
		}
		fprintf(fp, "\n");*/

		delete [] pMotion[i];
	}
	
	//fclose(fp);
	
	return 0;
}

//void AMCLoader::alterChannel( std::string name, unsigned int& value )
void AMCLoader::alterChannel( std::string name, unsigned int& value, std::string& orderTxt )
{
	std::transform(name.begin(), name.end(), name.begin(), ::tolower);
	name.erase(name.find_last_not_of(' ')+1);
	//name.erase( name.begin(), std::find_if( name.begin(), name.end(), std::not1( std::ptr_fun<int, int>(std::isspace) ) ) );

    if      ( !name.compare("tx") ) value |= mgBone::Xposition;
    else if ( !name.compare("ty") ) value |= mgBone::Yposition;
    else if ( !name.compare("tz") ) value |= mgBone::Zposition;
    else if ( !name.compare("rz") ) {
		value |= mgBone::Zrotation;
		orderTxt += "z";
	} else if ( !name.compare("rx") ) {
		value |= mgBone::Xrotation;
		orderTxt += "x";
	} else if ( !name.compare("ry") ) {
		value |= mgBone::Yrotation;
		orderTxt += "y";
	}
	else if ( !name.compare("ex") ) {
		value |= mgBone::Xexp;
		orderTxt += "x";
	} else if ( !name.compare("ey") ) {
		value |= mgBone::Yexp;
		orderTxt += "y";
	} else if ( !name.compare("ez") ) {
		value |= mgBone::Zexp;
		orderTxt += "z";
	}
}
