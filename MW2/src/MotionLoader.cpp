//#####################################################################
// Copyright 2010-2015, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
//#include "Loader/loHeader.h"
#include "Loader/MotionLoader.h"
#include "Loader/LoaderBase.h"
#include "Loader/BVHLoader.h"
#include "Loader/AMCLoader.h"
#include "Loader/FBXLoader.h"

MotionLoader::MOTIONTYPE MotionLoader::getExtension(const char* filename)
{
	std::string path(filename);
	std::string::size_type sIdx = path.rfind('.');

	MOTIONTYPE type = NONE;

	if(sIdx != std::string::npos)
	{
		std::string extension = path.substr(sIdx+1);

		if( !extension.compare("bvh") )
		{
			type = BVH;
		} else if ( !extension.compare("amc") || !extension.compare("asf") ) {
			type = AMC;
		}
		else if (!extension.compare("fbx")) {
			type = FBX;
		}
	}
	else
	{
		// No extension found
		printf("WARNING: MotionLoader::getExtension cannot find extension in the file %s.\n", filename);
	}
	return type;
}

int MotionLoader::loadMotionFile(const char* filename, const bool skeletonOnly)
{
	//std::string path(filename);
	//std::string::size_type sIdx = path.rfind('.');

	LoaderBase *loader = NULL;
	MOTIONTYPE type = getExtension(filename);

	switch(type)
	{
	case BVH:
		loader = new BVHLoader();
		break;
	case AMC:
		loader = new AMCLoader();
		break;
	case FBX:
		loader = new FBXLoader();
		break;
	default:
		printf("ERROR: fail to load a proper loader.\n");
		return -1;
		break;
	}

	int result;

	if(!_useSkeletonFile)
	{
		if (_skeleton)
		{
			delete _skeleton;
			_skeleton = NULL;
		}

		_skeleton = new mgSkeleton;
		
		
		result = loader->loadSkeleton(filename, _skeleton);

		if( result )
		{
			printf("ERROR: fail to read skeleton data from motion file.\n");
			return -1;
		}
	}
	
	if( skeletonOnly ) 
	{
		if( _motions )
		{
			delete _motions;
			_motions = NULL;
		}

		return 0;
	}

	if( _motions ) delete _motions;

	_motions = new mgData;
	loader->setTranslateScale(_transScale);
	result = loader->loadMotion(filename, _skeleton, _motions);
	
	delete loader;
	loader = NULL;

	if( result )
	{
		printf("ERROR: fail to read the motion data from %s.\n", filename);
		return -1;
	}

	return 0;
}

int MotionLoader::setSkeletonFile(const char* filename)
{
	if (filename == NULL)
	{
		if (_skeleton)
		{
			delete _skeleton;
			_skeleton = NULL;
		}

		_useSkeletonFile = false;

		return 0;
	}

	LoaderBase *loader = NULL;
	MOTIONTYPE type = getExtension(filename);

	switch(type)
	{
	case BVH:
		loader = new BVHLoader();
		break;
	case AMC:
		loader = new AMCLoader();
		break;
	case FBX:
		loader = new FBXLoader();
	default:
		printf("ERROR: fail to load a proper loader.\n");
		return -1;
		break;
	}

	if (_skeleton)
	{
		delete _skeleton;
		_skeleton = NULL;
	}

	_skeleton = new mgSkeleton;
	
	int result;
	result = loader->loadSkeleton(filename, _skeleton);

	if( result )
	{
		printf("ERROR: cannot read skeleton data from motion file.\n");
		return -1;
	}

	_useSkeletonFile = true;


	return 0;
}
