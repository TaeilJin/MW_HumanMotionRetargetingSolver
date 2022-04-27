//#####################################################################
// Copyright 2010-2015, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _LOADER_BASE_H
#define _LOADER_BASE_H

#include "Loader/Parser.h"
#include "MocapProcessor/mgSkeleton.h"
/**
    @class		LoaderBase
    @date		2014/02/18
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		A basic templeate for motion file loader.
    @warning	
*/

//static const unsigned int Xposition = 0x01;
//static const unsigned int Yposition = 0x02;
//static const unsigned int Zposition = 0x04;
//static const unsigned int Zrotation = 0x08;
//static const unsigned int Xrotation = 0x10;
//static const unsigned int Yrotation = 0x20;
//static const unsigned int Xquat = 0x40;
//static const unsigned int Yquat = 0x80;
//static const unsigned int Zquat = 0x100;
//static const unsigned int Wquat = 0x200;
//static const unsigned int translateChannel = Xposition | Yposition | Zposition;
//static const unsigned int rotationChannel = Zrotation | Xrotation | Yrotation;
//static const unsigned int quaternionChannel = Xquat | Yquat | Zquat | Wquat;

class LoaderBase:public Parser
{
public:
	// channels
	//static const unsigned int Xposition = 0x01;
 //   static const unsigned int Yposition = 0x02;
 //   static const unsigned int Zposition = 0x04;
 //   static const unsigned int Zrotation = 0x08;
 //   static const unsigned int Xrotation = 0x10;
 //   static const unsigned int Yrotation = 0x20;
	//static const unsigned int Xquat = 0x40;
	//static const unsigned int Yquat = 0x80;
	//static const unsigned int Zquat = 0x100;
	//static const unsigned int Wquat = 0x200;

	//static const unsigned int translateChannel = Xposition | Yposition | Zposition;
	//static const unsigned int rotationChannel = Zrotation | Xrotation | Yrotation;
	//static const unsigned int quaternionChannel = Xquat | Yquat | Zquat | Wquat;


	// return values
	enum {
		RT_OK		= 0x00,
		RT_NULLP	= 0x02,
		RT_NOINIT	= 0x04,
		RT_OPENERR	= 0x08
	} RT;

public:
	LoaderBase(){
		_transScale = 1.0;
	};
	~LoaderBase(){};

	virtual int loadMotion(const char* filename, mgSkeleton *skeleton, mgData *motion) = 0;
	virtual int loadSkeleton(const char* filename, mgSkeleton *skeleton) = 0;

	void setTranslateScale(CoordinateType scale) { _transScale = scale; }
	double getTranslateScale() { return _transScale; }

	//void alterChannel( char* name, int nameMaxLen, unsigned int& value ) = 0;
	//virtual void alterChannel( std::string name, unsigned int& value, std::string& orderTxt ) = 0;

protected:
	double _transScale;
};

#endif