//#####################################################################
// Copyright 2010-2015, Hynchul Choi, Sukwon Lee, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _BCHARACTER_LOADER_H_
#define _BCHARACTER_LOADER_H_
//
#include "Character/bCharacter.h"
#include "Loader/Parser.h"

//Note: originally bCharacterLoader inherited gParser in gMbsUtil.h
class bCharacterLoader:public Parser 
{
public:
					 bCharacterLoader(){};
	virtual			~bCharacterLoader(){};
	//virtual int		 loadModel(const char* filename, gMultibodySystem* system, gVisSystem* visSys=NULL);
	
	//2012-02-11: scale: scaling the size of the model
	virtual int		 loadModel(const char* filename, bCharacter* system, gReal scale=1.0);

private:
	gRotMat			 readRot();
	//gVisBody*		 readVisBody(gLink* link);

	int				 readBtCollisionShapes(const char* file, bCharacter* system);
	void			 readLink(bCharacter* system);

	// 2012-02-01 : added
	// 2012-02-11 : added scale parameter. See scaleSize() for details.
	//TODO: joint limits of prismatic joint need to be scaled as well.
	int				readFile(const char* fileHiearchy, bCharacter* system, gReal scale); 
	
	
	// 2012-02-01 : added.
	//int				readWorldConfig(gMultibodySystem* system);
	int				readHierarchy(bCharacter* system);
	int				readInertia(bCharacter* system);
	int				readGeometry(bCharacter* system, gReal scale);

	// 2012-04-26 : foot geometry info added. Deprecated.
	//int				readFootGeometry(bCharacter* system, gReal scale);

	// 2014-01-03 : chain info added
	int				readChains(bCharacter* system);

	// 2012-05-13 : read foot and arm index
	//int				readLimbsIdx(bCharacter* system);
	
	//2012-02-11: scale the size of the multibody system. 	
	void			scaleSize(bCharacter* system, gReal scale);
	
	gInertia		readInertia();
	gInertia		readInertiaParam();	// 2012-02-01 : added. it does same function as readInertia
	int				readHierarchy(const char* file, bCharacter* system);
};

#endif // _MBS_SYSTEM_LOADER_H_