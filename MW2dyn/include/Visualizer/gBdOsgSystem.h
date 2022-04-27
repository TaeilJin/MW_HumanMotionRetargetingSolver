//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _MBS_BD_OSG_SYSTEM_H_
#define _MBS_BD_OSG_SYSTEM_H_

#include "Character/bCharacter.h"
#include "Visualizer/gBDVisSystem.h"
/**
* MBS-BD Visualize System
**/
class gBDOSGSystem : public gBDVisSystem
{
public:
	enum FIGUREMODE{
		STICK,
		POLYGON
	};

public:
	gBDOSGSystem();
	virtual ~gBDOSGSystem() {};

	int setCharacter(const bCharacterSim *character, std::string name = "");
	void setFigureMode(FIGUREMODE mode) { _mode = mode; };
private:
	FIGUREMODE _mode;
};

#endif //_MBS_BD_OSG_SYSTEM_H_