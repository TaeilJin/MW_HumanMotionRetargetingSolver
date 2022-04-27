//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _MBS_HANDLER_H
#define _MBS_HANDLER_H

#if defined(WIN32) || defined(WIN64)
#include <Windows.h>
#endif

//#include "Visualizer/gVisHeader.h"
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>

typedef void (*EventFuncType)(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

class gEventHandler : public osgGA::GUIEventHandler
{
public:
	//typedef void (*EventFuncType)(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

public:
    gEventHandler();
	~gEventHandler();

   	bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

	int assignKey(osgGA::GUIEventAdapter::KeySymbol KEY, EventFuncType assignFunction, osgGA::GUIEventAdapter::ModKeyMask ModKey = (osgGA::GUIEventAdapter::ModKeyMask)0);
	//int assignButton(osgGA::GUIEventAdapter::MouseButtonMask BUTTON, EventFuncType assignFunction);
	int assignButton(osgGA::GUIEventAdapter::MouseButtonMask BUTTON, EventFuncType assignFunction, osgGA::GUIEventAdapter::ModKeyMask ModKey = (osgGA::GUIEventAdapter::ModKeyMask)0);

	//int setDoubleClickFunc(EventFuncType assignFunction) { doubleClicked=assignFunction; return 0;}

	// test
	EventFuncType move, relase;
private:

	// Handle Functions
	typedef std::pair<int, int>									CombModKey;
	typedef std::map<CombModKey,EventFuncType>					FunctionMap;
	typedef std::map<CombModKey,EventFuncType>::const_iterator	FunctionMIt;
	FunctionMap	_assignedKeyFunc;
	//FunctionMap _assignedBtnFunc;
	// <button, maks>
	
	std::map<CombModKey, EventFuncType> _assignedBtnFunc;
};

#endif