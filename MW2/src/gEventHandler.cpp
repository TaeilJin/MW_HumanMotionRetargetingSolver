//#####################################################################
// Copyright 2010-2015 Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#include "Visualizer/gEventHandler.h"
 
gEventHandler::gEventHandler()
{
	move = relase = NULL;
}
gEventHandler::~gEventHandler()
{
}

int gEventHandler::assignKey(osgGA::GUIEventAdapter::KeySymbol KEY, EventFuncType assignFunction, osgGA::GUIEventAdapter::ModKeyMask ModKey)
{
	FunctionMIt it;
	it = _assignedKeyFunc.find( CombModKey(KEY, ModKey) );
	if( it != _assignedKeyFunc.end() ) {
#ifdef _DEBUG
		printf("Duplicated Assigned Key\n");
#endif
		return -1;
	}
	_assignedKeyFunc[CombModKey(KEY, ModKey)] = assignFunction;
	return 0;
}

int gEventHandler::assignButton(osgGA::GUIEventAdapter::MouseButtonMask BUTTON, EventFuncType assignFunction, osgGA::GUIEventAdapter::ModKeyMask ModKey)
{
	FunctionMIt it;
	it = _assignedBtnFunc.find( CombModKey(BUTTON, ModKey) );
	if( it != _assignedBtnFunc.end() ) {
#ifdef _DEBUG
		printf("Duplicated Assigned Btn\n");
#endif
		return -1;
	}

	_assignedBtnFunc[CombModKey(BUTTON, ModKey)] = assignFunction;

	return 0;
}

bool gEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
	osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
	if (!view) return false;

	switch(ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::KEYDOWN):
		{
			CombModKey getKey(ea.getKey(), ea.getModKeyMask());

			FunctionMIt it = _assignedKeyFunc.find( getKey );
			if( it != _assignedKeyFunc.end() )
			{
				(it->second)(ea, aa);
			}
			break;
		}
	case(osgGA::GUIEventAdapter::PUSH):
		{
			CombModKey getBtn(ea.getButton(), ea.getModKeyMask());

			FunctionMIt it = _assignedBtnFunc.find( getBtn );
			if( it != _assignedBtnFunc.end() )
			{
				(it->second)(ea, aa);
			}
			break;
		}
	case(osgGA::GUIEventAdapter::MOVE):
		{
			if( move ) move(ea, aa);
			break;
		}
	case(osgGA::GUIEventAdapter::RELEASE):
		{
			if( relase ) relase(ea, aa);
			break;
		}
	}
	return false;
}