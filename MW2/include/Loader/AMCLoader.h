//#####################################################################
// Copyright 2010-2015, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _AMC_LOADER_H
#define _AMC_LOADER_H

#include "Loader/LoaderBase.h"
/**
    @class		AMCLoader
    @date		2014/02/18
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		load AMC/ASF motion file.
    @warning	
*/
class AMCLoader:public LoaderBase
{
public:
	int loadMotion(const char* filename, mgSkeleton *skeleton, mgData *motion);
	int loadSkeleton(const char* filename, mgSkeleton *skeleton);

private:
	int readASFfile(mgSkeleton *skeleton); //, mgBone* root);
	int	readMotion(mgSkeleton *skeleton, mgData *motion);

	//void alterChannel( std::string name, unsigned int& value );
	void alterChannel( std::string name, unsigned int& value, std::string& orderTxt );
};

#endif