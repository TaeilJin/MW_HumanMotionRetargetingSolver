                                               //#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#ifndef _MG_DATA_H
#define _MG_DATA_H

#include "Base/gBase.h"
/**
    @class		mgData
    @date		2014/02/18
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		A model for storing motion data. It consists of 2-Dim numbers.
    @warning	
*/
class mgData {
public:
	enum SAMPLING_METHOD{
		NEAREST_NEIGHBOR,
		LINEAR,
		GAUSSIAN
	};

	enum FILE_TYPE{
		BINARY,
		CSV,
		TAB
	};

public:
	mgData(bool owner=true):
	nMotion(0),
	nChannel(0),
	motions(NULL),
	pMotions(NULL),
	m_owner(owner)
	{
	}

	mgData(const unsigned frames, const unsigned channels, CoordinateType fps = 0.0):
		nMotion(frames),
		nChannel(channels),
		frameTime(fps),
		motions(NULL),
		pMotions(NULL),
		m_owner(true)
	{
		allocMemory();
	}

	mgData(const mgData& ref, const bool deepcopy = true)
	{
		nMotion = ref.nMotion;
		frameTime = ref.frameTime;
		nChannel = ref.nChannel;
		m_owner = deepcopy;
		
		if(deepcopy)
		{
			allocMemory();
			memcpy(pMotions, ref.pMotions, sizeof(CoordinateType) * nMotion * nChannel);
		} else {
			pMotions = ref.pMotions;
			motions = ref.motions;
		}
	}

	~mgData() {
		deallocMemory();
		/*if( m_owner && motions ) delete [] motions;
		if( m_owner && pMotions ) delete [] pMotions;*/
	}

	void allocMemory( const int frame, const int channel );
	void allocMemory();
	void deallocMemory();

	// motion sampler
	void sampler(CoordinateType* data, const double frame, SAMPLING_METHOD method = NEAREST_NEIGHBOR);

	void save(std::ostream& os, const FILE_TYPE type = TAB);
	void save(const char* filename, const FILE_TYPE type = TAB);

	CoordinateType** motions;
	CoordinateType* pMotions;

	unsigned int	nMotion;
	unsigned int	nChannel;
	CoordinateType	frameTime;

private:
	bool			m_owner;
};

#endif