//#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#include "MocapProcessor/mgMotionKinematicFeature.h"
#include <deque>

extern gVec3 MW_GROUND_NORMAL;

mgMotionKinematicFeature::mgMotionKinematicFeature(mgSkeleton* skel, mgData* motion):
_skeleton(skel), _motion(motion), _constVelThres(0.0), _sizeHalfWindow(0)
{
	positionsVec.resize(_skeleton->bones.size());
	cIdxTopIdx.resize(_skeleton->bones.size());
	_contactProb.resize(_skeleton->bones.size());
};

int mgMotionKinematicFeature::setMotionData(mgData *motion)
{
	_motion = motion;
	return (int)RT_OK;
}

int mgMotionKinematicFeature::addContactCandidate(std::string boneName)
{
	_stanceCandidate.push_back(boneName);
	return (int)RT_OK;
}

int mgMotionKinematicFeature::addContactCandidate(int boneID)
{
	_stanceCandidate.push_back(_skeleton->bones[boneID]->name);
	return (int)RT_OK;
}

int mgMotionKinematicFeature::detectContact(gVec3Array &position, boolArray &cFlag, bool useLimit, double limit)
{
	gVec3ArrayIt posIt = position.begin();
	gVec3 prevPos;
	//contact flag
	cFlag.clear();
	cFlag.reserve(_motion->nMotion);
	cFlag.push_back(0);

	// get the frame Id where the constraint position is steady in sequence.
	prevPos = *posIt;
	posIt++;

	if( useLimit )
	{
		while( posIt != position.end() )
		{
			if( (*posIt - prevPos).magnitude() < _constVelThres && (MW_GROUND_NORMAL, *posIt) < limit ) cFlag.push_back(1);
			else cFlag.push_back(0);
			prevPos = *posIt;
			posIt++;
		}
	} else {
		while( posIt != position.end() )
		{
			if( (*posIt - prevPos).magnitude() < _constVelThres ) cFlag.push_back(1);
			else cFlag.push_back(0);
			prevPos = *posIt;
			posIt++;
		}
	}

	return (int)RT_OK;
}

int mgMotionKinematicFeature::detectConstraint(std::string constName)
{
	int boneId = _skeleton->getBoneIdFromName(constName);
	
	/*
	if(contactsFlagMap.find(boneId) != contactsFlagMap.end() )
	{
#ifdef _DEBUG
		printf("Duplicated Constrainted Bone. (Const Bone Name: %s)\n", constName.c_str() );
#endif
		return (int)RT_DUPLICATE;
	}
	*/

	std::vector<uint8_t> *contactFlag = &contactsFlagMap[boneId];
	contactFlag->clear();

	// parameter
	double velocityThreshold =  _constVelThres;

	// get constraint 3d position array from all motions
	gVec3Array allPositions;
	getAllWPosition(constName, allPositions);

	// detect contact.
	detectContact(allPositions, *contactFlag, false, 0.0);

	boolFiltering(*contactFlag);

	// avg position
	positionsVec[boneId].clear();
	unsigned int firstPos, contactLength;
	gVec3 tempPos;
	for( int pos=0; pos<contactFlag->size(); pos++ )
	{
		if( contactFlag->at(pos) != 0 ) {

			tempPos.set(0.0,0.0,0.0);
			firstPos = pos;
			contactLength = 0;
			// Possible error. if compiler doesn't support short-circuit evaluation.
			while( pos<contactFlag->size() && contactFlag->at(pos) != 0 ) {
				tempPos += allPositions[pos];
				contactLength++;
				pos++;
			}
			tempPos /= contactLength;
			positionsVec[boneId].push_back(tempPos);
		}
	}

	return (int)RT_OK;
}

int mgMotionKinematicFeature::detectConstraintWithGroundLimit(std::string constName, double limit)
{
	int boneId = _skeleton->getBoneIdFromName(constName);
	
	/*
	if(contactsFlagMap.find(boneId) != contactsFlagMap.end() )
	{
#ifdef _DEBUG
		printf("Duplicated Constrainted Bone. (Const Bone Name: %s)\n", constName.c_str() );
#endif
		return (int)RT_DUPLICATE;
	}
	*/

	std::vector<uint8_t> *contactFlag = &contactsFlagMap[boneId];
	contactFlag->clear();

	// parameter
	double velocityThreshold =  _constVelThres;

	// get constraint 3d position array from all motions
	gVec3Array allPositions;
	getAllWPosition(constName, allPositions);

	// detect contact.
	detectContact(allPositions, *contactFlag, true, limit);

	boolFiltering(*contactFlag);

	// avg position
	gVec3Array& bonePosition = positionsVec[boneId];
	intArray& boneCTP = cIdxTopIdx[boneId];

	bonePosition.clear();
	boneCTP.clear();

	boneCTP.resize(contactFlag->size());

	unsigned int firstPos, contactLength;
	gVec3 tempPos;
	for( int pos=0; pos<contactFlag->size(); pos++ )
	{
		if( contactFlag->at(pos) != 0 ) 
		{
			tempPos.set(0.0,0.0,0.0);
			firstPos = pos;
			contactLength = 0;
			// Possible error. if compiler doesn't support short-circuit evaluation.
			while( pos<contactFlag->size() && contactFlag->at(pos) != 0 ) {
				tempPos += allPositions[pos];

				boneCTP[pos] = bonePosition.size();

				contactLength++;
				pos++;
			}
			if( pos<contactFlag->size() ) boneCTP[pos] = -1;

			tempPos /= contactLength;
			bonePosition.push_back(tempPos);
		} else {
			boneCTP[pos] = -1;
		}
	}

	return (int)RT_OK;
}

int mgMotionKinematicFeature::boolFiltering(boolArray &data, unsigned int sizeofHalfL1)
{
	std::deque<bool> convolute;
	unsigned int sum = 0, pos;

	for( pos=0; pos<data.size(); pos++ )
	{
		convolute.push_back(data[pos]);
		sum += data[pos];
		
		if( convolute.size() < sizeofHalfL1 + 1 ) continue;
		else if( convolute.size() > (sizeofHalfL1 * 2 + 1) ) {
			sum -= convolute.front();
			convolute.pop_front();
		}

		if( sum * 2 >= convolute.size() ) data[pos - sizeofHalfL1] = 1;
		else data[pos - sizeofHalfL1] = 0;
	}

	while( pos < sizeofHalfL1 + data.size())
	{
		sum -= convolute.front();
		convolute.pop_front();

		if( sum * 2 >= convolute.size() ) data[pos - sizeofHalfL1] = 1;
		else data[pos - sizeofHalfL1] = 0;
		pos++;
	}

	return 0;
}

int mgMotionKinematicFeature::boolFiltering(boolArray &data)
{
	unsigned int sizeofHalfL1 = _sizeHalfWindow; // window size

	// convolute. filtering noise.
	std::deque<bool> convolute;
	unsigned int sum = 0, pos;

	for( pos=0; pos<data.size(); pos++ )
	{
		convolute.push_back(data[pos]);
		sum += data[pos];
		
		if( convolute.size() < sizeofHalfL1 + 1 ) continue;
		else if( convolute.size() > (sizeofHalfL1 * 2 + 1) ) {
			sum -= convolute.front();
			convolute.pop_front();
		}

		if( sum * 2 >= convolute.size() ) data[pos - sizeofHalfL1] = 1;
		else data[pos - sizeofHalfL1] = 0;
	}

	while( pos < sizeofHalfL1 + data.size())
	{
		sum -= convolute.front();
		convolute.pop_front();

		if( sum * 2 >= convolute.size() ) data[pos - sizeofHalfL1] = 1;
		else data[pos - sizeofHalfL1] = 0;
		pos++;
	}

	return 0;
}

int mgMotionKinematicFeature::getAllW(const int id, gVec3Array &data)
{
	data.clear();	
	data.reserve(_motion->nMotion);
	gXMat worldMatrix;

	// apply motion
	for( unsigned int frame=0; frame<_motion->nMotion; frame++)
	{
		worldMatrix.setIdentity();
		_skeleton->getWMatrixAt(id, _motion->motions[frame], worldMatrix);
		data.push_back( worldMatrix.trn() );
	}
	return 0;
}

int mgMotionKinematicFeature::getAllW(const int id, gXMatArray &data)
{
	data.clear();	
	data.reserve(_motion->nMotion);
	gXMat worldMatrix;

	// apply motion
	for( unsigned int frame=0; frame<_motion->nMotion; frame++)
	{
		worldMatrix.setIdentity();
		_skeleton->getWMatrixAt(id, _motion->motions[frame], worldMatrix);
		data.push_back( worldMatrix );
	}
	return 0;
}

int mgMotionKinematicFeature::getAllWPosition(std::string constName, gVec3Array &data)
{
	data.clear();
	data.reserve(_motion->nMotion);
	gXMat worldMatrix;

	// apply motion
	for( unsigned int frame=0; frame<_motion->nMotion; frame++)
	{
		worldMatrix.setIdentity();
		_skeleton->getWMatrixAt(constName, _motion->motions[frame], worldMatrix);
		data.push_back( worldMatrix.trn() );
	}
	return 0;
}

int mgMotionKinematicFeature::setContactParameter(double constVelThres, int sizeHalfWindow)
{
	_constVelThres = constVelThres;
	_sizeHalfWindow = sizeHalfWindow;

	return (int)RT_OK;
}

int mgMotionKinematicFeature::detectConstraint()
{
	for( unsigned int j=0; j<_stanceCandidate.size(); j++ )
	{
		detectConstraint( _stanceCandidate[j]);
	}
	return (int)RT_OK;
}

int mgMotionKinematicFeature::detectConstraintWithGroundLimit(double limit)
{
	for( unsigned int j=0; j<_stanceCandidate.size(); j++ )
	{
		detectConstraintWithGroundLimit( _stanceCandidate[j], limit);
	}
	return (int)RT_OK;
}

int mgMotionKinematicFeature::getContactFlagMap(boolArrayMap& flagMap)
{
	flagMap = contactsFlagMap;
	return (int)RT_OK;
}

int mgMotionKinematicFeature::getContactPositionMap(std::vector<gVec3>& posVec)
{
	//posMap = positionsMap;
	printf("Deprecated Method.\n");
	assert(false);
	return (int)RT_OK;
}

int mgMotionKinematicFeature::getMotionWMap(const std::vector<int>& bonelist, gVec3ArrayMap& posMap)
{
	posMap.clear();

	for( unsigned int j=0; j<bonelist.size(); j++ )
	{
		getAllW( bonelist[j], posMap[bonelist[j]] );
	}
	return (int)RT_OK;
}

int mgMotionKinematicFeature::getMotionWMap(const std::vector<int>& bonelist, gXMatArrayMap& wMap)
{
	wMap.clear();

	for( unsigned int j=0; j<bonelist.size(); j++ )
	{
		getAllW( bonelist[j], wMap[bonelist[j]] );
	}
	return (int)RT_OK;
}

int mgMotionKinematicFeature::getMotionAllWPositionMap(gVec3ArrayMap& posMap)
{
	posMap.clear();

	for( unsigned int j=0; j<_skeleton->bones.size(); j++ )
	{
		getAllW( _skeleton->bones[j]->id, posMap[_skeleton->bones[j]->id] );
	}

	return (int)RT_OK;
}

int mgMotionKinematicFeature::getPosIdxWithCtIdx(const int boneId, const int contactIndex)
{
	//intArrayMapIt it = cIdxTopIdx.find(boneId);

	//if( it == cIdxTopIdx.end() )
	//{
	//	return -2;
	//}
	
	//return it->second[contactIndex];

	return cIdxTopIdx[boneId][contactIndex];
}

int mgMotionKinematicFeature::getContactProbability(int boneId, std::vector<double>& cp, int filterSize, int numOfApply)
{
	cp.clear();
	cp.resize(_motion->nMotion);

	boolArray& contact = contactsFlagMap[boneId];
	// initialize
	for( int i=0; i<_motion->nMotion; i++ )
	{
		cp[i] = (contact[i])?1.0:0.0;
	}
	/*
	// Smoothing
	std::list< double > queue;
	std::vector<double> gaussian(filterSize);

	double sigma = 1; // (double)filterSize / 8.0;
	double mu = (double)filterSize / 2.0;
	//double filtered;
	// gaussian

	double filterSum=0;
	for( int i=0; i<filterSize; i++ )
	{
		filterSum += gaussian[i] = exp(-1 * ( (i-mu)*(i-mu) / (2.0*sigma*sigma) ) ) / (sigma * sqrt( 2.0 * gPI ));
	}
	// normalize
	for( int i=0; i<filterSize; i++ )
	{
		gaussian[i] = gaussian[i] / filterSum;
	}
		
	double sum;
	for( int a=0; a<numOfApply; a++ )
	{
		for( int i=0; i<_motion->nMotion; i++ )
		{
			queue.push_back( cp[i] );
				
			if( queue.size() > filterSize ) {
				queue.pop_front();

				p=0;
				sum = 0.0;
				for( queue=queue.begin(); queue!=queue.end(); queue++ )
				{
					sum += (*queue) * gaussian[p];
					p++;
				}
				cp[(int)(i-mu)][0] =sum;
			}
		}
	}
*/
	return 0;
}

int mgMotionKinematicFeature::procContactProbability()
{
	for( int i=0; i<_skeleton->bones.size(); i++ )
	{
		getContactProbability(i, _contactProb[i], 6, 2);
	}

	return 0;
}