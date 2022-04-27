//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _POSE_TRANSFER_H_
#define _POSE_TRANSFER_H_

#include "mbs/gMultibodySystem.h"
#include "levmar.h"

/**
    @class		mgPoseTransfer
    @date		2015/03/26
    @author		Sung-Hee Lee
    @brief		This class provides method to transfer a pose from a character to another character. 
	Refer to Example8_CharacterRetargetting project for the usage of this class.
    @warning	
*/
class mgPoseTransfer{

public:

	struct idxDirTransfer{
		int idx0; //index of starting point 
		int idx1; //index of end point to define a direction vector
		double weight; //importance 
	};

	struct idxPosTransfer{
		int idx; //point index
		double weight; //importance
		double scale; //
	};
	struct idxDesired {
		int idx; // src point index
		double weight; //  importance
		gVec3 pos_desired; // desired position
	};
	arma::vec preferredTarCoords; //preferred coordinates (compact coordinates) of tar..note first 6 components (for baselink) are ignored. initially set to zero.
	double preferredTarCoordsWeight; //weight for the preferred coordinates
	double scale;
	double save_f_height;
	/**
	tar: target character to transfer pose to \n
	src: source character to import pose from \n
	tar/srcPoints: a set of sample points attached to characters. tarPoints and srcPoints must be consistent (i.e., if tarPoints[0]  \n
	is attached to tar's right elbow, srcPoints[0] must be attached to the src's right elbow) \n
	hence, it must be that tarPoints.size() = srcPoints.size() = namePoints.size() \n
	namePoints stores the name of the points used for identifying the points. \n
	directions: list of indices of two points to form a direction vector \n
	positions: list of point indices that will be glued from srcPoints to tarPoints \n
	\n
	how to transfer pose: \n
	We try to make the direction vector made by two points (from pointPairs) of tarCharacter match that of the corresponding vector of srcCharacter \n
	*/
	gMultibodySystem* src;
	gMultibodySystem* tar;	
	std::vector<std::string> namePoints;  //name of points
	std::vector<gAttachedPoint> srcPoints;//points in src
	std::vector<gAttachedPoint> tarPoints;//points in tar
	std::vector<idxDesired>		desiredPoints; // points for specific desired
	std::vector<idxDirTransfer> directions;
	std::vector<idxPosTransfer> positions;
	
	
	/* LevMarOpts: opts[0-4] = minim. options [\tau, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
	//                    * scale factor for initial \mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2 and the
	//                    * step used in difference approximation to the Jacobian. If \delta<0, the Jacobian is approximated
	//                    * with central differences which are more accurate (but slower!) compared to the forward differences
	//                    * employed by default. Set to NULL for defaults to be used.
	//                    */
	double LevMarOpts[5]; //option for levmar


	//                    /* O: information regarding the minimization. Set to NULL if don't care
	//                     * info[0]= ||e||_2 at initial p.
	//                     * info[1-4]=[ ||e||_2, ||J^T e||_inf,  ||Dp||_2, \mu/max[J^T J]_ii ], all computed at estimated p.
	//                     * info[5]= # iterations,
	//                     * info[6]=reason for terminating: 1 - stopped by small gradient J^T e
	//                     *                                 2 - stopped by small Dp
	//                     *                                 3 - stopped by itmax
	//                     *                                 4 - singular matrix. Restart from current p with increased \mu
	//                     *                                 5 - no further error reduction is possible. Restart with increased mu
	//                     *                                 6 - stopped by small ||e||_2
	//                     *                                 7 - stopped by invalid (i.e. NaN or Inf) "func" values; a user error
	//                     * info[7]= # function evaluations
	//                     * info[8]= # Jacobian evaluations
	//                     * info[9]= # linear systems solved, i.e. # attempts for reducing error
	//                     */
	double LevMarInfo[LM_INFO_SZ]; //performance report from levmar is stored here.

	/**
	motion will be transferred from srcCharacter to tarCharacter.
	*/
	mgPoseTransfer(gMultibodySystem* srcCharacter, gMultibodySystem* tarCharacter)
	{
		//assert(srcCharacter && tarCharacter);
		src = srcCharacter;
		tar = tarCharacter;
		preferredTarCoords.zeros(tar->dof());
		preferredTarCoordsWeight = 0.1;
		scale = 1.0;
		save_f_height = 0.0;
		double stopThresh = 1e-5;
		LevMarOpts[0]=1;
		LevMarOpts[1]=stopThresh;
		LevMarOpts[2]=stopThresh;
		LevMarOpts[3]=stopThresh;
		LevMarOpts[4]=LM_DIFF_DELTA;
	}

	virtual ~mgPoseTransfer(){}

	/**add points used for pose transfer.
	name: name of point. used for identification.
	srcLink/tarLink: link of src/tar to attach a point to
	srcPosLocal/tarPosLocal: position of a point wrt srcLink/tarLink's body frame	
	*/
	void addPoint(const char* name, gLink& srcLink, gVec3& srcPosLocal, gLink& tarLink, gVec3& tarPosLocal)
	{
		namePoints.push_back(name);
		gAttachedPoint ps(&srcLink,srcPosLocal);
		gAttachedPoint pt(&tarLink,tarPosLocal);
		srcPoints.push_back(ps);
		tarPoints.push_back(pt);		
	}

	void addDesiredJoint(const char* name, gLink& srcLink, gVec3& srcPosLocal)
	{
		namePoints.push_back(name);
		gAttachedPoint pt(&srcLink, srcPosLocal);
		tarPoints.push_back(pt);
	}

	/**
	add a direction objective for pose transfer
	direction is defined by a vector from pointName0 to pointName1
	weight: importance of this objective
	returns true if succeed.
	*/
	bool addDirectionObjective(const char* pointName0, const char* pointName1, double weight);

	/**
	add a position objective for pose transfer
	this tries to match the absolute position of the point of target character to that of source character.
	weight: importance of this objective
	returns true if succeed.
	*/
	bool addPositionObjective(const char* pointName, double weight);
	bool addPositionObjective(const char* pointName, double weight, double scale);
	bool addDesiredObjective(const char* pointName, double weight, gVec3 desiredpos);
	/*
	*this method transfers pose from source character to target character using levenberg-marquardt method
	offset: offset from sourse to target character
	returns 1 if succeed. 
	*/
	int transferPoseLevMar(gXMat& offset);
	
	int addDesiredPair(std::vector<gVec3> desiredPoses);
	int transferDesiredPoseLevMar(gXMat& offset, std::vector<gVec3> desiredJointPos);

	
};

#endif