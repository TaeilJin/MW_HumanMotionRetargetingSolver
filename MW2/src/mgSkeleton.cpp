//#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#include "MocapProcessor/mgSkeleton.h"
#include "mbs/gMbsUtil.h"
#include "mbs/gArmaUtil.h"
#include "MocapProcessor/mgUtility.h"

static void gXmultRp(gXMat& dst, const gXMat& X, const gRotMat& R, const gVec3& P)
{
	gReal* d = dst.ptr();

	const gReal* r = R.cptr();
	const gReal* x = X.cptr();
	const gReal* p = P.cptr();

	d[12] = x[0] * p[0] + x[4] * p[1] + x[8] * p[2] + x[12];
	d[13] = x[1] * p[0] + x[5] * p[1] + x[9] * p[2] + x[13];
	d[14] = x[2] * p[0] + x[6] * p[1] + x[10] * p[2] + x[14];

	d[0] = x[0] * r[0] + x[4] * r[1] + x[8] * r[2];
	d[4] = x[0] * r[3] + x[4] * r[4] + x[8] * r[5];
	d[8] = x[0] * r[6] + x[4] * r[7] + x[8] * r[8];

	d[1] = x[1] * r[0] + x[5] * r[1] + x[9] * r[2];
	d[5] = x[1] * r[3] + x[5] * r[4] + x[9] * r[5];
	d[9] = x[1] * r[6] + x[5] * r[7] + x[9] * r[8];

	d[2] = x[2] * r[0] + x[6] * r[1] + x[10] * r[2];
	d[6] = x[2] * r[3] + x[6] * r[4] + x[10] * r[5];
	d[10] = x[2] * r[6] + x[6] * r[7] + x[10] * r[8];

	d[3] = d[7] = d[11] = 0.;
	d[15] = 1.;
}

static void gX1multX2(gXMat& dst, const gXMat& X1, const gXMat& X2)
{
	gReal* d = dst.ptr();

	const gReal* x1 = X1.cptr();
	const gReal* x2 = X2.cptr();

	gReal r1, r2, r3;

	d[12] = x1[0] * x2[12] + x1[4] * x2[13] + x1[8] * x2[14] + x1[12];
	d[13] = x1[1] * x2[12] + x1[5] * x2[13] + x1[9] * x2[14] + x1[13];
	d[14] = x1[2] * x2[12] + x1[6] * x2[13] + x1[10] * x2[14] + x1[14];

	r1 = x1[0] * x2[0] + x1[4] * x2[1] + x1[8] * x2[2];
	r2 = x1[0] * x2[4] + x1[4] * x2[5] + x1[8] * x2[6];
	r3 = x1[0] * x2[8] + x1[4] * x2[9] + x1[8] * x2[10];
	d[0] = r1; d[4] = r2; d[8] = r3;

	r1 = x1[1] * x2[0] + x1[5] * x2[1] + x1[9] * x2[2];
	r2 = x1[1] * x2[4] + x1[5] * x2[5] + x1[9] * x2[6];
	r3 = x1[1] * x2[8] + x1[5] * x2[9] + x1[9] * x2[10];
	d[1] = r1; d[5] = r2; d[9] = r3;

	r1 = x1[2] * x2[0] + x1[6] * x2[1] + x1[10] * x2[2];
	r2 = x1[2] * x2[4] + x1[6] * x2[5] + x1[10] * x2[6];
	r3 = x1[2] * x2[8] + x1[6] * x2[9] + x1[10] * x2[10];
	d[2] = r1; d[6] = r2; d[10] = r3;

	d[3] = d[7] = d[11] = 0.;
	d[15] = 1.;
}

mgSkeleton::mgSkeleton(const mgSkeleton& skel)
{
	_retarget = skel._retarget;
	_retargetChannel = skel._retargetChannel;

	for (int i = 0; i < skel.bones.size(); i++)
	{
		mgBone* bone = new mgBone(i, this);
		bone->channel = skel.bones[i]->channel;
		//bone->children = skel.bones[i]->children;
		bone->direction_w = skel.bones[i]->direction_w;
		bone->eulerConv = skel.bones[i]->eulerConv;
		bone->H = skel.bones[i]->H;
		bone->length = skel.bones[i]->length;
		bone->limit = skel.bones[i]->limit;
		bone->localEndPoints = skel.bones[i]->localEndPoints;
		bone->name = skel.bones[i]->name;
		bone->nChannel = skel.bones[i]->nChannel;
		bone->order = skel.bones[i]->order;
		bone->parent = NULL;

		const mgBone* pBone = skel.bones[i]->parent;
		if (pBone != NULL)
		{
			bone->parent = bones[pBone->id];

			bone->parent->children.push_back(bone);
		}

		bones.push_back(bone);
	}

	boneRoot = bones[0];
	boneMap = skel.boneMap;
	dataPos = skel.dataPos;
	numT = skel.numT;
	numR = skel.numR;
	nTotalChannel = skel.nTotalChannel;
}

mgBone* mgSkeleton::createBone(std::string name)
{
	//mgBone* bone  = new mgBone(bones.size()); //, this);
	mgBone* bone  = new mgBone(bones.size(), this);
	bone->name = name;
	//bone->skeleton = this;

	bones.push_back(bone);
	boneMap[name] = bone->id;
	
	return bone;
}

int mgSkeleton::addChild(mgBone* parent, mgBone* child)
{
	parent->children.push_back(child);
	child->parent = parent;

	return 0;
}

int mgSkeleton::ik_with_jacobian(int endEffectorId, int baseId, gXMat tgtMat, CoordinateType* mFrame, bool onlyTrn)
{
	int functionStatus = 0;

	// get spatial Jacobian(se3) Matrix
	mgBone* bone = bones[endEffectorId];
	mgBone* tgtBone = bones[baseId];

	std::list<int> pathToEE;
	std::list<int>::iterator pathToEEIt;

	// get Total DOF
	int nTotalDof=0;
	while( bone && bone != tgtBone->parent )
	{
		pathToEE.push_front(bone->id);
		nTotalDof += bone->nChannel;
		bone = bone->parent;
	}

	arma::mat J(6, nTotalDof);
	arma::vec theta_dot(J.n_cols);

	//int status;
	// for SVD
	//arma::mat V(J.n_cols, J.n_cols);
	//arma::vec S(J.n_cols);
	//gsl_vector *work = gsl_vector_alloc(J.n_cols);

	// solve under-determine
	// weight
	arma::vec w(J.n_cols);
	int wIdx=0;
	double baseWeight_rot, baseWeight_trn, jointWeight;
	if( baseId == 0 )
	{
		baseWeight_rot = 1.5;
		baseWeight_trn = 1.0;
		jointWeight = 0.9;
	} else {
		baseWeight_rot = baseWeight_trn = jointWeight = 1.0;
	}
	
	while( wIdx < nTotalDof )
	{
		if( wIdx < 3 )
		{
			w(wIdx)=baseWeight_rot;
		} else if ( wIdx < 6 ) {
			w(wIdx)=baseWeight_trn;
		}else {
			w(wIdx)=jointWeight;
		}
		wIdx++;
	}
	double lamda = 0.1;


	// Desired EE
	gVec3 dPos = tgtMat.trn();
	gRotMat dRot = tgtMat.rot();
	gVec3 cPos;
	gRotMat cRot;
	gXMat cEEMat;
	gVec3 delta_v, delta_w;
	gTwist Vw_EE;
	gTwist delta_Screw;

	double epsilon = 1;
	double error = 100;
	double inv_delta_t = 0.1; // time slice
	//double magnitude_theta_dot;

	int pos;
	tgtBone = bones[baseId];

	delta_w.set(0,0,0);

	// Iterative J
	int iteration = 0;
	while( error > epsilon && iteration < 1000 )
	{
		iteration++;
		// compute error
		
		// get current End Effector worldMatrix.
		getWMatrixAt(endEffectorId, mFrame, cEEMat);
		cPos = cEEMat.trn();
		cRot = cEEMat.rot();
		
		delta_v = cRot.invMult(dPos - cPos);
		if( onlyTrn )
			delta_w.setZero();
		else 
			delta_w = gVec3::log(cRot.invMult(dRot));

		delta_Screw.set(delta_w, delta_v);
		// Error
		error = delta_Screw.magnitude();
		//error = (cPos - dPos).magnitude();

		//delta_Screw.normalize();
		if( delta_v.magnitudeSquared() > 1 ) {
			delta_Screw.normalize();
			//delta_Screw *= 0.1;
		}

		get_jacobian(J, endEffectorId, baseId, mFrame);

		// V = J*theta_dot
		// invJ * V = theta_dot
		arma::vec desired_screw(delta_Screw.ptr(), 6, false);//gsl_vector_view gsl_desired_screw = gsl_vector_view_array(delta_Screw.ptr(), 6);
		solve_Ax_y( theta_dot, J, desired_screw, w, NULL, lamda );
		
		//arma::mat U, V;
		//arma::vec s;

		//arma::svd(U, s, V, J);
		//
		//s(arma::find(s < 0)).zeros();
		//s = 1.0 / s;
		//arma::mat invS;
		//invS.zeros(V.n_rows, U.n_cols);
		//invS.submat(0,0, U.n_cols-1, U.n_cols-1) = arma::diagmat(s);
		//theta_dot = V * invS * U.t() * desired_screw;

		// put theta
		int jIdx=0;

		//double rx, ry, rz;
		//gRotMat rot_dot;
		//gVec3 rot_dot_vec;
		//mgBone* cBone;

		for( pathToEEIt = pathToEE.begin();
			pathToEEIt != pathToEE.end();
			pathToEEIt++ )
		{
			pos = dataPos[ *pathToEEIt ];
				
			mFrame[pos+0] += theta_dot( jIdx + 0 );
			mFrame[pos+1] += theta_dot( jIdx + 1 );
			mFrame[pos+2] += theta_dot( jIdx + 2 );
			jIdx += 3;

			//cBone = bones[ *pathToEEIt ];
			//if( *pathToEEIt == 0 )
			//{
			//	// Rotation
			//	rx = theta_dot(jIdx++);
			//	ry = theta_dot(jIdx++);
			//	rz = theta_dot(jIdx++);
			//	rot_dot_vec.set(rx,ry,rz);
			//	rot_dot = gRotMat::exp(rot_dot_vec);
			//	if( boneRoot->channel & mgBone::quaternionChannel )
			//	{
			//		gQuat baseRot = rot_dot.inQuat();
			//		// current Quat
			//		gQuat cBaseRot = gQuat(mFrame[pos+3], mFrame[pos+4], mFrame[pos+5], mFrame[pos+6]);
			//		cBaseRot.normalize();
			//		baseRot.normalize();
			//		baseRot = cBaseRot * baseRot;
			//		baseRot.normalize();
			//		mFrame[pos+3] = baseRot.x();
			//		mFrame[pos+4] = baseRot.y();
			//		mFrame[pos+5] = baseRot.z();
			//		mFrame[pos+6] = baseRot.w();
			//	} else if( boneRoot->channel & mgBone::expChannel ) {
			//		/*gVec3 trans;
			//		mgUtility::getTransAndRotMatFromData(mFrame, cBone, trans, cRot);
			//		cRot = cRot * rot_dot;
			//		mgUtility::setTransAndRotMatToData(mFrame, cBone, trans, cRot);*/
			//
			//		gVec3 trans, rot;
			//		mgUtility::getTransAndRotVecFromData(mFrame, cBone, trans, rot);
			//		rot += rot_dot_vec;
			//		mgUtility::setTransAndRotVecToData(mFrame, cBone, trans, rot);
			//
			//	} else {
			//		gVec3 euler = mgUtility::getZXYFromRotation(rot_dot);
			//
			//		mFrame[pos+3] += euler.z() * gRTD;
			//		mFrame[pos+4] += euler.x() * gRTD;
			//		mFrame[pos+5] += euler.y() * gRTD;
			//	}
			//	// Translation
			//	mFrame[pos+0] += theta_dot( jIdx++ );
			//	mFrame[pos+1] += theta_dot( jIdx++ );
			//	mFrame[pos+2] += theta_dot( jIdx++ );
			//} else {
			//	//rx = gsl_theta_dot( jIdx++ );
			//	//ry = gsl_theta_dot( jIdx++ );
			//	//rz = gsl_theta_dot( jIdx++ );
			//
			//	////mgUtility::setTransAndRotVecToData(mFrame, cBone, gVec3(), gVec3(rx, ry, rz)
			//
			//	//gVec3 rot = mgUtility::getZXYFromRotation( gRotMat::exp( gVec3(rx, ry, rz) ) );
			//
			//	////mFrame[pos+0] += rot.z() * gRTD;
			//	////mFrame[pos+1] += rot.x() * gRTD;
			//	////mFrame[pos+2] += rot.y() * gRTD;
			//	//				
			//	//mFrame[pos+0] += rot.x();
			//	//mFrame[pos+1] += rot.y();
			//	//mFrame[pos+2] += rot.z();
			//
			//	mFrame[pos+0] += theta_dot( jIdx + 0 );
			//	mFrame[pos+1] += theta_dot( jIdx + 1 );
			//	mFrame[pos+2] += theta_dot( jIdx + 2 );
			//	jIdx += 3;
			//}
		}
	}
	
	if( iteration >= 1000 )
	{
		gXMat hipMat;
		getWMatrixAt(baseId, mFrame, hipMat);
		//gVec3 hip = hipMat.trn();
		printf("IK fail : %f error %f distance.\n", error, (tgtMat.trn() - hipMat.trn()).magnitude());
	}


	return functionStatus;
}

int mgSkeleton::get_jacobian(arma::mat& J, int endEffectorId, int baseId, CoordinateType *mFrame)
{
	int functionStatus = 0;

	//gsl_matrix_view subMatrix;
	arma::mat::fixed<6,6> Jsub;
	gXMatList linkWMats;
	gTwist xi;
	gXMat eeWMat;

	gXMat gb;
	int jIdx=0;
	
	getWMatrixFromToAt(endEffectorId, baseId, mFrame, linkWMats);
	
	for( gXMatListIt matIt=linkWMats.begin();
		matIt != linkWMats.end();
		matIt++ )
	{
		//gb = linkWMats[endEffectorId].invMult( *matIt );
		gb = linkWMats.back().invMult( *matIt );

		if( baseId == 0 )
		{
			//subMatrix = gsl_matrix_submatrix(J, 0, jIdx, 6, 6);
			//make_matrix_Ad(&subMatrix.matrix, gb);			
			make_matrix_Ad(Jsub, gb);
			J.submat(0, jIdx, arma::size(6, 6)) = Jsub;
			jIdx += 6;
		} else {
			xi = gTwistRotX.xform(gb);
			mat_set_col(J, 0, jIdx++, 6, xi.cptr());
			xi = gTwistRotY.xform(gb);
			mat_set_col(J, 0, jIdx++, 6, xi.cptr());
			xi = gTwistRotZ.xform(gb);
			mat_set_col(J, 0, jIdx++, 6, xi.cptr());
		}
	}

#ifdef _DEBUG_PRINT
	debug_gsl_matrix(J);
#endif

	return functionStatus;
}

int mgSkeleton::convertFromGlobalToLocalPose(CoordinateType* gPose, CoordinateType* lPose)
{
	if( !gPose || !lPose ) 
	{
#ifdef _DEBUG
		printf("Error : NULL POSE\n");
#endif
		return -1;
	}

	gRotMat rot;
	int pos, pId;
	gVec3 rotData;

	gRotMat G, T;
	std::vector<gRotMat> boneMats(bones.size());

	for( unsigned int i=0; i<bones.size(); i++ )
	{
		if( _retarget[i] == -1 )
		{
#ifdef _DEBUG
			printf("ERROR : Bone(%s) is not mapped to BVH Bone\n", bones[i]->name.c_str());
#endif
			continue;
		}

		pos = _retarget[i];

		if( (_retargetChannel[ i ] ^ 0x3F) == 0 )
		{
			lPose[pos] = gPose[ pos++ ];
			lPose[pos] = gPose[ pos++ ];
			lPose[pos] = gPose[ pos++ ];

			rot.makeRotateZXY(gPose[ pos ] * gDTR, gPose[ pos + 1 ] * gDTR, gPose[ pos + 2 ] * gDTR);
			boneMats[i] = rot;

			lPose[pos] = gPose[ pos++ ];
			lPose[pos] = gPose[ pos++ ];
			lPose[pos] = gPose[ pos++ ];
		} else if( _retargetChannel[ i ] & 0x38 ) {
			rot.makeRotateZXY(gPose[ pos ] * gDTR, gPose[ pos + 1 ] * gDTR, gPose[ pos + 2 ] * gDTR);

			G.setIdentity();
			G = rot;

			pId = bones[i]->parent->id;
			T = boneMats[pId].invMult(G);
			boneMats[i] = G;

			rotData = mgUtility::getXYZFromRotation(T);

			lPose[pos++] = rotData.z() * gRTD;
			lPose[pos++] = rotData.x() * gRTD;
			lPose[pos++] = rotData.y() * gRTD;
		}
	}

	return 0;
}

int mgSkeleton::ik_with_elbowModel(int endEffectorId, int baseId, gXMat tgtMat, CoordinateType* mFrame)
{
	int functionStatus = 0;

	// get body Jacobian(se3) Matrix
	mgBone* bone = bones[endEffectorId];
	mgBone* tgtBone = bones[baseId];

	std::list<int> pathToEE;
	std::list<int>::iterator pathToEEIt;
	// get Total DOF
	int nTotalDof=7;
	while( bone && bone != tgtBone->parent )
	{
		pathToEE.push_front(bone->id);
		bone = bone->parent;
	}
	
	if( pathToEE.size() != 3 )
	{
		printf("Error : DOF of elbow shuold be 7 and elbow must have 3 limbs.\n");
		return -1;
	}

	arma::mat J(6, nTotalDof);
	arma::vec gsl_theta_dot(J.n_cols);

	//int status;

	// solve under-determine
	arma::vec w(J.n_cols, arma::fill::ones);
	//gsl_vector_set_all(w, 1.0);
	double lamda = 0.4;

	// Desired EE
	gVec3 dPos = tgtMat.trn();
	gRotMat dRot = tgtMat.rot();
	gVec3 cPos;
	gRotMat cRot;
	gXMat cEEMat;
	gVec3 delta_v, delta_w;
	gTwist Vw_EE;
	gTwist delta_Screw;

	double epsilon = 0.1;
	double error = 100;
	double inv_delta_t = 0.1; // time slice
	//double magnitude_theta_dot;

	int pos;
	tgtBone = bones[baseId];

	delta_w.set(0,0,0);

	// Iterative J
	int iteration = 0;
	while( error > epsilon && iteration < 1000 )
	{
		iteration++;
		// compute error
		
		// get current End Effector worldMatrix.
		getWMatrixAt(endEffectorId, mFrame, cEEMat);
		cPos = cEEMat.trn();
		cRot = cEEMat.rot();
		
		//vh->addPoint( osg::Vec3(cPos.x(), cPos.y(), cPos.z()), osg::Vec4(1,0,0,1), 5.);

		delta_v = cRot.invMult(dPos - cPos);
		delta_w = gVec3::log(cRot.invMult(dRot));

		delta_Screw.set(delta_w, delta_v);
		// Error
		error = delta_Screw.magnitude();
		//error = (cPos - dPos).magnitude();

		//delta_Screw.normalize();
		if( delta_v.magnitudeSquared() > 1 ) {
			delta_Screw.normalize();
			//delta_Screw *= 0.1;
		}

		get_elbow_jacobian(J, endEffectorId, baseId, mFrame);

		// V = J*theta_dot
		// invJ * V = theta_dot
		arma::vec gsl_desired_screw(delta_Screw.ptr(), 6, false); //gsl_vector_view gsl_desired_screw = gsl_vector_view_array(delta_Screw.ptr(), 6);

		solve_Ax_y( gsl_theta_dot, J, gsl_desired_screw, w, NULL, lamda );

		//magnitude_theta_dot = gDOT6(gsl_theta_dot->data, gsl_theta_dot->data);

		// put theta
		int jIdx=0;

		double theta_dot_i;

		for( pathToEEIt = pathToEE.begin();
			pathToEEIt != pathToEE.end();
			pathToEEIt++ )
		{
			pos = _retarget[ *pathToEEIt ];

			theta_dot_i = gsl_theta_dot( jIdx );
			if( theta_dot_i > 2 * gPI ) {
				printf("theta dot is over 2PI. error : %f\n", error);
				//theta_dot_i -= floor(theta_dot_i / (2*gPI));
			} else if ( theta_dot_i < -2 * gPI ) {
				printf("theta dot is below -2PI. error : %f\n", error);
				//theta_dot_i += floor(theta_dot_i / (2*gPI));
			}

			if( *pathToEEIt != baseId && *pathToEEIt != endEffectorId )
			{
				mFrame[pos] =0;
				mFrame[pos+1] += gsl_theta_dot( jIdx++ ) * gRTD;
				mFrame[pos+2] =0;
			} else {
				double rz = gsl_theta_dot( jIdx++ );
				double rx = gsl_theta_dot( jIdx++ );
				double ry =	gsl_theta_dot( jIdx++ );

				gVec3 euler = mgUtility::getZXYFromRotation(gRotMat::exp(gVec3(rx,ry,rz)));

				mFrame[pos] += euler.z() * gRTD;
				mFrame[pos+1] += euler.x() * gRTD;
				mFrame[pos+2] += euler.y() * gRTD;

				/*mFrame[dataPos] += gsl_vector_get( gsl_theta_dot, jIdx++ ) * gRTD;
				mFrame[pos+1] += gsl_vector_get( gsl_theta_dot, jIdx++ ) * gRTD;
				mFrame[pos+2] += gsl_vector_get( gsl_theta_dot, jIdx++ ) * gRTD;*/
			}
		}
	}
	//80.420158
	if( iteration >= 1000 )
	{
		gXMat hipMat;
		getWMatrixAt(baseId, mFrame, hipMat);
		//gVec3 hip = hipMat.trn();
		printf("IK fail : %f error %f distance.\n", error, (tgtMat.trn() - hipMat.trn()).magnitude());
	}
	//printf("IK result : %d iteration, %f error.\n", iteration, error);

	//gsl_vector_free(gsl_theta_dot);
	//gsl_matrix_free(J);
	//gsl_vector_free(w);

	return functionStatus;
}

int mgSkeleton::get_elbow_jacobian(arma::mat& J, int endEffectorId, int baseId, CoordinateType *mFrame)
{
	int functionStatus = 0;

	gXMatList linkWMats;
	gTwist xi;
	//gLink* link;
	//gLink* tgtLink = _mbs->link(baseId);
	mgBone* tgtBone = bones[baseId];
	mgBone* bone;

	gXMat gb;
	int jIdx=0;
	//int boneId;

	bone = bones[endEffectorId];
	getWMatrixFromToAt(endEffectorId, baseId, mFrame, linkWMats);
	
	for( gXMatListIt matIt=linkWMats.begin();
		matIt != linkWMats.end();
		matIt++ )
	{
		//gb = linkWMats[endEffectorId].invMult( matIt->second );
		gb = linkWMats.back().invMult( *matIt );

		//boneId = bone->id;
		//boneId = _mbs->getLinkIndex(link);


		if( bone->id != endEffectorId && bone->id != baseId )
		{			
			//X
			xi = gTwistRotX.xform(gb);
			mat_set_col(J, 0, jIdx++, 6, xi.cptr());
		} else {
			// Euler ZXY intrinsic
			// Z
			xi = gTwistRotZ.xform(gb);
			mat_set_col(J, 0, jIdx++, 6, xi.cptr());

			// X
			xi = gTwistRotX.xform(gb);
			mat_set_col(J, 0, jIdx++, 6, xi.cptr());

			// Y
			xi = gTwistRotY.xform(gb);
			mat_set_col(J, 0, jIdx++, 6, xi.cptr());
		}

		//link = link->parent();
		bone = bone->parent;
	}

#ifdef _DEBUG_PRINT
	debug_gsl_matrix(J);
#endif

	return functionStatus;
}

int mgSkeleton::getWMatrixAt(int boneId, const CoordinateType* data, gXMat& wMat, double scale) const
{
	mgBone* bone = bones[boneId];
	gXMat worldMat, lMat;
	worldMat.setIdentity();

	while( bone )
	{
		getLMatrixAt(bone->id, data, lMat);
		worldMat = lMat * worldMat;
		bone = bone->parent;
	}
	wMat = worldMat;

	return 0;
}

int mgSkeleton::getWHMatrixAt(int boneId, gXMat& WHM, double scale) const
{
	mgBone* bone = bones[boneId];
	gXMat worldMat, temp;
	gVec3 transVec;
	gRotMat tRot;
	gQuat tQuat;
	worldMat.setIdentity();

	while( bone )
	{
		temp = bone->H;
		worldMat = temp * worldMat;
		bone = bone->parent;
	}
	WHM = worldMat;

	return 0;
}

int mgSkeleton::getWMatrixAt(std::string boneName, CoordinateType* data, gXMat& wMat, double scale)
{
	mgBone* bone = bones[boneMap[boneName]];

	return getWMatrixAt(boneMap[boneName], data, wMat, scale);
}

int mgSkeleton::getLMatrixAt(int boneId, const CoordinateType* data, gXMat& lMat, double scale) const
{
	gRotMat tRot;
	gVec3 transVec;
	mgBone *bone = bones[boneId];
	mgUtility::getTransAndRotMatFromData(data, bone, transVec, tRot, scale);
	lMat = gXMat(transVec) * bone->H * gXMat(tRot);

	return 0;
}

int mgSkeleton::getAllLMatrixAt(CoordinateType* data, std::vector<gXMat> &boneMats, double scale)
{
	gXMat trans;
	for( int i=0; i<bones.size(); i++ )
	{
		getLMatrixAt(i, data, trans, scale);
		boneMats.push_back(trans);
	}
	return 0;
}

int mgSkeleton::getAllWMatrixAt(CoordinateType* data, std::vector<gXMat> &boneMats, double scale)
{
	std::vector<bool> fBoneMats;

	boneMats.clear();
	boneMats.resize(bones.size());
	fBoneMats.resize(bones.size(), false);

	gXMat linkWorld, linkMat, parentWorld;
	BONEVIt boneIt;
	int boneId, parentBId;
	mgBone* bone;
	//for( boneIt=bones.begin();boneIt!=bones.end(); boneIt++)
	for( boneId=0; boneId<bones.size(); boneId++ )
	{
		bone = bones[boneId];
		// Init
		//boneId = (*boneIt)->id;
		parentWorld.setIdentity();

		//if( (*boneIt)->parent ) {
		if( bone->parent ) {
			parentBId = bone->parent->id;
			if( fBoneMats[ parentBId ] ) {
				parentWorld = boneMats[parentBId];
			}
		}
		getLMatrixAt( boneId, data, linkMat, scale );
		linkWorld = parentWorld * linkMat;

		fBoneMats[boneId] = true;
		boneMats[boneId] = linkWorld;
	}

	return (int)RT_OK;
}

int mgSkeleton::getAllWMatrixAt(gXMatVec &boneMats, const CoordinateType* data, double scale) const
{
	// assume: a parent link must have lower id than child links.

	const int nLink = bones.size();

	gXMat linkMat;

	int linkId, parentId;
	for (linkId = 0; linkId < nLink; linkId++)
	{
		//const gLink* link = multiBody->link(linkId);
		mgBone* bone = bones[linkId];

		gVec3 transVec;
		gRotMat tRot;
		mgUtility::getTransAndRotMatFromData(data, bone, transVec, tRot);

		gXmultRp(linkMat, bone->H, tRot, transVec);

		if (bone->parent) {
			gX1multX2(boneMats[linkId], boneMats[bone->parent->id], linkMat);
		}
		else {
			boneMats[linkId] = linkMat;
		}
	}



	/*
	std::vector<bool> fBoneMats(bones.size(), false);

	if( boneMats.size() != bones.size() )
	{
		// warning: contain could be changed. It is the problem when gXMatVec used a pointer initializer.
		boneMats.resize(bones.size());
	}

	gXMat linkWorld, linkMat, parentWorld;
	int boneId, parentBId;
	mgBone* bone;

	for( boneId=0; boneId<bones.size(); boneId++ )
	{
		bone = bones[boneId];
		parentWorld.setIdentity();

		if( bone->parent ) {
			parentBId = bone->parent->id;
			if( fBoneMats[ parentBId ] ) {
				parentWorld = boneMats[parentBId];
			} else {
				printf("error: (getAllWMatrixAt), bone %s does not have a parent matrix. \
					   The order of the bones should be aligned in decendent manner.\n", bone->name.c_str());
			}
		}

		getLMatrixAt( linkMat, boneId, data, scale );
		linkWorld = parentWorld * linkMat;

		fBoneMats[boneId] = true;
		boneMats[boneId] = linkWorld;
	}
	*/

	return (int)RT_OK;
}

int mgSkeleton::getWMatrixFromToAt(int decendantBoneId, int ancestorBoneId, CoordinateType *data, gXMatList &boneMats, double scale)
{
	boneMats.clear();

	gXMat eye;
	eye.setIdentity();

	gXMat linkWorld, linkMat, parentWorld;
	mgBone* bone = bones[decendantBoneId];
	mgBone* tgtBone = bones[ancestorBoneId];

	getWMatrixAt(bone->id, data, linkWorld, scale);
	boneMats.push_back(linkWorld);
	//bone = bone->parent;

	while( bone && bone != tgtBone )
	{
		getLMatrixAt(bone->id, data, linkMat, scale);
		linkWorld = linkWorld * linkMat.invMult( eye );
		boneMats.push_front(linkWorld);

		bone = bone->parent;
	}


	return (int)RT_OK;
}

int mgSkeleton::getAllCoord(CoordinateType* data, std::vector<gXMat> &boneMats, double scale)
{
	boneMats.clear();
	boneMats.resize(bones.size());

	gRotMat tRot;
	gQuat tQuat;
	gVec3 transVec;
	mgBone *bone;
	int pos;

	for( int i=0; i<bones.size(); i++ )
	{
		bone = bones[i];
		pos = dataPos[i];

		mgUtility::getTransAndRotMatFromData(data, bone, transVec, tRot, scale);
		boneMats[i].setTrn(transVec);
		boneMats[i].setRot(tRot);
		//trans.setTrn(transVec);
		//rot.setRot(tRot);
	}

	return 0;
}

gVec3 mgSkeleton::getRootPositionFromPose(CoordinateType *pose, double scale)
{
	return gVec3(pose[0] * scale, pose[1] * scale, pose[2] * scale);
}

int mgSkeleton::getAllWPosition(std::string constName, mgData& motion, gVec3Array &data)
{
	data.clear();	
	data.reserve(motion.nMotion);
	gXMat worldMatrix;

	// apply motion
	for( unsigned int frame=0; frame<motion.nMotion; frame++)
	{
		worldMatrix.setIdentity();
		getWMatrixAt(constName, motion.motions[frame], worldMatrix);
		data.push_back( worldMatrix.trn() );
	}
	return 0;
}

int mgSkeleton::setWMatrixAt(const int boneId, CoordinateType* data, gXMat& wMat, double scale)
{
	mgBone* bone = bones[boneId];
	mgUtility::setTransAndRotMatToData(data, bone, wMat.trn(), wMat.rot(), scale);
	return 0;
}


// TODO: add quat.
//int mgSkeleton::applyTransformToPose(CoordinateType* data, gXMat trans)
//{
//	BONEVIt boneIt;
//	int pos=0;
//
//	gXMat rootMat;
//	gRotMat rot;
//	gVec3 rootRot;
//
//	for( boneIt=bones.begin();boneIt!=bones.end(); boneIt++)
//	{
//		pos = dataPos[(*boneIt)->id];
//
//		if( ((*boneIt)->channel ^ 0x3F) == 0 )
//		{
//			if( (*boneIt)->channel & 0x3C0 )
//			{
//				printf("Warning: not yet implemented Quaternion part in this function.(applyTransformToPose)\n");
//			}
//
//			rootMat.setTrn( gVec3(
//				data[pos+0],
//				data[pos+1],
//				data[pos+2]
//				) );
//
//			rot.makeRotateZXY(data[pos+3]*gDTR, data[pos+4]*gDTR, data[pos+5]*gDTR);
//			rootMat.setRot(rot);
//			rootMat = trans * rootMat;
//			rootRot = mgUtility::getZXYFromRotation(rootMat.rot());
//
//			data[pos++] = rootMat.trn().x();
//			data[pos++] = rootMat.trn().y();
//			data[pos++] = rootMat.trn().z();
//
//			data[pos++] = rootRot.z() * gRTD;
//			data[pos++] = rootRot.x() * gRTD;
//			data[pos++] = rootRot.y() * gRTD;
//
//			break;
//		}
//	}
//
//	return 0;
//}

/*
int mgSkeleton::getMotion(BVHelper::CoordinateType *data)
{
	int dataIdx;
	gLink* link;
	gVec3 trn, rotVec;
	gRotMat rot;
	gQuat rotQuat;

	for( int bondId=0; bondId<bones.size(); bondId++ )
	{
		dataIdx = _retarget[bondId];
		// ERROR

		link = _mbs->link(bondId);

		if( _skeleton->bones[bondId]->channel & 0x07 )
		{
			trn = link->localFrame().trn();
			data[dataIdx++] = trn.x();
			data[dataIdx++] = trn.y();
			data[dataIdx++] = trn.z();
		}
		if( _skeleton->bones[bondId]->channel & 0x38 )
		{
			gRotMat rot = link->localFrame().rot();
			rotVec = BVHelper::getZXYFromRotation(rot);
			data[dataIdx++] = rotVec.z() * gRTD;
			data[dataIdx++] = rotVec.x() * gRTD;
			data[dataIdx++] = rotVec.y() * gRTD;
		}
		if( _skeleton->bones[bondId]->channel & 0x3C0 )
		{
			gRotMat rot = link->localFrame().rot();
			rotQuat = rot.inQuat();
			data[dataIdx++] = rotQuat.x();
			data[dataIdx++] = rotQuat.y();
			data[dataIdx++] = rotQuat.z();
			data[dataIdx++] = rotQuat.w();
		}
	}

	return 0;
}
*/