
#ifndef _TI_RBFNIK_H_
#define _TI_RBFNIK_H_

#include <iostream>
#include <vector>
// Character
#include "Character/bCharacter.h"
#include "Character/bCharacterSolver.h"
#include "MBS/gArmaUtil.h"
// Loader
#include "Character/bCharacterLoader.h"
#include "loader/MotionLoader.h"
#include "loader/mgSkeletonToBCharacter.h"
#include "MocapProcessor/mgUtility.h"
// Visualizer
#include "Visualizer/gOSGViewHelper.h"
#include "Visualizer/gEventHandler.h"
#include "Visualizer/gBDOSGSystem.h"
// f2c
#include "f2c.h"
// Motion Transfer
#include "MocapProcessor/mgPoseTransfer.h"

//RBFN Weight 구하기
class tiRBFNW{
	public:
		//////////////////////////////////////////////////////////
		///20150428 : make Bounding box to get centers of Sample
		//////////////////////////////////////////////////////////
		MotionLoader *moLoader_h;
	
		///levmar setting.
		double LevMarOpts[5]; //option for levmar
		//* LevMarOpts: opts[0-4] = minim. options [\tau, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
		////                    * scale factor for initial \mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2 and the
		////                    * step used in difference approximation to the Jacobian. If \delta<0, the Jacobian is approximated
		////                    * with central differences which are more accurate (but slower!) compared to the forward differences
		////                    * employed by default. Set to NULL for defaults to be used.
		////                    */
		double LevMarInfo[LM_INFO_SZ]; //performance report from levmar is stored here.
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

		//////////////////////////////////////////////////////////////////////////////////////////////////
		///// Make Bounding box & get Hand position, Humerus , clavicle vector , Bounding box Centers , 
		//////////////////////////////////////////////////////////////////////////////////////////////////
		double nMotionFramecnt;
		arma::vec vCxW,vCyW,vCzW,vHxW,vHyW,vHzW;
		arma::mat mAllQuat;  //Motion 의 Hand 와의 Chain 인 애들의 Quaternion 값(모두 ball joint 라고 가정)
		arma::mat mAllHand;	 //Motion 의 Hand position 좌표 (x,y,z)
		arma::mat mAllHCvec; // Motion 의 Humerus 와 Clavicle vector 를 가져온다. 
		arma::mat mBBPos;	 // bounding box 에 저장 된 Center

		///hierachy 가 다른 src 모션을 target 캐릭터에 입힌다.
		void preMotionTransfer(bCharacter *src,bCharacter *avatar,mgPoseTransfer *poseTrans); //hierachy 가 다른 두개의 캐릭터에 pos dir 을 입힘. (tar:only 믹사모 , src: IPI)
		void MotionTransfer(bCharacter *src,bCharacter *avatar , bool initRetarget,mgPoseTransfer *poseTrans); //Transfer started
	
		///make Bounding Box
		void makeBoundingBoxofSampleSparse(bCharacter *motion,bCharacter *avatarclone,bCharacter *avatar,mgPoseTransfer *poseTrans);	//motion 을 avatar clone으로 pose transfer 실행시켜 원하는 tar:avatar 에 입힘
		void setQ(arma::mat *m){(*this).mAllQuat = *m;}	//tiRBFNIK 에 motion frame 속에서 캐릭터가 가지는 q를 다 가져온다.		
		void setHand(arma::mat *m){	mAllHand = *m;}	// tiRBFNIK에 motion frame 속에서 캐릭터의 오른손 position 에 대해 다 가져온다.
		void setHCvec(arma::mat *m){mAllHCvec = *m;}	//tiRBFNIK 에 Humerus , Clavicle vector (unit) 를 전부다 가져온다. 
		void setBBPos(arma::mat *m){	mBBPos = *m;}	//tiRBFNIK 에 만들어진 Bounding box 의 Bounding 시키는 point 를 가져온다.
	
	
		////////////////////////////////////////////////////////////////////////////////
		/// Make Weight of Humerus Clavicle vector(unit) that related with handposition
		////////////////////////////////////////////////////////////////////////////////
		arma::mat mWeight;   // Weight : output
		arma::mat mSigma;	 // mSigma : output

		///2. get Centers of Bounding box & compute RBFN Method to find w, sigma
		///make objective function f(w,sigma) = V (vecotor ex: Clavicle vector , Humerus Vector )
		///compute LEVMAR to find W, Sigma
		arma::vec computeCxWeightLevMar(void);
		arma::vec computeCyWeightLevMar(void);
		arma::vec computeCzWeightLevMar(void);
		arma::vec computeHxWeightLevMar(void);
		arma::vec computeHyWeightLevMar(void);
		arma::vec computeHzWeightLevMar(void);

		///get Weight, Sigma , BBPos by Excel
		void getBBPosToExcel(arma::mat mBBPos);
		void getWeightToExcel(arma::mat mWeight);
		void getSigmaToExcel(arma::mat mSigma);

		tiRBFNW(MotionLoader *moLoader_h)
		{
			//motion 에 대한 값을 받아오기 위해
			///Using bounding box & weight of humerus,clavicle 
			(*this).moLoader_h =moLoader_h;
		
			///////////
			///LEVMAR//
			///////////
			double stopThresh = 1e-5;	//levmar 에 대한 Option 설정
			LevMarOpts[0]=1;
			LevMarOpts[1]=stopThresh;
			LevMarOpts[2]=stopThresh;
			LevMarOpts[3]=stopThresh;
			LevMarOpts[4]=LM_DIFF_DELTA;

		}
		virtual ~tiRBFNW(){}
};

class tiRBFNIK
{

	public:
	/////////////////////////////////////////////////////////////////
	/// IK using Humerus vector & Clavicle vector (only for Right arm) 
	///////////////////////////////////////////////////////////////
	///levmar setting.
	double LevMarOpts[5]; //option for levmar
	//* LevMarOpts: opts[0-4] = minim. options [\tau, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
	////                    * scale factor for initial \mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2 and the
	////                    * step used in difference approximation to the Jacobian. If \delta<0, the Jacobian is approximated
	////                    * with central differences which are more accurate (but slower!) compared to the forward differences
	////                    * employed by default. Set to NULL for defaults to be used.
	////                    */
	double LevMarInfo[LM_INFO_SZ]; //performance report from levmar is stored here.
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

	double computeCxUsingWeight(arma::vec EEPos);
	double computeCyUsingWeight(arma::vec EEPos);
	double computeCzUsingWeight(arma::vec EEPos);
	
	double computeHxUsingWeight(arma::vec EEPos);
	double computeHyUsingWeight(arma::vec EEPos);
	double computeHzUsingWeight(arma::vec EEPos);
	
	gVec3 v_DesClavicle;	//Desired Clavicle vector
	gVec3 v_DesHumerus;		//Desired Humerus vector 
	gVec3 v_DesHand;		//Desired Hand vector
	gVec3 v_DesDir;			//Deisred Hand Direction

	arma::vec preferredTarCoords; //preferred coordinates (compact coordinates) of tar..note first 6 components (for baselink) are ignored. initially set to zero.
	double preferredTarCoordsWeight; //weight for the preferred coordinates
	
	bCharacter* tar;
	
	///3. compute IK using (Hx,Hy,Hz), (Cx,Cy,Cz) , Weight Matrix (W) , Desired position Xd
	int computeIKusingHC(gVec3 EEPos);
	int computeIKusingHC_bound(gVec3 EEPos);
	int computeIKusingHC_bound_frame(gXMat EEFrame,int LinkChainIdx); // arm IK LinkChainIdx:(Right[0] or Left[1]) for Desired Frame
	
	int trackHead(gVec3 EEPos);
	arma::mat mWeight;
	arma::mat mSigma;
	arma::mat mBBPos;

	///"get Weight, Sigma , BBPos by txt" function
	static arma::mat getBBPosByTxt(const char* fileName);
	static arma::mat getWeightByTxt(const char* fileName);
	static arma::mat getSigmaByTxt(const char* fileName);

	tiRBFNIK(bCharacter* tarCharacter,arma::mat mW,arma::mat mS,arma::mat mB)
	{
		//motion 에 대한 값을 받아오기 위해
		///Using bounding box & weight of humerus,clavicle 
		mWeight = mW;
		mSigma = mS;
		mBBPos = mB;
		////////
		///IK //
		////////
		(*this).tar = tarCharacter; //target character
		preferredTarCoords.zeros(((*this).tar->m_rArmIdx.size()-1)*3); //parameter : right arm 의 모든 각도
		preferredTarCoordsWeight = 0.5;	//weight for default pose (p = 0)
		
		///////////
		///LEVMAR//
		///////////
		double stopThresh = 1e-5;	//levmar 에 대한 Option 설정
		LevMarOpts[0]=1;
		LevMarOpts[1]=stopThresh;
		LevMarOpts[2]=stopThresh;
		LevMarOpts[3]=stopThresh;
		LevMarOpts[4]=LM_DIFF_DELTA;

	}
	virtual ~tiRBFNIK(){}

};
#endif
