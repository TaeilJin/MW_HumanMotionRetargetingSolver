#include "EX4_retargetmotion.h"


void EX4_retargetmotion::SetupCalcMetric(char* inputFilename, char* gtFilename )
{
	// get input motion file
	MotionLoader loader_input;
	loader_input.loadMotionFile(inputFilename);
	mgData* motion_input = loader_input.getMotion();
	mgSkeleton* skeleton_input = loader_input.getSkeleton();
	srcFrameTime = motion_input->frameTime;

	// get GT motion file
	MotionLoader loader_GT;
	loader_GT.loadMotionFile(gtFilename);
	mgData* motion_GT = loader_GT.getMotion();
	mgSkeleton* skeleton_GT = loader_GT.getSkeleton();
	srcFrameTime = motion_GT->frameTime;

	//
	std::cout << "|--- load GT motion file ---|" << std::endl;
	// load src motion (pose sequence) : nMotion is a total number of frame for motion data
	arma::mat refCoord(g_src->sizeCompactCoordArray(), motion_GT->nMotion, arma::fill::zeros);
	for (int f = 0; f < motion_GT->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			g_src,
			skeleton_GT,
			motion_GT->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}
	arma::mat g_GTCoord = refCoord;

	std::cout << "|--- load input motion file ---|" << std::endl;
	// load src motion (pose sequence) : nMotion is a total number of frame for motion data
	arma::mat refCoord_input(g_src->sizeCompactCoordArray(), motion_input->nMotion, arma::fill::zeros);
	for (int f = 0; f < motion_input->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			g_src,
			skeleton_input,
			motion_input->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord_input.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}
	arma::mat g_inputCoord = refCoord_input;

	std::cout << "|--- load source GT motion  ---|" << std::endl;
	arma::mat inputPositions(3 * 22, g_inputCoord.n_cols, arma::fill::zeros);
	arma::mat GTPositions(3 * 22, g_GTCoord.n_cols, arma::fill::zeros);
	for (int iter = 0; iter < g_inputCoord.n_cols; iter++) {
		
		//input pose
		arma::vec pose = g_inputCoord.col(iter);
		//input update
		g_src->setFromCompactCoordArray(pose);
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();
		//input position
		arma::vec Quat_pose; Quat_pose.resize(22 * 3);
		int j_cnt = 0;
		for (int j = 0; j < g_src->numLinks(); j++) {
			if (j == 0 || j == 1 || j == 2 || j == 3 || j == 4 || j == 5 ||
				j == 6 || j == 7 || j == 8 || j == 9 ||
				j == 25 || j == 26 || j == 27 || j == 28 ||
				j == 44 || j == 45 || j == 46 || j == 47 ||
				j == 48 || j == 49 || j == 50 || j == 51) {
				Quat_pose[3 * j_cnt + 0] = g_src->link(j)->frame().trn().x();
				Quat_pose[3 * j_cnt + 1] = g_src->link(j)->frame().trn().y();
				Quat_pose[3 * j_cnt + 2] = g_src->link(j)->frame().trn().z();
				j_cnt++;
			}
		}
		inputPositions(0, iter, arma::SizeMat(Quat_pose.n_elem, 1)) = Quat_pose;


		//input pose
		arma::vec pose_gt = g_GTCoord.col(iter);
		//input update
		g_src->setFromCompactCoordArray(pose_gt);
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();
		//input position
		j_cnt = 0;
		for (int j = 0; j < g_src->numLinks(); j++) {
			if (j == 0 || j == 1 || j == 2 || j == 3 || j == 4 || j == 5 ||
				j == 6 || j == 7 || j == 8 || j == 9 ||
				j == 25 || j == 26 || j == 27 || j == 28 ||
				j == 44 || j == 45 || j == 46 || j == 47 ||
				j == 48 || j == 49 || j == 50 || j == 51){
				Quat_pose[3 * j_cnt + 0] = g_src->link(j)->frame().trn().x();
				Quat_pose[3 * j_cnt + 1] = g_src->link(j)->frame().trn().y();
				Quat_pose[3 * j_cnt + 2] = g_src->link(j)->frame().trn().z();
				j_cnt++;
			}

		}
		//std::cout << " joint number " << j_cnt << std::endl;
		GTPositions(0, iter, arma::SizeMat(Quat_pose.n_elem, 1)) = Quat_pose;

	}

	// Metric
	std::cout << "|--- GT Error  ---|" << std::endl;
	arma::mat GTerror(3 * 22, g_GTCoord.n_cols, arma::fill::zeros);
	for (int elem = 0; elem < GTPositions.n_cols; elem++) {
		GTerror(0, elem, arma::SizeMat(3 * 22, 1)) = arma::abs(inputPositions(0, elem, arma::SizeMat(3 * 22, 1)) - GTPositions(0, elem, arma::SizeMat(3 * 22, 1)));
		
		//std::cout << GTerror << std::endl;
	}
	
	arma::vec mean_pose = arma::mean(GTerror);
	double total_mean_pose = arma::mean(mean_pose);
	std::cout << " mean pose error " << total_mean_pose << " / character height " << total_mean_pose / 147.444<< std::endl;
}

void EX4_retargetmotion::SetupScene(char* srcfilename, char* srcCharactertxt, char* tarCharactertxt)
{
	std::cout << "|--- load src motion file ---|" << std::endl;
	// get src motion file
	MotionLoader loader;
	loader.loadMotionFile(srcfilename);
	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();
	srcFrameTime = motion->frameTime;


	std::cout << "|--- load src character file ---|" << std::endl;
	// load src bCharacter from txt file
	g_src = new bCharacter();
	g_srcSim = new bCharacterSim(g_src);
	g_srcVis = new gBDOSGSystem();
	if (loadAvatarModelFromFile(g_src, g_srcSim, g_srcVis, srcCharactertxt, 1.0) != TRUE) {
		std::cout << "|---- write src character file ---|" << std::endl;
		const double mass = 70.;
		mgSkeletonToBCharacter::saveToBCharacter(skeleton, srcCharactertxt, mass);
		std::cout << "|---- load new src character file : warning should check a free joint ---|" << std::endl;
		loadAvatarModelFromFile(g_src, g_srcSim, g_srcVis, srcCharactertxt, 1.0);
	}

	std::cout << "|--- load src motion file ---|" << std::endl;
	// load src motion (pose sequence) : nMotion is a total number of frame for motion data
	arma::mat refCoord(g_src->sizeCompactCoordArray(), motion->nMotion, arma::fill::zeros);
	for (int f = 0; f < motion->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			g_src,
			skeleton,
			motion->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}
	g_refCoord = refCoord;
	std::cout << "|--- load tar character file ---|" << std::endl;
	// generate tar bCharacter from txt file
	g_tar = new bCharacter();
	g_tarSim = new bCharacterSim(g_tar);
	g_tarVis = new gBDOSGSystem();
	double check_load = loadAvatarModelFromFile(g_tar, g_tarSim, g_tarVis, tarCharactertxt, 1.0);

	std::cout << "|--- initialization process is finished ---|\n" << std::endl;

	std::cout << "|--- UPDATE RETARGET SCENE       ---|" << std::endl;
	std::cout << "|--- press  Y to forward motion  ---|" << std::endl;
	std::cout << "|--- press  H to backward motion ---|" << std::endl;
	std::cout << "|--- press  U to forward 1 frame ---|" << std::endl;
	std::cout << "|--- press  J to backward 1 frame---|" << std::endl;
	std::cout << "|--- press  P to see next bvh    ---|" << std::endl;
	g_osg_cha->addChild(g_srcVis->getOSGGroup());
	g_osg_cha->addChild(g_tarVis->getOSGGroup());

}

void EX4_retargetmotion::SetupScene(char* srcfilename, char* srcCharactertxt, char*tarCharactertxt, float scale)
{
	std::cout << "|--- load src motion file ---|" << std::endl;
	// get src motion file
	MotionLoader loader;
	loader.setTranslateScale(scale);
	loader.loadMotionFile(srcfilename);
	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();
	srcFrameTime = motion->frameTime;
	

	std::cout << "|--- load src character file ---|" << std::endl;
	// load src bCharacter from txt file
	g_src = new bCharacter();
	g_srcSim = new bCharacterSim(g_src);
	g_srcVis = new gBDOSGSystem();
	if (loadAvatarModelFromFile(g_src, g_srcSim, g_srcVis, srcCharactertxt, scale) != TRUE) {
		std::cout << "|---- write src character file ---|" << std::endl;
		const double mass = 70.;
		mgSkeletonToBCharacter::saveToBCharacter(skeleton, srcCharactertxt, mass);
		std::cout << "|---- load new src character file : warning should check a free joint ---|" << std::endl;
		loadAvatarModelFromFile(g_src, g_srcSim, g_srcVis, srcCharactertxt, scale);
	}

	std::cout << "|--- load src motion file ---|" << std::endl;
	// load src motion (pose sequence) : nMotion is a total number of frame for motion data
	arma::mat refCoord(g_src->sizeCompactCoordArray(), motion->nMotion, arma::fill::zeros);
	for (int f = 0; f < motion->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			g_src,
			skeleton,
			motion->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}
	g_refCoord = refCoord;
	std::cout << "|--- load tar character file ---|" << std::endl;
	// generate tar bCharacter from txt file
	g_tar = new bCharacter();
	g_tarSim = new bCharacterSim(g_tar);
	g_tarVis = new gBDOSGSystem();
	double check_load = loadAvatarModelFromFile(g_tar, g_tarSim, g_tarVis, tarCharactertxt, scale);

	std::cout << "|--- initialization process is finished ---|\n" << std::endl;

	std::cout << "|--- UPDATE RETARGET SCENE       ---|" << std::endl;
	std::cout << "|--- press  Y to forward motion  ---|" << std::endl;
	std::cout << "|--- press  H to backward motion ---|" << std::endl;
	std::cout << "|--- press  U to forward 1 frame ---|" << std::endl;
	std::cout << "|--- press  J to backward 1 frame---|" << std::endl;
	std::cout << "|--- press  P to see next bvh    ---|" << std::endl;
	g_osg_cha->addChild(g_srcVis->getOSGGroup());
	g_osg_cha->addChild(g_tarVis->getOSGGroup());

}

void EX4_retargetmotion::initRetarget(int RP)
{
	// initialization of poseTransfer
	switch (RP) {
		case Mocap2MixamoRest:
			//initTransfer_Mocap(g_src, g_tar);
			//initTransfer_Mocap_DK(g_src, g_tar);
			initTransfer_Mocap(g_src, g_tar, "Source_BY+Target_SMPL.txt");
			break;
		case Axis2MixamoRest:
			initTransfer(g_src, g_tar);
			break;
		case Kinect2MixamoRest:
			initTransfer_Kinect(g_src, g_tar);
			break;
		case Mixamo92MixamoRest:
			initTransfer_Mixamo9(g_src, g_tar);
			//initTransfer_MixamoJackie(g_src, g_tar);
			break;
		case CMU2MixamoRest:
			initTransfer_CMU(g_src, g_tar);
			break;
		case Amass2MixamoRest:
			initTransfer_Amass(g_src, g_tar);
			break;
		case HDM2MixamoRest:
			initTransfer_HDM05(g_src, g_tar);
			break;
	}
}

void EX4_retargetmotion::initRetarget(int RP, char* textfile)
{
	initTransfer_Mocap(g_src, g_tar, textfile);// "Source_BY+Target_SMPL.txt");
}



void EX4_retargetmotion::UpdateScene(int even_simulationTime, int even_iter, bool b_motionretarget, double y_offset)
{
	//
	g_viewer->frame(even_simulationTime);

	arma::vec pose = g_refCoord.col(even_iter);
		
	g_src->setFromCompactCoordArray(pose);
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();

	//retarget
	retargetPose(b_motionretarget,y_offset);

	g_srcVis->update();
	g_tarVis->update();
	
}

void EX4_retargetmotion::retargetPose(bool bool_motionretarget,double y_offset)
{
	// desired positions update
	for (int p = 0; p < poseTrans->srcPoints.size(); p++) {

		poseTrans->tarPoints[p].updateKinematicsUptoPos();
		poseTrans->srcPoints[p].updateKinematicsUptoPos();
		/*gOSGShape::setColor(osg::Vec4(1.0, 0, 0, 1.0));
		g_osg_debug->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(poseTrans->tarPoints[p].posWorld()), 5.0));
		gOSGShape::setColor(osg::Vec4(0.0, 0, 1.0, 1.0));
		g_osg_debug->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(poseTrans->srcPoints[p].posWorld()), 5.0));*/

	}

	//motion retargeting
	if (bool_motionretarget) {
		//tar pose retargeting
		poseTrans->desiredPoints[0].pos_desired = poseTrans->scale * g_src->link(0)->frame().trn(); //poseTrans->scale
		gXMat offset; offset.setTrn(0, 0, 0);
		poseTrans->transferPoseLevMar(offset);
		g_tar->updateKinematicsUptoPos();
		g_tar->updateKinematicBodiesOfCharacterSim();
		
		/*	//post processing (foot height)
			double min_f_height = std::min(g_tar->findLink("RightToeBase")->frame().trn().y(), g_tar->findLink("LeftToeBase")->frame().trn().y());
			min_f_height = std::min(min_f_height, std::min(g_tar->findLink("RightHand")->frame().trn().y(), g_tar->findLink("LeftHand")->frame().trn().y()));

			if(even_iter < g_refCoord.n_cols)
			if (min_f_height < 1e-3) {
				poseTrans->save_f_height += min_f_height;
			}
		*/
		//double max_f_height = std::max(g_tar->findLink("RightFoot")->frame().trn().y(), g_tar->findLink("LeftFoot")->frame().trn().y());
		//if (max_f_height > 15) {
		//	gVec3 new_pelvis = g_src->link(0)->frame().trn();
		//	new_pelvis.setY(new_pelvis.y() + 10.0); //std::abs(poseTrans->save_f_height)
		//	g_tar->setBasePosition(new_pelvis);
		//	g_tar->updateKinematicsUptoPos();
		//	g_tar->updateKinematicBodiesOfCharacterSim();
		//}
		gVec3 new_pelvis = g_src->link(0)->frame().trn();
		new_pelvis.setY(new_pelvis.y() + y_offset); //std::abs(poseTrans->save_f_height)
		g_tar->setBasePosition(new_pelvis);
		g_tar->updateKinematicsUptoPos();
		g_tar->updateKinematicBodiesOfCharacterSim();
	}
}

void EX4_retargetmotion::saveRetargetMotion(bool bool_save, char* tarBVHFILE, float frametime, double y_offset, float scale)
{
	if (bool_save) {
		// retarget src motion ( do what we need )
		arma::mat tarQuaternions(g_tar->sizeSafeCoordArray(), g_refCoord.n_cols, arma::fill::zeros);
		for (int iter = 0; iter < tarQuaternions.n_cols; iter++) {

			//printf("\r%d/%d [", count, MAX); // 진행 상태 출력  
			//percent = (float)count / MAX * 100; // 퍼센트 계산  
			//bar_count = percent / tick; // 프로그레스바 갯수 계산  
			//for (i = 0; i < LEN; i++) { // LEN길이의 프로그레스바 출력  
			//	if (bar_count > i) { // 프로그레스바 길이보다 i가 작으면 
			//		printf("%c", bar);
			//	}
			//	else { // i가 더 커지면  
			//		printf("%c", blank);
			//	}
			//}
			//printf("] %0.2f%%", percent); // 퍼센트 출력  
			//count++; // 카운트 1증가  
			//Sleep(SPEED); // SPEEDms 대기

			if (tarQuaternions.n_rows != (g_tar->numLinks() * 4 + 3))
				std::cout << " dof is not matched! see the matrix and character" << std::endl;
			//src pose 
			g_src->setFromCompactCoordArray(g_refCoord.col(iter));
			g_src->updateKinematicsUptoPos();
			g_src->updateKinematicBodiesOfCharacterSim();

			////post src processing
			//gXMat baseMat = g_src->link(0)->frame();
			//gXMat baseRot; baseRot.setIdentity();
			//gRotMat rotX; rotX.makeRotateX(90 * gDTR);
			//baseRot.setRot(rotX);
			//baseMat = baseRot.invMult(baseMat);

			//g_src->setFromCompactCoordArray(g_refCoord.col(iter));
			//g_src->setBasePose(baseMat);
			//g_src->updateKinematicsUptoPos();
			//g_src->updateKinematicBodiesOfCharacterSim();

			//tar pose retargeting
			retargetPose(true, y_offset);

			//tar pose save
			arma::vec Quat_pose; Quat_pose.resize(g_tar->numLinks() * 4 + 3);

			g_tar->getSafeCoordArray(Quat_pose);
			tarQuaternions(0, iter, arma::SizeMat(Quat_pose.n_elem, 1)) = Quat_pose;
		}
		std::cout << "|--- retarget is finished & save bvh is starting ---|" << std::endl;
		g_saveBVH->saveBVHFile(g_tar, tarBVHFILE, tarQuaternions, mgBone::_AXISORDER::ZYX, frametime,scale);
		std::cout << "|--- save is finished ---|" << std::endl;
	}
}

void EX4_retargetmotion::savePreProcessingMotion(bool bool_save, char* tarBVHFILE)
{
	if (bool_save) {
		// retarget src motion ( do what we need )
		arma::mat tarQuaternions(g_tar->sizeSafeCoordArray(), g_refCoord.n_cols, arma::fill::zeros);
		for (int iter = 0; iter < tarQuaternions.n_cols; iter++) {

			//printf("\r%d/%d [", count, MAX); // 진행 상태 출력  
			//percent = (float)count / MAX * 100; // 퍼센트 계산  
			//bar_count = percent / tick; // 프로그레스바 갯수 계산  
			//for (i = 0; i < LEN; i++) { // LEN길이의 프로그레스바 출력  
			//	if (bar_count > i) { // 프로그레스바 길이보다 i가 작으면 
			//		printf("%c", bar);
			//	}
			//	else { // i가 더 커지면  
			//		printf("%c", blank);
			//	}
			//}
			//printf("] %0.2f%%", percent); // 퍼센트 출력  
			//count++; // 카운트 1증가  
			//Sleep(SPEED); // SPEEDms 대기

			if (tarQuaternions.n_rows != (g_tar->numLinks() * 4 + 3))
				std::cout << " dof is not matched! see the matrix and character" << std::endl;
			//src pose 
			g_src->setFromCompactCoordArray(g_refCoord.col(iter));
			g_src->updateKinematicsUptoPos();
			g_src->updateKinematicBodiesOfCharacterSim();

			gXMat baseMat = g_src->link(0)->frame();
			/*gXMat baseRot; baseRot.setIdentity();
			gRotMat rotX; rotX.makeRotateX(90 * gDTR);
			baseRot.setRot(rotX);
			baseMat = baseRot.invMult(baseMat);*/

			g_tar->setFromCompactCoordArray(g_refCoord.col(iter));
			g_tar->setBasePose(baseMat);
			g_tar->updateKinematicsUptoPos();
			g_tar->updateKinematicBodiesOfCharacterSim();

			//tar pose save
			arma::vec Quat_pose; Quat_pose.resize(g_tar->numLinks() * 4 + 3);

			g_tar->getSafeCoordArray(Quat_pose);
			tarQuaternions(0, iter, arma::SizeMat(Quat_pose.n_elem, 1)) = Quat_pose;
		}
		std::cout << "|--- retarget is finished & save bvh is starting ---|" << std::endl;
		g_saveBVH->saveBVHFile(g_tar, tarBVHFILE, tarQuaternions, mgBone::_AXISORDER::ZYX, 1/120.0);
		std::cout << "|--- save is finished ---|" << std::endl;
	}
	
}
