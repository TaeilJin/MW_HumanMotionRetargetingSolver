#include "EX4_retargetmotion_aj.h"


void EX4_retargetmotion_aj::SetupScene(char* srcfilename, char* srcCharactertxt, char*tarCharactertxt)
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

void EX4_retargetmotion_aj::initRetarget(int RP)
{
	// initialization of poseTransfer
	switch (RP) {
		case Mocap2MixamoRest:
			initTransfer_Mocap(g_src, g_tar);
			break;
		case Axis2MixamoRest:
			initTransfer(g_src, g_tar);
			break;
		case Kinect2MixamoRest:
			initTransfer_Kinect(g_src, g_tar);
			break;
		case Mixamo92MixamoRest:
			initTransfer_Mixamo9(g_src, g_tar);
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



void EX4_retargetmotion_aj::UpdateScene(int even_simulationTime, int even_iter, bool b_motionretarget)
{
	//
	g_viewer->frame(even_simulationTime);

	arma::vec pose = g_refCoord.col(even_iter);
		
	g_src->setFromCompactCoordArray(pose);
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();

	/*gXMat baseMat = g_src->link(0)->frame();
	gXMat baseRot; baseRot.setIdentity();
	gRotMat rotX; rotX.makeRotateX(90 * gDTR);
	baseRot.setRot(rotX);
	baseMat = baseRot.invMult(baseMat);

	g_src->setFromCompactCoordArray(pose);
	g_src->setBasePose(baseMat);
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();*/

	//retarget
	retargetPose(b_motionretarget);

	g_srcVis->update();
	g_tarVis->update();
	
}

void EX4_retargetmotion_aj::retargetPose(bool bool_motionretarget)
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
		poseTrans->desiredPoints[0].pos_desired = g_src->link(0)->frame().trn(); //poseTrans->scale
		gXMat offset; offset.setTrn(0, 0, 0);
		poseTrans->transferPoseLevMar(offset);
		g_tar->updateKinematicsUptoPos();
		g_tar->updateKinematicBodiesOfCharacterSim();
		
		//// post processing (foot height)
		//double min_f_height = std::min(g_tar->findLink("RightFoot")->frame().trn().y(), g_tar->findLink("LeftFoot")->frame().trn().y());
		//min_f_height = std::min(min_f_height, std::min(g_tar->findLink("RightHand")->frame().trn().y(), g_tar->findLink("LeftHand")->frame().trn().y()));
		//
		//if (min_f_height < 1e-3) {
		//	gVec3 new_pelvis = g_tar->link(0)->frame().trn();
		//	new_pelvis.setY(new_pelvis.y() + std::abs(min_f_height));
		//	g_tar->setBasePosition(new_pelvis);
		//	g_tar->updateKinematicsUptoPos();
		//	g_tar->updateKinematicBodiesOfCharacterSim();
		//	poseTrans->save_f_height = min_f_height;
		//}
		//double max_f_height = std::max(g_tar->findLink("RightFoot")->frame().trn().y(), g_tar->findLink("LeftFoot")->frame().trn().y());
		//if (max_f_height > 15) {
		//	gVec3 new_pelvis = g_src->link(0)->frame().trn();
		//	new_pelvis.setY(new_pelvis.y() + 10.0); //std::abs(poseTrans->save_f_height)
		//	g_tar->setBasePosition(new_pelvis);
		//	g_tar->updateKinematicsUptoPos();
		//	g_tar->updateKinematicBodiesOfCharacterSim();
		//}
	}
}

void EX4_retargetmotion_aj::saveRetargetMotion(bool bool_save, char* tarBVHFILE, float frametime)
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
			retargetPose(true);

			//tar pose save
			arma::vec Quat_pose; Quat_pose.resize(g_tar->numLinks() * 4 + 3);

			g_tar->getSafeCoordArray(Quat_pose);
			tarQuaternions(0, iter, arma::SizeMat(Quat_pose.n_elem, 1)) = Quat_pose;
		}
		std::cout << "|--- retarget is finished & save bvh is starting ---|" << std::endl;
		g_saveBVH->saveBVHFile(g_tar, tarBVHFILE, tarQuaternions, mgBone::_AXISORDER::ZYX, frametime);
		std::cout << "|--- save is finished ---|" << std::endl;
	}
}

void EX4_retargetmotion_aj::savePreProcessingMotion(bool bool_save, char* tarBVHFILE)
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
