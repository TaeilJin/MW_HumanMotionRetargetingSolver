#include "EX5_makeBVHFile.h"


void EX5_makeBVHFile::SetupScene(char* srcfilename, char* srcCharactertxt)
{
	std::cout << "|--- load src motion file ---|" << std::endl;
	// get src motion file
	MotionLoader loader;
	loader.loadMotionFile(srcfilename);
	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();

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

	std::cout << "|--- initialization process is finished ---|\n" << std::endl;

	std::cout << "|--- UPDATE RETARGET SCENE       ---|" << std::endl;
	std::cout << "|--- press  Y to forward motion  ---|" << std::endl;
	std::cout << "|--- press  H to backward motion ---|" << std::endl;
	std::cout << "|--- press  U to forward 1 frame ---|" << std::endl;
	std::cout << "|--- press  J to backward 1 frame---|" << std::endl;
	std::cout << "|--- press  P to see next bvh    ---|" << std::endl;
	g_osg_cha->addChild(g_srcVis->getOSGGroup());
	

}

void EX5_makeBVHFile::initTransfer(bCharacter* src)
{
	g_poseTrans = new mgPoseTransfer(src, src);
	g_poseTrans->addDesiredJoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t9", *src->findLink("Spine"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t10", *src->findLink("Spine1"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t11", *src->findLink("Neck1"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t12", *src->findLink("Head"), gVec3(0, 0, 0));

	g_poseTrans->addDesiredJoint("t13", *src->findLink("LeftArm"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t14", *src->findLink("LeftForeArm"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t15", *src->findLink("LeftHand"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t16", *src->findLink("LeftHandIndex1"), gVec3(0, 0, 0));

	g_poseTrans->addDesiredJoint("t17", *src->findLink("RightArm"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t18", *src->findLink("RightForeArm"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t19", *src->findLink("RightHand"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t20", *src->findLink("RightHandIndex1"), gVec3(0, 0, 0));

	g_poseTrans->addDesiredJoint("t5", *src->findLink("RightUpLeg"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t6", *src->findLink("RightLeg"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t7", *src->findLink("RightFoot"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t8", *src->findLink("RightToeBase"), gVec3(0, 0, 0));

	g_poseTrans->addDesiredJoint("t1", *src->findLink("LeftUpLeg"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t2", *src->findLink("LeftLeg"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t3", *src->findLink("LeftFoot"), gVec3(0, 0, 0));
	g_poseTrans->addDesiredJoint("t4", *src->findLink("LeftToeBase"), gVec3(0, 0, 0));
}

void EX5_makeBVHFile::SetupMakeBVH(bCharacter* g_src, char* target_joint_positions)
{
	//load target joint positions
	arma::mat temp; temp.load(target_joint_positions);
	g_target_positions = temp;

	// setup levmar features
	initTransfer(g_src);
}


void EX5_makeBVHFile::UpdateScene(int even_simulationTime, int even_iter)
{
	//
	g_viewer->frame(even_simulationTime);
	g_osg_debug->removeChildren(0, g_osg_debug->getNumChildren());

	/*arma::rowvec v_des = g_target_positions.row(even_iter);
	std::vector<gVec3> desireds = UpdateTargetPose(g_src, v_des);

	gXMat offset;
	g_poseTrans->transferDesiredPoseLevMar(offset, desireds);*/

	g_src->updateKinematicBodiesOfCharacterSim();
	g_srcVis->update();
	
}

std::vector<gVec3> EX5_makeBVHFile::UpdateTargetPose(bCharacter* g_tar, arma::rowvec g_tarCoord)
{
	std::vector<gVec3> points; 
	for (int j = 0; j < 21; j++) {
		gVec3 point;
		point.setX(g_tarCoord(0+3*j));
		point.setY(g_tarCoord(1+3*j));
		point.setZ(g_tarCoord(2+3*j));
		points.push_back(point);
	}
	for (int k = 0; k < points.size(); k++) {
		g_osg_debug->addChild(gOSGShape::createPoint(gVec32OsgVec(points[k]),50.0));
	}

	return points;
}

void EX5_makeBVHFile::saveMakedMotion(arma::mat tar_des_positions, char* tarBVHFile)
{
	arma::mat tarQuaternions(g_src->sizeSafeCoordArray(), tar_des_positions.n_rows, arma::fill::zeros);

	for (int i = 0; i < tar_des_positions.n_rows; i++) {
		
		arma::rowvec v_des = tar_des_positions.row(i);
		std::vector<gVec3> desireds = UpdateTargetPose(g_src, v_des);

		gXMat offset;
		g_poseTrans->transferDesiredPoseLevMar(offset, desireds);

		//get quaternion
		arma::vec quat_pose; quat_pose.resize(g_src->numLinks() * 4 + 3);
		g_src->getSafeCoordArray(quat_pose);

		//save motion
		tarQuaternions(0, i, arma::SizeMat(quat_pose.n_elem, 1)) = quat_pose;
	}
	std::cout << "|--- retarget is finished & save bvh is starting ---|" << std::endl;
	g_saveBVH->saveBVHFile(g_src, tarBVHFile, tarQuaternions, mgBone::_AXISORDER::ZYX, 1 / 30.0);
	std::cout << "|--- file name ---|" << tarBVHFile << std::endl;
	std::cout << "|--- save is finished ---|" << std::endl;
}
 