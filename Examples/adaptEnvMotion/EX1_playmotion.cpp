#include "EX1_playmotion.h"


void EX1_playmotion::SetupScene(char* srcfilename, char* srcCharactertxt)
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
	std::cout << "|--- load tar character file ---|" << std::endl;
	// generate tar bCharacter from txt file
	bCharacter* tar = new bCharacter();
	bCharacterSim* tarSim = new bCharacterSim(tar);
	gBDOSGSystem* tarVis = new gBDOSGSystem();
	double check_load = loadAvatarModelFromFile(tar, tarSim, tarVis, "mixamo_rest.txt", 1.0);

	std::cout << "|--- initialization process is finished ---|\n" << std::endl;

	std::cout << "|--- UPDATE RETARGET SCENE       ---|" << std::endl;
	std::cout << "|--- press  Y to forward motion  ---|" << std::endl;
	std::cout << "|--- press  H to backward motion ---|" << std::endl;
	std::cout << "|--- press  U to forward 1 frame ---|" << std::endl;
	std::cout << "|--- press  J to backward 1 frame---|" << std::endl;
	std::cout << "|--- press  P to see next bvh    ---|" << std::endl;
	g_osg_cha->addChild(g_srcVis->getOSGGroup());
	

}

void EX1_playmotion::UpdateScene(int even_simulationTime, int even_iter)
{
	//
	g_viewer->frame(even_simulationTime);

	arma::vec pose = g_refCoord.col(even_iter);
		
	g_src->setFromCompactCoordArray(pose);
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();

	g_srcVis->update();
	
}
