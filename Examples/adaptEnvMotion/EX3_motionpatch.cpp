#include "EX3_motionpatch.h"


void EX3_motionpatch::extractKeyPose(arma::mat motion, std::vector<int>& keyposeindex) {
	//
	int start_frame = 2;
	int total_frames = motion.n_cols -start_frame;
	if (total_frames < 3) {
		start_frame = 0;
		total_frames = motion.n_cols;
	}
	int posestate = 0;
		
	int dof = motion.n_rows;
	// extracting key pose 
	g_src->setFromCompactCoordArray(motion.col(start_frame));
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();
	
	gVec3 pos_preHip = gVec3(100, 100, 100);
	gVec3 forward_preHip = g_src->link(0)->frame().rotZ();
	double len_lower = (g_src->link(g_src->m_lLegIdx[1])->frame().trn() - g_src->link(g_src->m_lLegIdx[2])->frame().trn()).magnitude();
	double len_upleg = (g_src->link(g_src->m_lLegIdx[0])->frame().trn() - g_src->link(g_src->m_rLegIdx[0])->frame().trn()).magnitude();
	
	std::vector<int> check_keystate(total_frames);
	
	for (int idx_motion = start_frame; idx_motion < total_frames; idx_motion++) {

		g_src->setFromCompactCoordArray(motion.col(idx_motion));
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();

		// extract frame that hip is dramatically changed
		gVec3 forward_curHip = g_src->link(0)->frame().rotZ();
		double dotprod = (forward_curHip, forward_preHip); dotprod = acos(dotprod) * gRTD;

		gVec3 pos_curHip = g_src->link(0)->frame().trn();
		double height = (pos_curHip - pos_preHip).magnitude();
		//key pose 
		if (dotprod  > 90.0 || height > 90.0 || idx_motion == total_frames-1) {
			check_keystate[idx_motion] = posestate++;
			forward_preHip = forward_curHip;
			pos_preHip = pos_curHip;
		}
		else
			check_keystate[idx_motion] = posestate;
	}
	std::vector<int> toggledindex;
	findToggledIndx(check_keystate, toggledindex);
	for (int k = 0; k < toggledindex.size(); k++)
		std::cout << toggledindex[k] << std::endl;
	keyposeindex = toggledindex;

}

void EX3_motionpatch::extractChairRoot(arma::mat motion, std::vector<int> keypose_indices, std::vector<int>& interactionstates, std::vector<gXMat>& gMat_ChairRoots)
{
	std::vector<int> interactionindex(keypose_indices.size());
	std::vector<gXMat> gMat_chairRoots(keypose_indices.size());
	for (int k = 0; k < keypose_indices.size(); k++) {
		// key frame to next key frame
		int pose_idx = keypose_indices[k];
		int start_frame = pose_idx;
		int end_frame;
		if (k == keypose_indices.size() - 1)
			end_frame = motion.n_cols;
		else
			end_frame = keypose_indices[k + 1];

		//load pose from key pose
		g_src->setFromCompactCoordArray(motion.col(pose_idx));
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();

		g_src->storeCoord();
		gVec3 key_rotZ = g_src->link(0)->frame().trn();
		gVec3 key_hipvel = g_src->link(0)->frame().trn();
		double mean_diff_z = 0; double mean_diff_hip = 0;
		for (int p = start_frame; p < end_frame; p++) {
			g_src->setFromCompactCoordArray(motion.col(p));
			g_src->updateKinematicsUptoPos();
			g_src->updateKinematicBodiesOfCharacterSim();

			gVec3 cur_rotZ = g_src->link(0)->frame().trn();
			gVec3 cur_hip = g_src->link(0)->frame().trn();

			double diff = (cur_rotZ - key_rotZ).magnitude();
			double diff_hip = (cur_hip - key_hipvel).magnitude();
			mean_diff_z += diff;
			mean_diff_hip += diff_hip;
		}
		mean_diff_z = mean_diff_z / (end_frame - start_frame);
		mean_diff_hip = mean_diff_hip / (end_frame - start_frame);
		g_src->restoreCoord();
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();

		if (mean_diff_z < 5 && mean_diff_hip < 5 || k == keypose_indices.size() -1 ) {
			interactionindex[k] = 1;

			// input positions
			gVec3 pos_rUpleg = g_src->link(g_src->m_rLegIdx[0])->frame().trn();
			gVec3 pos_lUpleg = g_src->link(g_src->m_lLegIdx[0])->frame().trn();
			gVec3 pos_rshldr = g_src->link(g_src->m_rArmIdx[0])->frame().trn();
			gVec3 pos_lshldr = g_src->link(g_src->m_lArmIdx[0])->frame().trn();
			// across vector
			gVec3 pos_chair = g_src->link(0)->frame().trn(); pos_chair.setY(0.0);
			gVec3 vec_shldr = pos_lshldr - pos_rshldr; vec_shldr.normalize();
			gVec3 vec_upleg = pos_lUpleg - pos_rUpleg; vec_upleg.normalize();
			gVec3 vec_across = (vec_shldr + vec_upleg); vec_across.normalize();
			createNewFramefromUpperBody(g_src->link(0)->frame().trn(), vec_across, gMat_chairRoots[k]);
			// 이전의 움직이는 keypose 는 같은 chair root 를 가진다고 가정한다.
			for (int j = 0; j < k; j++) {
				if (interactionindex[j] == 0) {
					gMat_chairRoots[j] = gMat_chairRoots[k];
					interactionindex[j] = 2; // 1: interacting, 2: moving
				}
			}
		}
		else
			interactionindex[k] = 0;
	}
	for (int j = 0; j < interactionindex.size(); j++) {
		std::cout << "key frame " << keypose_indices[j] << " interaction " << interactionindex[j] << " " << j << " / " << interactionindex.size() << std::endl;
	}
	if (interactionindex.size() > 0) {
		interactionstates = interactionindex;
		gMat_ChairRoots = gMat_chairRoots;
	}
	else
		std::cout << " there is no interaction 1 " << std::endl;
}

void EX3_motionpatch::setPatch(int bodypart, gXMat chairroot, plane& plane)
{
	if (bodypart == 0) {
		//hip position
		gVec3 pos_leftup = g_src->link(g_src->m_lLegIdx[0])->frame().trn(); pos_leftup = chairroot.invMultVec4(pos_leftup);
		gVec3 pos_rightup = g_src->link(g_src->m_rLegIdx[0])->frame().trn(); pos_rightup = chairroot.invMultVec4(pos_rightup);

		gVec3 position = (pos_rightup + pos_leftup) /= 2; position = chairroot.multVec4(position);
		//hip normal
		int start_index = g_src->m_rLegIdx[0]; int left_index = g_src->m_lLegIdx[0]; int forward_index = g_src->m_rLegIdx[1]; bool left_start = false;
		if (g_src->link(g_src->m_lLegIdx[1])->frame().trn().y() < g_src->link(g_src->m_rLegIdx[1])->frame().trn().y()) {
			start_index = g_src->m_lLegIdx[0]; left_index = g_src->m_rLegIdx[0]; forward_index = g_src->m_lLegIdx[1]; left_start = true;
		}

		gVec3 dir_s_left = (g_src->link(left_index)->frame().trn() - g_src->link(start_index)->frame().trn()); dir_s_left.normalize();
		if (left_start == true)
			dir_s_left = -1 * dir_s_left;
		gVec3 dir_s_forward = (g_src->link(forward_index)->frame().trn() - g_src->link(start_index)->frame().trn()); dir_s_forward.normalize();
		gVec3 normal = dir_s_forward % dir_s_left; normal.normalize();
		
		//hip plane update		
		plane.exist = true;
		gXMat gmat_rarm_plane;
		genInteractionFrame(position, normal, chairroot, gmat_rarm_plane);
		//// output plane pos 4
		double e_w = 10;
		double e_h = 10;
		double width = 20;
		double height = 20;
		plane.center = gmat_rarm_plane.trn();
		plane.normal = gmat_rarm_plane.rotY();
		plane.createPlaneOnFrame(gmat_rarm_plane, e_w, e_h, width, height, false);
	}
	if (bodypart == 1) {
		//backrest position
		gVec3 position = g_src->link(g_src->m_headIdx[0])->frame().trn(); 
		//hip normal
		gVec3 dir_spine = g_src->link(g_src->m_headIdx[0])->frame().trn() - g_src->link(0)->frame().trn(); dir_spine.normalize();
		dir_spine = chairroot.invMultVec3(dir_spine);
		gVec3 normal = gVec3(1, 0, 0) % dir_spine; normal.normalize();
		normal = chairroot.multVec3(normal);
		//hip plane update		
		plane.exist = true;
		gXMat gmat_plane;
		genInteractionFrame_forSpine(position, normal, chairroot, gmat_plane);
		//// output plane pos 4
		double e_w = 10;
		double e_h = 10;
		double width = 20;
		double height = 20;
		plane.center = gmat_plane.trn();
		plane.normal = gmat_plane.rotY();
		plane.createPlaneOnFrame(gmat_plane, e_w, e_h, width, height, false);
	}
	if (bodypart == 2) {
		//armrestl position
		gVec3 position = g_src->link(g_src->m_lArmIdx[1])->frame().trn();
		//hip normal
		gVec3 normal = gVec3(0, 1, 0);

		//hip plane update		
		plane.exist = true;
		gXMat gmat_plane;
		genInteractionFrame(position, normal, chairroot, gmat_plane);
		//// output plane pos 4
		double e_w = 10;
		double e_h = 10;
		double width = 20;
		double height = 20;
		plane.center = gmat_plane.trn();
		plane.normal = gmat_plane.rotY();
		plane.createPlaneOnFrame(gmat_plane, e_w, e_h, width, height, false);
	}
	if (bodypart == 3) {
		//armrestr position
		gVec3 position = g_src->link(g_src->m_rArmIdx[1])->frame().trn();
		//hip normal
		gVec3 normal = gVec3(0, 1, 0);

		//hip plane update		
		plane.exist = true;
		gXMat gmat_plane;
		genInteractionFrame(position, normal, chairroot, gmat_plane);
		//// output plane pos 4
		double e_w = 10;
		double e_h = 10;
		double width = 20;
		double height = 20;
		plane.center = gmat_plane.trn();
		plane.normal = gmat_plane.rotY();
		plane.createPlaneOnFrame(gmat_plane, e_w, e_h, width, height, false);
	}
	if (bodypart == 4) {
		//armrestl position
		gVec3 position = g_src->link(g_src->m_lArmIdx[2])->frame().trn();
		//hip normal
		gVec3 normal = gVec3(0, 1, 0);

		//hip plane update		
		plane.exist = true;
		gXMat gmat_plane;
		genInteractionFrame(position, normal, chairroot, gmat_plane);
		//// output plane pos 4
		double e_w = 10;
		double e_h = 10;
		double width = 20;
		double height = 20;
		plane.center = gmat_plane.trn();
		plane.normal = gmat_plane.rotY();
		plane.createPlaneOnFrame(gmat_plane, e_w, e_h, width, height, false);
	}
	if (bodypart == 5) {
		//armrestr position
		gVec3 position = g_src->link(g_src->m_rArmIdx[2])->frame().trn();
		//hip normal
		gVec3 normal = gVec3(0, 1, 0);

		//hip plane update		
		plane.exist = true;
		gXMat gmat_plane;
		genInteractionFrame(position, normal, chairroot, gmat_plane);
		//// output plane pos 4
		double e_w = 10;
		double e_h = 10;
		double width = 20;
		double height = 20;
		plane.center = gmat_plane.trn();
		plane.normal = gmat_plane.rotY();
		plane.createPlaneOnFrame(gmat_plane, e_w, e_h, width, height, false);
	}
	if (bodypart == 6) {
		//armrestl position
		gVec3 position = g_src->link(g_src->m_lLegIdx[3])->frame().trn();
		//hip normal
		gVec3 normal = gVec3(0, 1, 0);

		//hip plane update		
		plane.exist = true;
		gXMat gmat_plane;
		genInteractionFrame(position, normal, chairroot, gmat_plane);
		//// output plane pos 4
		double e_w = 10;
		double e_h = 10;
		double width = 20;
		double height = 20;
		plane.center = gmat_plane.trn();
		plane.normal = gmat_plane.rotY();
		plane.createPlaneOnFrame(gmat_plane, e_w, e_h, width, height, true);
	}
	if (bodypart == 7) {
		//armrestr position
		gVec3 position = g_src->link(g_src->m_rLegIdx[3])->frame().trn();
		//hip normal
		gVec3 normal = gVec3(0, 1, 0);

		//hip plane update		
		plane.exist = true;
		gXMat gmat_plane;
		genInteractionFrame(position, normal, chairroot, gmat_plane);
		//// output plane pos 4
		double e_w = 10;
		double e_h = 10;
		double width = 20;
		double height = 20;
		plane.center = gmat_plane.trn();
		plane.normal = gmat_plane.rotY();
		plane.createPlaneOnFrame(gmat_plane, e_w, e_h, width, height, true);
	}
}

void EX3_motionpatch::genInteractionFrame(gVec3 plane_position, gVec3 plane_normal, gXMat chairroot, gXMat& interaction_frame)
{
	gVec3 x_vec = plane_normal % chairroot.rotZ(); x_vec.normalize();
	gRotMat rotmat_; rotmat_.setColumn(0, x_vec); rotmat_.setColumn(1, plane_normal); rotmat_.setColumn(2, chairroot.rotZ());
	interaction_frame.setRot(rotmat_); interaction_frame.setTrn(plane_position);
}

void EX3_motionpatch::genInteractionFrame_forSpine(gVec3 plane_position, gVec3 plane_normal, gXMat chairroot, gXMat& interaction_frame)
{
	gVec3 x_vec = plane_normal % chairroot.rotX(); x_vec.normalize();
	gRotMat rotmat_; rotmat_.setColumn(0, x_vec); rotmat_.setColumn(1, plane_normal); rotmat_.setColumn(2, chairroot.rotX());
	interaction_frame.setRot(rotmat_); interaction_frame.setTrn(plane_position);
}

// init scene 
void EX3_motionpatch::SetupScene(char* srcfilename, char* srcCharactertxt, char*outfilefoldername)
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
	//-------------------------------------------------- do what you want
	g_lowerHeight = (g_src->link(g_src->m_lLegIdx[0])->frame().trn() - g_src->link(g_src->m_lLegIdx[2])->frame().trn()).magnitude();
	g_armHeight = (g_src->link(g_src->m_lArmIdx[0])->frame().trn() - g_src->link(g_src->m_lArmIdx[1])->frame().trn()).magnitude();
	// extract key pose
	if (g_refCoord.n_cols > 0) {
		extractKeyPose(g_refCoord, g_keypose_Indices);
		// extract Matrix
		extractChairRoot(g_refCoord, g_keypose_Indices, g_interaction_states, g_gMat_chairs);
		// init 
		g_interact_plane.root_frame = g_gMat_chairs[0];
		g_interact_plane.seat.exist = false;
		g_interact_plane.backrest.exist = false;
		g_interact_plane.armrest_R.exist = false;
		g_interact_plane.armrest_L.exist = false;
		g_interact_plane.handrest_R.exist = false;
		g_interact_plane.handrest_L.exist = false;
		g_interact_plane.footrest_L.exist = false;
		g_interact_plane.footrest_R.exist = false;

		readEnvironmentInfoTXT(g_interact_plane, outfilefoldername);
	}
}
int k = 0; int next_frame = 1;
// update scene is updated character by frame
void EX3_motionpatch::UpdateScene(int even_simulationTime, int even_iter)
{
	//
	g_viewer->frame(even_simulationTime);
	
	g_osg_debug->removeChildren(0, g_osg_debug->getNumChildren());
	gXMat gmat = g_gMat_chairs[0];
	drawFrame(gmat, g_osg_debug);

	gOSGShape::setColor(osg::Vec4(1.0, 0.0, 1.0, 0.1));
	if (g_interact_plane.seat.exist == true)
		g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_plane.seat.pos_00), gVtoOsg(g_interact_plane.seat.pos_01), gVtoOsg(g_interact_plane.seat.pos_10), gVtoOsg(g_interact_plane.seat.pos_11)));
	gOSGShape::setColor(osg::Vec4(1.0, 0.0, 1.0, 0.1));
	if (g_interact_plane.backrest.exist == true)
		g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_plane.backrest.pos_00), gVtoOsg(g_interact_plane.backrest.pos_01), gVtoOsg(g_interact_plane.backrest.pos_10), gVtoOsg(g_interact_plane.backrest.pos_11)));
	gOSGShape::setColor(osg::Vec4(0.0, 1.0, 0.0, 0.1));
	if (g_interact_plane.armrest_L.exist == true)
		g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_plane.armrest_L.pos_00), gVtoOsg(g_interact_plane.armrest_L.pos_01), gVtoOsg(g_interact_plane.armrest_L.pos_10), gVtoOsg(g_interact_plane.armrest_L.pos_11)));
	if (g_interact_plane.armrest_R.exist == true)
		g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_plane.armrest_R.pos_00), gVtoOsg(g_interact_plane.armrest_R.pos_01), gVtoOsg(g_interact_plane.armrest_R.pos_10), gVtoOsg(g_interact_plane.armrest_R.pos_11)));
	gOSGShape::setColor(osg::Vec4(0.0, 0.0, 1.0, 0.1));
	if (g_interact_plane.handrest_L.exist == true)
		g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_plane.handrest_L.pos_00), gVtoOsg(g_interact_plane.handrest_L.pos_01), gVtoOsg(g_interact_plane.handrest_L.pos_10), gVtoOsg(g_interact_plane.handrest_L.pos_11)));
	if (g_interact_plane.handrest_R.exist == true)
		g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_plane.handrest_R.pos_00), gVtoOsg(g_interact_plane.handrest_R.pos_01), gVtoOsg(g_interact_plane.handrest_R.pos_10), gVtoOsg(g_interact_plane.handrest_R.pos_11)));
	gOSGShape::setColor(osg::Vec4(0.0, 1.0, 1.0, 0.1));
	if (g_interact_plane.footrest_L.exist == true)
		g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_plane.footrest_L.pos_00), gVtoOsg(g_interact_plane.footrest_L.pos_01), gVtoOsg(g_interact_plane.footrest_L.pos_10), gVtoOsg(g_interact_plane.footrest_L.pos_11)));
	if (g_interact_plane.footrest_R.exist == true)
		g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_plane.footrest_R.pos_00), gVtoOsg(g_interact_plane.footrest_R.pos_01), gVtoOsg(g_interact_plane.footrest_R.pos_10), gVtoOsg(g_interact_plane.footrest_R.pos_11)));
		
	updateCharacterFrame(even_iter);
	g_srcVis->update();
	
}