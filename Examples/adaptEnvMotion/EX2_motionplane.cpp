#include "EX2_motionplane.h"


void EX2_motionplane::extractKeyPose(arma::mat motion, std::vector<int>& keyposeindex) {
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

		//gVec3 pos_curHip = g_src->link(0)->frame().trn();
		gVec3 pos_curHip = g_src->link(g_src->m_lArmIdx[2])->frame().trn();
		double height = (pos_curHip - pos_preHip).magnitude();
		height = pos_curHip.y() - pos_preHip.y();
		//key pose 
		if (dotprod  > 90.0 || abs(height) > 10.0 || idx_motion == total_frames-1) {
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

void EX2_motionplane::extractChairRoot(arma::mat motion, std::vector<int> keypose_indices, std::vector<int>& interactionstates, std::vector<gXMat>& gMat_ChairRoots)
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

//chain_index : hip: 0
std::vector<gVec3> EX2_motionplane::gatherJointPositions(arma::mat motion, int start_frame, int end_frame, int joint_index)
{
	std::vector<gVec3> positions(end_frame - start_frame);
	// gather motion data
	g_src->storeCoord();
	for (int k = start_frame, p = 0; k < end_frame; k++, p++) {
		arma::vec frame_pose = motion.col(k);
		g_src->setFromCompactCoordArray(frame_pose);
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();

		//positions
		positions[p] = g_src->link(joint_index)->frame().trn();
	}
	g_src->restoreCoord();
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();

	return positions;
}

std::vector<gVec3> EX2_motionplane::gatherJointPositions(arma::mat motion, int start_frame, int end_frame, int joint_index, int joint2_index)
{
	std::vector<gVec3> positions(end_frame - start_frame);
	// gather motion data
	g_src->storeCoord();
	for (int k = start_frame, p = 0; k < end_frame; k++, p++) {
		arma::vec frame_pose = motion.col(k);
		g_src->setFromCompactCoordArray(frame_pose);
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();

		//hip positions
		positions[p] = (g_src->link(joint_index)->frame().trn() + g_src->link(joint2_index)->frame().trn()) /= 2;

	}
	g_src->restoreCoord();
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();

	return positions;
}

// get normal
std::vector<gVec3> EX2_motionplane::gatherJointNormals(arma::mat motion, int start_frame, int end_frame, int joint_index)
{
	std::vector<gVec3> normals(end_frame - start_frame);
	// gather motion data
	g_src->storeCoord();
	for (int k = start_frame, p = 0; k < end_frame; k++, p++) {
		arma::vec frame_pose = motion.col(k);
		g_src->setFromCompactCoordArray(frame_pose);
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();

		normals[p] = g_src->link(joint_index)->frame().rotY();
	}
	g_src->restoreCoord();
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();

	return normals;
}

std::vector<gVec3> EX2_motionplane::gatherJointDir(arma::mat motion, int start_frame, int end_frame, int joint_start, int joint2_end)
{
	std::vector<gVec3> normals(end_frame - start_frame);
	// gather motion data
	g_src->storeCoord();
	for (int k = start_frame, p = 0; k < end_frame; k++, p++) {
		arma::vec frame_pose = motion.col(k);
		g_src->setFromCompactCoordArray(frame_pose);
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();

		//hip 
		gVec3 normal = g_src->link(joint2_end)->frame().trn() - g_src->link(joint_start)->frame().trn(); normal.normalize();
		normals[p] = normal;
		
	}
	g_src->restoreCoord();
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();

	return normals;
}


std::vector<gVec3> EX2_motionplane::gatherJointNormals(arma::mat motion, int start_frame, int end_frame, int joint_index_start, int joint_index_end, int joint_index_forward, bool left_start)
{
	std::vector<gVec3> normals(end_frame - start_frame);
	// gather motion data
	g_src->storeCoord();
	for (int k = start_frame, p = 0; k < end_frame; k++, p++) {
		arma::vec frame_pose = motion.col(k);
		g_src->setFromCompactCoordArray(frame_pose);
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();

		//hip positions
		gVec3 dir_s_left = (g_src->link(joint_index_end)->frame().trn()- g_src->link(joint_index_start)->frame().trn()); dir_s_left.normalize();
		if (left_start == true)
			dir_s_left = -1 * dir_s_left;
		gVec3 dir_s_forward = (g_src->link(joint_index_forward)->frame().trn() - g_src->link(joint_index_start)->frame().trn()); dir_s_forward.normalize();
		gVec3 normal = dir_s_forward % dir_s_left; //normal.normalize();
		normals[p] = normal;
	}
	g_src->restoreCoord();
	g_src->updateKinematicsUptoPos();
	g_src->updateKinematicBodiesOfCharacterSim();

	return normals;
}

void EX2_motionplane::extractInteractPlanes(arma::mat motion, std::vector<int> interactionstates, std::vector<int> keypose, std::vector<gXMat> gMat_chair)
{
	g_interact_planes.resize(keypose.size());

	for (int k = 0; k < keypose.size(); k++) {
		g_interact_planes[k].root_frame = gMat_chair[k];
		if (interactionstates[k] == 1) {
			int key_start = keypose[k];
			int key_end = 0;
			if (k == keypose.size() - 1)
				key_end = motion.n_cols;
			else
				key_end = keypose[k + 1];
			
			g_src->setFromCompactCoordArray(motion.col(key_start));
			g_src->updateKinematicsUptoPos();
			g_src->updateKinematicBodiesOfCharacterSim();
			
			// gather hip positions
			if (g_src->link(0)->frame().trn().y() < g_lowerHeight*0.9) {
				// 앉기
				int start_index = g_src->m_rLegIdx[0]; 
				int left_index = g_src->m_lLegIdx[0]; 
				int forward_index = g_src->m_rLegIdx[1]; 
				bool left_start = false;
				if (g_src->link(g_src->m_lLegIdx[1])->frame().trn().y() < g_src->link(g_src->m_rLegIdx[1])->frame().trn().y()) {
					start_index = g_src->m_lLegIdx[0]; left_index = g_src->m_rLegIdx[0]; forward_index = g_src->m_lLegIdx[1]; left_start = true;
				}
				std::vector<gVec3> hip_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_lLegIdx[0], g_src->m_rLegIdx[0]);
				std::vector<gVec3> hip_normals = gatherJointNormals(motion, key_start, key_end, start_index, left_index, forward_index,left_start);
				plane seat;
				gXMat gmat_seat_plane;
				gVec3 seat_position; gVec3 seat_normal;
				seat.exist = findPlanePositionNormal(hip_positions, hip_normals,
					-1000, 1000, -1000, 1000, -1000, 1000,
					-1000, 1000, -1000, 1000, -1000, 1000, gMat_chair[k], seat_position, seat_normal);

				if (seat.exist) {
					genInteractionFrame(seat_position, seat_normal, gMat_chair[k], gmat_seat_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					seat.center = gmat_seat_plane.trn();
					seat.normal = gmat_seat_plane.rotY();
					seat.createPlaneOnFrame(gmat_seat_plane, e_w, e_h, width, height,false);
				}
				g_interact_planes[k].seat = seat;

				// backrest
				std::vector<gVec3> spine_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_headIdx[0]);
				std::vector<gVec3> spine_directions = gatherJointDir(motion, key_start, key_end, 0, g_src->m_headIdx[0]);
				plane backrest;
				gXMat gmat_backrest_plane;
				gVec3 backrest_position; gVec3 backrest_normal;
				backrest.exist = findPlanePositionNormal_forSpine(spine_positions, spine_directions,
					-1000, 1000, -1000, 1000, -1000, 1000,
					-0.1, 0.1, -1000, 1000, -1, -0.1, gMat_chair[k], backrest_position, backrest_normal);

				if (backrest.exist) {
					genInteractionFrame_forSpine(backrest_position, backrest_normal, gMat_chair[k], gmat_backrest_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					backrest.center = gmat_backrest_plane.trn();
					backrest.normal = gmat_backrest_plane.rotY();
					backrest.createPlaneOnFrame(gmat_backrest_plane, e_w, e_h, width, height, false);
				}
				g_interact_planes[k].backrest = backrest;
				// left arm
				std::vector<gVec3> larm_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_lArmIdx[1]);
				plane larm;
				gXMat gmat_larm_plane;
				gVec3 arm_position; gVec3 arm_normal;
				larm.exist = findPlanePositionNormal_withToq(larm_positions, 
					-1000, 1000, -1000, 1000, -1000, 1000, 
					-1000, 1000, -0.7, -0.0, -1000, 1000,
					10, larm_positions.size()/2,  g_src->link(g_src->m_lArmIdx[1])->frame().trn(),g_src->link(g_src->m_lArmIdx[0])->frame().trn(),gMat_chair[k], arm_position,arm_normal);
				if (larm.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_larm_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					larm.center = gmat_larm_plane.trn();
					larm.normal = gmat_larm_plane.rotY();
					larm.createPlaneOnFrame(gmat_larm_plane, e_w, e_h, width, height,false);
				}
				g_interact_planes[k].armrest_L = larm;

				// right arm
				std::vector<gVec3> rarm_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_rArmIdx[1]);
				plane rarm;
				gXMat gmat_rarm_plane;
				rarm.exist = findPlanePositionNormal_withToq(rarm_positions,
					-1000, 1000, -1000, 1000, -1000, 1000, 
					-1000, 1000, -0.7, -0.0, -1000, 1000, 10, rarm_positions.size() / 2,
					g_src->link(g_src->m_rArmIdx[1])->frame().trn(), g_src->link(g_src->m_rArmIdx[0])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				if (rarm.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_rarm_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					rarm.center = gmat_rarm_plane.trn();
					rarm.normal = gmat_rarm_plane.rotY();
					rarm.createPlaneOnFrame(gmat_rarm_plane, e_w, e_h, width, height,false);
				}
				g_interact_planes[k].armrest_R = rarm;
				
				// right hand
				std::vector<gVec3> rhand_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_rArmIdx[2]);
				plane rhand;
				gXMat gmat_rhand_plane;
				rhand.exist = findPlanePositionNormal_withToq(rhand_positions,
					-1000, 1000, -1000, 1000, 0.0, 1000,
					-1000, 1000, -0.3, -0.0, -1.0, 1.0, 10, rhand_positions.size() / 2,
					g_src->link(g_src->m_rArmIdx[2])->frame().trn(), g_src->link(g_src->m_rArmIdx[1])->frame().trn(),gMat_chair[k], arm_position, arm_normal);
				if (rhand.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_rhand_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					rhand.center = gmat_rhand_plane.trn();
					rhand.normal = gmat_rhand_plane.rotY();
					rhand.createPlaneOnFrame(gmat_rhand_plane, e_w, e_h, width, height,false);
				}
				g_interact_planes[k].handrest_R = rhand;

				// left hand
				std::vector<gVec3> lhand_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_lArmIdx[2]);
				plane lhand;
				gXMat gmat_lhand_plane;
				lhand.exist = findPlanePositionNormal_withToq(lhand_positions,
					-1000, 1000, -1000, 1000, 0.0, 1000,
					-1000, 1000, -0.3, -0.0, -1.0, 1.0, 10, lhand_positions.size() / 2,
					g_src->link(g_src->m_lArmIdx[2])->frame().trn(), g_src->link(g_src->m_rArmIdx[1])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				if (lhand.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_lhand_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					lhand.center = gmat_lhand_plane.trn();
					lhand.normal = gmat_lhand_plane.rotY();
					lhand.createPlaneOnFrame(gmat_lhand_plane, e_w, e_h, width, height,false);
				}
				g_interact_planes[k].handrest_L = lhand;

				// right foot
				std::vector<gVec3> rfoot_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_rLegIdx[3]);
				plane rfoot;
				gXMat gmat_rfoot_plane;
				rfoot.exist = findPlanePositionNormal_withReference(rfoot_positions,
					-1000, 1000, -1000, 1000, -1000, 1000, 7,
					g_src->link(g_src->m_rLegIdx[3])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				if (rfoot.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_rfoot_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					rfoot.center = gmat_rfoot_plane.trn();
					rfoot.normal = gmat_rfoot_plane.rotY();
					rfoot.createPlaneOnFrame(gmat_rfoot_plane, e_w, e_h, width, height,true);
				}
				g_interact_planes[k].footrest_R = rfoot;

				// left foot
				std::vector<gVec3> lfoot_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_lLegIdx[3]);
				plane lfoot;
				gXMat gmat_lfoot_plane;
				lfoot.exist = findPlanePositionNormal_withReference(lfoot_positions,
					-1000, 1000, -1000, 1000, -1000, 1000, 7,
					g_src->link(g_src->m_lLegIdx[3])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				if (lfoot.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_lfoot_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					lfoot.center = gmat_lfoot_plane.trn();
					lfoot.normal = gmat_lfoot_plane.rotY();
					lfoot.createPlaneOnFrame(gmat_lfoot_plane, e_w, e_h, width, height,true);
				}
				g_interact_planes[k].footrest_L = lfoot;

				std::cout << "seat " << g_interact_planes[k].seat.exist << " / " << interactionstates.size() << std::endl;
				std::cout << "backrest " << g_interact_planes[k].backrest.exist << " / " << interactionstates.size() << std::endl;
				std::cout << "armrest_R " << g_interact_planes[k].armrest_R.exist << " / " << interactionstates.size() << std::endl;
				std::cout << "armrest_L " << g_interact_planes[k].armrest_L.exist << " / " << interactionstates.size() << std::endl;
				std::cout << "handrest_R " << g_interact_planes[k].handrest_R.exist << " / " << interactionstates.size() << std::endl;
				std::cout << "handrest_L " << g_interact_planes[k].handrest_L.exist << " / " << interactionstates.size() << std::endl;
				std::cout << "footrest_L " << g_interact_planes[k].footrest_L.exist << " / " << interactionstates.size() << std::endl;
				std::cout << "footrest_R " << g_interact_planes[k].footrest_R.exist << " / " << interactionstates.size() << std::endl;
			}
			else {
				// 서기
				g_interact_planes[k].seat.exist = false;
				g_interact_planes[k].backrest.exist = false;
				g_interact_planes[k].armrest_R.exist = false;
				g_interact_planes[k].armrest_L.exist = false;
				
				// left arm
				std::vector<gVec3> larm_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_lArmIdx[1]);
				plane larm;
				gXMat gmat_larm_plane;
				gVec3 arm_position; gVec3 arm_normal;
				larm.exist = findPlanePositionNormal_withToq(larm_positions,
					-1000, 1000, -1000, 1000, -1000, 1000, 
					-1000, 1000, -0.7, -0.0, -1000, 1000, 7, larm_positions.size() / 3,
					g_src->link(g_src->m_lArmIdx[1])->frame().trn(), g_src->link(g_src->m_lArmIdx[0])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				if (larm.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_larm_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					larm.center = gmat_larm_plane.trn();
					larm.normal = gmat_larm_plane.rotY();
					larm.createPlaneOnFrame(gmat_larm_plane, e_w, e_h, width, height, false);
				}
				g_interact_planes[k].armrest_L = larm;

				// right arm
				std::vector<gVec3> rarm_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_rArmIdx[1]);
				plane rarm;
				gXMat gmat_rarm_plane;
				rarm.exist = findPlanePositionNormal_withToq(rarm_positions,
					-1000, 1000, -1000, 1000, -1000, 1000, 
					-1000, 1000, -0.8, -0.0, -1000, 1000, 7, rarm_positions.size() / 3,
					g_src->link(g_src->m_rArmIdx[1])->frame().trn(), g_src->link(g_src->m_rArmIdx[0])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				if (rarm.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_rarm_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					rarm.center = gmat_rarm_plane.trn();
					rarm.normal = gmat_rarm_plane.rotY();
					rarm.createPlaneOnFrame(gmat_rarm_plane, e_w, e_h, width, height, false);
				}
				g_interact_planes[k].armrest_R = rarm;

				// right hand
				std::vector<gVec3> rhand_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_rArmIdx[2]);
				plane rhand;
				gXMat gmat_rhand_plane;
				
				rhand.exist = findPlanePositionNormal_withToq(rhand_positions,
					-1000, 1000, -1000, 1000, -1000, 1000,
					-1000, 1000, -0.8, -0.0, -1000, 1000, 7, rhand_positions.size() / 3,
					g_src->link(g_src->m_rArmIdx[2])->frame().trn(), g_src->link(g_src->m_rArmIdx[1])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				/*rhand.exist = findPlanePositionNormal_withReference(rhand_positions,
					-1000, 1000, g_lowerHeight * 0.8, g_src->link(g_src->m_rArmIdx[1])->frame().trn().y(), 10, g_armHeight,
					5.0, g_src->link(g_src->m_rArmIdx[2])->frame().trn(), gMat_chair[k], arm_position, arm_normal);*/
				if (rhand.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_rhand_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					rhand.center = gmat_rhand_plane.trn();
					rhand.normal = gmat_rhand_plane.rotY();
					rhand.createPlaneOnFrame(gmat_rhand_plane, e_w, e_h, width, height, false);
				}
				g_interact_planes[k].handrest_R = rhand;

				// left hand
				std::vector<gVec3> lhand_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_lArmIdx[2]);
				plane lhand;
				gXMat gmat_lhand_plane;
				lhand.exist = findPlanePositionNormal_withToq(lhand_positions,
					-1000, 1000, -1000, 1000, -1000, 1000, 
					-1000, 1000, -0.7, -0.0, -1000, 1000, 7, lhand_positions.size() / 3,
					g_src->link(g_src->m_lArmIdx[2])->frame().trn(), g_src->link(g_src->m_lArmIdx[1])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				/*lhand.exist = findPlanePositionNormal_withReference(lhand_positions,
					-1000, 1000, g_lowerHeight * 0.8, g_src->link(g_src->m_lArmIdx[1])->frame().trn().y(), 10, 1000,
					5.0, g_src->link(g_src->m_lArmIdx[2])->frame().trn(), gMat_chair[k], arm_position, arm_normal);*/
				if (lhand.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_lhand_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					lhand.center = gmat_lhand_plane.trn();
					lhand.normal = gmat_lhand_plane.rotY();
					lhand.createPlaneOnFrame(gmat_lhand_plane, e_w, e_h, width, height, false);
				}
				g_interact_planes[k].handrest_L = lhand;

				// right foot
				std::vector<gVec3> rfoot_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_rLegIdx[3]);
				plane rfoot;
				gXMat gmat_rfoot_plane;
				rfoot.exist = findPlanePositionNormal_withReference(rfoot_positions,
					-1000, 1000, -1000, 1000, -1000, 1000, 7,
					g_src->link(g_src->m_rLegIdx[3])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				if (rfoot.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_rfoot_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					rfoot.center = gmat_rfoot_plane.trn();
					rfoot.normal = gmat_rfoot_plane.rotY();
					rfoot.createPlaneOnFrame(gmat_rfoot_plane, e_w, e_h, width, height, true);
				}
				g_interact_planes[k].footrest_R = rfoot;

				// left foot
				std::vector<gVec3> lfoot_positions = gatherJointPositions(motion, key_start, key_end, g_src->m_lLegIdx[3]);
				plane lfoot;
				gXMat gmat_lfoot_plane;
				lfoot.exist = findPlanePositionNormal_withReference(lfoot_positions,
					-1000, 1000, -1000, 1000, -1000, 1000, 7,
					g_src->link(g_src->m_lLegIdx[3])->frame().trn(), gMat_chair[k], arm_position, arm_normal);
				if (lfoot.exist) {
					genInteractionFrame(arm_position, arm_normal, gMat_chair[k], gmat_lfoot_plane);
					//// output plane pos 4
					double e_w = 10;
					double e_h = 10;
					double width = 20;
					double height = 20;
					lfoot.center = gmat_lfoot_plane.trn();
					lfoot.normal = gmat_lfoot_plane.rotY();
					lfoot.createPlaneOnFrame(gmat_lfoot_plane, e_w, e_h, width, height, true);
				}
				g_interact_planes[k].footrest_L = lfoot;

				std::cout << "seat " << g_interact_planes[k].seat.exist <<  " / " << interactionstates.size() << std::endl;
				std::cout << "backrest " << g_interact_planes[k].backrest.exist <<  " / " << interactionstates.size() << std::endl;
				std::cout << "armrest_R " << g_interact_planes[k].armrest_R.exist <<  " / " << interactionstates.size() << std::endl;
				std::cout << "armrest_L " << g_interact_planes[k].armrest_L.exist <<  " / " << interactionstates.size() << std::endl;
				std::cout << "handrest_R " << g_interact_planes[k].handrest_R.exist <<  " / " << interactionstates.size() << std::endl;
				std::cout << "handrest_L " << g_interact_planes[k].handrest_L.exist <<  " / " << interactionstates.size() << std::endl;
				std::cout << "footrest_L " << g_interact_planes[k].footrest_L.exist <<  " / " << interactionstates.size() << std::endl;
				std::cout << "footrest_R " << g_interact_planes[k].footrest_R.exist <<  " / " << interactionstates.size() << std::endl;

				
			}
			
		}
		else {
			g_interact_planes[k].seat.exist = false;
			g_interact_planes[k].backrest.exist = false;
			g_interact_planes[k].armrest_R.exist = false;
			g_interact_planes[k].armrest_L.exist = false;
			g_interact_planes[k].handrest_R.exist = false;
			g_interact_planes[k].handrest_L.exist = false;
			g_interact_planes[k].footrest_L.exist = false;
			g_interact_planes[k].footrest_R.exist = false;
		}
	}
}

bool EX2_motionplane::findPlanePositionNormal(std::vector<gVec3> positions, std::vector<gVec3> normals, double cp_xl, double cp_xu, double cp_yl, double cp_yu, double cp_zl, double cp_zu, double cn_xl, double cn_xu, double cn_yl, double cn_yu, double cn_zl, double cn_zu, gXMat chair_root, gVec3& plane_position, gVec3& plane_normal)
{
	gVec3 min_position(1000, 1000, 1000);
	int min_index = 10;
	for (int i = 0; i < positions.size(); i++){
		gVec3 pos = positions[i]; pos = chair_root.invMultVec4(pos);
		gVec3 normal = normals[i]; normal = chair_root.invMultVec3(normal);

		if (pos.z() < cp_zu && pos.z() > cp_zl &&
			pos.y() < cp_yu && pos.y() > cp_yl &&
			pos.x() < cp_xu && pos.x() > cp_xl &&
			normal.z() < cn_zu && normal.z() > cn_zl &&
			normal.y() < cn_yu && normal.y() > cn_yl &&
			normal.x() < cn_xu && normal.x() > cn_xl 
			) {
			if (pos.y() < min_position.y()) {
				min_position = pos;
				min_index = i;
			}
		}
		
	}
	bool has_frame = false;
	if (min_position.y() > 1000)
		has_frame = false;
	else {
		has_frame = true;
		plane_position = min_position;
		plane_normal = normals[min_index];

		plane_position = chair_root.multVec4(plane_position);
		plane_normal = chair_root.multVec3(plane_normal);
	}
	return has_frame;

}

bool EX2_motionplane::findPlanePositionNormal_withToq(std::vector<gVec3> positions, 
	double cp_xl, double cp_xu, double cp_yl, double cp_yu, double cp_zl, double cp_zu, 
	double cn_xl, double cn_xu, double cn_yl, double cn_yu, double cn_zl, double cn_zu,
	double const_torq, double number, gVec3 reference_joint,
	gVec3 torqued_joint, gXMat chair_root, gVec3& plane_position, gVec3& plane_normal)
{
	int n_clip_frames_forPlane = positions.size();

	gVec3 mean_normal_l = gVec3(0, 1, 0);
	gVec3 min_pos_l = gVec3(1000, 1000, 1000);
	int n_interaction_normal = 0;

	for (int p = 0; p < n_clip_frames_forPlane; p++) {
		//left hand 3 positions
		gVec3 p0 = positions[p + 0];
		
		gVec3 vec_ref = (p0 - reference_joint);

		gVec3 vec = (p0 - torqued_joint); vec.normalize();
		
		p0 = chair_root.invMultVec4(p0);
		vec = chair_root.invMultVec3(vec);
		if (p0.z() < cp_zu && p0.z() > cp_zl &&
			p0.y() < cp_yu && p0.y() > cp_yl &&
			p0.x() < cp_xu && p0.x() > cp_xl &&
			vec.z() < cn_zu && vec.z() > cn_zl &&
			vec.y() < cn_yu && vec.y() > cn_yl &&
			vec.x() < cn_xu && vec.x() > cn_xl &&
			vec_ref.magnitude() < const_torq
			) {
			n_interaction_normal++;
			if (p0.y() < min_pos_l.y()) {
				min_pos_l = p0;
			}
		}	


	}
	
	bool has_frame = false;
	if (n_interaction_normal < number)
		has_frame = false;
	else {
		has_frame = true;
		plane_position = min_pos_l;
		plane_normal = mean_normal_l;

		plane_position = chair_root.multVec4(plane_position);
		plane_normal = chair_root.multVec3(plane_normal);
	}
	return has_frame;
}

bool EX2_motionplane::findPlanePositionNormal_withReference(std::vector<gVec3> positions, 
	double cp_xl, double cp_xu, double cp_yl, double cp_yu, double cp_zl, double cp_zu, double const_vel, 
	gVec3 reference_joint, gXMat chair_root, gVec3& plane_position, gVec3& plane_normal)
{

	int n_clip_frames_forPlane = positions.size()-2 +1;

	gVec3 mean_normal_l = gVec3(0, 1, 0);
	gVec3 min_pos_l = gVec3(1000, 1000, 1000);
	int n_interaction_normal = 0;

	for (int p = 0; p < n_clip_frames_forPlane; p++) {
		//left hand 3 positions
		gVec3 p0 = positions[p + 0];
		gVec3 p1 = positions[p + 1];

		gVec3 gravity = gVec3(0, -9.8, 0); gravity.normalize();
		gVec3 vec = (p0 - reference_joint); 
		double vel = vec.magnitude();
		p0 = chair_root.invMultVec4(p0);
		if (p0.z() < cp_zu && p0.z() > cp_zl &&
			p0.y() < cp_yu && p0.y() > cp_yl &&
			p0.x() < cp_xu && p0.x() > cp_xl &&
			vel < const_vel
			) {
			n_interaction_normal++;
			if (p0.y() < min_pos_l.y()) {
				min_pos_l = p0;
			}
		}


	}

	bool has_frame = false;
	if (n_interaction_normal < 1)
		has_frame = false;
	else {
		has_frame = true;
		plane_position = min_pos_l;
		plane_normal = mean_normal_l;

		plane_position = chair_root.multVec4(plane_position);
		plane_normal = chair_root.multVec3(plane_normal);
	}
	return has_frame;
}

bool EX2_motionplane::findPlanePositionNormal_forSpine(std::vector<gVec3> positions, std::vector<gVec3> normals, 
	double cp_xl, double cp_xu, double cp_yl, double cp_yu, double cp_zl, double cp_zu, 
	double cn_xl, double cn_xu, double cn_yl, double cn_yu, double cn_zl, double cn_zu, 
	gXMat chair_root, gVec3& plane_position, gVec3& plane_normal)
{
	int n_clip_frames_forPlane = positions.size();

	gVec3 min_normal_l = gVec3(1000, 1000, 1000);
	gVec3 min_dir_l = gVec3(1000, 1000, 1000);
	gVec3 min_pos_l = gVec3(1000, 1000, 1000);
	int n_interaction_normal = 0;
	int min_index = 10;
	for (int p = 0; p < n_clip_frames_forPlane; p++) {
		//left hand 3 positions
		gVec3 p0 = positions[p];
		gVec3 dir = normals[p];

		dir = chair_root.invMultVec3(dir);
		gVec3 normal = gVec3(1,0,0) % dir; normal.normalize();

		p0 = chair_root.invMultVec4(p0);
		if (p0.z() < cp_zu && p0.z() > cp_zl &&
			p0.y() < cp_yu && p0.y() > cp_yl &&
			p0.x() < cp_xu && p0.x() > cp_xl &&
			dir.z() < cn_zu && dir.z() > cn_zl &&
			dir.y() < cn_yu && dir.y() > cn_yl &&
			dir.x() < cn_xu && dir.x() > cn_xl
			) {
			n_interaction_normal++;
			if (dir.z() < min_dir_l.z()) {
				min_pos_l = p0;
				min_dir_l = dir;
				min_normal_l = normal;
			}
		}


	}

	bool has_frame = false;
	if (n_interaction_normal < 1)
		has_frame = false;
	else {
		has_frame = true;
		plane_position = min_pos_l;
		plane_normal = min_normal_l;

		plane_position = chair_root.multVec4(plane_position);
		plane_normal = chair_root.multVec3(plane_normal);
	}
	return has_frame;
}

void EX2_motionplane::genInteractionFrame(gVec3 plane_position, gVec3 plane_normal, gXMat chairroot, gXMat& interaction_frame)
{
	gVec3 x_vec = plane_normal % chairroot.rotZ(); x_vec.normalize();
	gRotMat rotmat_; rotmat_.setColumn(0, x_vec); rotmat_.setColumn(1, plane_normal); rotmat_.setColumn(2, chairroot.rotZ());
	interaction_frame.setRot(rotmat_); interaction_frame.setTrn(plane_position);
}

void EX2_motionplane::genInteractionFrame_forSpine(gVec3 plane_position, gVec3 plane_normal, gXMat chairroot, gXMat& interaction_frame)
{
	gVec3 x_vec = plane_normal % chairroot.rotX(); x_vec.normalize();
	gRotMat rotmat_; rotmat_.setColumn(0, x_vec); rotmat_.setColumn(1, plane_normal); rotmat_.setColumn(2, chairroot.rotX());
	interaction_frame.setRot(rotmat_); interaction_frame.setTrn(plane_position);
}

bool EX2_motionplane::findMeanPosNormal_const(std::vector<gVec3> pos_datas, double x_u, double x_l, double y_u, double y_l, double z_u, double z_l, gXMat chair_root, gXMat& interaction_frame)
{
	int n_clip_frames_forPlane = pos_datas.size() - 3 + 1;

	gVec3 mean_normal_l = gVec3(0, 0, 0);
	gVec3 mean_pos_l = gVec3(0, 0, 0);
	int n_interaction_normal = 0;
	for (int p = 0; p < n_clip_frames_forPlane; p++) {
		//left hand 3 positions
		gVec3 p0 = pos_datas[p + 0];
		gVec3 p1 = pos_datas[p + 1];
		gVec3 p2 = pos_datas[p + 2];
		g_osg_debug->addChild(gOSGShape::createPoint(gVtoOsg(p0), 5.0));
		gVec3 vec_1 = p1 - p0; vec_1.normalize();
		gVec3 vec_2 = p2 - p0; vec_2.normalize();
		gVec3 normal = vec_1 % vec_2; normal.normalize();

		normal = chair_root.invMultVec3(normal);
		if (normal.z() < z_u && normal.z() > z_l &&
			normal.y() < y_u && normal.y() > y_l &&
			normal.x() < x_u && normal.x() > x_l
			) {
			mean_normal_l += normal;
			mean_pos_l += (p0 + p1 + p2);
			n_interaction_normal++;
		}
	}
	mean_normal_l = (mean_normal_l /= n_interaction_normal); mean_normal_l.normalize();
	mean_pos_l = (mean_pos_l /= n_interaction_normal * 3);

	mean_normal_l = chair_root.multVec3(mean_normal_l);

	gVec3 x_vec = mean_normal_l % chair_root.rotZ(); x_vec.normalize();


	bool has_frame = false;
	if (n_interaction_normal < 1)
		has_frame = false;
	else {
		has_frame = true;
		gRotMat rotmat_; rotmat_.setColumn(0, x_vec); rotmat_.setColumn(1, mean_normal_l); rotmat_.setColumn(2, chair_root.rotZ());
		interaction_frame.setRot(rotmat_); interaction_frame.setTrn(mean_pos_l);
	}
	return has_frame;
}
bool EX2_motionplane::findMeanPosNormal_const_hand(std::vector<gVec3> pos_datas, double x_u, double x_l, double y_u, double y_l, double z_u, double z_l, double hand_y_u, double hand_y_l, gXMat chair_root, gXMat& interaction_frame)
{
	int n_clip_frames_forPlane = pos_datas.size() - 3 + 1;

	gVec3 mean_normal_l = gVec3(0, 0, 0);
	gVec3 mean_pos_l = gVec3(0, 0, 0);
	int n_interaction_normal = 0;
	for (int p = 0; p < n_clip_frames_forPlane; p++) {
		//left hand 3 positions
		gVec3 p0 = pos_datas[p + 0];
		gVec3 p1 = pos_datas[p + 1];
		gVec3 p2 = pos_datas[p + 2];
		g_osg_debug->addChild(gOSGShape::createPoint(gVtoOsg(p0), 5.0));
		gVec3 vec_1 = p1 - p0; vec_1.normalize();
		gVec3 vec_2 = p2 - p0; vec_2.normalize();
		gVec3 normal = vec_1 % vec_2; normal.normalize();

		normal = chair_root.invMultVec3(normal);
		if (normal.z() < z_u && normal.z() > z_l &&
			normal.y() < y_u && normal.y() > y_l &&
			normal.x() < x_u && normal.x() > x_l &&
			p0.y() < hand_y_u && p0.y() > hand_y_l &&
			p1.y() < hand_y_u && p1.y() > hand_y_l&&
			p2.y() < hand_y_u && p2.y() > hand_y_l
			) {
			mean_normal_l += normal;
			mean_pos_l += (p0 + p1 + p2);
			n_interaction_normal++;
		}
	}
	mean_normal_l = (mean_normal_l /= n_interaction_normal); mean_normal_l.normalize();
	mean_pos_l = (mean_pos_l /= n_interaction_normal * 3);

	mean_normal_l = chair_root.multVec3(mean_normal_l);

	gVec3 x_vec = mean_normal_l % chair_root.rotZ(); x_vec.normalize();


	bool has_frame = false;
	if (n_interaction_normal < 1)
		has_frame = false;
	else {
		has_frame = true;
		gRotMat rotmat_; rotmat_.setColumn(0, x_vec); rotmat_.setColumn(1, mean_normal_l); rotmat_.setColumn(2, chair_root.rotZ());
		interaction_frame.setRot(rotmat_); interaction_frame.setTrn(mean_pos_l);
	}
	return has_frame;
}
bool EX2_motionplane::findMeanPosNormal_const(std::vector<gVec3> pos_datas, std::vector<gVec3> normal_datas, double x_u, double x_l, double y_u, double y_l, double z_u, double z_l, gXMat chair_root, gXMat& interaction_frame)
{

	int n_clip_frames_forPlane = pos_datas.size();

	gVec3 mean_normal_l = gVec3(0, 0, 0);
	gVec3 mean_pos_l = gVec3(0, 0, 0);
	int n_interaction_normal = 0;
	for (int p = 0; p < n_clip_frames_forPlane; p++) {
		// get points
		gVec3 p0 = pos_datas[p];
		// get normal
		gVec3 normal = normal_datas[p];

		// inverse points
		p0 = chair_root.invMultVec4(p0);
		normal = chair_root.invMultVec3(normal);

		if (normal.z() < z_u && normal.z() > z_l &&
			normal.y() < y_u && normal.y() > y_l &&
			normal.x() < x_u && normal.x() > x_l) {
			mean_normal_l += normal;
			mean_pos_l += p0;
			n_interaction_normal++;
		}
	}
	mean_normal_l = (mean_normal_l /= n_interaction_normal); mean_normal_l.normalize();
	mean_pos_l = mean_pos_l /= n_interaction_normal;  

	mean_normal_l = chair_root.multVec3(mean_normal_l);
	mean_pos_l = chair_root.multVec4(mean_pos_l);

	gVec3 x_vec = mean_normal_l % chair_root.rotZ(); x_vec.normalize();


	bool has_frame = false;
	if (n_interaction_normal < 1)
		has_frame = false;
	else {
		
		has_frame = true;
		gRotMat rotmat_; rotmat_.setColumn(0, x_vec); rotmat_.setColumn(1, mean_normal_l); rotmat_.setColumn(2, chair_root.rotZ());
		interaction_frame.setRot(rotmat_); interaction_frame.setTrn(mean_pos_l);
	}
	return has_frame;
}
bool EX2_motionplane::findMeanPosNormal_const(std::vector<gVec3> pos_datas, double x_u, double x_l, double y_u, double y_l, double z_u, double z_l, double ph_u, double ph_l, gXMat chair_root, gXMat& interaction_frame)
{
	int n_clip_frames_forPlane = pos_datas.size() - 3 + 1;

	gVec3 mean_normal_l = gVec3(0, 0, 0);
	gVec3 mean_pos_l = gVec3(0, 0, 0);
	int n_interaction_normal = 0;
	for (int p = 0; p < n_clip_frames_forPlane; p++) {
		//left hand 3 positions
		gVec3 p0 = pos_datas[p + 0];
		gVec3 p1 = pos_datas[p + 1];
		gVec3 p2 = pos_datas[p + 2];

		gVec3 vec_1 = p1 - p0; vec_1.normalize();
		gVec3 vec_2 = p2 - p0; vec_2.normalize();
		gVec3 normal = vec_1 % vec_2; normal.normalize();

		normal = chair_root.invMultVec3(normal);
		gVec3 pos_mean = (p0 + p1 + p2)/=3;
		if (normal.z() < z_u && normal.z() > z_l &&
			normal.y() < y_u && normal.y() > y_l &&
			normal.x() < x_u && normal.x() > x_l &&
			pos_mean.y() < ph_u && pos_mean.y() >ph_l) {
			mean_normal_l += normal;
			mean_pos_l += pos_mean;
			n_interaction_normal++;
		}
	}
	mean_normal_l = (mean_normal_l /= n_interaction_normal); mean_normal_l.normalize();
	mean_pos_l = (mean_pos_l /= n_interaction_normal);

	mean_normal_l = chair_root.multVec3(mean_normal_l);

	gVec3 x_vec = mean_normal_l % chair_root.rotZ(); x_vec.normalize();


	bool has_frame = false;
	if (n_interaction_normal < 1)
		has_frame = false;
	else {
		has_frame = true;
		gRotMat rotmat_; rotmat_.setColumn(0, x_vec); rotmat_.setColumn(1, mean_normal_l); rotmat_.setColumn(2, chair_root.rotZ());
		interaction_frame.setRot(rotmat_); interaction_frame.setTrn(mean_pos_l);
	}
	return has_frame;
}

bool EX2_motionplane::findMeanPosNormal_const_spine(std::vector<gVec3> pos_datas, std::vector<gVec3> normal_datas, double x_u, double x_l, double y_u, double y_l, double z_u, double z_l, gVec3 hip, gXMat chair_root, gXMat& interaction_frame)
{
	int n_clip_frames_forPlane = pos_datas.size();

	gVec3 mean_normal_l = gVec3(0, 0, 0);
	gVec3 mean_pos_l = gVec3(0, 0, 0);
	int n_interaction_normal = 0;

	for (int p = 0; p < n_clip_frames_forPlane; p++) {
		//left hand 3 positions
		gVec3 p0 = pos_datas[p];

		gVec3 dir = normal_datas[p]; 

		gVec3 normal = chair_root.invMultVec3(dir);
		
		if (normal.z() < z_u && normal.z() > z_l &&
			normal.y() < y_u && normal.y() > y_l &&
			normal.x() < x_u && normal.x() > x_l
			) {
			mean_normal_l += chair_root.multVec3(normal);
			mean_pos_l += chair_root.multVec4(p0);
			n_interaction_normal++;
		}

	}
	mean_normal_l = (mean_normal_l)/=n_interaction_normal;
	mean_pos_l = (mean_pos_l)/=n_interaction_normal;

	gVec3 backrest_normal = mean_normal_l; backrest_normal.normalize();
	gVec3 vec_forward = backrest_normal % chair_root.rotX(); vec_forward.normalize();

	bool has_frame = false;
	if (n_interaction_normal < 1)
		has_frame = false;
	else {
		has_frame = true;
		gRotMat rotmat_; rotmat_.setColumn(0,backrest_normal); rotmat_.setColumn(1, vec_forward); rotmat_.setColumn(2, chair_root.rotX());
		interaction_frame.setRot(rotmat_); interaction_frame.setTrn(mean_pos_l);
	}
	return has_frame;
}
bool EX2_motionplane::findMeanPosNormal_const_toq(std::vector<gVec3> pos_datas, double const_torq, gVec3 torqued_joint, gVec3 vec_forward, gXMat& interaction_frame)
{
	int n_clip_frames_forPlane = pos_datas.size()- 3 + 1;

	gVec3 mean_normal_l = gVec3(0, 0, 0);
	gVec3 mean_pos_l = gVec3(0, 0, 0);
	int n_interaction_normal = 0;

	for (int p = 0; p < n_clip_frames_forPlane; p++) {
		//left hand 3 positions
		gVec3 p0 = pos_datas[p + 0];
		gVec3 p1 = pos_datas[p + 1];
		gVec3 p2 = pos_datas[p + 2];

		gVec3 vec_1 = p1 - p0; vec_1.normalize();
		gVec3 vec_2 = p2 - p0; vec_2.normalize();
		gVec3 normal = vec_1 % vec_2; normal.normalize();
		double degree = (normal, gVec3(0, 1, 0)); degree = acos(degree) * gRTD;

		gVec3 gravity = gVec3(0, -9.8, 0); gravity.normalize();
		gVec3 vec = (p1 - torqued_joint); vec.normalize();
		double toq1 = (vec % gravity).magnitude();
		vec = (p2 - torqued_joint); vec.normalize();
		double toq2 = (vec % gravity).magnitude();
		vec = (p0 - torqued_joint); vec.normalize();
		double toq3 = (vec % gravity).magnitude();
		toq1 = (toq1 + toq2 + toq3) / 3;
		if (degree < 15 && degree >-10 &&
			abs(toq1) > const_torq
			) {
			mean_normal_l += normal;
			mean_pos_l += (p0 + p1 + p2);
			n_interaction_normal++;
		}
		

	}
	mean_normal_l = (mean_normal_l /= n_interaction_normal); mean_normal_l.normalize();
	mean_pos_l = (mean_pos_l /= n_interaction_normal * 3);

	gVec3 x_vec = mean_normal_l % vec_forward; x_vec.normalize();

	bool has_frame = false;
	if (n_interaction_normal < 1)
		has_frame = false;
	else {
		has_frame = true;
		gRotMat rotmat_; rotmat_.setColumn(0, x_vec); rotmat_.setColumn(1, mean_normal_l); rotmat_.setColumn(2, vec_forward);
		interaction_frame.setRot(rotmat_); interaction_frame.setTrn(mean_pos_l);
	}
	return has_frame;
}


// init scene 
void EX2_motionplane::SetupScene(char* srcfilename, char* srcCharactertxt, char*outfilefoldername)
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
	extractKeyPose(g_refCoord, g_keypose_Indices);
	// extract Matrix
	extractChairRoot(g_refCoord, g_keypose_Indices,g_interaction_states,g_gMat_chairs);
	// extract Interaction plane
	//extractInteractPlanes(g_refCoord, g_interaction_states, g_keypose_Indices, g_gMat_chairs);
	// save BVH files of key pose
	//exportKeyMotionClipsBVH(g_refCoord, outfilefoldername);
	// save TXT files of environment planes
	//exportKeyEnvironmentInfoTXT(g_interact_planes, outfilefoldername);
	
}

// update scene is updated character by frame
void EX2_motionplane::UpdateScene(int even_simulationTime, int even_iter)
{
	//
	g_viewer->frame(even_simulationTime);
	
	
	g_osg_debug->removeChildren(0, g_osg_debug->getNumChildren());
	drawFrame(g_gMat_chairs[even_iter], g_osg_debug);

		/*gOSGShape::setColor(osg::Vec4(1.0, 0.0, 1.0, 0.1));
		if (g_interact_planes[k].seat.exist == true)
			g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_planes[k].seat.pos_00), gVtoOsg(g_interact_planes[k].seat.pos_01), gVtoOsg(g_interact_planes[k].seat.pos_10), gVtoOsg(g_interact_planes[k].seat.pos_11)));
		gOSGShape::setColor(osg::Vec4(1.0, 0.0, 1.0, 0.1));
		if (g_interact_planes[k].backrest.exist == true)
			g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_planes[k].backrest.pos_00), gVtoOsg(g_interact_planes[k].backrest.pos_01), gVtoOsg(g_interact_planes[k].backrest.pos_10), gVtoOsg(g_interact_planes[k].backrest.pos_11)));
		gOSGShape::setColor(osg::Vec4(0.0, 1.0, 0.0, 0.1));
		if (g_interact_planes[k].armrest_L.exist == true)
			g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_planes[k].armrest_L.pos_00), gVtoOsg(g_interact_planes[k].armrest_L.pos_01), gVtoOsg(g_interact_planes[k].armrest_L.pos_10), gVtoOsg(g_interact_planes[k].armrest_L.pos_11)));
		if (g_interact_planes[k].armrest_R.exist == true)
			g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_planes[k].armrest_R.pos_00), gVtoOsg(g_interact_planes[k].armrest_R.pos_01), gVtoOsg(g_interact_planes[k].armrest_R.pos_10), gVtoOsg(g_interact_planes[k].armrest_R.pos_11)));
		gOSGShape::setColor(osg::Vec4(0.0, 0.0, 1.0, 0.1));
		if (g_interact_planes[k].handrest_L.exist == true)
			g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_planes[k].handrest_L.pos_00), gVtoOsg(g_interact_planes[k].handrest_L.pos_01), gVtoOsg(g_interact_planes[k].handrest_L.pos_10), gVtoOsg(g_interact_planes[k].handrest_L.pos_11)));
		if (g_interact_planes[k].handrest_R.exist == true)
			g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_planes[k].handrest_R.pos_00), gVtoOsg(g_interact_planes[k].handrest_R.pos_01), gVtoOsg(g_interact_planes[k].handrest_R.pos_10), gVtoOsg(g_interact_planes[k].handrest_R.pos_11)));
		gOSGShape::setColor(osg::Vec4(0.0, 1.0, 1.0, 0.1));
		if (g_interact_planes[k].footrest_L.exist == true)
			g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_planes[k].footrest_L.pos_00), gVtoOsg(g_interact_planes[k].footrest_L.pos_01), gVtoOsg(g_interact_planes[k].footrest_L.pos_10), gVtoOsg(g_interact_planes[k].footrest_L.pos_11)));
		if (g_interact_planes[k].footrest_R.exist == true)
			g_osg_debug->addChild(gOSGShape::createPlane(gVtoOsg(g_interact_planes[k].footrest_R.pos_00), gVtoOsg(g_interact_planes[k].footrest_R.pos_01), gVtoOsg(g_interact_planes[k].footrest_R.pos_10), gVtoOsg(g_interact_planes[k].footrest_R.pos_11)));*/
	
	
	int keypose = g_keypose_Indices[even_iter];
	updateCharacterFrame(keypose);
	g_srcVis->update();
	
}