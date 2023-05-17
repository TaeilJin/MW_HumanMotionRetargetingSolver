

#include "MBS/MBSLoader.h"
#include "mgPoseTransfer_IK.h"
#include "saveBVH.h"

#ifdef  UNITY_MW_DLL_TEST_EXPORTS
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT __declspec(dllimport)
#endif //  DLL_TEST_EXPORTS


double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);
mgPoseIKSolver* g_poseTrans; // Xsense->Ybot
mgPoseIKSolver* g_poseTrans_IK; // Ybot
mgPoseIKSolver* g_poseTrans_Avatar; // Ybot -> Avatar

//-- retarget Xsense->Ybot
gMultibodySystem* mbs_src;
gMultibodySystem* mbs_tar;

//-- Ybot
gMultibodySystem* mbs_human;
gMultibodySystem* mbs_human_pre;
//-- retarget Ybot -> Avatar
gMultibodySystem* mbs_avatar;


void SetJntRotDirOBJ(mgPoseIKSolver* poseTrans, char* txt_id, char* src_jnt, char* tar_jnt) {
	
	if(tar_jnt == "Head")
		poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0.1));
	else
		poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
}
void SetJntRotDirOBJ(mgPoseIKSolver* poseTrans, char* txt_id, char* src_jnt, gVec3 s_X, gVec3 s_Z, char* tar_jnt, gVec3 t_X, gVec3 t_Z) {
	gLink* srcLink = poseTrans->src->findLink(src_jnt);
	gLink* tarLink = poseTrans->tar->findLink(tar_jnt);

	if (srcLink == NULL || tarLink == NULL)
		std::cout << "you should check joint names" << std::endl;
	else {
		poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
		char p_x[45]; char p_z[45]; char p_y[45];
		strcpy(p_x, txt_id); strcat(p_x, "_x");
		strcpy(p_y, txt_id); strcat(p_y, "_y");
		strcpy(p_z, txt_id); strcat(p_z, "_z");
		//poseTrans->addPoint(p_x, *poseTrans->src->findLink(src_jnt), s_X, *poseTrans->tar->findLink(tar_jnt), t_X);
		//poseTrans->addPoint(p_y, *poseTrans->src->findLink(src_jnt), gVec3(0, 1, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 1, 0));
		//poseTrans->addPoint(p_z, *poseTrans->src->findLink(src_jnt), s_Z, *poseTrans->tar->findLink(tar_jnt), t_Z);

		poseTrans->addPoint(p_x, *poseTrans->src->findLink(src_jnt), poseTrans->src->findLink(src_jnt)->frame().rotX(), 
			*poseTrans->tar->findLink(tar_jnt), poseTrans->tar->findLink(tar_jnt)->frame().rotX());
		poseTrans->addPoint(p_y, *poseTrans->src->findLink(src_jnt), poseTrans->src->findLink(src_jnt)->frame().rotY(),
			*poseTrans->tar->findLink(tar_jnt), poseTrans->tar->findLink(tar_jnt)->frame().rotY());
		poseTrans->addPoint(p_z, *poseTrans->src->findLink(src_jnt), poseTrans->src->findLink(src_jnt)->frame().rotZ(),
			*poseTrans->tar->findLink(tar_jnt), poseTrans->tar->findLink(tar_jnt)->frame().rotZ());



		poseTrans->addDirectionObjective(txt_id, p_x, 1.0);
		poseTrans->addDirectionObjective(txt_id, p_z, 1.0);
		poseTrans->addDirectionObjective(txt_id, p_y, 1.0);
	}
}
void SetJntRotDirOBJ(mgPoseIKSolver* poseTrans, char* txt_id, char* src_jnt, gVec3 s_X, gVec3 s_Z, gVec3 s_Y, char* tar_jnt, gVec3 t_X, gVec3 t_Z, gVec3 t_Y) {
	gLink* srcLink = poseTrans->src->findLink(src_jnt);
	gLink* tarLink = poseTrans->tar->findLink(tar_jnt);

	if (srcLink == NULL || tarLink == NULL)
		std::cout << "you should check joint names" << std::endl;
	else {
		poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
		char p_x[45]; char p_z[45]; char p_y[45];
		strcpy(p_x, txt_id); strcat(p_x, "_x");
		strcpy(p_y, txt_id); strcat(p_y, "_y");
		strcpy(p_z, txt_id); strcat(p_z, "_z");
		poseTrans->addPoint(p_x, *poseTrans->src->findLink(src_jnt), s_X, *poseTrans->tar->findLink(tar_jnt), t_X);
		poseTrans->addPoint(p_y, *poseTrans->src->findLink(src_jnt), s_Y, *poseTrans->tar->findLink(tar_jnt), t_Y);
		poseTrans->addPoint(p_z, *poseTrans->src->findLink(src_jnt), s_Z, *poseTrans->tar->findLink(tar_jnt), t_Z);

		poseTrans->addDirectionObjective(txt_id, p_x, 1.0);
		poseTrans->addDirectionObjective(txt_id, p_z, 1.0);
		poseTrans->addDirectionObjective(txt_id, p_y, 1.0);
	}
}

extern "C"
{
	DLL_EXPORT int INIT_RETARGET(gMultibodySystem* src, gMultibodySystem* tar) {
		
		mbs_src = new gMultibodySystem();
		mbs_src = src;

		mbs_tar = new gMultibodySystem();
		mbs_tar = tar;

		g_poseTrans_Avatar = new mgPoseIKSolver(mbs_src, mbs_tar);
		g_poseTrans_Avatar->scale = 1.0;

		return g_poseTrans_Avatar->tar->numLinks();
	}
	DLL_EXPORT int MAPPING_JOINTS(const char* mapped_name, const char* src_name, const char* tar_name) {

		//g_poseTrans_Avatar->addPoint(mapped_name, *g_poseTrans_Avatar->src->findLink(src_name), gVec3(0, 0, 0), *g_poseTrans_Avatar->tar->findLink(tar_name), gVec3(0, 0, 0));
		SetJntRotDirOBJ(g_poseTrans_Avatar, (char*)mapped_name, (char*)src_name, (char*)tar_name);
		//std::cout << (char*)src_name << (char*)tar_name << " tarpoints " << g_poseTrans_Avatar->tarPoints[0].body() << std::endl;
		//std::cout<<" tarpoints " << g_poseTrans_Avatar->tarPoints[0].body() << std::endl;
		//g_poseTrans_Avatar->tarPoints[0].updateKinematicsUptoPos();
		return g_poseTrans_Avatar->tarPoints.size();
	}
	DLL_EXPORT int MAPPING_JOINTS_withAXIS(const char* mapped_name, const char* src_name, const char* tar_name,
		gVec3 g_src_X, gVec3 g_src_Y, gVec3 g_src_Z,
		gVec3 g_tar_X, gVec3 g_tar_Y, gVec3 g_tar_Z) {

		//g_poseTrans_Avatar->addPoint(mapped_name, *g_poseTrans_Avatar->src->findLink(src_name), gVec3(0, 0, 0), *g_poseTrans_Avatar->tar->findLink(tar_name), gVec3(0, 0, 0));
		//SetJntRotDirOBJ(g_poseTrans_Avatar, (char*)mapped_name, (char*)src_name, (char*)tar_name);
		gVec3 src_x = mbs_src->findLink((char*)src_name)->frame().rotX();
		gVec3 src_y = mbs_src->findLink((char*)src_name)->frame().rotY();
		gVec3 src_z = mbs_src->findLink((char*)src_name)->frame().rotZ();

		//std::cout << (char*)src_name << (char*)tar_name << " src_x " <<  src_x << " tar rot x " << mat_tar.rotX() << " tar_x" << mat_tar.invMultVec3(src_x) << std::endl;
		//std::cout << (char*)src_name << (char*)tar_name << " src_y " << src_y << mat_tar.invMultVec3(src_y) << std::endl;
		//std::cout << (char*)src_name << (char*)tar_name << " src_z " << src_z << mat_tar.invMultVec3(src_z) << std::endl;

		SetJntRotDirOBJ(g_poseTrans_Avatar, (char*)mapped_name,
			(char*)src_name, g_src_X, g_src_Z, g_src_Y,
			(char*)tar_name, g_tar_X, g_tar_Z, g_tar_Y);

		//std::cout << (char*)src_name << (char*)tar_name << " tarpoints " << g_poseTrans_Avatar->tarPoints[0].body() << std::endl;
		//std::cout<<" tarpoints " << g_poseTrans_Avatar->tarPoints[0].body() << std::endl;
		//g_poseTrans_Avatar->tarPoints[0].updateKinematicsUptoPos();
		return g_poseTrans_Avatar->tarPoints.size();
	}
	DLL_EXPORT int MAPPING_DIRS(const char* j_start, const char* j_to, float weightDir) {
		//direction objectives
		g_poseTrans_Avatar->addDirectionObjective(j_start, j_to, weightDir);
		std::cout << " directions " << g_poseTrans_Avatar->directions.size() << " WEIGHT " << weightDir<< " " << j_start << j_to<< g_poseTrans_Avatar->namePoints[0] << std::endl;
		return 1;
	}
	DLL_EXPORT int MAPPING_POS(const char* j_to, float weightPos) {
		//pelvis position
		g_poseTrans_Avatar->addDesiredObjective(j_to, weightPos, gVec3(0, 0, 0));
		return 1;
	}
	DLL_EXPORT gMultibodySystem* DO_RETARGET_OUTPUT(float y_offset) {
		//
		g_poseTrans_Avatar->src->updateKinematicsUptoPos();
		
		// desired positions update
		for (int p = 0; p < g_poseTrans_Avatar->srcPoints.size(); p++) {

			g_poseTrans_Avatar->tarPoints[p].updateKinematicsUptoPos();
			g_poseTrans_Avatar->srcPoints[p].updateKinematicsUptoPos();

		}

		//motion retargeting
		g_poseTrans_Avatar->desiredPoints[0].pos_desired = g_poseTrans_Avatar->scale * g_poseTrans_Avatar->src->link(0)->frame().trn(); //g_poseTrans->scale
		

		gXMat offset; offset.setTrn(0, 0, 0);
		g_poseTrans_Avatar->transferPoseLevMar(offset);
		g_poseTrans_Avatar->tar->updateKinematicsUptoPos();

		gVec3 new_pelvis = g_poseTrans_Avatar->tar->link(0)->frame().trn();
		new_pelvis.setY(new_pelvis.y() + y_offset); //std::abs(g_poseTrans->save_f_height)
		g_poseTrans_Avatar->tar->setBasePosition(new_pelvis);
		g_poseTrans_Avatar->tar->updateKinematicsUptoPos();

		return g_poseTrans_Avatar->tar;
	}

	/* OPTIMIZATION INVERSE KINEMATICS */
	DLL_EXPORT int LevmarIK(gMultibodySystem* mbs) {
		mbs_human = new gMultibodySystem();
		mbs_human = mbs;

		mbs_human_pre = new gMultibodySystem();
		mbs_human_pre = mbs;
		// 
		g_poseTrans_IK = new mgPoseIKSolver(mbs, mbs);

		return 1;
	}
	DLL_EXPORT int SETTING_DESJOINTS(const char* mapped_name, const char* src_name) {

		g_poseTrans_IK->addDesiredJoint(mapped_name, *mbs_human->findLink(src_name), gVec3(0, 0, 0));

		return g_poseTrans_IK->desiredPoints.size();
	}
	DLL_EXPORT gMultibodySystem* DO_POSE(std::vector<gVec3> desireds) {
		//
		gXMat offset;
		mbs_human->setBasePosition(desireds[0]);
		mbs_human->updateKinematicsUptoPos();
		g_poseTrans_IK->transferDesiredPoseLevMar(offset, desireds);

		arma::vec lca(mbs_human->dof());
		mbs_human->getCompactCoordArray(lca);
		mbs_human_pre->setFromCompactCoordArray(lca);
		mbs_human_pre->updateKinematicsUptoPos();

		return g_poseTrans_IK->tar;
	}

	/*--- */

	DLL_EXPORT int INITRETARGET_AVATAR(gMultibodySystem* src, gMultibodySystem* tar) {

		//mbs_src = new gMultibodySystem();
		//mbs_human = src;

		mbs_avatar = new gMultibodySystem();
		mbs_avatar = tar;

		double maxHeight = mbs_human->findLink("mixamorig:Head")->frame().trn().y() - mbs_human->findLink("mixamorig:Hips")->frame().trn().y();
		double maxHeightT = mbs_avatar->findLink("Head")->frame().trn().y() - mbs_avatar->findLink("Hips")->frame().trn().y();
		double scale = maxHeightT / maxHeight;

		g_poseTrans_Avatar = new mgPoseIKSolver(mbs_human, mbs_avatar);
		g_poseTrans_Avatar->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		SetJntRotDirOBJ(g_poseTrans_Avatar, "t0", "mixamorig:Hips", "Hips");// gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "t1", "mixamorig:Spine", "Spine");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "t2", "mixamorig:Spine1","Spine1");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "t3", "mixamorig:Spine2", "Spine2");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "t4", "mixamorig:Neck", "Neck");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "t5", "mixamorig:Head", "Head");

		//left leg chain
		SetJntRotDirOBJ(g_poseTrans_Avatar, "la0", "mixamorig:LeftUpLeg", "LeftUpLeg");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "la1", "mixamorig:LeftLeg", "LeftLeg");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "la2", "mixamorig:LeftFoot", "LeftFoot");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "la3", "mixamorig:LeftToeBase", "LeftToeBase");

		//right leg chain
		SetJntRotDirOBJ(g_poseTrans_Avatar, "ra0", "mixamorig:RightUpLeg", "RightUpLeg");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "ra1", "mixamorig:RightLeg", "RightLeg");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "ra2", "mixamorig:RightFoot", "RightFoot");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "ra3", "mixamorig:RightToeBase", "RightToeBase");


		//left arm chain
		SetJntRotDirOBJ(g_poseTrans_Avatar, "ll0", "mixamorig:LeftShoulder", "LeftShoulder");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "ll1", "mixamorig:LeftArm", "LeftArm");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "ll2", "mixamorig:LeftForeArm", "LeftForeArm");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "ll3", "mixamorig:LeftHand", "LeftHand");

		//right arm chain
		SetJntRotDirOBJ(g_poseTrans_Avatar, "rl0", "mixamorig:RightShoulder", "RightShoulder");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "rl1", "mixamorig:RightArm", "RightArm");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "rl2", "mixamorig:RightForeArm", "RightForeArm");
		SetJntRotDirOBJ(g_poseTrans_Avatar, "rl3", "mixamorig:RightHand", "RightHand");


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


		//direction objectives
		g_poseTrans_Avatar->addDirectionObjective("t0", "t1", weightDir);
		g_poseTrans_Avatar->addDirectionObjective("t1", "t2", weightDir);
		g_poseTrans_Avatar->addDirectionObjective("t2", "t3", weightDir);
		g_poseTrans_Avatar->addDirectionObjective("t3", "t4", weightDir);
		g_poseTrans_Avatar->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		g_poseTrans_Avatar->addDirectionObjective("la0", "la1", weightDir * 5.0);
		g_poseTrans_Avatar->addDirectionObjective("la1", "la2", weightDir * 5.0);
		g_poseTrans_Avatar->addDirectionObjective("la2", "la3", weightDir*5.0);

		//right leg chain
		g_poseTrans_Avatar->addDirectionObjective("ra0", "ra1", weightDir * 5.0);
		g_poseTrans_Avatar->addDirectionObjective("ra1", "ra2", weightDir * 5.0);
		g_poseTrans_Avatar->addDirectionObjective("ra2", "ra3", weightDir*5.0);

		//right arm chain
		g_poseTrans_Avatar->addDirectionObjective("rl0", "rl1", weightDir);
		g_poseTrans_Avatar->addDirectionObjective("rl1", "rl2", weightDir);
		g_poseTrans_Avatar->addDirectionObjective("rl2", "rl3", weightDir);

		//right arm chain
		g_poseTrans_Avatar->addDirectionObjective("ll0", "ll1", weightDir);
		g_poseTrans_Avatar->addDirectionObjective("ll1", "ll2", weightDir);
		g_poseTrans_Avatar->addDirectionObjective("ll2", "ll3", weightDir);

		//pelvis position
		g_poseTrans_Avatar->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

		return 1;
	}
	
	DLL_EXPORT gMultibodySystem* DO_RETARGETING_AVATAR(float y_offset) {
		//
		// desired positions update
		for (int p = 0; p < g_poseTrans_Avatar->srcPoints.size(); p++) {

			g_poseTrans_Avatar->tarPoints[p].updateKinematicsUptoPos();
			g_poseTrans_Avatar->srcPoints[p].updateKinematicsUptoPos();

		}

		//motion retargeting
		g_poseTrans_Avatar->desiredPoints[0].pos_desired = g_poseTrans_Avatar->scale * mbs_human->link(0)->frame().trn(); //g_poseTrans->scale
		gXMat offset; offset.setTrn(0, 0, 0);
		g_poseTrans_Avatar->transferPoseLevMar(offset);
		mbs_avatar->updateKinematicsUptoPos();

		gVec3 new_pelvis = mbs_human->link(0)->frame().trn();
		new_pelvis.setY(new_pelvis.y() + y_offset); //std::abs(g_poseTrans->save_f_height)
		mbs_avatar->setBasePosition(new_pelvis);
		mbs_avatar->updateKinematicsUptoPos();

		return g_poseTrans_Avatar->tar;
	}

	DLL_EXPORT int INIT_IK_AVATAR(gMultibodySystem* mbs) {
		mbs_avatar = new gMultibodySystem();
		mbs_avatar = mbs;
		// 
		g_poseTrans_Avatar = new mgPoseIKSolver(mbs, mbs);
		g_poseTrans_Avatar->addDesiredJoint("t0", *mbs->findLink("Hips"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t1", *mbs->findLink("Spine"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t2", *mbs->findLink("Spine1"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t3", *mbs->findLink("Spine2"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t4", *mbs->findLink("Neck"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t5", *mbs->findLink("Head"), gVec3(0, 0, 0));

		g_poseTrans_Avatar->addDesiredJoint("t6", *mbs->findLink("LeftShoulder"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t7", *mbs->findLink("LeftArm"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t8", *mbs->findLink("LeftForeArm"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t9", *mbs->findLink("LeftHand"), gVec3(0, 0, 0));

		g_poseTrans_Avatar->addDesiredJoint("t10", *mbs->findLink("RightShoulder"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t11", *mbs->findLink("RightArm"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t12", *mbs->findLink("RightForeArm"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t13", *mbs->findLink("RightHand"), gVec3(0, 0, 0));

		g_poseTrans_Avatar->addDesiredJoint("t14", *mbs->findLink("RightUpLeg"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t15", *mbs->findLink("RightLeg"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t16", *mbs->findLink("RightFoot"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t17", *mbs->findLink("RightToeBase"), gVec3(0, 0, 0));

		g_poseTrans_Avatar->addDesiredJoint("t18", *mbs->findLink("LeftUpLeg"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t19", *mbs->findLink("LeftLeg"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t20", *mbs->findLink("LeftFoot"), gVec3(0, 0, 0));
		g_poseTrans_Avatar->addDesiredJoint("t21", *mbs->findLink("LeftToeBase"), gVec3(0, 0, 0));


		return 1;
	}

	DLL_EXPORT gMultibodySystem* DO_POSE_AVATAR(std::vector<gVec3> desireds) {
		//
		gXMat offset;
		mbs_avatar->setBasePosition(desireds[0]);
		mbs_avatar->updateKinematicsUptoPos();
		g_poseTrans_Avatar->transferDesiredPoseLevMar(offset, desireds);
		mbs_avatar->updateKinematicsUptoPos();
		return g_poseTrans_Avatar->tar;
	}


	DLL_EXPORT int INITRETARGET(gMultibodySystem* src, gMultibodySystem* tar) {
		
		mbs_src = new gMultibodySystem();
		mbs_src = src;

		mbs_tar = new gMultibodySystem();
		mbs_tar = tar;

		double maxHeight = mbs_src->findLink("Head")->frame().trn().y() - mbs_src->findLink("Hips")->frame().trn().y();
		double maxHeightT = mbs_tar->findLink("mixamorig:Head")->frame().trn().y() - mbs_tar->findLink("mixamorig:Hips")->frame().trn().y();
		double scale = maxHeightT / maxHeight;

		g_poseTrans = new mgPoseIKSolver(mbs_src, mbs_tar);
		g_poseTrans->scale = scale;
		// you need to manually match the src chracter joint name and corresponding tar character joint name
		//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
		SetJntRotDirOBJ(g_poseTrans, "t0", "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "t1", "Chest", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "t2", "Chest2", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "t3", "Chest3", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//left leg chain
		SetJntRotDirOBJ(g_poseTrans, "la0", "LeftHip", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "la1", "LeftKnee", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "la2", "LeftAnkle", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "la2", "LeftToe", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right leg chain
		SetJntRotDirOBJ(g_poseTrans, "ra0", "RightHip", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "ra1", "RightKnee", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "ra2", "RightAnkle", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "la2", "RightToe", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));


		//left arm chain
		SetJntRotDirOBJ(g_poseTrans, "ll0", "LeftCollar", "mixamorig:LeftShoulder");
		SetJntRotDirOBJ(g_poseTrans, "ll1", "LeftShoulder", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "ll2", "LeftElbow", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "ll3", "LeftWrist", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10));

		//right arm chain
		SetJntRotDirOBJ(g_poseTrans, "rl0", "RightCollar", "mixamorig:RightShoulder");
		SetJntRotDirOBJ(g_poseTrans, "rl1", "RightShoulder", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "rl2", "RightElbow", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
		SetJntRotDirOBJ(g_poseTrans, "rl3", "RightWrist", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10));


		double weightDir = 1.;//importance of direction vector in pose transfer
		double weightPos = 1.01;//importance of end-effector orientation in pose transfer


		//direction objectives
		g_poseTrans->addDirectionObjective("t0", "t1", weightDir);
		g_poseTrans->addDirectionObjective("t1", "t2", weightDir);
		g_poseTrans->addDirectionObjective("t2", "t3", weightDir);
		g_poseTrans->addDirectionObjective("t3", "t4", weightDir);
		g_poseTrans->addDirectionObjective("t4", "t5", weightDir);

		//left leg chain 
		g_poseTrans->addDirectionObjective("la0", "la1", weightDir * 5.0);
		g_poseTrans->addDirectionObjective("la1", "la2", weightDir * 5.0);
		//poseTrans->addDirectionObjective("la2", "la3", weightDir*5.0);

		//right leg chain
		g_poseTrans->addDirectionObjective("ra0", "ra1", weightDir * 5.0);
		g_poseTrans->addDirectionObjective("ra1", "ra2", weightDir * 5.0);
		//poseTrans->addDirectionObjective("ra2", "ra3", weightDir*5.0);

		//right arm chain
		g_poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
		g_poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
		g_poseTrans->addDirectionObjective("rl2", "rl3", weightDir);

		//right arm chain
		g_poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
		g_poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
		g_poseTrans->addDirectionObjective("ll2", "ll3", weightDir);

		//pelvis position
		g_poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

		return 1;
	}

	DLL_EXPORT gMultibodySystem* DO_RETARGETING(float y_offset) {
		//
		// desired positions update
		for (int p = 0; p < g_poseTrans->srcPoints.size(); p++) {

			g_poseTrans->tarPoints[p].updateKinematicsUptoPos();
			g_poseTrans->srcPoints[p].updateKinematicsUptoPos();

		}

		//motion retargeting
		g_poseTrans->desiredPoints[0].pos_desired = g_poseTrans->scale * mbs_src->link(0)->frame().trn(); //g_poseTrans->scale
		gXMat offset; offset.setTrn(0, 0, 0);
		g_poseTrans->transferPoseLevMar(offset);
		mbs_tar->updateKinematicsUptoPos();
		
		gVec3 new_pelvis = mbs_tar->link(0)->frame().trn();
		new_pelvis.setY(new_pelvis.y()+ y_offset); //std::abs(g_poseTrans->save_f_height)
		mbs_tar->setBasePosition(new_pelvis);
		mbs_tar->updateKinematicsUptoPos();
		
		return g_poseTrans->tar;
	}


	//DLL_EXPORT int INIT_IK(gMultibodySystem* mbs) {
	//	mbs_human = new gMultibodySystem();
	//	mbs_human = mbs;

	//	mbs_human_pre = new gMultibodySystem();
	//	mbs_human_pre = mbs;
	//	// 
	//	g_poseTrans_IK = new mgPoseIKSolver(mbs, mbs);
	//	g_poseTrans_IK->addDesiredJoint("t0", *mbs->findLink("mixamorig:Hips"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t1", *mbs->findLink("mixamorig:Spine"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t2", *mbs->findLink("mixamorig:Spine1"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t3", *mbs->findLink("mixamorig:Spine2"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t4", *mbs->findLink("mixamorig:Neck"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t5", *mbs->findLink("mixamorig:Head"), gVec3(0, 0, 0));

	//	g_poseTrans_IK->addDesiredJoint("t6", *mbs->findLink("mixamorig:LeftShoulder"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t7", *mbs->findLink("mixamorig:LeftArm"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t8", *mbs->findLink("mixamorig:LeftForeArm"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t9", *mbs->findLink("mixamorig:LeftHand"), gVec3(0, 0, 0));

	//	g_poseTrans_IK->addDesiredJoint("t10", *mbs->findLink("mixamorig:RightShoulder"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t11", *mbs->findLink("mixamorig:RightArm"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t12", *mbs->findLink("mixamorig:RightForeArm"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t13", *mbs->findLink("mixamorig:RightHand"), gVec3(0, 0, 0));

	//	g_poseTrans_IK->addDesiredJoint("t14", *mbs->findLink("mixamorig:RightUpLeg"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t15", *mbs->findLink("mixamorig:RightLeg"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t16", *mbs->findLink("mixamorig:RightFoot"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t17", *mbs->findLink("mixamorig:RightToeBase"), gVec3(0, 0, 0));

	//	g_poseTrans_IK->addDesiredJoint("t18", *mbs->findLink("mixamorig:LeftUpLeg"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t19", *mbs->findLink("mixamorig:LeftLeg"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t20", *mbs->findLink("mixamorig:LeftFoot"), gVec3(0, 0, 0));
	//	g_poseTrans_IK->addDesiredJoint("t21", *mbs->findLink("mixamorig:LeftToeBase"), gVec3(0, 0, 0));

	//	
	//	return 1;
	//}


	
	//
	std::vector<gVec3> func_constrained_desireds(std::vector<gVec3> desireds) {
		
		float threshold = 1;
		std::vector<gVec3> newPoints = desireds;
		for (int j = 0; j < 22; j++) {
			if (j != 3 && j != 5 && j != 9 && j != 13)
			{
				//gVec3 point = (desireds[j]) - mbs_human_pre->link(j)->frame().trn();
				
				gVec3 point = mbs_human_pre->link(j)->frame().invMultVec4(desireds[j]);
				
				if (point.magnitude() > threshold) {
					point.normalize();
					gVec3 newPoint = mbs_human_pre->link(j)->frame().multVec4((threshold * point));
					newPoints[j] = newPoint;
				}
				
			}
		}
		//j != 0 &&
		return newPoints;
	}
	DLL_EXPORT gMultibodySystem* DO_CONS_POSE(std::vector<gVec3> desireds) {
		//
		gXMat offset;
		
		std::vector<gVec3> newPoints = func_constrained_desireds(desireds);
		mbs_human->setBasePosition(newPoints[0]);
		mbs_human->updateKinematicsUptoPos();
		g_poseTrans->transferDesiredPoseLevMar(offset, newPoints);
		
		arma::vec lca(mbs_human->dof()); 
		mbs_human->getCompactCoordArray(lca);
		mbs_human_pre->setFromCompactCoordArray(lca);
		mbs_human_pre->updateKinematicsUptoPos();

		return mbs_human_pre;
	}

}