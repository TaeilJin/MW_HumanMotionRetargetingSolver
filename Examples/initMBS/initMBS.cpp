
#include <Windows.h>
#include "MBS/MBSLoader.h"
#include "saveBVH.h"

#ifdef  UNITY_MW_DLL_TEST_EXPORTS
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT __declspec(dllimport)
#endif //  DLL_TEST_EXPORTS


double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);

gMultibodySystem* mbs;
gMultibodySystem* mbs_output;
gMultibodySystem* mbs_avatar;
gMultibodySystem* src;


extern "C"
{
	//typedef int(*INIT_IK)(gMultibodySystem* mbs);
	typedef int(*INIT_RETARGET)(gMultibodySystem* src, gMultibodySystem* tar);
	typedef int(*INITRETARGET)(gMultibodySystem* src, gMultibodySystem* tar);
	typedef int(*INITRETARGET_AVATAR)(gMultibodySystem* src, gMultibodySystem* tar);
	typedef int(*INIT_IK_AVATAR)(gMultibodySystem* mbs);
	typedef int(*MAPPING_JOINTS)(const char* mapped_name, const char* src_name, const char* tar_name);
	typedef int(*MAPPING_JOINTS_withAXIS)(const char* mapped_name, const char* src_name, const char* tar_name, 
		gVec3 g_src_X, gVec3 g_src_Y, gVec3 g_src_Z,
		gVec3 g_tar_X, gVec3 g_tar_Y, gVec3 g_tar_Z);
	typedef int(*MAPPING_DIRS)(const char* j_start, const char* j_to, float weightDir);
	typedef int(*MAPPING_POS)( const char* j_to, float weightDir);
	typedef gMultibodySystem* (*DO_RETARGET_OUTPUT)(float y_offset);

	typedef gMultibodySystem* (*DO_POSE)(std::vector<gVec3> desireds);
	typedef gMultibodySystem* (*DO_RETARGETING)(float y_offset);
	typedef gMultibodySystem* (*DO_RETARGETING_AVATAR)(float y_offset);
	typedef gMultibodySystem* (*DO_POSE_AVATAR)(std::vector<gVec3> desireds);

	/* optimization ik */
	typedef int (*LevmarIK)(gMultibodySystem* mbs);

	DLL_EXPORT int INIT_MBS_CHA(LPCSTR chaSrcTXTFile, LPCSTR chaTarTXTFile) {
		MBSLoader tar_loader;
		src = new gMultibodySystem();
		tar_loader.loadModelUnity(chaSrcTXTFile, src, 1.0);
		//for (int j = 0; j < src->numLinks(); j++)
		//	std::cout << " joint " << src->link(j)->name() << " x " << src->link(j)->frame().rotX() << " y " <<
		//	src->link(j)->frame().rotY() << " z " << src->link(j)->frame().rotZ() << std::endl;
		MBSLoader mbs_loader;
		mbs = new gMultibodySystem();
		mbs_loader.loadModelUnity(chaTarTXTFile, mbs, 1.0);
		//for (int j = 0; j < mbs->numLinks(); j++)
		//	std::cout << " joint " << mbs->link(j)->name() << " x " << mbs->link(j)->frame().rotX() << " y " <<
		//	mbs->link(j)->frame().rotY() << " z " << mbs->link(j)->frame().rotZ() << std::endl;
		return mbs->numLinks();
	}
	DLL_EXPORT int SETUP_RETARGET() {
		HMODULE hDll = LoadLibrary(L"poseIK.DLL");
		INIT_RETARGET init_MWSolver;
		init_MWSolver = (INIT_RETARGET)GetProcAddress(hDll, "INIT_RETARGET");
		init_MWSolver(src, mbs);

		return src->numLinks();
	}
	DLL_EXPORT int SETUP_MAPPING_JOINT(LPCSTR mapped_name, LPCSTR SRC_name, LPCSTR TAR_name) {
		HMODULE hDll = LoadLibrary(L"poseIK.DLL");
		MAPPING_JOINTS init_MWSolver;
		init_MWSolver = (MAPPING_JOINTS)GetProcAddress(hDll, "MAPPING_JOINTS");
		
		return init_MWSolver(mapped_name, SRC_name, TAR_name);
	}
	DLL_EXPORT int SETUP_MAPPING_JOINT_withAXIS(LPCSTR mapped_name, LPCSTR SRC_name, LPCSTR TAR_name,
		float* src_X, float* src_Y, float* src_Z,
		float* tar_X, float* tar_Y, float* tar_Z) {
		HMODULE hDll = LoadLibrary(L"poseIK.DLL");
		MAPPING_JOINTS_withAXIS init_MWSolver;
		init_MWSolver = (MAPPING_JOINTS_withAXIS)GetProcAddress(hDll, "MAPPING_JOINTS_withAXIS");

		gVec3 g_src_X(src_X[0], src_X[1], src_X[2]);
		gVec3 g_src_Y(src_Y[0], src_Y[1], src_Y[2]);
		gVec3 g_src_Z(src_Z[0], src_Z[1], src_Z[2]);

		gVec3 g_tar_X(tar_X[0], tar_X[1], tar_X[2]);
		gVec3 g_tar_Y(tar_Y[0], tar_Y[1], tar_Y[2]);
		gVec3 g_tar_Z(tar_Z[0], tar_Z[1], tar_Z[2]);

		return init_MWSolver(mapped_name, SRC_name, TAR_name,
			g_src_X, g_src_Y, g_src_Z,
			g_tar_X, g_tar_Y, g_tar_Z);
	}

	DLL_EXPORT int SETUP_MAPPING_DIR(LPCSTR JNT_START, LPCSTR JNT_TO, float weight) {
		HMODULE hDll = LoadLibrary(L"poseIK.DLL");
		MAPPING_DIRS init_MWSolver;
		init_MWSolver = (MAPPING_DIRS)GetProcAddress(hDll, "MAPPING_DIRS");
		init_MWSolver(JNT_START, JNT_TO, weight);

		return src->numLinks();
	}
	DLL_EXPORT int SETUP_MAPPING_POS(LPCSTR JNT_TO, float weight) {
		HMODULE hDll = LoadLibrary(L"poseIK.DLL");
		MAPPING_POS init_MWSolver;
		init_MWSolver = (MAPPING_POS)GetProcAddress(hDll, "MAPPING_POS");
		init_MWSolver(JNT_TO, weight);

		return src->numLinks();
	}
	DLL_EXPORT void SET_TAR_POSE_MW(float* input_POSE) {
		gVec3 position;
		position.setX(-1 * input_POSE[0]);
		position.setY(input_POSE[1]);
		position.setZ(input_POSE[2]);

		//std::cout << " src numlinks " << src->numLinks() << " " << position<< " " <<input_POSE[0]<<std::endl;
		//joints local rotation
		for (int i = 0; i < mbs->numLinks(); i++) {
			//
			gQuat quat_rH; quat_rH.set(input_POSE[3 + 4 * i + 0], input_POSE[3 + 4 * i + 1], input_POSE[3 + 4 * i + 2], input_POSE[3 + 4 * i + 3]);
			gQuat quat_lH;
			//convert righthand quaternion to lefthand quaternion
			quat_lH.setX(quat_rH.x());
			quat_lH.setY(-1.0 * quat_rH.y());
			quat_lH.setZ(-1.0 * quat_rH.z());
			quat_lH.setW(quat_rH.w());
			quat_lH.normalize();
			mbs->link(i)->setFromSafeCoordArray(quat_lH.cptr());
			//gRotMat rot_LeftHanded = quat_rH.inRotMatrix();
			//gRotMat rot_RightHanded;
			//double* convert_L2R = new double[9];
			//convert_L2R[0] = rot_LeftHanded.e(0);    convert_L2R[3] = -1 * rot_LeftHanded.e(3); convert_L2R[6] = -1 * rot_LeftHanded.e(6);
			//convert_L2R[1] = -1 * rot_LeftHanded.e(1); convert_L2R[4] = rot_LeftHanded.e(4);    convert_L2R[7] = rot_LeftHanded.e(7);
			//convert_L2R[2] = -1 * rot_LeftHanded.e(2); convert_L2R[5] = rot_LeftHanded.e(5);    convert_L2R[8] = rot_LeftHanded.e(8);
			//rot_RightHanded.set(convert_L2R);

			//gXMat mat_G = mbs->link(i)->localFrame();
			//mat_G.setRot(rot_RightHanded);

			////convert righthand quaternion to lefthand quaternion
			//mbs->link(i)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());
			mbs->updateKinematicsUptoPos();
		}

		mbs->setBasePosition(position);
		mbs->updateKinematicsUptoPos();

	}
	DLL_EXPORT void SET_SRC_POSE_MW(float* input_POSE) {
		gVec3 position;
		position.setX(-1 * input_POSE[0]);
		position.setY(input_POSE[1]);
		position.setZ(input_POSE[2]);

		//std::cout << " src numlinks " << src->numLinks() << " " << position<< " " <<input_POSE[0]<<std::endl;
		//joints local rotation
		for (int i = 0; i < src->numLinks(); i++) {
			////
			gQuat quat_rH; quat_rH.set(input_POSE[3 + 4 * i + 0], input_POSE[3 + 4 * i + 1], input_POSE[3 + 4 * i + 2], input_POSE[3 + 4 * i + 3]);
			
			gQuat quat_lH;
			//convert righthand quaternion to lefthand quaternion
			quat_lH.setX(quat_rH.x());
			quat_lH.setY(-1.0 * quat_rH.y());
			quat_lH.setZ(-1.0 * quat_rH.z());
			quat_lH.setW(quat_rH.w());
			quat_lH.normalize();
			src->link(i)->setFromSafeCoordArray(quat_lH.cptr());
			
			//gRotMat rot_LeftHanded = quat_rH.inRotMatrix();
			//gRotMat rot_RightHanded;
			//double* convert_L2R = new double[9];
			//convert_L2R[0] = rot_LeftHanded.e(0);    convert_L2R[3] = -1 * rot_LeftHanded.e(3); convert_L2R[6] = -1 * rot_LeftHanded.e(6);
			//convert_L2R[1] = -1 * rot_LeftHanded.e(1); convert_L2R[4] = rot_LeftHanded.e(4);    convert_L2R[7] = rot_LeftHanded.e(7);
			//convert_L2R[2] = -1 * rot_LeftHanded.e(2); convert_L2R[5] = rot_LeftHanded.e(5);    convert_L2R[8] = rot_LeftHanded.e(8);
			//rot_RightHanded.set(convert_L2R);

			//gXMat mat_G = src->link(i)->localFrame();
			//mat_G.setRot(rot_RightHanded);

			////convert righthand quaternion to lefthand quaternion
			//src->link(i)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());

			src->updateKinematicsUptoPos();
		}

		src->setBasePosition(position);
		src->updateKinematicsUptoPos();

		//for (int j = 0; j < src->numLinks(); j++)
		//	std::cout << " joint " << src->link(j)->name() << " x " << src->link(j)->frame().rotX() << " y " <<
		//	src->link(j)->frame().rotY() << " z " << src->link(j)->frame().rotZ() << std::endl;

	}

	DLL_EXPORT void DO_RETARGET(float Y_offset) {

		//MW solver 상의 아바타도 업데이트
		HMODULE hDll = LoadLibrary(L"poseIK.DLL");

		DO_RETARGET_OUTPUT FullBody_IK;
		FullBody_IK = (DO_RETARGET_OUTPUT)GetProcAddress(hDll, "DO_RETARGET_OUTPUT");

		mbs = FullBody_IK(Y_offset);
		mbs->updateKinematicsUptoPos();
	}

	DLL_EXPORT void OUTPUT_SRC_POSE_UNITY_OUTPUT(float* output_POSE) {
		src->updateKinematicsUptoPos();
		//hip world position
		gVec3 pos = src->link(0)->frame().trn();
		//convert righthand quaternion to lefthand quaternion
		output_POSE[0] = -1 * pos.x(); output_POSE[1] = pos.y(); output_POSE[2] = pos.z();
		//std::cout << " src " << src->numLinks() << std::endl;
		//joints local rotation
		for (int i = 0; i < src->numLinks(); i++) {
			gRotMat rot_MW = src->link(i)->localFrame().rot();
			//
			gQuat quat_rH = rot_MW.inQuat();
			gQuat quat_lH;
			//convert righthand quaternion to lefthand quaternion
			quat_lH.setX(quat_rH.x());
			quat_lH.setY(-1.0 * quat_rH.y());
			quat_lH.setZ(-1.0 * quat_rH.z());
			//quat_lH.setY(quat_rH.y());
			//quat_lH.setZ(quat_rH.z());
			quat_lH.setW(quat_rH.w());

			//convert righthand quaternion to lefthand quaternion
			output_POSE[3 + 4 * i + 0] = quat_lH.x(); output_POSE[3 + 4 * i + 1] = quat_lH.y(); output_POSE[3 + 4 * i + 2] = quat_lH.z(); output_POSE[3 + 4 * i + 3] = quat_lH.w();
		}
	}

	DLL_EXPORT void OUTPUT_TAR_POSE_UNITY_OUTPUT(float* output_POSE) {
		mbs->updateKinematicsUptoPos();
		//hip world position
		gVec3 pos = mbs->link(0)->frame().trn();
		//convert righthand quaternion to lefthand quaternion
		output_POSE[0] = -1 * pos.x(); output_POSE[1] = pos.y(); output_POSE[2] = pos.z();
		//std::cout << " mbs " << mbs->numLinks() << std::endl;
		//joints local rotation
		for (int i = 0; i < mbs->numLinks(); i++) {
			gRotMat rot_MW = mbs->link(i)->localFrame().rot();
			//
			gQuat quat_rH = rot_MW.inQuat();
			gQuat quat_lH;
			//convert righthand quaternion to lefthand quaternion
			quat_lH.setX(quat_rH.x());
			quat_lH.setY(-1.0 * quat_rH.y());
			quat_lH.setZ(-1.0 * quat_rH.z());
			//quat_lH.setY(quat_rH.y());
			//quat_lH.setZ(quat_rH.z());
			quat_lH.setW(quat_rH.w());

			//convert righthand quaternion to lefthand quaternion
			output_POSE[3 + 4 * i + 0] = quat_lH.x(); output_POSE[3 + 4 * i + 1] = quat_lH.y(); output_POSE[3 + 4 * i + 2] = quat_lH.z(); output_POSE[3 + 4 * i + 3] = quat_lH.w();
		}
	}

	/* Optimization IK */
	DLL_EXPORT int INIT_MBS(LPCSTR chaSrcTXTFile) {
		MBSLoader tar_loader;
		mbs_avatar = new gMultibodySystem();
		tar_loader.loadModelUnity(chaSrcTXTFile, mbs_avatar, 1.0);
		//for (int j = 0; j < src->numLinks(); j++)
		//	std::cout << " joint " << src->link(j)->name() << " x " << src->link(j)->frame().rotX() << " y " <<
		//	src->link(j)->frame().rotY() << " z " << src->link(j)->frame().rotZ() << std::endl;
		return mbs_avatar->numLinks();
	}
	
	DLL_EXPORT int INIT_IK() {
		//mbs = new gMultibodySystem();
		//	
		//	MBSLoader mbs_loader;
		//	//mbs_loader.loadModel("D://TJ_develop//MW//MW_HumanMotionRetargeting//Examples//initMBS//mixamo_rest.txt", mbs, 1.0);
		//	mbs_loader.loadModelUnity(chaTarTXTFile, mbs, 1.0);

		HMODULE hDll = LoadLibrary(L"poseIK.DLL");

		LevmarIK init_MWSolver_IK;
		init_MWSolver_IK = (LevmarIK)GetProcAddress(hDll, "INIT_IK");

		//Init MW Solver
		return init_MWSolver_IK(mbs_avatar);
	}





	//DLL_EXPORT int SETUP_CONNECTION_RETARGET(LPCSTR chaSrcTXTFile, LPCSTR chaTarTXTFile, LPCSTR chaAvatarTXTFile) {
	//	
	//	MBSLoader tar_loader;
	//	src = new gMultibodySystem();
	//	tar_loader.loadModelUnity(chaSrcTXTFile, src, 1.0);

	//	MBSLoader mbs_loader;
	//	mbs = new gMultibodySystem();
	//	mbs_loader.loadModelUnity(chaTarTXTFile, mbs, 1.0);

	//	MBSLoader mbs_out_loader;
	//	mbs_output = new gMultibodySystem();
	//	mbs_out_loader.loadModelUnity(chaTarTXTFile, mbs_output, 1.0);

	//	MBSLoader mbs_avatar_loader;
	//	mbs_avatar = new gMultibodySystem();
	//	mbs_avatar_loader.loadModelUnity(chaAvatarTXTFile, mbs_avatar, 1.0);

	//	//-----------------------------------//
	//	HMODULE hDll = LoadLibrary(L"poseIK.DLL");
	//	INITRETARGET init_MWSolver;
	//	init_MWSolver = (INITRETARGET)GetProcAddress(hDll, "INITRETARGET");
	//	init_MWSolver(src, mbs);

	//	INIT_IK init_MWSolver_IK;
	//	init_MWSolver_IK = (INIT_IK)GetProcAddress(hDll, "INIT_IK");
	//	init_MWSolver_IK(mbs_output);

	//	INITRETARGET_AVATAR init_MWSolver_Avatar;
	//	init_MWSolver_Avatar = (INITRETARGET_AVATAR)GetProcAddress(hDll, "INITRETARGET_AVATAR");
	//	init_MWSolver_Avatar(mbs_output, mbs_avatar);
	//	/*INIT_IK_AVATAR init_MWSolver_IK_AVATAR;
	//	init_MWSolver_IK_AVATAR = (INIT_IK_AVATAR)GetProcAddress(hDll, "INIT_IK_AVATAR");
	//	init_MWSolver_IK_AVATAR(mbs_avatar);*/
	//	
	//	return mbs->numLinks();
	//}

	//DLL_EXPORT int SETUP_CONNECTION(LPCSTR chaTarTXTFile) {
	//	mbs = new gMultibodySystem();
	//	
	//	MBSLoader mbs_loader;
	//	//mbs_loader.loadModel("D://TJ_develop//MW//MW_HumanMotionRetargeting//Examples//initMBS//mixamo_rest.txt", mbs, 1.0);
	//	mbs_loader.loadModelUnity(chaTarTXTFile, mbs, 1.0);

	//	//HMODULE hDll = LoadLibrary(L"D://TJ_develop//MW//MW_HumanMotionRetargeting//Examples//poseIK//x64//Release//poseIK.DLL");
	//	HMODULE hDll = LoadLibrary(L"poseIK.DLL");

	//	INIT_IK init_MWSolver_IK;
	//	init_MWSolver_IK = (INIT_IK)GetProcAddress(hDll, "INIT_IK");

	//	//Init MW Solver
	//	return init_MWSolver_IK(mbs);
	//	//return mbs->numLinks();
	//}

	DLL_EXPORT void SET_BASE_POSITION_MW(float* input_pos) {

		gVec3 position(-1.0 * input_pos[0], input_pos[1], input_pos[2]);

		mbs->setBasePosition(position);
		mbs->updateKinematicsUptoPos();

	}

	DLL_EXPORT void SET_JOINT_QUATERNION_MW(LPCSTR jointname, float* input_Quat) {
		gQuat quat; quat.set(input_Quat[0], input_Quat[1], input_Quat[2], input_Quat[3]);
		gRotMat rot_LeftHanded = quat.inRotMatrix();
		gRotMat rot_RightHanded;
		double* convert_L2R = new double[9];
		convert_L2R[0] = rot_LeftHanded.e(0);    convert_L2R[3] = -1 * rot_LeftHanded.e(3); convert_L2R[6] = -1 * rot_LeftHanded.e(6);
		convert_L2R[1] = -1 * rot_LeftHanded.e(1); convert_L2R[4] = rot_LeftHanded.e(4);    convert_L2R[7] = rot_LeftHanded.e(7);
		convert_L2R[2] = -1 * rot_LeftHanded.e(2); convert_L2R[5] = rot_LeftHanded.e(5);    convert_L2R[8] = rot_LeftHanded.e(8);
		rot_RightHanded.set(convert_L2R);

		gXMat mat_G = mbs->findLink(jointname)->localFrame();
		mat_G.setRot(rot_RightHanded);

		mbs->findLink(jointname)->setFromSafeCoordArray(mat_G.rotInQuat().cptr());
		mbs->updateKinematicsUptoPos();

	}

	

	DLL_EXPORT void OUTPUT_JOINT_QUATERNION_UNITY(int jointindex, float* output_Quat) {

		mbs->updateKinematicsUptoPos();

		gRotMat rot_MW = mbs->link(jointindex)->localFrame().rot();

		//
		gQuat quat_rH = rot_MW.inQuat();
		gQuat quat_lH;
		//convert righthand quaternion to lefthand quaternion
		quat_lH.setX(quat_rH.x());
		quat_lH.setY(-1.0 * quat_rH.y());
		quat_lH.setZ(-1.0 * quat_rH.z());
		quat_lH.setW(quat_rH.w());

		output_Quat[0] = quat_lH.x(); output_Quat[1] = quat_lH.y(); output_Quat[2] = quat_lH.z(); output_Quat[3] = quat_lH.w();
		//


	}

	DLL_EXPORT void OUTPUT_JOINT_POSE_UNITY(float* output_POSE) {
		mbs->updateKinematicsUptoPos();
		//hip world position
		gVec3 pos = mbs->link(0)->frame().trn();
		//convert righthand quaternion to lefthand quaternion
		output_POSE[0] = -1 * pos.x(); output_POSE[1] = pos.y(); output_POSE[2] = pos.z();
		//joints local rotation
		for (int i = 0; i < mbs->numLinks(); i++) {
			gRotMat rot_MW = mbs->link(i)->localFrame().rot();
			//
			gQuat quat_rH = rot_MW.inQuat();
			gQuat quat_lH;
			//convert righthand quaternion to lefthand quaternion
			quat_lH.setX(quat_rH.x());
			quat_lH.setY(-1.0 * quat_rH.y());
			quat_lH.setZ(-1.0 * quat_rH.z());
			quat_lH.setW(quat_rH.w());

			//convert righthand quaternion to lefthand quaternion
			output_POSE[3 + 4 * i + 0] = quat_lH.x(); output_POSE[3 + 4 * i + 1] = quat_lH.y(); output_POSE[3 + 4 * i + 2] = quat_lH.z(); output_POSE[3 + 4 * i + 3] = quat_lH.w();
		}
	}

	DLL_EXPORT void OUTPUT_JOINT_POSITION_UNITY(float* output_POS) {

		mbs->updateKinematicsUptoPos();

		for (int i = 0; i < 22; i++) {
			gVec3 pos = mbs->link(i)->frame().trn();
			//convert righthand quaternion to lefthand quaternion
			output_POS[3*i + 0] = -1 * pos.x(); output_POS[3 * i + 1] = pos.y(); output_POS[3 * i + 2] = pos.z();
		}
	}

	DLL_EXPORT void OUTPUT_BASE_POSITION_UNITY(float* output_POS) {

		mbs->updateKinematicsUptoPos();

		gVec3 pos = mbs->baseLink()->frame().trn();
		//convert righthand quaternion to lefthand quaternion
		output_POS[0] = -1 * pos.x(); output_POS[1] = pos.y(); output_POS[2] = pos.z();

	}

	DLL_EXPORT void DO_FULLBODY_IK(float* desireds) {

		//MW solver 상의 아바타도 업데이트
		HMODULE hDll = LoadLibrary(L"poseIK.DLL");

		DO_POSE FullBody_IK;
		FullBody_IK = (DO_POSE)GetProcAddress(hDll, "DO_POSE");

		std::vector<gVec3> points;
		for (int j = 0; j < 22; j++) {
			gVec3 point;
			point.setX(-1 * desireds[0 + 3 * j]);
			point.setY(desireds[1 + 3 * j]);
			point.setZ(desireds[2 + 3 * j]);
			points.push_back(point);
		}

		mbs_output = FullBody_IK(points);
		//mbs = FullBody_IK_CONS(points);
		mbs_output->updateKinematicsUptoPos();
	}


	DLL_EXPORT void OUTPUT_JOINT_POSE_UNITY_AVATAR(float* output_POSE) {
		mbs_avatar->updateKinematicsUptoPos();
		//hip world position
		gVec3 pos = mbs_avatar->link(0)->frame().trn();
		//convert righthand quaternion to lefthand quaternion
		output_POSE[0] = -1 * pos.x(); output_POSE[1] = pos.y(); output_POSE[2] = pos.z();
		//joints local rotation
		for (int i = 0; i < mbs_avatar->numLinks(); i++) {
			gRotMat rot_MW = mbs_avatar->link(i)->localFrame().rot();
			//
			gQuat quat_rH = rot_MW.inQuat();
			gQuat quat_lH;
			//convert righthand quaternion to lefthand quaternion
			quat_lH.setX(quat_rH.x());
			quat_lH.setY(-1.0 * quat_rH.y());
			quat_lH.setZ(-1.0 * quat_rH.z());
			quat_lH.setW(quat_rH.w());

			//convert righthand quaternion to lefthand quaternion
			output_POSE[3 + 4 * i + 0] = quat_lH.x(); output_POSE[3 + 4 * i + 1] = quat_lH.y(); output_POSE[3 + 4 * i + 2] = quat_lH.z(); output_POSE[3 + 4 * i + 3] = quat_lH.w();
		}
	}

	DLL_EXPORT void DO_FULLBODY_IK_AVATAR(float* desireds) {

		//MW solver 상의 아바타도 업데이트
		HMODULE hDll = LoadLibrary(L"poseIK.DLL");

		DO_POSE_AVATAR FullBody_IK_AVATAR;
		FullBody_IK_AVATAR = (DO_POSE_AVATAR)GetProcAddress(hDll, "DO_POSE_AVATAR");

		std::vector<gVec3> points;
		for (int j = 0; j < 22; j++) {
			gVec3 point;
			point.setX(-1 * desireds[0 + 3 * j]);
			point.setY(desireds[1 + 3 * j]);
			point.setZ(desireds[2 + 3 * j]);
			points.push_back(point);
		}

		mbs_avatar = FullBody_IK_AVATAR(points);
		//mbs = FullBody_IK_CONS(points);
		mbs_avatar->updateKinematicsUptoPos();
	}

	
	

	DLL_EXPORT LPCSTR INIT_JOINT_LIST(int i) {

		LPCSTR joint_name = mbs->link(i)->name();

		return joint_name;
	}

}