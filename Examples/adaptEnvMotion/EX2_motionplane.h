#pragma once
#include <Windows.h>
#include <iostream>
#include <algorithm>

#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/LineWidth>
#include <osgDB/ReadFile>

#include "Character/bCharacterLoader.h"
#include "Loader/MotionLoader.h"
#include "Loader/mgSkeletonToBCharacter.h"
#include "MocapProcessor/mgMBSUtil.h"
#include "MocapProcessor/mgUtility.h"
#include "Visualizer/gBdOsgSystem.h"
#include "Visualizer/gOSGSkin.h"
#include "Visualizer/gOSGShape.h"

#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Animation>
#include <osgAnimation/Skeleton>
#include <osgAnimation/Bone>
#include <osgAnimation/UpdateBone>
#include <osgAnimation/StackedRotateAxisElement>
#include <osgAnimation/StackedMatrixElement>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/StackedScaleElement>
#include <osg/TriangleIndexFunctor>
#include <osgDB/Options>
#include <osg/BlendFunc>

#include "saveBVH.h"
#include "saveTXT.h"

class EX2_motionplane
{

public:
	btDynamicsWorld* m_dynamicsWorld;
	osg::ref_ptr<osgViewer::Viewer> g_viewer;
	osg::ref_ptr<osg::Group> g_osg_cha;
	osg::ref_ptr<osg::Group> g_osg_debug;

	bCharacter* g_src;
	bCharacterSim* g_srcSim;
	gBDOSGSystem* g_srcVis;
	arma::mat g_refCoord;

	double g_lowerHeight, g_armHeight;
	int g_iter = 0;
	int g_simulationTime = 0;
	bool BTsetup(btDynamicsWorld* in_dynamicworld) {
		m_dynamicsWorld = in_dynamicworld;
		return true;
	}
	bool OSGsetup(osg::ref_ptr<osgViewer::Viewer> viewer, osg::ref_ptr<osg::Group>& osg_cha, osg::ref_ptr<osg::Group>& osg_debug) {
		// 1. osgGroup for draw character on scene [osg::Group]
		// 2. osgGroup for draw debugging on scene [osg::Group]
		g_osg_cha = osg_cha;
		g_osg_debug = osg_debug;
		g_viewer = viewer;
		return true;
	}
	bool loadAvatarModelFromFile(bCharacter* in_avatar, bCharacterSim* in_avatarSim, gBDOSGSystem* visSys, const char* filename, const double scale)
	{
		bCharacterLoader loader;
		bool exist = FALSE;
		if (!loader.loadModel(filename, in_avatar, scale)) { printf("fail to load!\n"); exist = FALSE; }
		else
		{
			exist = TRUE;
			in_avatar->postLoading();

			//double lowerBodyLength = 
			//	fabs(m_avatar->baseLink()->pos().y() - m_avatar->getLFootLink()->pos().y()) //root to ankle
			//	+ fabs( (m_avatar->getLFootLink()->frame().multVec3(m_avatar->getLFootGeom().sole())).y() ); //ankle to ground
			//m_avatar->setBasePosition(gVec3(0,lowerBodyLength + 10.0 ,0)); //set initial position 10.0 centimeter off ground
			//avatar->updateKinematicsUptoPos();

			//create avatarSim
			in_avatar->setupCharacterSim(in_avatarSim, m_dynamicsWorld, btVector3(0, 0, 0));
			in_avatarSim->postLoading();

			// kinematic 
			in_avatarSim->setBtBodiesDynamicOrKinematic(btCollisionObject::CF_KINEMATIC_OBJECT);


			visSys->setDebugMode(gBDVisSystem::ALL);

			double shapeWidth = gOSGShape::_width;
			osg::Vec4 color = gOSGShape::color;
			color.a() = 0.4;
			gOSGShape::_width = 0.5;
			gOSGShape::setColor(color);
			visSys->setCharacter(in_avatarSim, "in_avatar");
			osg::ref_ptr<osg::Group> avatarGroup;
			avatarGroup = visSys->getOSGGroup("in_avatar");

			gOSGShape::_width = shapeWidth;

			//visSys.setRenderMode(gBDVisSystem::POLYGON, "in_avatar");
			visSys->setRenderMode(gBDVisSystem::WIREFRAME, "in_avatar");

			osg::ref_ptr<osg::Group> group = visSys->getOSGGroup("in_avatar");
			group->getStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			group->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

			osg::ref_ptr<osg::BlendFunc> bf = new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
			group->getStateSet()->setAttributeAndModes(bf);

		}
		return exist;
	}
	// init function 1. load src character [txt] 2. load motion clip	 [txt]
	void SetupScene(char* srcfilename, char* srcCharactertxt, char* outfilefoldername);
	
	// extract keypose frame input source motion, output keypose frame indices
	std::vector<int> g_keypose_Indices;
	std::vector<int> g_interaction_states;
	void extractKeyPose(arma::mat motions, std::vector<int>& toggledindex);
	void findToggledIndx(std::vector<int> v, std::vector<int>& v_index) {
		for (int i = 0; i < v.size() - 1; i++) {
			if (v[i] != v[i + 1])
				v_index.push_back(i + 1);
		}
		v_index.push_back(v.size() - 1);
		if (v_index.size() < 0) {
			std::cout << " first frame is a key pose" << std::endl;
			v_index.push_back(0);
		}

	}
	//extract chair root and interacted keypose indices
	std::vector<gXMat> g_gMat_chairs;
	void extractChairRoot(arma::mat motion, std::vector<int> keypose_indices, std::vector<int>& interactionstates, std::vector<gXMat>& gMat_ChairRoots);
	void createNewFramefromUpperBody(gVec3 pos_hip, gVec3 vec_dir_left, gXMat& gMat_chair_root) {
		// across vector
		gVec3 pos_chair = pos_hip; pos_chair.setY(0.0);
		gVec3 vec_across = vec_dir_left; vec_across.normalize();
		// up vector
		gVec3 vec_up = gVec3(0, 1, 0);
		// forward vector
		gVec3 vec_forward = vec_across % vec_up; vec_forward.normalize();
		// left vector
		gVec3 vec_left = vec_up % vec_forward; vec_left.normalize();
		// set chair root matrix
		gRotMat rot_chair; rot_chair.setColumn(0, vec_left); rot_chair.setColumn(1, vec_up); rot_chair.setColumn(2, vec_forward);
		gXMat gMat_chair; gMat_chair.setRot(rot_chair); gMat_chair.setTrn(pos_chair);
		// update interaction_planes
		gMat_chair_root = gMat_chair;
	};
	
	//plane extracted from chairroot and motion clip
	struct plane {
		gVec3 center;
		gVec3 normal;
		gVec3 pos_00;
		gVec3 pos_01;
		gVec3 pos_10;
		gVec3 pos_11;
		bool exist;

		void createPlaneVertice(gVec3 pos_start, gVec3 dir1, double height, gVec3 dir2, double width) {
			pos_00 = pos_start;
			pos_01 = pos_start + (dir1 * height);
			pos_10 = pos_start + (dir2 * width);
			pos_11 = pos_start + ((dir1 * height) + (dir2 * width));
		};
		void createPlaneOnFrame(gXMat frame_world, double e_w, double e_h, double width, double height,bool isfoot) {
			// output plane pos 4
			gVec3 pos_start_arm = gVec3(-1 * e_w, -7, -1 * e_h);
			if(isfoot)
				pos_start_arm = gVec3(-1 * e_w, 0.0, -1 * e_h);
			pos_start_arm = frame_world.multVec4(pos_start_arm);
			gVec3 dir_width = frame_world.rotX();
			gVec3 dir_height = frame_world.rotZ();
			// create plane
			createPlaneVertice(pos_start_arm, dir_height, height, dir_width, width);
		}
	};
	struct interact_planes {
		plane seat;
		plane backrest;

		plane armrest_L;
		plane armrest_R;

		plane handrest_L;
		plane handrest_R;

		plane footrest_L;
		plane footrest_R;

		gXMat root_frame;
	};
	std::vector<interact_planes> g_interact_planes;
	void extractInteractPlanes(arma::mat motion, std::vector<int> interactionstates, std::vector<int> keypose, std::vector<gXMat> gMat_chair);
	
	bool findPlanePositionNormal(std::vector<gVec3> positions, std::vector<gVec3> normals,
		double cp_xl, double cp_xu, double cp_yl, double cp_yu, double cp_zl, double cp_zu,
		double cn_xl, double cn_xu, double cn_yl, double cn_yu, double cn_zl, double cn_zu,
		gXMat chair_root, gVec3& plane_position, gVec3& plane_normal);
	bool findPlanePositionNormal_withToq(std::vector<gVec3> positions, 
		double cp_xl, double cp_xu, double cp_yl, double cp_yu, double cp_zl, double cp_zu,
		double cn_xl, double cn_xu, double cn_yl, double cn_yu, double cn_zl, double cn_zu,
		double const_torq, double number,gVec3 reference_joint,
		gVec3 torqued_joint, gXMat chair_root, gVec3& plane_position, gVec3& plane_normal);
	bool findPlanePositionNormal_withReference(std::vector<gVec3> positions,
		double cp_xl, double cp_xu, double cp_yl, double cp_yu, double cp_zl, double cp_zu,
		double const_vel,
		gVec3 reference_joint, gXMat chair_root, gVec3& plane_position, gVec3& plane_normal);
	bool findPlanePositionNormal_forSpine(std::vector<gVec3> positions, std::vector<gVec3> normals,
		double cp_xl, double cp_xu, double cp_yl, double cp_yu, double cp_zl, double cp_zu,
		double cn_xl, double cn_xu, double cn_yl, double cn_yu, double cn_zl, double cn_zu,
		gXMat chair_root, gVec3& plane_position, gVec3& plane_normal);

	void genInteractionFrame(gVec3 plane_position, gVec3 plane_normal, gXMat chairroot, gXMat& interaction_frame);
	void genInteractionFrame_forSpine(gVec3 plane_position, gVec3 plane_normal, gXMat chairroot, gXMat& interaction_frame);

	
	
	bool findMeanPosNormal_const(std::vector<gVec3> pos_datas, 
		double x_u, double x_l, double y_u, double y_l, double z_u, double z_l, 
		gXMat chair_root, gXMat& interaction_frame);
	bool findMeanPosNormal_const_hand(std::vector<gVec3> pos_datas,
		double x_u, double x_l, double y_u, double y_l, double z_u, double z_l,
		double hand_y_u, double hand_y_l,
		gXMat chair_root, gXMat& interaction_frame);
	bool findMeanPosNormal_const(std::vector<gVec3> pos_datas,std::vector<gVec3> normal_datas,
		double x_u, double x_l, double y_u, double y_l, double z_u, double z_l,
		gXMat chair_root, gXMat& interaction_frame);
	bool findMeanPosNormal_const(std::vector<gVec3> pos_datas,
		double x_u, double x_l, double y_u, double y_l, double z_u, double z_l,
		double ph_u, double ph_l,
		gXMat chair_root, gXMat& interaction_frame);
	bool findMeanPosNormal_const_spine(std::vector<gVec3> pos_datas, std::vector<gVec3> normal_datas,
		double x_u, double x_l, double y_u, double y_l, double z_u, double z_l, 
		gVec3 hip, gXMat chair_root, gXMat& interaction_frame);

	bool findMeanPosNormal_const_toq(std::vector<gVec3> pos_datas,
		double const_torq,
		gVec3 torqued_joint, gVec3 vec_forward, gXMat& interaction_frame);
	
	//
	// export chair root, 
	saveTXT* saveTXT;
	void exportKeyEnvironmentInfoTXT(std::vector<interact_planes> planes, char* foldername) {
		
		for (int i = 0; i < planes.size(); i++) {
			char retarget_file_path[_MAX_PATH];// retarget 결과를 저장할 경로
			strcpy(retarget_file_path, foldername);
			strcat(retarget_file_path, std::to_string(i).c_str());
			strcat(retarget_file_path, "_key.txt");
			
			interact_planes plane = planes[i];
			//start write
			FILE* fp = fopen(retarget_file_path, "w");
			//1. chair root frame
			gXMat gmat = plane.root_frame;
			saveTXT->saveTmat(fp, gmat,"RootMatPOSXYZ");
			//2. planes
			if (plane.seat.exist == true) {
				gVec3 center = gmat.invMultVec4(plane.seat.center);
				saveTXT->saveVec3(fp, center, false,"seat");
				gVec3 normal = gmat.invMultVec4(plane.seat.normal);
				saveTXT->saveVec3(fp, normal, true);
			}
			if (plane.backrest.exist == true) {
				gVec3 center = gmat.invMultVec4(plane.backrest.center);
				saveTXT->saveVec3(fp, center, false, "backrest");
				gVec3 normal = gmat.invMultVec4(plane.backrest.normal);
				saveTXT->saveVec3(fp, normal, true);
			}
			if (plane.armrest_L.exist == true) {
				gVec3 center = gmat.invMultVec4(plane.armrest_L.center);
				saveTXT->saveVec3(fp, center, false, "armrest_L");
				gVec3 normal = gmat.invMultVec4(plane.armrest_L.normal);
				saveTXT->saveVec3(fp, normal, true);
			}
			if (plane.armrest_R.exist == true) {
				gVec3 center = gmat.invMultVec4(plane.armrest_R.center);
				saveTXT->saveVec3(fp, center, false, "armrest_R");
				gVec3 normal = gmat.invMultVec4(plane.armrest_R.normal);
				saveTXT->saveVec3(fp, normal, true);
			}
			if (plane.handrest_L.exist == true) {
				gVec3 center = gmat.invMultVec4(plane.handrest_L.center);
				saveTXT->saveVec3(fp, center, false, "handrest_L");
				gVec3 normal = gmat.invMultVec4(plane.handrest_L.normal);
				saveTXT->saveVec3(fp, normal, true);
			}
			if (plane.handrest_R.exist == true) {
				gVec3 center = gmat.invMultVec4(plane.handrest_R.center);
				saveTXT->saveVec3(fp, center, false, "handrest_R");
				gVec3 normal = gmat.invMultVec4(plane.handrest_R.normal);
				saveTXT->saveVec3(fp, normal, true);

			}
			if (plane.footrest_L.exist == true) {
				gVec3 center = gmat.invMultVec4(plane.footrest_L.center);
				saveTXT->saveVec3(fp, center, false, "footrest_L");
				gVec3 normal = gmat.invMultVec4(plane.footrest_L.normal);
				saveTXT->saveVec3(fp, center, true);
			}
			if (plane.footrest_R.exist == true) {
				gVec3 center = gmat.invMultVec4(plane.footrest_R.center);
				saveTXT->saveVec3(fp, center, false, "footrest_R");
				gVec3 normal = gmat.invMultVec4(plane.footrest_R.normal);
				saveTXT->saveVec3(fp, center, true);
			}

			//end write file
			fclose(fp);
		}
	}
	//
	saveBVH* saveBVH;
	void exportKeyMotionClipsBVH(arma::mat motion, char* filename) {
		for (int i = 0; i < g_keypose_Indices.size(); i++) {
			int key_start = g_keypose_Indices[i];
			int key_end = 0;
			if (i == g_keypose_Indices.size() - 1)
				key_end = motion.n_cols;
			else
				key_end = g_keypose_Indices[i + 1];

			//save BVH
			char retarget_file_path[_MAX_PATH];// retarget 결과를 저장할 경로
			strcpy(retarget_file_path, filename);
			strcat(retarget_file_path, std::to_string(i).c_str());
			strcat(retarget_file_path, "_key.bvh");

			saveBVH->saveBVH_start2end(motion, g_src, key_start, key_end, retarget_file_path);
		}
	}

	// update function 
	// 1. simulation iteration (for viewer)
	// 2. pose frame iteration (for motiondata)
	void UpdateScene(int even_simulationTime, int even_iter);

	// update character
	void updateCharacterFrame(int i_frame) {
		arma::vec pose = g_refCoord.col(i_frame);

		g_src->setFromCompactCoordArray(pose);
		g_src->updateKinematicsUptoPos();
		g_src->updateKinematicBodiesOfCharacterSim();
	}
	std::vector<gVec3> gatherJointPositions(arma::mat motion, int start_frame, int end_frame, int joint_index);
	std::vector<gVec3> gatherJointPositions(arma::mat motion, int start_frame, int end_frame, int joint_index, int joint_index2);
	std::vector<gVec3> gatherJointNormals(arma::mat motion, int start_frame, int end_frame, int joint_index);
	std::vector<gVec3> gatherJointDir(arma::mat motion, int start_frame, int end_frame, int joint_index, int joint_index2);

	std::vector<gVec3> gatherJointNormals(arma::mat motion, int start_frame, int end_frame, int joint_index_start, int joint_index_end, int joint_index_forward,bool left_start);

	// draw function
	osg::Vec3 gVtoOsg(gVec3 vec) {
		osg::Vec3 ovec;
		ovec.set(vec.x(), vec.y(), vec.z());
		return ovec;
	}
	void drawFrame(gXMat transposeMatrix, osg::ref_ptr<osg::Group>& debugGroup) {
		//draw chair root
		gOSGShape::setColor(osg::Vec4(0.0, 1.0, 0.0, 1.0));
		debugGroup->addChild(gOSGShape::createLineShape(gVtoOsg(transposeMatrix.trn()), gVtoOsg(transposeMatrix.rotY()), 10.0, 1.0));
		gOSGShape::setColor(osg::Vec4(0.0, 0.0, 1.0, 1.0));
		debugGroup->addChild(gOSGShape::createLineShape(gVtoOsg(transposeMatrix.trn()), gVtoOsg(transposeMatrix.rotZ()), 10.0, 2.0));
		gOSGShape::setColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));
		debugGroup->addChild(gOSGShape::createLineShape(gVtoOsg(transposeMatrix.trn()), gVtoOsg(transposeMatrix.rotX()), 10.0, 2.0));
	}
};

