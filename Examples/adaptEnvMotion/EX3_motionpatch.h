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

class EX3_motionpatch
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
	interact_planes g_interact_plane;
	void setPatch(int bodypart, gXMat chairroot, plane& plane);
	void genInteractionFrame(gVec3 plane_position, gVec3 plane_normal, gXMat chairroot, gXMat& interaction_frame);
	void genInteractionFrame_forSpine(gVec3 plane_position, gVec3 plane_normal, gXMat chairroot, gXMat& interaction_frame);
	void makePlane(plane& plane, gXMat chairroot, gVec3 pos, gVec3 normal,bool offset) {
		//hip plane update		
		plane.exist = true;
		gXMat gmat_rarm_plane;
		genInteractionFrame(pos, normal, chairroot, gmat_rarm_plane);
		//// output plane pos 4
		double e_w = 10;
		double e_h = 10;
		double width = 20;
		double height = 20;
		plane.center = gmat_rarm_plane.trn();
		plane.normal = gmat_rarm_plane.rotY();
		plane.createPlaneOnFrame(gmat_rarm_plane, e_w, e_h, width, height, offset);
	}
	//
	// export chair root, 
	saveTXT* saveTXT;
	void exportEnvironmentInfoTXT(interact_planes plane, char* foldername) {
		char retarget_file_path[_MAX_PATH];// retarget 결과를 저장할 경로
		strcpy(retarget_file_path, foldername);
		strcat(retarget_file_path, ".txt");

		//start write
		FILE* fp = fopen(retarget_file_path, "w");
		//1. chair root frame
		gXMat gmat = plane.root_frame;
		saveTXT->saveTmat(fp, gmat, "RootMatPOSXYZ");
		//2. planes
		if (plane.seat.exist == true) {
			gVec3 center = gmat.invMultVec4(plane.seat.center);
			saveTXT->saveVec3(fp, center, false, "seat");
			gVec3 normal = gmat.invMultVec3(plane.seat.normal);
			saveTXT->saveVec3(fp, normal, true);
		}
		if (plane.backrest.exist == true) {
			gVec3 center = gmat.invMultVec4(plane.backrest.center);
			saveTXT->saveVec3(fp, center, false, "backrest");
			gVec3 normal = gmat.invMultVec3(plane.backrest.normal);
			saveTXT->saveVec3(fp, normal, true);
		}
		if (plane.armrest_L.exist == true) {
			gVec3 center = gmat.invMultVec4(plane.armrest_L.center);
			saveTXT->saveVec3(fp, center, false, "armrest_L");
			gVec3 normal = gmat.invMultVec3(plane.armrest_L.normal);
			saveTXT->saveVec3(fp, normal, true);
		}
		if (plane.armrest_R.exist == true) {
			gVec3 center = gmat.invMultVec4(plane.armrest_R.center);
			saveTXT->saveVec3(fp, center, false, "armrest_R");
			gVec3 normal = gmat.invMultVec3(plane.armrest_R.normal);
			saveTXT->saveVec3(fp, normal, true);
		}
		if (plane.handrest_L.exist == true) {
			gVec3 center = gmat.invMultVec4(plane.handrest_L.center);
			saveTXT->saveVec3(fp, center, false, "handrest_L");
			gVec3 normal = gmat.invMultVec3(plane.handrest_L.normal);
			saveTXT->saveVec3(fp, normal, true);
		}
		if (plane.handrest_R.exist == true) {
			gVec3 center = gmat.invMultVec4(plane.handrest_R.center);
			saveTXT->saveVec3(fp, center, false, "handrest_R");
			gVec3 normal = gmat.invMultVec3(plane.handrest_R.normal);
			saveTXT->saveVec3(fp, normal, true);

		}
		if (plane.footrest_L.exist == true) {
			gVec3 center = gmat.invMultVec4(plane.footrest_L.center);
			saveTXT->saveVec3(fp, center, false, "footrest_L");
			gVec3 normal = gmat.invMultVec3(plane.footrest_L.normal);
			saveTXT->saveVec3(fp, normal, true);
		}
		if (plane.footrest_R.exist == true) {
			gVec3 center = gmat.invMultVec4(plane.footrest_R.center);
			saveTXT->saveVec3(fp, center, false, "footrest_R");
			gVec3 normal = gmat.invMultVec3(plane.footrest_R.normal);
			saveTXT->saveVec3(fp, normal, true);
		}

		//end write file
		fclose(fp);
	}
	
	std::vector<std::string> splitString(std::string str, std::string delimiter = " ")
	{
		int start = 0;
		int end = str.find(delimiter);
		std::vector<std::string> line_data;
		while (end != -1) {
			//std::cout << str.substr(start, end - start) << std::endl;
			line_data.push_back(str.substr(start, end - start));
			start = end + delimiter.size();
			end = str.find(delimiter, start);
		}
		//std::cout << str.substr(start, end - start);
		line_data.push_back(str.substr(start, end - start));

		return line_data;
	}
	
	void readEnvironmentInfoTXT(interact_planes& plane, char* foldername) {
		char retarget_file_path[_MAX_PATH];// retarget 결과를 저장할 경로
		strcpy(retarget_file_path, foldername);
		strcat(retarget_file_path, ".txt");

		std::ifstream ifs;
		ifs.open(retarget_file_path);
		std::string line_txt;
		//while (std::getline(ifs, line_txt))
		while (std::getline(ifs, line_txt)) {
			//std::getline(ifs, line_txt);
			//std::cout << line_txt;
			std::vector<std::string> line_data = splitString(line_txt, " ");

			if (line_data[0] == "RootMatPOSXYZ") {
				float scale = 0.01f;
				gVec3 pos; pos.set(std::stof(line_data[1]), std::stof(line_data[2]), std::stof(line_data[3]));
				gVec3 rotX; rotX.set(std::stof(line_data[4]), std::stof(line_data[5]), std::stof(line_data[6]));
				gVec3 rotY; rotY.set(std::stof(line_data[7]), std::stof(line_data[8]), std::stof(line_data[9]));
				gVec3 rotZ; rotZ.set(std::stof(line_data[10]), std::stof(line_data[11]), std::stof(line_data[12]));

				gXMat chairroot; 
				gRotMat rot; rot.setColumn(0, rotX); rot.setColumn(1, rotY); rot.setColumn(2, rotZ);
				chairroot.setRot(rot); chairroot.setTrn(pos);
				
				plane.root_frame = chairroot;
				
			}
			if (line_data[0] == "seat") {
				float scale = 0.01f;
				gVec3 pos; pos.set(std::stof(line_data[1]), std::stof(line_data[2]), std::stof(line_data[3]));
				gVec3 normal; normal.set(std::stof(line_data[4]), std::stof(line_data[5]), std::stof(line_data[6]));
				pos = plane.root_frame.multVec4(pos); normal = plane.root_frame.multVec3(normal);
				makePlane(plane.seat, plane.root_frame, pos, normal,false);
				
			}
			if (line_data[0] == "backrest") {
				float scale = 0.01f;
				gVec3 pos; pos.set(std::stof(line_data[1]), std::stof(line_data[2]), std::stof(line_data[3]));
				gVec3 normal; normal.set(std::stof(line_data[4]), std::stof(line_data[5]), std::stof(line_data[6]));
				pos = plane.root_frame.multVec4(pos); normal = plane.root_frame.multVec3(normal);
				makePlane(plane.backrest, plane.root_frame, pos, normal, false);
				
			}

			if (line_data[0] == "armrest_L") {
				float scale = 0.01f;
				gVec3 pos; pos.set(std::stof(line_data[1]), std::stof(line_data[2]), std::stof(line_data[3]));
				gVec3 normal; normal.set(std::stof(line_data[4]), std::stof(line_data[5]), std::stof(line_data[6]));
				pos = plane.root_frame.multVec4(pos); normal = plane.root_frame.multVec3(normal);
				makePlane(plane.armrest_L, plane.root_frame, pos, normal, false);
				
			}
			if (line_data[0] == "armrest_R") {
				float scale = 0.01f;
				gVec3 pos; pos.set(std::stof(line_data[1]), std::stof(line_data[2]), std::stof(line_data[3]));
				gVec3 normal; normal.set(std::stof(line_data[4]), std::stof(line_data[5]), std::stof(line_data[6]));
				pos = plane.root_frame.multVec4(pos); normal = plane.root_frame.multVec3(normal);
				makePlane(plane.armrest_R, plane.root_frame, pos, normal, false);
				
			}

			if (line_data[0] == "handrest_L") {
				float scale = 0.01f;
				gVec3 pos; pos.set(std::stof(line_data[1]), std::stof(line_data[2]), std::stof(line_data[3]));
				gVec3 normal; normal.set(std::stof(line_data[4]), std::stof(line_data[5]), std::stof(line_data[6]));
				pos = plane.root_frame.multVec4(pos); normal = plane.root_frame.multVec3(normal);
				makePlane(plane.handrest_L, plane.root_frame, pos, normal, false);
				
			}
			if (line_data[0] == "handrest_R") {
				float scale = 0.01f;
				gVec3 pos; pos.set(std::stof(line_data[1]), std::stof(line_data[2]), std::stof(line_data[3]));
				gVec3 normal; normal.set(std::stof(line_data[4]), std::stof(line_data[5]), std::stof(line_data[6]));
				pos = plane.root_frame.multVec4(pos); normal = plane.root_frame.multVec3(normal);
				makePlane(plane.handrest_R, plane.root_frame, pos, normal, false);
				
			}

			if (line_data[0] == "footrest_L") {
				float scale = 0.01f;
				gVec3 pos; pos.set(std::stof(line_data[1]), std::stof(line_data[2]), std::stof(line_data[3]));
				gVec3 normal; normal.set(std::stof(line_data[4]), std::stof(line_data[5]), std::stof(line_data[6]));
				pos = plane.root_frame.multVec4(pos); normal = plane.root_frame.multVec3(normal);
				makePlane(plane.footrest_L, plane.root_frame, pos, normal,true);
				
			}
			if (line_data[0] == "footrest_R") {
				float scale = 0.01f;
				gVec3 pos; pos.set(std::stof(line_data[1]), std::stof(line_data[2]), std::stof(line_data[3]));
				gVec3 normal; normal.set(std::stof(line_data[4]), std::stof(line_data[5]), std::stof(line_data[6]));
				pos = plane.root_frame.multVec4(pos); normal = plane.root_frame.multVec3(normal);
				makePlane(plane.footrest_R, plane.root_frame, pos, normal, true);
				
			}

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

