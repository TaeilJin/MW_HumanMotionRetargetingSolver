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

class saveBVH
{

public:
	
	std::string puttap(int n) {
		std::string output;
		for (int i = 0; i < n; i++) {
			output += "\t";
		}
		return output;
	}

	enum RotSeq { zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy, xzx };

	///////////////////////////////
	// Quaternion to Euler
	///////////////////////////////
	/*
	enum _AXISORDER {
	XYZ,
	XZY,
	YXZ,
	YZX,
	ZXY,
	ZYX,
	XY, XZ,
	YX, YZ,
	ZX, ZY,
	X, Y, Z
	};
	*/
	void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
		res[0] = atan2(r11, r12);
		res[1] = acos(r21);
		res[2] = atan2(r31, r32);
	}

	void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
		res[0] = atan2(r31, r32);
		res[1] = asin(r21);
		res[2] = atan2(r11, r12);
	}

	// note: 
	// return values of res[] depends on rotSeq.
	// i.e.
	// for rotSeq zyx, 
	// x = res[0], y = res[1], z = res[2]
	// for rotSeq xyz
	// z = res[0], y = res[1], x = res[2]
	// ...
	void quaternion2Euler(gQuat q, double res[], RotSeq rotSeq)
	{
		switch (rotSeq) {
		case zyx:
			threeaxisrot(
				2 * (q.x() * q.y() + q.w() * q.z()),
				q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
				-2 * (q.x() * q.z() - q.w() * q.y()),
				2 * (q.y() * q.z() + q.w() * q.x()),
				q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
				res);
			break;

		case zyz:
			twoaxisrot(2 * (q.y() * q.z() - q.w() * q.x()),
				2 * (q.x() * q.z() + q.w() * q.y()),
				q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
				2 * (q.y() * q.z() + q.w() * q.x()),
				-2 * (q.x() * q.z() - q.w() * q.y()),
				res);
			break;

		case zxy:
			threeaxisrot(-2 * (q.x() * q.y() - q.w() * q.z()),
				q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(),
				2 * (q.y() * q.z() + q.w() * q.x()),
				-2 * (q.x() * q.z() - q.w() * q.y()),
				q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
				res);
			break;

		case zxz:
			twoaxisrot(2 * (q.x() * q.z() + q.w() * q.y()),
				-2 * (q.y() * q.z() - q.w() * q.x()),
				q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
				2 * (q.x() * q.z() - q.w() * q.y()),
				2 * (q.y() * q.z() + q.w() * q.x()),
				res);
			break;

		case yxz:
			threeaxisrot(2 * (q.x() * q.z() + q.w() * q.y()),
				q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
				-2 * (q.y() * q.z() - q.w() * q.x()),
				2 * (q.x() * q.y() + q.w() * q.z()),
				q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(),
				res);
			break;

		case yxy:
			twoaxisrot(2 * (q.x() * q.y() - q.w() * q.z()),
				2 * (q.y() * q.z() + q.w() * q.x()),
				q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(),
				2 * (q.x() * q.y() + q.w() * q.z()),
				-2 * (q.y() * q.z() - q.w() * q.x()),
				res);
			break;

		case yzx:
			threeaxisrot(-2 * (q.x() * q.z() - q.w() * q.y()),
				q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
				2 * (q.x() * q.y() + q.w() * q.z()),
				-2 * (q.y() * q.z() - q.w() * q.x()),
				q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(),
				res);
			break;

		case yzy:
			twoaxisrot(2 * (q.y() * q.z() + q.w() * q.x()),
				-2 * (q.x() * q.y() - q.w() * q.z()),
				q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(),
				2 * (q.y() * q.z() - q.w() * q.x()),
				2 * (q.x() * q.y() + q.w() * q.z()),
				res);
			break;

		case xyz:
			threeaxisrot(-2 * (q.y() * q.z() - q.w() * q.x()),
				q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
				2 * (q.x() * q.z() + q.w() * q.y()),
				-2 * (q.x() * q.y() - q.w() * q.z()),
				q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
				res);
			break;

		case xyx:
			twoaxisrot(2 * (q.x() * q.y() + q.w() * q.z()),
				-2 * (q.x() * q.z() - q.w() * q.y()),
				q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
				2 * (q.x() * q.y() - q.w() * q.z()),
				2 * (q.x() * q.z() + q.w() * q.y()),
				res);
			break;

		case xzy:
			threeaxisrot(2 * (q.y() * q.z() + q.w() * q.x()),
				q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z(),
				-2 * (q.x() * q.y() - q.w() * q.z()),
				2 * (q.x() * q.z() + q.w() * q.y()),
				q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
				res);
			break;

		case xzx:
			twoaxisrot(2 * (q.x() * q.z() - q.w() * q.y()),
				2 * (q.x() * q.y() + q.w() * q.z()),
				q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
				2 * (q.x() * q.z() + q.w() * q.y()),
				-2 * (q.x() * q.y() - q.w() * q.z()),
				res);
			break;
		default:
			std::cout << "Unknown rotation sequence" << std::endl;
			break;
		}
	}

	void savejoint(FILE* fp, mgBone* bone, int i) {
		std::string new_t = puttap(i);
		fprintf(fp, "%sJOINT %s\n", new_t.c_str(), bone->name.c_str());
		fprintf(fp, "%s{\n", new_t.c_str());

		gVec3 pos = bone->H.trn();
		std::cout << bone->order << std::endl;
		/*
		enum _AXISORDER {
			XYZ,
			XZY,
			YXZ,
			YZX,
			ZXY,
			ZYX,
			XY, XZ,
			YX, YZ,
			ZX, ZY,
			X, Y, Z
		};
		*/
		new_t = puttap(i + 1); // i+1 times tap (index +1 )  == child tap num

		fprintf(fp, "%sOFFSET %g %g %g\n", new_t.c_str(), pos.x(), pos.y(), pos.z());
		const char* Zrotation = "Zrotation";
		const char* Yrotation = "Yrotation";
		const char* Xrotation = "Xrotation";
		if (bone->order == 0) {
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Xrotation, Yrotation, Zrotation); // XYZ
		}
		else if (bone->order == 1) {
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Xrotation, Zrotation, Yrotation); // XZY
		}
		else if (bone->order == 2) {
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Yrotation, Xrotation, Zrotation); //YXZ
		}
		else if (bone->order == 3) {
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Yrotation, Zrotation, Xrotation); //YZX
		}
		else if (bone->order == 4) {
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Zrotation, Xrotation, Yrotation); //ZXY
		}
		else if (bone->order == 5) {
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Zrotation, Yrotation, Xrotation); //ZYX
		}
		else {
			std::cout << " rotation order is not 3 dimension! " << std::endl;
		}
		// recursive
		bool end_site = true;

		for (int j = 0; j < bone->children.size(); j++) {
			mgBone* child = bone->children[j];
			savejoint(fp, child, i + 1);
			end_site = false;
		}

		/*if (end_site == true) {
			new_t = puttap(i + 1);
			fprintf(fp, "%sEnd Site\n", new_t.c_str());
			fprintf(fp, "%s{\n", new_t.c_str());
			new_t = puttap(i + 2);
			fprintf(fp, "%sOFFSET %g %g %g\n", new_t.c_str(), 0.0, 0.0, 0.0);
			new_t = puttap(i + 1);
			fprintf(fp, "%s}\n", new_t.c_str());
		}*/

		new_t = puttap(i);
		fprintf(fp, "%s}\n", new_t.c_str());

		//
	}

	void savejoint(FILE* fp, gLink* bone, int i, mgBone::_AXISORDER axisorder,float scale) {
		std::string new_t = puttap(i);
		fprintf(fp, "%sJOINT %s\n", new_t.c_str(), bone->name());
		fprintf(fp, "%s{\n", new_t.c_str());

		gVec3 pos = bone->localFrameDefault().trn() * scale;

		/*
		enum _AXISORDER {
		XYZ,
		XZY,
		YXZ,
		YZX,
		ZXY,
		ZYX,
		XY, XZ,
		YX, YZ,
		ZX, ZY,
		X, Y, Z
		};
		*/
		new_t = puttap(i + 1); // i+1 times tap (index +1 )  == child tap num

		fprintf(fp, "%sOFFSET %g %g %g\n", new_t.c_str(), pos.x(), pos.y(), pos.z());
		const char* Zrotation = "Zrotation";
		const char* Yrotation = "Yrotation";
		const char* Xrotation = "Xrotation";
		switch (axisorder) {
		case mgBone::_AXISORDER::XYZ:
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Xrotation, Yrotation, Zrotation); // XYZ
			break;
		case mgBone::_AXISORDER::XZY:
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Xrotation, Zrotation, Yrotation); // XZY
			break;
		case mgBone::_AXISORDER::YXZ:
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Yrotation, Xrotation, Zrotation); //YXZ
			break;
		case mgBone::_AXISORDER::YZX:
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Yrotation, Zrotation, Xrotation); //YZX
			break;
		case mgBone::_AXISORDER::ZXY:
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Zrotation, Xrotation, Yrotation); //ZXY
			break;
		case mgBone::_AXISORDER::ZYX:
			fprintf(fp, "%sCHANNELS 3 %s %s %s\n", new_t.c_str(), Zrotation, Yrotation, Xrotation); //ZYX
			break;
		}
		// recursive
		bool end_site = true;

		for (int j = 0; j < bone->numChildren(); j++) {
			gLink* child = bone->child(j);
			savejoint(fp, child, i + 1, axisorder,scale);
			end_site = false;
		}

		// sometimes we don't need a end  site
		/*if (end_site == true) {
			new_t = puttap(i + 1);
			fprintf(fp, "%sEnd Site\n", new_t.c_str());
			fprintf(fp, "%s{\n", new_t.c_str());
			new_t = puttap(i + 2);
			fprintf(fp, "%sOFFSET %g %g %g\n", new_t.c_str(), 0.0, 0.0, 0.0);
			new_t = puttap(i + 1);
			fprintf(fp, "%s}\n", new_t.c_str());
		}*/

		new_t = puttap(i);
		fprintf(fp, "%s}\n", new_t.c_str());

		//
	}
	int saveBVHFile(bCharacter* character, const char* filename, arma::mat Quat_Motion, mgBone::_AXISORDER axisorder, float frametime = 1.0f / 60.0f, float scale = 1.0f) {

		// character -> hierarchy, Offset -> Hierarchy
		// rotation order -> Motion
		FILE* fp = fopen(filename, "w");
		if (!fp)
		{
			printf("error: cannot open a file(%s) to write.\n", filename);
			return -1;
		}
		// Root Bone Hierarchy
		gLink* rootLink = character->link(0);
		//gVec3 pos = rootLink->localFrameDefault().trn();
		gVec3 pos = rootLink->localFrameDefault().trn() * scale;


		fprintf(fp, "HIERARCHY\n");
		fprintf(fp, "ROOT %s\n", rootLink->name());
		fprintf(fp, "{\n");
		//OFFSET
		std::string t = puttap(1);
		fprintf(fp, "%sOFFSET %g %g %g \n", t.c_str(), pos.x(), pos.y(), pos.z());
		//Channels
		const char* Zrotation = "Zrotation";
		const char* Yrotation = "Yrotation";
		const char* Xrotation = "Xrotation";
		switch (axisorder) {
		case (mgBone::_AXISORDER::XYZ):
			fprintf(fp, "%sCHANNELS 6 Xposition Yposition Zposition %s %s %s\n", t.c_str(), Xrotation, Yrotation, Zrotation); // XYZ
			break;
		case (mgBone::_AXISORDER::XZY):
			fprintf(fp, "%sCHANNELS 6 Xposition Yposition Zposition %s %s %s\n", t.c_str(), Xrotation, Zrotation, Yrotation); // XZY
			break;
		case (mgBone::_AXISORDER::YXZ):
			fprintf(fp, "%sCHANNELS 6 Xposition Yposition Zposition %s %s %s\n", t.c_str(), Yrotation, Xrotation, Zrotation); //YXZ
			break;
		case (mgBone::_AXISORDER::YZX):
			fprintf(fp, "%sCHANNELS 6 Xposition Yposition Zposition %s %s %s\n", t.c_str(), Yrotation, Zrotation, Xrotation); //YZX
			break;
		case (mgBone::_AXISORDER::ZXY):
			fprintf(fp, "%sCHANNELS 6 Xposition Yposition Zposition %s %s %s\n", t.c_str(), Zrotation, Xrotation, Yrotation); //ZXY
			break;
		case (mgBone::_AXISORDER::ZYX):
			fprintf(fp, "%sCHANNELS 6 Xposition Yposition Zposition %s %s %s\n", t.c_str(), Zrotation, Yrotation, Xrotation); //ZYX
			break;
		}

		//for loop 
		for (int j = 0; j < rootLink->numChildren(); j++) {
			gLink* child = rootLink->child(j);
			savejoint(fp, child, 1, axisorder, scale);
		}
		fprintf(fp, "}\n");

		////Motion
		fprintf(fp, "MOTION\n");
		fprintf(fp, "Frames: %d\n", Quat_Motion.n_cols);
		fprintf(fp, "Frame Time: %g\n", frametime);

		for (int i = 0; i < Quat_Motion.n_cols; i++) {
			arma::vec Quat_Pose = Quat_Motion.col(i);
			for (int j = 0; j < character->numLinks(); j++) {
				if (j == 0) {
					gVec3 pos;
					pos.setX(Quat_Pose[4]);
					pos.setY(Quat_Pose[5]);
					pos.setZ(Quat_Pose[6]);
					//
					pos = pos * scale;
					//
					gQuat q;
					q.setX(Quat_Pose[0]);
					q.setY(Quat_Pose[1]);
					q.setZ(Quat_Pose[2]);
					q.setW(Quat_Pose[3]);
					q.normalize();
					//quaternion to euler with rotation order
					/*enum _AXISORDER {
					XYZ,
					XZY,
					YXZ,
					YZX,
					ZXY,
					ZYX,
					XY, XZ,
					YX, YZ,
					ZX, ZY,
					X, Y, Z
					};*/
					double* euler = new double[3];
					switch (axisorder) {
					case mgBone::_AXISORDER::XYZ:
						quaternion2Euler(q, euler, xyz);
						fprintf(fp, "%g %g %g %g %g %g ", pos.x(), pos.y(), pos.z(), euler[0] * gRTD, euler[1] * gRTD, euler[2] * gRTD); // XYZ
						break;
					case mgBone::_AXISORDER::XZY:
						quaternion2Euler(q, euler, xzy);
						fprintf(fp, "%g %g %g %g %g %g ", pos.x(), pos.y(), pos.z(), euler[0] * gRTD, euler[2] * gRTD, euler[1] * gRTD); // XZY
						break;
					case mgBone::_AXISORDER::YXZ:
						quaternion2Euler(q, euler, yxz);
						fprintf(fp, "%g %g %g %g %g %g ", pos.x(), pos.y(), pos.z(), euler[1] * gRTD, euler[0] * gRTD, euler[2] * gRTD); //YXZ
						break;
					case mgBone::_AXISORDER::YZX:
						quaternion2Euler(q, euler, yzx);
						fprintf(fp, "%g %g %g %g %g %g ", pos.x(), pos.y(), pos.z(), euler[1] * gRTD, euler[2] * gRTD, euler[0] * gRTD); //YZX
						break;
					case mgBone::_AXISORDER::ZXY:
						quaternion2Euler(q, euler, zxy);
						fprintf(fp, "%g %g %g %g %g %g ", pos.x(), pos.y(), pos.z(), euler[2] * gRTD, euler[0] * gRTD, euler[1] * gRTD); //ZXY
						break;
					case mgBone::_AXISORDER::ZYX:
						quaternion2Euler(q, euler, zyx);
						fprintf(fp, "%g %g %g %g %g %g ", pos.x(), pos.y(), pos.z(), euler[2] * gRTD, euler[1] * gRTD, euler[0] * gRTD); //ZYX
						break;
					}

				}
				else {
					gQuat q;
					q.setX(Quat_Pose[(4 * j + 3) + 0]);
					q.setY(Quat_Pose[(4 * j + 3) + 1]);
					q.setZ(Quat_Pose[(4 * j + 3) + 2]);
					q.setW(Quat_Pose[(4 * j + 3) + 3]);
					q.normalize();
					//quaternion to euler with rotation order
					/*enum _AXISORDER {
					XYZ,
					XZY,
					YXZ,
					YZX,
					ZXY,
					ZYX,
					XY, XZ,
					YX, YZ,
					ZX, ZY,
					X, Y, Z
					};*/
					double* euler = new double[3];
					switch (axisorder) {
					case mgBone::_AXISORDER::XYZ:
						quaternion2Euler(q, euler, xyz);
						fprintf(fp, "%g %g %g ", euler[0] * gRTD, euler[1] * gRTD, euler[2] * gRTD); // XYZ
						break;
					case mgBone::_AXISORDER::XZY:
						quaternion2Euler(q, euler, xzy);
						fprintf(fp, "%g %g %g ", euler[0] * gRTD, euler[2] * gRTD, euler[1] * gRTD); // XZY
						break;
					case mgBone::_AXISORDER::YXZ:
						quaternion2Euler(q, euler, yxz);
						fprintf(fp, "%g %g %g ", euler[1] * gRTD, euler[0] * gRTD, euler[2] * gRTD); //YXZ
						break;
					case mgBone::_AXISORDER::YZX:
						quaternion2Euler(q, euler, yzx);
						fprintf(fp, "%g %g %g ", euler[1] * gRTD, euler[2] * gRTD, euler[0] * gRTD); //YZX
						break;
					case mgBone::_AXISORDER::ZXY:
						quaternion2Euler(q, euler, zxy);
						fprintf(fp, "%g %g %g ", euler[2] * gRTD, euler[0] * gRTD, euler[1] * gRTD); //ZXY
						break;
					case mgBone::_AXISORDER::ZYX:
						quaternion2Euler(q, euler, zyx);
						fprintf(fp, "%g %g %g ", euler[2] * gRTD, euler[1] * gRTD, euler[0] * gRTD); //ZYX
						break;
					}
				}
			}
			fprintf(fp, "\n");
		}

		//end file
		fclose(fp);

		return 0;
	}
	void saveBVH_start2end(arma::mat motion, bCharacter* character, int key_start, int key_end, char* filename) {

		int totalframes = key_end - key_start;
		arma::mat tarQuaternions(character->sizeSafeCoordArray(), totalframes, arma::fill::zeros);
		for (int k = key_start, p = 0; k < key_end; k++, p++) {
			character->setFromCompactCoordArray(motion.col(k));
			character->updateKinematicsUptoPos();
			character->updateKinematicBodiesOfCharacterSim();

			//
			if (tarQuaternions.n_rows != (character->numLinks() * 4 + 3))
				std::cout << " dof is not matched! see the matrix and character" << std::endl;

			//tar pose save
			arma::vec Quat_pose; Quat_pose.resize(character->numLinks() * 4 + 3);
			character->getSafeCoordArray(Quat_pose);
			tarQuaternions(0, p, arma::SizeMat(Quat_pose.n_elem, 1)) = Quat_pose;
		}
		saveBVHFile(character, filename, tarQuaternions, mgBone::_AXISORDER::ZYX);
	}
};

