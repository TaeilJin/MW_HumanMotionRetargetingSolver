
#include <Windows.h>
#include <iostream>
#include <algorithm>

#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/LineWidth>
#include <osgDB/ReadFile>

#include "Visualizer/gOSGShape.h"
#include "Loader/MotionLoader.h"
#include "Loader/mgSkeletonToBCharacter.h"
//#include "AvatarWorld.h"
#include "Visualizer/gBdOsgSystem.h"

#include "MocapProcessor/mgMBSUtil.h"
#include "MocapProcessor/mgUtility.h"
#include "Visualizer/gEventHandler.h"

#include "Visualizer/gOSGSkin.h"

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

#include "igl/readOBJ.h"
#include "mgPoseTransfer.h"
#include "../MW2/include/Base/gMath.h"
#include <io.h>
#include <experimental/filesystem>

#define NCPtoNSP(n) (n+2) // convert # of control point to # of spline control points, which used in BSpline
#define NCPtoNSE(n) (n-1) // convert # of control point to # of spline segment, which used in BSpline


double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);


// test
arma::mat refCoord;

osg::ref_ptr<osg::Group> debugGroup = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup2 = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup3 = new osg::Group;

//
osg::Vec3 drawBone(osg::ref_ptr<osg::Node> node, osg::ref_ptr<osg::Group> view_group)
{
	osg::ref_ptr<osgAnimation::Bone> b = dynamic_cast<osgAnimation::Bone*>(node.get());
	if (!b)
	{
		osg::ref_ptr<osg::Group> group = dynamic_cast<osg::Group*>(node.get());
		if (group)
		{
			for (int i = 0; i < group->getNumChildren(); ++i)
			{
				drawBone(group->getChild(i), view_group);
			}
		}
		return osg::Vec3(0, 0, 0);
	}

	osg::Matrix wMat = b->getWorldMatrices()[0];
	//osg::Matrix wMat = b->getMatrixInSkeletonSpace();
	//osg::Vec3 pos = b->getMatrix().getTrans();
	osg::Vec3 pos = wMat.getTrans();

	gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
	view_group->addChild(gOSGShape::createPoint(pos, 3.));

	for (int i = 0; i < b->getNumChildren(); ++i)
	{
		osg::Vec3 c_pos = drawBone(b->getChild(i), view_group);

		gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
		view_group->addChild(gOSGShape::createLineShape(pos, c_pos, 1.));

	}

	return pos;
}

osg::Vec3 drawBone(mgBone* bone, double* data, osg::ref_ptr<osg::Group> view_group)
{
	gXMat wMat;
	bone->skeleton->getWMatrixAt(bone->id, data, wMat);

	gVec3 pos_g = wMat.trn();
	osg::Vec3 pos(pos_g.x(), pos_g.y(), pos_g.z());

	gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
	view_group->addChild(gOSGShape::createPoint(pos, 3.));

	for (int i = 0; i < bone->children.size(); ++i)
	{
		osg::Vec3 c_pos = drawBone(bone->children[i], data, view_group);

		gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
		view_group->addChild(gOSGShape::createLineShape(pos, c_pos, 1.));
	}

	return pos;
}

#include "Loader/FBXLoader.h"

osg::ref_ptr<osg::Node> getNodeFromName(osg::ref_ptr<osg::Node> node, std::string& name)
{
	osg::Group* group = node->asGroup();
	if (!group) return NULL;

	if (node->getName() == name)
		return node;

	for (int i = 0; i < group->getNumChildren(); ++i)
	{
		osg::ref_ptr<osg::Node> n = getNodeFromName(group->getChild(i), name);
		if (n) return n;
	}
	return NULL;
}

gXMat getNodeTransform(osg::ref_ptr<osg::Node> node, FBXLoader* fbx)
{
	return gXMat();
}

//get gVec3 to OSGVec
osg::Vec3 gVec3_2_OsgVec(gVec3 gVec) {
	osg::Vec3 p(gVec.x(), gVec.y(), gVec.z());
	return p;
};
//get gVec3 to OSGVec
gVec3 OsgVec_2_gVec3(osg::Vec3 oVec) {
	gVec3 p(oVec.x(), oVec.y(), oVec.z());
	return p;
};
//print gVec3
void printgVec3(char* txt, gVec3 gVec) {
	std::cout << txt << " " << " x " << gVec.x() << " y " << gVec.y() << " z " << gVec.z() << std::endl;
}

//event handler
int iter = 0;
double simulationTime = 0;

bool bool_go = false;
bool bool_go_back = false;
bool bool_motionstop = true;
bool bool_motionretarget = false;
bool bool_motionretarget_save = false;
inline bool toggleUpdateMotion() { bool_go = !bool_go; return bool_go; }
inline bool toggleUpdateMotionBack() { bool_go_back = !bool_go_back; return bool_go_back; }

// event handler
void keyEventToggleAnimation(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (toggleUpdateMotion())
		std::cout << "Activate animation mode.\n";
	else
		std::cout << "Deactivate animation mode.\n";
}
void keyEventToggleAnimationBack(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (toggleUpdateMotionBack())
		std::cout << "Activate animation mode.\n";
	else
		std::cout << "Deactivate animation mode.\n";
}
void keyEventOneFrameGo(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	simulationTime += 1. / 30.;
	iter++;
}
void keyEventOneFrameBack(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	simulationTime -= 1. / 30.;
	iter--;
}
void keyEventOneDataGo(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool_motionstop = false;
}
void keyEventDoRetarget(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool_motionretarget = !bool_motionretarget;
}
void keyEventDoRetargetSave(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool_motionretarget_save = true;
}
void drawMesh(Eigen::MatrixXd V) {
	for (int r = 0; r < V.rows(); r++) {
		debugGroup->addChild(gOSGShape::createPoint(osg::Vec3(V(r, 0), V(r, 1), V(r, 2)), 5.0));
	}
}
//----------------------------------------------------you may don't need to check upper coding lines (core and individual's functions are mixed. )

#include "Character/bCharacterLoader.h"
#include <osg/BlendFunc>
btBroadphaseInterface				*m_broadphase;
btCollisionDispatcher				*m_dispatcher;
btConstraintSolver					*m_solver;
btDefaultCollisionConfiguration		*m_collisionConfiguration;
btDynamicsWorld						*m_dynamicsWorld;
btRigidBody* m_groundBody;

void initWorld()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);
	m_solver = new btSequentialImpulseConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.001f;
	m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.01f;
	m_dynamicsWorld->getDispatchInfo().m_useContinuous = true;

	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
	m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_USE_2_FRICTION_DIRECTIONS | SOLVER_USE_WARMSTARTING | SOLVER_CACHE_FRIENDLY;
	m_dynamicsWorld->setGravity(btVector3(MW_GRAVITY_VECTOR.x(), MW_GRAVITY_VECTOR.y(), MW_GRAVITY_VECTOR.z()));
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(20.), btScalar(10.), btScalar(20.)));
		//m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		//groundTransform.setRotation(btQuaternion(0, -10*gDTR, 0 ));
		//groundTransform.setOrigin(btVector3(0.0,-10.0,0));
		groundTransform.setOrigin(btVector3(0.0, -10.0, 0));
		//#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
		fixedGround->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
		fixedGround->setFriction(0.9);
		fixedGround->setRestitution(0.1);
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		//localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
		m_groundBody = new btRigidBody(0.0, 0, groundShape);
		m_groundBody->setWorldTransform(groundTransform);
		m_groundBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
		m_groundBody->setFriction(0.9);
		m_dynamicsWorld->addRigidBody(m_groundBody);

		//visSys->addVisBody(groundBody);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}
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
// poseTransfer process
// poseTrans input : bCharacter* src, bCharacter* tar
// poseTrans output : retargetted tar pose ( you should update new pose using updateKinematicsUptoPos();)
// initTranfer: setting desired position, desired direction objectives
// poseTrans->transferPoseLevMar(offset); : do pose Transfer
mgPoseTransfer* poseTrans;





void SetJntRotDirOBJ(mgPoseTransfer* poseTrans, char* txt_id, char* src_jnt, char* tar_jnt) {
	poseTrans->addPoint(txt_id, *poseTrans->src->findLink(src_jnt), gVec3(0, 0, 0), *poseTrans->tar->findLink(tar_jnt), gVec3(0, 0, 0));
	
}
void SetJntRotDirOBJ(mgPoseTransfer* poseTrans, char* txt_id, char* src_jnt, gVec3 s_X, gVec3 s_Z, char* tar_jnt, gVec3 t_X, gVec3 t_Z) {
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
		poseTrans->addPoint(p_y, *poseTrans->src->findLink(src_jnt), gVec3(0,1,0), *poseTrans->tar->findLink(tar_jnt), gVec3(0,1,0));
		poseTrans->addPoint(p_z, *poseTrans->src->findLink(src_jnt), s_Z, *poseTrans->tar->findLink(tar_jnt), t_Z);

		poseTrans->addDirectionObjective(txt_id, p_x, 1.0);
		poseTrans->addDirectionObjective(txt_id, p_z, 1.0);
		poseTrans->addDirectionObjective(txt_id, p_y, 1.0);
	}
}
void SetJntRotDirOBJ(mgPoseTransfer* poseTrans, char* txt_id, char* src_jnt, gVec3 s_X, gVec3 s_Z, gVec3 s_Y, char* tar_jnt, gVec3 t_X, gVec3 t_Z, gVec3 t_Y) {
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

enum RetargetPair { Mocap2MixamoRest, Axis2MixamoRest, Kinect2MixamoRest, Mixamo92MixamoRest, CMU2MixamoRest};
std::vector<std::string> RetargetPair_s;

void initTransfer(bCharacter* src, bCharacter* tar) {

	double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hips")->frame().trn().y();
	double maxHeightT = tar->findLink("mixamorig:Head")->frame().trn().y() -  tar->findLink("mixamorig:Spine1")->frame().trn().y();
	double scale =  maxHeightT /maxHeight;

	poseTrans = new mgPoseTransfer(src, tar);
	poseTrans->scale = scale;
	// you need to manually match the src chracter joint name and corresponding tar character joint name
	//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
	SetJntRotDirOBJ(poseTrans, "t0", "Hips",gVec3(10,0,0),gVec3(0,0,10), "mixamorig:Hips",gVec3(10,0,0),gVec3(0,0,10));
	SetJntRotDirOBJ(poseTrans, "t1", "Spine", gVec3(10,0,0), gVec3(0, 0, 10), "mixamorig:Spine", gVec3(10,0,0), gVec3(0, 0, 10) );
	SetJntRotDirOBJ(poseTrans, "t2", "Spine1", gVec3(10,0,0), gVec3(0, 0, 10), "mixamorig:Spine1", gVec3(10,0,0), gVec3(0, 0, 10) );
	SetJntRotDirOBJ(poseTrans, "t3", "Spine2", gVec3(10,0,0), gVec3(0, 0, 10), "mixamorig:Spine2",gVec3(10,0,0), gVec3(0, 0, 10) );
	SetJntRotDirOBJ(poseTrans, "t4", "Neck",gVec3(10,0,0), gVec3(0, 0, 10) , "mixamorig:Neck", gVec3(10,0,0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10,0,0), gVec3(0, 0, 10),  "mixamorig:Head", gVec3(10,0,0), gVec3(0, 0, 10));
	
	//left leg chain
	SetJntRotDirOBJ(poseTrans, "la0", "LeftUpLeg", gVec3(10,0,0), gVec3(0,0,10) , "mixamorig:LeftUpLeg", gVec3(10,0,0), gVec3(0,0,10));
	SetJntRotDirOBJ(poseTrans, "la1", "LeftLeg", gVec3(10,0,0), gVec3(0,0,10) , "mixamorig:LeftLeg",  gVec3(10,0,0), gVec3(0,0,10));
	SetJntRotDirOBJ(poseTrans, "la2", "LeftFoot", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:LeftFoot", gVec3(10,0,0), gVec3(0,0,10));
	
	//right leg chain
	SetJntRotDirOBJ(poseTrans, "ra0", "RightUpLeg", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:RightUpLeg", gVec3(10,0,0), gVec3(0,0,10));
	SetJntRotDirOBJ(poseTrans, "ra1", "RightLeg", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:RightLeg", gVec3(10,0,0), gVec3(0,0,10));
	SetJntRotDirOBJ(poseTrans, "ra2", "RightFoot", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:RightFoot", gVec3(10,0,0), gVec3(0,0,10));
	
	
	//left arm chain
	SetJntRotDirOBJ(poseTrans, "ll0", "LeftShoulder", "mixamorig:LeftShoulder");
	SetJntRotDirOBJ(poseTrans, "ll1", "LeftArm", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:LeftArm", gVec3(10,0,0), gVec3(0,0,10));
	SetJntRotDirOBJ(poseTrans, "ll2", "LeftForeArm", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:LeftForeArm", gVec3(10,0,0), gVec3(0,0,10));
	SetJntRotDirOBJ(poseTrans, "ll3", "LeftHand", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:LeftHand", gVec3(10,0,0), gVec3(0,0,10));
	
	//right arm chain
	SetJntRotDirOBJ(poseTrans, "rl0", "RightShoulder", "mixamorig:RightShoulder");
	SetJntRotDirOBJ(poseTrans, "rl1", "RightArm", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:RightArm", gVec3(10,0,0), gVec3(0,0,10));
	SetJntRotDirOBJ(poseTrans, "rl2", "RightForeArm", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:RightForeArm", gVec3(10,0,0), gVec3(0,0,10));
	SetJntRotDirOBJ(poseTrans, "rl3", "RightHand", gVec3(10,0,0), gVec3(0,0,10), "mixamorig:RightHand", gVec3(10,0,0), gVec3(0,0,10));


	double weightDir = 1.;//importance of direction vector in pose transfer
	double weightPos = 1.01;//importance of end-effector orientation in pose transfer


	//direction objectives
	poseTrans->addDirectionObjective("t0", "t1", weightDir);
	poseTrans->addDirectionObjective("t1", "t2", weightDir);
	poseTrans->addDirectionObjective("t2", "t3", weightDir);
	poseTrans->addDirectionObjective("t3", "t4", weightDir);
	poseTrans->addDirectionObjective("t4", "t5", weightDir);
	
	//left leg chain 
	poseTrans->addDirectionObjective("la0", "la1", weightDir*5.0);
	poseTrans->addDirectionObjective("la1", "la2", weightDir*5.0);
	//poseTrans->addDirectionObjective("la2", "la3", weightDir*5.0);

	//right leg chain
	poseTrans->addDirectionObjective("ra0", "ra1", weightDir*5.0);
	poseTrans->addDirectionObjective("ra1", "ra2", weightDir*5.0);
	//poseTrans->addDirectionObjective("ra2", "ra3", weightDir*5.0);

	//right arm chain
	poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
	poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
	poseTrans->addDirectionObjective("rl2", "rl3", weightDir);
	
	//right arm chain
	poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
	poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
	poseTrans->addDirectionObjective("ll2", "ll3", weightDir);
	
	//pelvis position
	poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0)); 

}
void initTransfer_Kinect(bCharacter* src, bCharacter* tar) {

	double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hip")->frame().trn().y();
	double maxHeightT = tar->findLink("mixamorig:Head")->frame().trn().y() - tar->findLink("mixamorig:Spine1")->frame().trn().y();
	double scale = maxHeightT / maxHeight;

	poseTrans = new mgPoseTransfer(src, tar);
	poseTrans->scale = scale;
	// you need to manually match the src chracter joint name and corresponding tar character joint name
	//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
	SetJntRotDirOBJ(poseTrans, "t0", "Hip", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t1", "LowerSpine", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t2", "MiddleSpine", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t3", "Chest", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//left leg chain
	SetJntRotDirOBJ(poseTrans, "la0", "LThigh", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "la1", "LShin", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "la2", "LFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "la3", "LToe", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//right leg chain
	SetJntRotDirOBJ(poseTrans, "ra0", "RThigh", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ra1", "RShin", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ra2", "RFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ra3", "RToe", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//left arm chain
	SetJntRotDirOBJ(poseTrans, "ll0", "LClavicle", "mixamorig:LeftShoulder");
	SetJntRotDirOBJ(poseTrans, "ll1", "LShoulder", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ll2", "LForearm", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ll3", "LHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//right arm chain
	SetJntRotDirOBJ(poseTrans, "rl0", "RClavicle", "mixamorig:RightShoulder");
	SetJntRotDirOBJ(poseTrans, "rl1", "RShoulder", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "rl2", "RForearm", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "rl3", "RHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10));


	double weightDir = 1.;//importance of direction vector in pose transfer
	double weightPos = 1.01;//importance of end-effector orientation in pose transfer


							//direction objectives
	poseTrans->addDirectionObjective("t0", "t1", weightDir);
	poseTrans->addDirectionObjective("t1", "t2", weightDir);
	poseTrans->addDirectionObjective("t2", "t3", weightDir);
	poseTrans->addDirectionObjective("t3", "t4", weightDir);
	poseTrans->addDirectionObjective("t4", "t5", weightDir);

	//left leg chain 
	poseTrans->addDirectionObjective("la0", "la1", weightDir*5.0);
	poseTrans->addDirectionObjective("la1", "la2", weightDir*5.0);
	poseTrans->addDirectionObjective("la2", "la3", weightDir*5.0);

	//right leg chain
	poseTrans->addDirectionObjective("ra0", "ra1", weightDir*5.0);
	poseTrans->addDirectionObjective("ra1", "ra2", weightDir*5.0);
	poseTrans->addDirectionObjective("ra2", "ra3", weightDir*5.0);

	//right arm chain
	poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
	poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
	poseTrans->addDirectionObjective("rl2", "rl3", weightDir);

	//right arm chain
	poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
	poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
	poseTrans->addDirectionObjective("ll2", "ll3", weightDir);

	//pelvis position
	poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

}
void initTransfer_Mocap(bCharacter* src, bCharacter* tar) {

	double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hips")->frame().trn().y();
	double maxHeightT = tar->findLink("mixamorig:Head")->frame().trn().y() - tar->findLink("mixamorig:Spine1")->frame().trn().y();
	double scale = maxHeightT / maxHeight;

	poseTrans = new mgPoseTransfer(src, tar);
	poseTrans->scale = scale;
	// you need to manually match the src chracter joint name and corresponding tar character joint name
	//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
	SetJntRotDirOBJ(poseTrans, "t0", "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t1", "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t2", "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
	//SetJntRotDirOBJ(poseTrans, "t3", "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//left leg chain
	SetJntRotDirOBJ(poseTrans, "ll0", "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ll1", "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ll2", "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ll3", "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//right leg chain
	SetJntRotDirOBJ(poseTrans, "rl0", "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "rl1", "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "rl2", "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "rl3", "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//left arm chain
	SetJntRotDirOBJ(poseTrans, "la0", "LeftShoulder", "mixamorig:LeftShoulder");
	SetJntRotDirOBJ(poseTrans, "la1", "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "la2", "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "la3", "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//right arm chain
	SetJntRotDirOBJ(poseTrans, "ra0", "RightShoulder", "mixamorig:RightShoulder");
	SetJntRotDirOBJ(poseTrans, "ra1", "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ra2", "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ra3", "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "mixamorig:RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10));


	double weightDir = 1.;//importance of direction vector in pose transfer
	double weightPos = 1.01;//importance of end-effector orientation in pose transfer


							//direction objectives
	poseTrans->addDirectionObjective("t0", "t1", weightDir);
	poseTrans->addDirectionObjective("t1", "t2", weightDir);
	poseTrans->addDirectionObjective("t2", "t4", weightDir);
	//poseTrans->addDirectionObjective("t3", "t4", weightDir);
	poseTrans->addDirectionObjective("t4", "t5", weightDir);

	//left leg chain 
	poseTrans->addDirectionObjective("ll0", "ll1", weightDir*5.0);
	poseTrans->addDirectionObjective("ll1", "ll2", weightDir*5.0);
	poseTrans->addDirectionObjective("la2", "la3", weightDir*5.0);

	//right leg chain
	poseTrans->addDirectionObjective("rl0", "rl1", weightDir*5.0);
	poseTrans->addDirectionObjective("rl1", "rl2", weightDir*5.0);
	poseTrans->addDirectionObjective("rl2", "rl3", weightDir*5.0);

	//right arm chain
	poseTrans->addDirectionObjective("ra0", "ra1", weightDir);
	poseTrans->addDirectionObjective("ra1", "ra2", weightDir);
	poseTrans->addDirectionObjective("ra2", "ra3", weightDir);

	//right arm chain
	poseTrans->addDirectionObjective("la0", "la1", weightDir);
	poseTrans->addDirectionObjective("la1", "la2", weightDir);
	poseTrans->addDirectionObjective("la2", "la3", weightDir);

	//pelvis position
	poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

}
void initTransfer_Mixamo9(bCharacter* src, bCharacter* tar) {

	double maxHeight = src->findLink("mixamorig9:Head")->frame().trn().y() - src->findLink("mixamorig9:Hips")->frame().trn().y();
	double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Spine1")->frame().trn().y();
	double scale = maxHeightT / maxHeight;

	poseTrans = new mgPoseTransfer(src, tar);
	poseTrans->scale = scale;
	// you need to manually match the src chracter joint name and corresponding tar character joint name
	//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
	SetJntRotDirOBJ(poseTrans, "t0", "mixamorig9:Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t1", "mixamorig9:Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t2", "mixamorig9:Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t3", "mixamorig9:Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t4", "mixamorig9:Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t5", "mixamorig9:Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//left leg chain
	SetJntRotDirOBJ(poseTrans, "ll0", "mixamorig9:LeftUpLeg", gVec3(-10, 0, 0), gVec3(0, 0, 10), gVec3(0, -10, 0), "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "ll1", "mixamorig9:LeftLeg", gVec3(-10, 0, 0), gVec3(0, 0, 10), gVec3(0, -10, 0), "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "ll2", "mixamorig9:LeftFoot", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "ll3", "mixamorig9:LeftToeBase", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10), "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));

	//right leg chain
	SetJntRotDirOBJ(poseTrans, "rl0", "mixamorig9:RightUpLeg", gVec3(-10, 0, 0), gVec3(0, 0, 10), gVec3(0, -10, 0), "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "rl1", "mixamorig9:RightLeg", gVec3(-10, 0, 0), gVec3(0, 0, 10), gVec3(0, -10, 0), "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "rl2", "mixamorig9:RightFoot", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10), "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "rl3", "mixamorig9:RightToeBase", gVec3(-10, 0, 0), gVec3(0, 10, 0), gVec3(0, 0, 10), "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));

	//left arm chain
	SetJntRotDirOBJ(poseTrans, "la0", "mixamorig9:LeftShoulder", "LeftShoulder");
	SetJntRotDirOBJ(poseTrans, "la1", "mixamorig9:LeftArm", gVec3(0, 10, 0), gVec3(-10, 0, 0), gVec3(0, 0, -10), "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "la2", "mixamorig9:LeftForeArm", gVec3(0, 10, 0), gVec3(-10, 0, 0), gVec3(0, 0, -10), "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "la3", "mixamorig9:LeftHand", gVec3(0, 10, 0), gVec3(-10, 0, 0), gVec3(0, 0, -10), "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));

	//right arm chain
	SetJntRotDirOBJ(poseTrans, "ra0", "mixamorig9:RightShoulder", "RightShoulder");
	SetJntRotDirOBJ(poseTrans, "ra1", "mixamorig9:RightArm", gVec3(0, -10, 0), gVec3(10, 0, 0), gVec3(0, 0, -10), "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "ra2", "mixamorig9:RightForeArm", gVec3(0, -10, 0), gVec3(10, 0, 0), gVec3(0, 0, -10), "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));
	SetJntRotDirOBJ(poseTrans, "ra3", "mixamorig9:RightHand", gVec3(0, -10, 0), gVec3(10, 0, 0), gVec3(0, 0, -10), "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), gVec3(0, 10, 0));


	double weightDir = 1.;//importance of direction vector in pose transfer
	double weightPos = 1.01;//importance of end-effector orientation in pose transfer


							//direction objectives
	poseTrans->addDirectionObjective("t0", "t1", weightDir);
	poseTrans->addDirectionObjective("t1", "t2", weightDir);
	poseTrans->addDirectionObjective("t2", "t4", weightDir);
	poseTrans->addDirectionObjective("t3", "t4", weightDir);
	poseTrans->addDirectionObjective("t4", "t5", weightDir);

	//left leg chain 
	poseTrans->addDirectionObjective("ll0", "ll1", weightDir * 5.0);
	poseTrans->addDirectionObjective("ll1", "ll2", weightDir * 5.0);
	poseTrans->addDirectionObjective("la2", "la3", weightDir * 5.0);

	//right leg chain
	poseTrans->addDirectionObjective("rl0", "rl1", weightDir * 5.0);
	poseTrans->addDirectionObjective("rl1", "rl2", weightDir * 5.0);
	poseTrans->addDirectionObjective("rl2", "rl3", weightDir * 5.0);

	//right arm chain
	poseTrans->addDirectionObjective("ra0", "ra1", weightDir);
	poseTrans->addDirectionObjective("ra1", "ra2", weightDir);
	poseTrans->addDirectionObjective("ra2", "ra3", weightDir);

	//right arm chain
	poseTrans->addDirectionObjective("la0", "la1", weightDir);
	poseTrans->addDirectionObjective("la1", "la2", weightDir);
	poseTrans->addDirectionObjective("la2", "la3", weightDir);

	//pelvis position
	poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

}
void initTransfer_CMU(bCharacter* src, bCharacter* tar) {

	double maxHeight = src->findLink("Head")->frame().trn().y() - src->findLink("Hips")->frame().trn().y();
	double maxHeightT = tar->findLink("Head")->frame().trn().y() - tar->findLink("Spine1")->frame().trn().y();
	double scale = maxHeightT / maxHeight;

	poseTrans = new mgPoseTransfer(src, tar);
	poseTrans->scale = scale;
	// you need to manually match the src chracter joint name and corresponding tar character joint name
	//poseTrans->addPoint("t0", *src->findLink("Hips"), gVec3(0, 0, 0), *tar->findLink("Hips"), gVec3(0, 0, 0));
	SetJntRotDirOBJ(poseTrans, "t0", "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10), "Hips", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t1", "LowerBack", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t2", "Spine", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t3", "Spine1", gVec3(10, 0, 0), gVec3(0, 0, 10), "Spine2", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t4", "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10), "Neck", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "t5", "Head", gVec3(10, 0, 0), gVec3(0, 0, 10), "Head", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//left leg chain
	SetJntRotDirOBJ(poseTrans, "la0", "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "la1", "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "la2", "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "la3", "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//right leg chain
	SetJntRotDirOBJ(poseTrans, "ra0", "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightUpLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ra1", "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightLeg", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ra2", "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightFoot", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ra3", "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightToeBase", gVec3(10, 0, 0), gVec3(0, 0, 10));


	//left arm chain
	SetJntRotDirOBJ(poseTrans, "ll0", "LeftShoulder", "LeftShoulder");
	SetJntRotDirOBJ(poseTrans, "ll1", "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ll2", "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "ll3", "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "LeftHand", gVec3(10, 0, 0), gVec3(0, 0, 10));

	//right arm chain
	SetJntRotDirOBJ(poseTrans, "rl0", "RightShoulder", "RightShoulder");
	SetJntRotDirOBJ(poseTrans, "rl1", "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "rl2", "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightForeArm", gVec3(10, 0, 0), gVec3(0, 0, 10));
	SetJntRotDirOBJ(poseTrans, "rl3", "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10), "RightHand", gVec3(10, 0, 0), gVec3(0, 0, 10));


	double weightDir = 1.;//importance of direction vector in pose transfer
	double weightPos = 1.01;//importance of end-effector orientation in pose transfer


							//direction objectives
	poseTrans->addDirectionObjective("t0", "t1", weightDir);
	poseTrans->addDirectionObjective("t1", "t2", weightDir);
	poseTrans->addDirectionObjective("t2", "t3", weightDir);
	poseTrans->addDirectionObjective("t3", "t4", weightDir);
	poseTrans->addDirectionObjective("t4", "t5", weightDir);

	//left leg chain 
	poseTrans->addDirectionObjective("la0", "la1", weightDir * 5.0);
	poseTrans->addDirectionObjective("la1", "la2", weightDir * 5.0);
	poseTrans->addDirectionObjective("la2", "la3", weightDir * 5.0);

	//right leg chain
	poseTrans->addDirectionObjective("ra0", "ra1", weightDir * 5.0);
	poseTrans->addDirectionObjective("ra1", "ra2", weightDir * 5.0);
	poseTrans->addDirectionObjective("ra2", "ra3", weightDir * 5.0);

	//right arm chain
	poseTrans->addDirectionObjective("rl0", "rl1", weightDir);
	poseTrans->addDirectionObjective("rl1", "rl2", weightDir);
	poseTrans->addDirectionObjective("rl2", "rl3", weightDir);

	//right arm chain
	poseTrans->addDirectionObjective("ll0", "ll1", weightDir);
	poseTrans->addDirectionObjective("ll1", "ll2", weightDir);
	poseTrans->addDirectionObjective("ll2", "ll3", weightDir);

	//pelvis position
	poseTrans->addDesiredObjective("t0", 1.0, gVec3(0, 0, 0));

}
std::string puttap(int n) {
	std::string output;
	for (int i = 0; i < n; i++) {
		output += "\t";
	}
	return output;
}

enum RotSeq { zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy, xzx };

void savejoint(FILE* fp, mgBone* bone, int i ) {
	std::string new_t = puttap(i);
	fprintf(fp, "%sJOINT %s\n", new_t.c_str(), bone->name.c_str());
	fprintf(fp, "%s{\n", new_t.c_str());

	gVec3 pos = bone->H.trn();
	std::cout << bone->order <<std::endl;
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
	new_t = puttap(i+1); // i+1 times tap (index +1 )  == child tap num

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

	if (end_site == true) {
		new_t = puttap(i + 1);
		fprintf(fp, "%sEnd Site\n", new_t.c_str());
		fprintf(fp, "%s{\n", new_t.c_str());
		new_t = puttap(i + 2);
		fprintf(fp, "%sOFFSET %g %g %g\n", new_t.c_str(), 0.0, 0.0, 0.0);
		new_t = puttap(i + 1);
		fprintf(fp, "%s}\n", new_t.c_str());
	}

	new_t = puttap(i);
	fprintf(fp, "%s}\n", new_t.c_str());

	//
}

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
			2 * (q.x()*q.y() + q.w()*q.z()),
			q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
			-2 * (q.x()*q.z() - q.w()*q.y()),
			2 * (q.y()*q.z() + q.w()*q.x()),
			q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
			res);
		break;

	case zyz:
		twoaxisrot(2 * (q.y()*q.z() - q.w()*q.x()),
			2 * (q.x()*q.z() + q.w()*q.y()),
			q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
			2 * (q.y()*q.z() + q.w()*q.x()),
			-2 * (q.x()*q.z() - q.w()*q.y()),
			res);
		break;

	case zxy:
		threeaxisrot(-2 * (q.x()*q.y() - q.w()*q.z()),
			q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
			2 * (q.y()*q.z() + q.w()*q.x()),
			-2 * (q.x()*q.z() - q.w()*q.y()),
			q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
			res);
		break;

	case zxz:
		twoaxisrot(2 * (q.x()*q.z() + q.w()*q.y()),
			-2 * (q.y()*q.z() - q.w()*q.x()),
			q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
			2 * (q.x()*q.z() - q.w()*q.y()),
			2 * (q.y()*q.z() + q.w()*q.x()),
			res);
		break;

	case yxz:
		threeaxisrot(2 * (q.x()*q.z() + q.w()*q.y()),
			q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
			-2 * (q.y()*q.z() - q.w()*q.x()),
			2 * (q.x()*q.y() + q.w()*q.z()),
			q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
			res);
		break;

	case yxy:
		twoaxisrot(2 * (q.x()*q.y() - q.w()*q.z()),
			2 * (q.y()*q.z() + q.w()*q.x()),
			q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
			2 * (q.x()*q.y() + q.w()*q.z()),
			-2 * (q.y()*q.z() - q.w()*q.x()),
			res);
		break;

	case yzx:
		threeaxisrot(-2 * (q.x()*q.z() - q.w()*q.y()),
			q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
			2 * (q.x()*q.y() + q.w()*q.z()),
			-2 * (q.y()*q.z() - q.w()*q.x()),
			q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
			res);
		break;

	case yzy:
		twoaxisrot(2 * (q.y()*q.z() + q.w()*q.x()),
			-2 * (q.x()*q.y() - q.w()*q.z()),
			q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
			2 * (q.y()*q.z() - q.w()*q.x()),
			2 * (q.x()*q.y() + q.w()*q.z()),
			res);
		break;

	case xyz:
		threeaxisrot(-2 * (q.y()*q.z() - q.w()*q.x()),
			q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
			2 * (q.x()*q.z() + q.w()*q.y()),
			-2 * (q.x()*q.y() - q.w()*q.z()),
			q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
			res);
		break;

	case xyx:
		twoaxisrot(2 * (q.x()*q.y() + q.w()*q.z()),
			-2 * (q.x()*q.z() - q.w()*q.y()),
			q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
			2 * (q.x()*q.y() - q.w()*q.z()),
			2 * (q.x()*q.z() + q.w()*q.y()),
			res);
		break;

	case xzy:
		threeaxisrot(2 * (q.y()*q.z() + q.w()*q.x()),
			q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),
			-2 * (q.x()*q.y() - q.w()*q.z()),
			2 * (q.x()*q.z() + q.w()*q.y()),
			q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
			res);
		break;

	case xzx:
		twoaxisrot(2 * (q.x()*q.z() - q.w()*q.y()),
			2 * (q.x()*q.y() + q.w()*q.z()),
			q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
			2 * (q.x()*q.z() + q.w()*q.y()),
			-2 * (q.x()*q.y() - q.w()*q.z()),
			res);
		break;
	default:
		std::cout << "Unknown rotation sequence" << std::endl;
		break;
	}
}


void savejoint(FILE* fp, gLink* bone, int i,mgBone::_AXISORDER axisorder) {
	std::string new_t = puttap(i);
	fprintf(fp, "%sJOINT %s\n", new_t.c_str(), bone->name());
	fprintf(fp, "%s{\n", new_t.c_str());

	gVec3 pos = bone->localFrameDefault().trn();
	
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
		savejoint(fp, child, i + 1, axisorder);
		end_site = false;
	}

	if (end_site == true) {
		new_t = puttap(i + 1);
		fprintf(fp, "%sEnd Site\n", new_t.c_str());
		fprintf(fp, "%s{\n", new_t.c_str());
		new_t = puttap(i + 2);
		fprintf(fp, "%sOFFSET %g %g %g\n", new_t.c_str(), 0.0, 0.0, 0.0);
		new_t = puttap(i + 1);
		fprintf(fp, "%s}\n", new_t.c_str());
	}

	new_t = puttap(i);
	fprintf(fp, "%s}\n", new_t.c_str());

	//
}
int saveBVHFile(bCharacter * character, char*filename, arma::mat Quat_Motion, mgBone::_AXISORDER axisorder, float frametime = 1.0f / 30.0f) {
	
	// character -> hierarchy, Offset -> Hierarchy
	// rotation order -> Motion
	FILE *fp = fopen(filename, "w");
	if (!fp)
	{
		printf("error: cannot open a file(%s) to write.\n", filename);
		return -1;
	}
	// Root Bone Hierarchy
	gLink* rootLink = character->link(0);
	gVec3 pos = rootLink->localFrameDefault().trn();
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
		gLink *child = rootLink->child(j);
		savejoint(fp, child, 1,axisorder);
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

void SrcSceneTotal(char*srcfilename, char*srcCharactertxt, osg::ref_ptr<osgViewer::Viewer> viewer, osg::ref_ptr<osg::Group> debugGroup2) {
	std::cout << "|--- load src motion file ---|" << std::endl;
	// get src motion file
	MotionLoader loader;
	loader.loadMotionFile(srcfilename);
	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();

	std::cout << "|--- load src character file ---|" << std::endl;
	// load src bCharacter from txt file
	bCharacter* src = new bCharacter();
	bCharacterSim* srcSim = new bCharacterSim(src);
	gBDOSGSystem* srcVis = new gBDOSGSystem();
	if (loadAvatarModelFromFile(src, srcSim, srcVis, srcCharactertxt, 1.0) != TRUE) {
		std::cout << "|---- write src character file ---|" << std::endl;
		const double mass = 70.;
		mgSkeletonToBCharacter::saveToBCharacter(skeleton, srcCharactertxt, mass);
		std::cout << "|---- load new src character file : warning should check a free joint ---|" << std::endl;
		loadAvatarModelFromFile(src, srcSim, srcVis, srcCharactertxt, 1.0);
	}

	std::cout << "|--- load src motion file ---|" << std::endl;
	// load src motion (pose sequence) : nMotion is a total number of frame for motion data
	arma::mat refCoord(src->sizeCompactCoordArray(), motion->nMotion, arma::fill::zeros);
	for (int f = 0; f < motion->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			src,
			skeleton,
			motion->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}

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
	iter = 0; //global value 
	debugGroup2->addChild(srcVis->getOSGGroup());
	debugGroup2->addChild(tarVis->getOSGGroup());
	while (bool_motionstop) {
		viewer->frame(simulationTime);

		if (iter < 0)
			iter = 0;
		if (iter >= motion->nMotion)
			iter = 0;

		src->setFromCompactCoordArray(refCoord.col(iter));
		src->updateKinematicsUptoPos();
		src->updateKinematicBodiesOfCharacterSim();

		srcVis->update();

		bool_motionretarget_save = false;
		if (bool_go == true) {
			simulationTime += 1. / 30.;
			iter++;
		}
		else if (bool_go_back == true) {
			simulationTime -= 1. / 30.;
			iter--;
		}

		simulationTime += 1. / 30.;
		iter++;

	}
	debugGroup2->removeChildren(0, debugGroup2->getNumChildren());
	viewer->done();
	bool_motionstop = true;
	bool_motionretarget = false;
	bool_motionretarget_save = false;
}

void retargetSrc_saveBVH(char* srcfilename, char* srcCharactertxt, char* tarBVHfilename, RetargetPair RP) {
	// get src motion file
	MotionLoader loader;
	loader.loadMotionFile(srcfilename);
	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();


	// load src bCharacter from txt file
	bCharacter* src = new bCharacter();
	bCharacterSim* srcSim = new bCharacterSim(src);
	gBDOSGSystem* srcVis = new gBDOSGSystem();
	if (loadAvatarModelFromFile(src, srcSim, srcVis, srcCharactertxt, 1.0) != TRUE) {
		std::cout << "|---- write src character file ---|" << std::endl;
		const double mass = 70.;
		mgSkeletonToBCharacter::saveToBCharacter(skeleton, srcCharactertxt, mass);
		std::cout << "|---- load new src character file : warning should check a free joint ---|" << std::endl;
		loadAvatarModelFromFile(src, srcSim, srcVis, srcCharactertxt, 1.0);
	}

	// load src motion (pose sequence) : nMotion is a total number of frame for motion data
	arma::mat refCoord(src->sizeCompactCoordArray(), motion->nMotion, arma::fill::zeros);
	for (int f = 0; f < motion->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			src,
			skeleton,
			motion->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}

	// generate tar bCharacter from txt file
	bCharacter* tar = new bCharacter();
	bCharacterSim* tarSim = new bCharacterSim(tar);
	gBDOSGSystem* tarVis = new gBDOSGSystem();
	double check_load = loadAvatarModelFromFile(tar, tarSim, tarVis, "mixamo_rest.txt", 1.0);

	// initialization of poseTransfer
	switch (RP) {
	case Mocap2MixamoRest:
		initTransfer_Mocap(src, tar);
		break;
	case Axis2MixamoRest:
		initTransfer(src, tar);
		break;
	case Kinect2MixamoRest:
		initTransfer_Kinect(src, tar);
		break;
	case Mixamo92MixamoRest:
		initTransfer_Mixamo9(src, tar);
		break;
	case CMU2MixamoRest:
		initTransfer_CMU(src,tar);
		break;
	}
	std::cout << "|--- RETARGET & SAVE SCENE ---|" << std::endl;
	std::cout << "|--- retarget is start ---|" << std::endl;
	// progress bar
	const char bar = '='; // 프로그레스바 문자  
	const char blank = ' '; // 비어있는 프로그레스바 문자  
	const int LEN = 20; // 프로그레스바 길이  
	const int MAX = motion->nMotion; // 진행작업 최대값 
	const int SPEED = 50; // 카운트 증가 대기시간  
	int count = 0; // 현재 진행된 작업  
	int i; // 반복문 전용 변수  
	float tick = (float)100 / LEN; // 몇 %마다 프로그레스바 추가할지 계산 
	int bar_count; // 프로그레스바 갯수 저장 변수  
	float percent; // 퍼센트 저장 변수

	// retarget src motion ( do what we need )
	arma::mat tarQuaternions(tar->sizeSafeCoordArray(), motion->nMotion, arma::fill::zeros);
	for (int iter = 0; iter < tarQuaternions.n_cols; iter++) {
		
		//printf("\r%d/%d [", count, MAX); // 진행 상태 출력  
		//percent = (float)count / MAX * 100; // 퍼센트 계산  
		//bar_count = percent / tick; // 프로그레스바 갯수 계산  
		//for (i = 0; i<LEN; i++) { // LEN길이의 프로그레스바 출력  
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

		if (tarQuaternions.n_rows != (tar->numLinks() * 4 + 3))
			std::cout << " dof is not matched! see the matrix and character" << std::endl;
		//src pose 
		src->setFromCompactCoordArray(refCoord.col(iter));
		src->updateKinematicsUptoPos();
		src->updateKinematicBodiesOfCharacterSim();

	
		//tar pose retargeting
		poseTrans->desiredPoints[0].pos_desired = poseTrans->scale * src->link(0)->frame().trn();
		gXMat offset; offset.setTrn(0, 0, 0);
		poseTrans->transferPoseLevMar(offset);
		tar->updateKinematicsUptoPos();
		tar->updateKinematicBodiesOfCharacterSim();

		//tar pose save
		arma::vec Quat_pose; Quat_pose.resize(tar->numLinks() * 4 + 3);

		// post processing (foot height)
		double min_f_height = std::min(tar->findLink("RightToeBase")->frame().trn().y(), tar->findLink("LeftToeBase")->frame().trn().y());
		min_f_height = std::min(min_f_height, std::min(tar->findLink("RightHand")->frame().trn().y(), tar->findLink("LeftHand")->frame().trn().y()));
		if (min_f_height < 1e-3) {
			gVec3 new_pelvis = tar->link(0)->frame().trn();
			new_pelvis.setY(new_pelvis.y() + std::abs(min_f_height));
			tar->setBasePosition(new_pelvis);
			tar->updateKinematicsUptoPos();
			tar->updateKinematicBodiesOfCharacterSim();
		}

		tar->getSafeCoordArray(Quat_pose);
		tarQuaternions(0, iter, arma::SizeMat(Quat_pose.n_elem, 1)) = Quat_pose;
	}
	std::cout << "|--- retarget is finished & save bvh is starting ---|" << std::endl;
	saveBVHFile(tar, tarBVHfilename, tarQuaternions, mgBone::_AXISORDER::ZYX);
	std::cout << "|--- save is finished ---|" << std::endl;
}

void RetargetingSceneTotal(char*srcfilename, char*srcCharactertxt, char*targetBVHFile, RetargetPair RP, osg::ref_ptr<osgViewer::Viewer> viewer, osg::ref_ptr<osg::Group> debugGroup2) {
	
	std::cout << "|--- load src motion file ---|" << std::endl;
	// get src motion file
	MotionLoader loader;
	loader.loadMotionFile(srcfilename);
	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();
	
	std::cout << "|--- load src character file ---|" << std::endl;
	// load src bCharacter from txt file
	bCharacter* src = new bCharacter();
	bCharacterSim* srcSim = new bCharacterSim(src);
	gBDOSGSystem* srcVis = new gBDOSGSystem();
	if (loadAvatarModelFromFile(src, srcSim, srcVis, srcCharactertxt, 1.0) != TRUE) {
		std::cout << "|---- write src character file ---|" << std::endl;
		const double mass = 70.;
		mgSkeletonToBCharacter::saveToBCharacter(skeleton, srcCharactertxt, mass);
		std::cout << "|---- load new src character file : warning should check a free joint ---|" << std::endl;
		loadAvatarModelFromFile(src, srcSim, srcVis, srcCharactertxt, 1.0);
	}
	
	std::cout << "|--- load src motion file ---|" << std::endl;
	// load src motion (pose sequence) : nMotion is a total number of frame for motion data
	arma::mat refCoord(src->sizeCompactCoordArray(), motion->nMotion, arma::fill::zeros);
	for (int f = 0; f < motion->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			src,
			skeleton,
			motion->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}
	
	std::cout << "|--- load tar character file ---|" << std::endl;
	// generate tar bCharacter from txt file
	bCharacter* tar = new bCharacter();
	bCharacterSim* tarSim = new bCharacterSim(tar);
	gBDOSGSystem* tarVis = new gBDOSGSystem();
	double check_load = loadAvatarModelFromFile(tar, tarSim, tarVis, "mixamo_rest.txt", 1.0);
	
	std::cout << "|--- init retarget func ---|" << std::endl;
	// initialization of poseTransfer
	switch (RP) {
	case Mocap2MixamoRest:
		initTransfer_Mocap(src, tar);
		break;
	case Axis2MixamoRest:
		initTransfer(src, tar);
		break;
	case Kinect2MixamoRest:
		initTransfer_Kinect(src, tar);
		break;
	case Mixamo92MixamoRest:
		initTransfer_Mixamo9(src, tar);
		break;
	case CMU2MixamoRest:
		initTransfer_CMU(src, tar);
		break;
	}
	std::cout << "|--- initialization process is finished ---|\n" << std::endl;

	std::cout << "|--- UPDATE RETARGET SCENE       ---|" << std::endl;
	std::cout << "|--- press  Y to forward motion  ---|" << std::endl;
	std::cout << "|--- press  H to backward motion ---|" << std::endl;
	std::cout << "|--- press  U to forward 1 frame ---|" << std::endl;
	std::cout << "|--- press  J to backward 1 frame---|" << std::endl;
	std::cout << "|--- press  P to see next bvh    ---|" << std::endl;
	std::cout << "|--- toggle R to retarget        ---|" << std::endl;
	std::cout << "|--- press  S to retarget & save ---|\n" << std::endl;
	iter = 0; //global value 
	debugGroup2->addChild(srcVis->getOSGGroup());
	debugGroup2->addChild(tarVis->getOSGGroup());
	while (bool_motionstop) {
		viewer->frame(simulationTime);

		if (iter < 0)
			iter = 0;
		if (iter >= motion->nMotion)
			iter = 0;

		src->setFromCompactCoordArray(refCoord.col(iter));
		src->updateKinematicsUptoPos();
		src->updateKinematicBodiesOfCharacterSim();

		debugGroup->removeChildren(0, debugGroup->getNumChildren());
		for (int p = 0; p < poseTrans->srcPoints.size(); p++) {
			
			poseTrans->tarPoints[p].updateKinematicsUptoPos();
			poseTrans->srcPoints[p].updateKinematicsUptoPos();
			gOSGShape::setColor(osg::Vec4(1.0, 0, 0, 1.0));
			debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(poseTrans->tarPoints[p].posWorld()),5.0));
			gOSGShape::setColor(osg::Vec4(0.0, 0, 1.0, 1.0));
			debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(poseTrans->srcPoints[p].posWorld()), 5.0));

		}

		//motion retargeting and save
		if (bool_motionretarget) {
			//tar pose retargeting
			poseTrans->desiredPoints[0].pos_desired = poseTrans->scale * src->link(0)->frame().trn();
			gXMat offset; offset.setTrn(0, 0, 0);
			poseTrans->transferPoseLevMar(offset);
			tar->updateKinematicsUptoPos();
			tar->updateKinematicBodiesOfCharacterSim();

			// post processing (foot height)
			double min_f_height = std::min(tar->findLink("RightToeBase")->frame().trn().y(), tar->findLink("LeftToeBase")->frame().trn().y());
			if (min_f_height < 1e-3) {
				gVec3 new_pelvis = tar->link(0)->frame().trn();
				new_pelvis.setY(new_pelvis.y() + std::abs(min_f_height));
				tar->setBasePosition(new_pelvis);
				tar->updateKinematicsUptoPos();
				tar->updateKinematicBodiesOfCharacterSim();
			}
		}
		tarVis->update();
		srcVis->update();

		//motion retargeting and save
		if (bool_motionretarget_save) {
			retargetSrc_saveBVH(srcfilename, srcCharactertxt,targetBVHFile, RP);
		}
		bool_motionretarget_save = false;
		if (bool_go == true) {
			simulationTime += 1. / 30.;
			iter++;
		}
		else if (bool_go_back == true) {
			simulationTime -= 1. / 30.;
			iter--;
		}

		simulationTime += 1. / 30.;
		iter++;

	}
	debugGroup2->removeChildren(0, debugGroup2->getNumChildren());
	viewer->done();
	bool_motionstop = true;
	bool_motionretarget = false;
	bool_motionretarget_save = false;
}

struct _finddata_t fd;
int isFileOrDir()
{
	if (fd.attrib & _A_SUBDIR)
		return 0; // 디렉토리면 0 반환
	else
		return 1; // 그밖의 경우는 "존재하는 파일"이기에 1 반환

}
std::vector<char*> files; std::vector<char*>g_fnames;
void FileSearch(char file_path[])
{
	intptr_t handle; int check = 0;
	char* file_path2 = new char[_MAX_PATH];

	strcat(file_path, "\\");
	strcpy(file_path2, file_path);
	strcat(file_path, "*");

	if ((handle = _findfirst(file_path, &fd)) == -1)
	{
		printf("No such file or directory\n");
		return;
	}
	while (_findnext(handle, &fd) == 0)
	{
		char* file_pt = new char [_MAX_PATH];
		char* file_name = new char[_MAX_FNAME];
		// file total path
		strcpy(file_pt, file_path2); 
		strcat(file_pt, fd.name); 
		// file name
		strcpy(file_name, fd.name);
		// 파일인지 디렉토리 인지 식별
		check = isFileOrDir();    

		if (check == 0 && fd.name[0] != '.')
		{
			std::cout << "-* it is a child directory : we assume that it search only current folder " << std::endl;
			//FileSearch(file_pt);    //하위 디렉토리 검색 재귀함수
		}
		else if (check == 1 && fd.size != 0 && fd.name[0] != '.')
		{
			printf("-* filename : %s, \n", fd.name);
			files.push_back(file_pt);

			char* pos = new char[256];
			pos = strchr(file_name, '.');
			*pos = '\0';
			
			g_fnames.push_back(file_name);
		}
	}
	_findclose(handle);

	std::cout << "--*load " << files.size() << " files \n" << std::endl;
}
using namespace std;

vector<string> SplitPath(string path, char sep) { vector<string> out; stringstream stream(path); string temp; while (getline(stream, temp, sep)) { out.push_back(temp); } return out; }

int main(int argc, char **argv)
{
	// construct the viewer.
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	osg::ref_ptr<osg::Group> scene = new osg::Group;
	osg::ref_ptr<osg::MatrixTransform> rootTrans = new osg::MatrixTransform;
	osg::ref_ptr<osg::MatrixTransform> refTrans = new osg::MatrixTransform;

	//initializeVisualization();

	scene->addChild(debugGroup);
	scene->addChild(debugGroup2);
	scene->addChild(debugGroup3);

	viewer->setSceneData(scene);


	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 10;
	traits->y = 10;
	traits->width = 1024;
	traits->height = 768;
	traits->windowDecoration = true;
	traits->supportsResize = false;
	traits->windowName = "test";

	osg::ref_ptr<osg::GraphicsContext> graphicscontext = osg::GraphicsContext::createGraphicsContext(traits);
	graphicscontext->realize();

	viewer->getCamera()->setGraphicsContext(graphicscontext);

	osg::Camera* camera = viewer->getCamera();

	camera->setClearColor(osg::Vec4(1., 1., 1., 1.0));
	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	camera->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(traits->width) / static_cast<double>(traits->height), 1.0f, 10000.0f);

	osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
	viewer->setCameraManipulator(manipulator);
	manipulator->setAutoComputeHomePosition(true);
	//manipulator->setHomePosition(osg::Vec3(-100, 100, -100), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0), false);
	manipulator->home(0);

	int margin = 300;
	int w = margin * 2;
	int h = margin * 2;
	double size = 10;

	double startPoint_h = 0;
	double startPoint_w = 0;

	float width = 1.0f;

	gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
	for (int i = 0; i <= (h / size + 1); i++)
		scene->addChild(gOSGShape::createLineShape(osg::Vec3(startPoint_w, 0.0, i*size + startPoint_h), osg::Vec3(1, 0, 0), w + size, width));
	for (int i = 0; i <= (w / size + 1); i++)
		scene->addChild(gOSGShape::createLineShape(osg::Vec3(i*size + startPoint_w, 0.0, startPoint_h), osg::Vec3(0, 0, 1), h + size, width));
	scene->addChild(gOSGShape::createAxis(5.0, 5.0));

	//initializeVisualization();
	initWorld(); // init btworld for viewing avatar vis system
	//--------------------- initial setup is finished. (you may not need to see upper coding lines)

	// keyboard event handler
	osg::ref_ptr<gEventHandler> handler = new gEventHandler;
	handler->assignKey(osgGA::GUIEventAdapter::KEY_Y, keyEventToggleAnimation);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_H, keyEventToggleAnimationBack);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_U, keyEventOneFrameGo);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_J, keyEventOneFrameBack);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_P, keyEventOneDataGo);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_R, keyEventDoRetarget);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_S, keyEventDoRetargetSave);

	viewer->addEventHandler(handler);
	//--------------------------------------------------VIEWER 설정 ----------------------------------------------------------------------------//
	
	//source file 불러오기
	std::cout << "|-TYPE DIRECTORY PATH-| " << std::endl;
	char source_file_path[_MAX_PATH] = "E:/motiondata/BODYAGENT_BVHFILE/P1"; //"E:/motiondata/Environment"; //"E:/motiondata/Mocap"; //"E:/motiondata/BODYAGENT_BVHFILE/P1";
	char retarget_file_path_origin[_MAX_PATH] = "E:/motiondata/BODYAGENT_BVHFILE/P1/retarget";
	char rootname[_MAX_PATH] = "P1";
	
	int i_type = 0;
	printf(" source file path: ");
	while (1) {
		source_file_path[i_type] = getchar();
		if (source_file_path[i_type++] == '\n') break;
	}
	source_file_path[i_type - 1] = 0;
	printf(" target file path: ");
	i_type = 0;
	while (1) {
		retarget_file_path_origin[i_type] = getchar();
		if (retarget_file_path_origin[i_type++] == '\n') break;
	}
	retarget_file_path_origin[i_type - 1] = 0;

	vector<string> foldername;
	foldername = SplitPath(source_file_path, '\\');
	strcpy(rootname, foldername.back().c_str());
	std::cout << " source character name: " << rootname << std::endl;
	

	//enum RetargetPair { Mocap2MixamoRest, Axis2MixamoRest, Kinect2MixamoRest, Mixamo92MixamoRest, CMU2MixamoRest};
	RetargetPair_s.push_back("Mocap2MixamoRest");
	RetargetPair_s.push_back("Axis2MixamoRest");
	RetargetPair_s.push_back("Kinect2MixamoRest");
	RetargetPair_s.push_back("Mixamo92MixamoRest");
	RetargetPair_s.push_back("CMU2MixamoRest");

	for (int i = 0; i < RetargetPair_s.size(); i++) { std::cout << "index: " << i << " enum name :" << RetargetPair_s[i] << std::endl; }
	printf(" select src-target pair enum: ");
	
	RetargetPair inputRP = Axis2MixamoRest;
	int enum_index = 0;
	while (1) {
		scanf_s("%d", &enum_index);
		
		if (enum_index < 5) {
			switch (enum_index) {
			case 0:
				inputRP = Mocap2MixamoRest;
				std::cout << " input RP : Mocap2MixamoRest \n" << std::endl;
				break;
			case 1:
				inputRP = Axis2MixamoRest;
				std::cout << " input RP : Axis2MixamoRest \n" << std::endl;
				break;
			case 2:
				inputRP = Kinect2MixamoRest;
				std::cout << " input RP : Kinect2MixamoRest \n" << std::endl;
				break;
			case 3:
				inputRP = Mixamo92MixamoRest;
				std::cout << " input RP : Kinect2MixamoRest \n" << std::endl;
				break;
			case 4:
				inputRP = CMU2MixamoRest;
				std::cout<< " input RP : CMU2MixamoRest \n" << std::endl;
				break;
			default:
				std::cout << " Fail! : you shuld type valid one 0: Mocap2mixamorest 1:Axis2 2:Kinect2 3:Mixamo9(leonard) 4:CMU " << std::endl;
				break;
			}
			break;
		}
		else {
			std::cout << " type again(doesn't match with src_tar pair " << std::endl;
		}
	}
	
	//---------------------------------------------------------------------------------
	std::cout << "|-SOURCE FILE ROOT PATH-| " << source_file_path << std::endl;
	files.clear(); g_fnames.clear();
	FileSearch(source_file_path);
	
	//source file 의 개수 만큼 scene 이 존재한다.
	for (int i = 0; i < files.size(); i++) {
		std::cout << "|-SRC FILE-| " << i << " |-NAME-| " << g_fnames[i] << std::endl;
		char retarget_file_path[_MAX_PATH];// retarget 결과를 저장할 경로
		char src_txt_file_path[_MAX_PATH]; // src bvh file 의 캐릭터 정보
	
		//src character txt file 생성 및 불러오기
		strcpy(src_txt_file_path, retarget_file_path_origin);
		strcat(src_txt_file_path, "/");
		strcat(src_txt_file_path, rootname);
		strcat(src_txt_file_path, ".txt");
		
		//tar bvh file name 생성
		strcpy(retarget_file_path, retarget_file_path_origin);
		strcat(retarget_file_path, "/");
		strcat(retarget_file_path, g_fnames[i]);
		strcat(retarget_file_path, "_ret_MIXAMO_rest.bvh");
		
		printf("--src txtfile path : | %s |\n", src_txt_file_path);
		printf("--target file path : | %s |\n", retarget_file_path);
		//start
		//RetargetingSceneTotal(files[i], src_txt_file_path, retarget_file_path, inputRP, viewer, debugGroup2);
		// just retargeting files (don't use viewer)
		retargetSrc_saveBVH(files[i], src_txt_file_path, retarget_file_path, inputRP);
	}
	
	return 0;
}