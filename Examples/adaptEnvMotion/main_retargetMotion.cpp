
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
#include "../MW2/include/Base/gMath.h"
#include <io.h>
#include <experimental/filesystem>

#define NCPtoNSP(n) (n+2) // convert # of control point to # of spline control points, which used in BSpline
#define NCPtoNSE(n) (n-1) // convert # of control point to # of spline segment, which used in BSpline
using namespace std;

double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);


// test
arma::mat refCoord;

osg::ref_ptr<osg::Group> debugGroup = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup2 = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup3 = new osg::Group;

#include "EX3_motionpatch.h"
#include "EX4_retargetmotion.h"
#include "EX4_retargetmotion_aj.h"
//EX4_retargetmotion* ex4_retargetscene = new EX4_retargetmotion();
EX3_motionpatch* ex3_scene = new EX3_motionpatch();
EX4_retargetmotion_aj* ex4_retargetscene = new EX4_retargetmotion_aj();
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

bool bool_set_seat_patch = false;
bool bool_set_backrest_patch = false;

bool bool_set_armrestl_patch = false;
bool bool_set_armrestr_patch = false;

bool bool_set_handrestl_patch = false;
bool bool_set_handrestr_patch = false;

bool bool_set_footrestl_patch = false;
bool bool_set_footrestr_patch = false;

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
	std::cout << " press s " << std::endl;
	bool_motionretarget_save = true;
}
void keyEventDoSetSeatPatch(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	std::cout << " press q " << std::endl;
	bool_set_seat_patch = true;
}
void keyEventDoSetBackrestPatch(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	std::cout << " press w " << std::endl;
	bool_set_backrest_patch = true;
}
void keyEventDoSetArmrestLPatch(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	std::cout << " press e " << std::endl;
	bool_set_armrestl_patch = true;
}
void keyEventDoSetArmrestRPatch(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	std::cout << " press r " << std::endl;
	bool_set_armrestr_patch = true;
}
void keyEventDoSetHandrestLPatch(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	std::cout << " press z " << std::endl;
	bool_set_handrestl_patch = true;
}
void keyEventDoSetHandrestRPatch(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	std::cout << " press x " << std::endl;
	bool_set_handrestr_patch = true;
}
void keyEventDoSetFootrestLPatch(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	std::cout << " press c " << std::endl;
	bool_set_footrestl_patch = true;
}
void keyEventDoSetFootrestRPatch(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	std::cout << " press v " << std::endl;
	bool_set_footrestr_patch = true;

	ex4_retargetscene->poseTrans->desiredPoints[0].weight = 0.0;

}
void keyEventDoSetPatchReset(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	std::cout << " press l " << std::endl;
	ex3_scene->g_interact_plane.seat.exist = false;
	ex3_scene->g_interact_plane.backrest.exist = false;
	ex3_scene->g_interact_plane.armrest_R.exist = false;
	ex3_scene->g_interact_plane.armrest_L.exist = false;
	ex3_scene->g_interact_plane.handrest_R.exist = false;
	ex3_scene->g_interact_plane.handrest_L.exist = false;
	ex3_scene->g_interact_plane.footrest_L.exist = false;
	ex3_scene->g_interact_plane.footrest_R.exist = false;
}
void drawMesh(Eigen::MatrixXd V) {
	for (int r = 0; r < V.rows(); r++) {
		debugGroup->addChild(gOSGShape::createPoint(osg::Vec3(V(r, 0), V(r, 1), V(r, 2)), 5.0));
	}
}
//----------------------------------------------------you may don't need to check upper coding lines (core and individual's functions are mixed. )

#include "Character/bCharacterLoader.h"
#include <osg/BlendFunc>
btBroadphaseInterface* m_broadphase;
btCollisionDispatcher* m_dispatcher;
btConstraintSolver* m_solver;
btDefaultCollisionConfiguration* m_collisionConfiguration;
btDynamicsWorld* m_dynamicsWorld;
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

std::vector<char*> files; std::vector<char*>g_fnames;
struct _finddata_t fd;
int isFileOrDir()
{
	if (fd.attrib & _A_SUBDIR)
		return 0; // 디렉토리면 0 반환
	else
		return 1; // 그밖의 경우는 "존재하는 파일"이기에 1 반환

}

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
		char* file_pt = new char[_MAX_PATH];
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

vector<string> SplitPath(string path, char sep) { vector<string> out; stringstream stream(path); string temp; while (getline(stream, temp, sep)) { out.push_back(temp); } return out; }


int main(int argc, char** argv)
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
		scene->addChild(gOSGShape::createLineShape(osg::Vec3(startPoint_w, 0.0, i * size + startPoint_h), osg::Vec3(1, 0, 0), w + size, width));
	for (int i = 0; i <= (w / size + 1); i++)
		scene->addChild(gOSGShape::createLineShape(osg::Vec3(i * size + startPoint_w, 0.0, startPoint_h), osg::Vec3(0, 0, 1), h + size, width));
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
	handler->assignKey(osgGA::GUIEventAdapter::KEY_T, keyEventDoRetarget);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_S, keyEventDoRetargetSave);

	handler->assignKey(osgGA::GUIEventAdapter::KEY_Q, keyEventDoSetSeatPatch);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_W, keyEventDoSetBackrestPatch);

	handler->assignKey(osgGA::GUIEventAdapter::KEY_E, keyEventDoSetArmrestLPatch);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_R, keyEventDoSetArmrestRPatch);
	
	handler->assignKey(osgGA::GUIEventAdapter::KEY_Z, keyEventDoSetHandrestLPatch);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_X, keyEventDoSetHandrestRPatch);

	handler->assignKey(osgGA::GUIEventAdapter::KEY_C, keyEventDoSetFootrestLPatch);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_V, keyEventDoSetFootrestRPatch);

	handler->assignKey(osgGA::GUIEventAdapter::KEY_L, keyEventDoSetPatchReset);

	
	viewer->addEventHandler(handler);
	//--------------------------------------------------VIEWER 설정 ----------------------------------------------------------------------------//

	//source file 불러오기
	std::cout << "|-TYPE DIRECTORY PATH-| " << std::endl;
	//char source_file_path[_MAX_PATH] = "D:/TJ_develop/motiondata/AxisNeuron/retarget/adapted/"; //"E:/motiondata/Environment"; //"E:/motiondata/Mocap"; //"E:/motiondata/BODYAGENT_BVHFILE/P1";
	char source_file_path[_MAX_PATH] = "D:/TJ_develop/motiondata/mixamo_trimdata/";
	char retarget_file_path_origin[_MAX_PATH] = "D:/TJ_develop/motiondata/mixamo_trimdata/keyData";
	char rootname[_MAX_PATH] = "P1";

	ex4_retargetscene->generateTarCharacterSpecfile("aj_retarget.bvh", "aj_rest.txt"); // 


	//---------------------------------------------------------------------------------
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

	//set source character file
	vector<string> foldername;
	foldername = SplitPath(source_file_path, '\\');
	strcpy(rootname, foldername.back().c_str());
	std::cout << " source character name: " << rootname << std::endl;

	//
	ex4_retargetscene->setAllRP();
	for (int i = 0; i < ex4_retargetscene->RetargetPair_s.size(); i++) { std::cout << "index: " << i << " enum name :" << ex4_retargetscene->RetargetPair_s[i] << std::endl; }
	printf(" select src-target pair enum: ");

	EX4_retargetmotion::RetargetPair inputRP = EX4_retargetmotion::Axis2MixamoRest;
	int enum_index = 0;
	while (1) {
		scanf_s("%d", &enum_index);

		if (enum_index < 7) {
			switch (enum_index) {
			case 0:
				inputRP = EX4_retargetmotion::Mocap2MixamoRest;
				std::cout << " input RP : Mocap2MixamoRest \n" << std::endl;
				break;
			case 1:
				inputRP = EX4_retargetmotion::Axis2MixamoRest;
				std::cout << " input RP : Axis2MixamoRest \n" << std::endl;
				break;
			case 2:
				inputRP = EX4_retargetmotion::Kinect2MixamoRest;
				std::cout << " input RP : Kinect2MixamoRest \n" << std::endl;
				break;
			case 3:
				inputRP = EX4_retargetmotion::Mixamo92MixamoRest;
				std::cout << " input RP : Kinect2MixamoRest \n" << std::endl;
				break;
			case 4:
				inputRP = EX4_retargetmotion::CMU2MixamoRest;
				std::cout << " input RP : CMU2MixamoRest \n" << std::endl;
				break;
			case 5:
				inputRP = EX4_retargetmotion::Amass2MixamoRest;
				std::cout << " input RP : Amass2MixamoRest \n" << std::endl;
				break;
			case 6:
				inputRP = EX4_retargetmotion::HDM2MixamoRest;
				std::cout << " input RP : HDM2MixamoRest \n" << std::endl;
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
	//
	std::cout << "|-SOURCE FILE ROOT PATH-| " << source_file_path << std::endl;
	files.clear(); g_fnames.clear();
	FileSearch(source_file_path);

	//----------------------------------------------------------------------------------
	

	//source file 의 개수 만큼 scene 이 존재한다.
	for (int i = 0; i < files.size(); i++) {
		std::cout << "|-SRC FILE-| " << i << " |-NAME-| " << g_fnames[i] << std::endl;
		
		foldername = SplitPath(files[i], '.');
		string test = foldername.back().c_str();
		if (test == "bvh" || test =="fbx" || test =="csv") {
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
			//init setup
			ex4_retargetscene->BTsetup(m_dynamicsWorld);
			ex4_retargetscene->OSGsetup(viewer, debugGroup, debugGroup2);
			//ex4_retargetscene->SetupScene(files[i], src_txt_file_path, "mixamo_rest.txt");
			ex4_retargetscene->SetupScene(files[i], src_txt_file_path, "aj_rest.txt"); 
			ex4_retargetscene->initRetarget(inputRP);
			
			

			// save process
			bool_motionstop = false;
			ex4_retargetscene->saveRetargetMotion(true, retarget_file_path,ex4_retargetscene->srcFrameTime);
			//ex4_retargetscene->savePreProcessingMotion(true, retarget_file_path);

			// runtime viewer example
			while (bool_motionstop) {
				
				if (iter < 0)
					iter = 0;
				if (iter >= ex4_retargetscene->g_refCoord.n_cols)
					iter = 0;

				ex4_retargetscene->UpdateScene(simulationTime, iter, bool_motionretarget);

				if (bool_motionretarget_save == true) {
					ex4_retargetscene->saveRetargetMotion(true, retarget_file_path, ex4_retargetscene->srcFrameTime);
					bool_motionretarget_save = false;
				}

				if (bool_go == true) {
					simulationTime += 1. / 60.;
					iter++;
				}
				else if (bool_go_back == true) {
					simulationTime -= 1. / 60.;
					iter--;
				}

				debugGroup2->removeChildren(0, debugGroup2->getNumChildren());
			}

			

			////viewer done
			debugGroup->removeChildren(0, debugGroup->getNumChildren());
			
			viewer->done();
			bool_motionstop = true;
		}
		else {
			std::cout << "we only focus on the bvh files " << std::endl;
		}
		
	}

	return 0;
}