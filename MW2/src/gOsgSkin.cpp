//#####################################################################
// Copyright 2010-2015, Taeil Jin, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifdef _WIN64
	#include <Windows.h>
#endif

#include <osgDB/readFile>
#include <osgUtil/UpdateVisitor>
#include <osgAnimation/UpdateBone>
#include "mbs/gMultibodySystem.h"
#include "Visualizer/gOsgUtil.h"
#include "Visualizer/gOsgShape.h"
#include "Visualizer/gOsgSkin.h"
//#include "ReaderWriterFBX.h"

//starting from node, recursively find osgAnimation::Bone node and remove its updateCallback.
void gOsgSkin::traverseAndRemoveUpdateCallbacksFromBones(osg::Node* node)
{	
	//node->setUpdateCallback(NULL);
	osgAnimation::UpdateBone* pUB = dynamic_cast<osgAnimation::UpdateBone*>(node->getUpdateCallback());
	if( pUB )
	{
		node->setUpdateCallback(NULL);
	}


	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if( group ){				
		//traverse children		
		for(int i=0;i<group->getNumChildren();++i)
		{
			 traverseAndRemoveUpdateCallbacksFromBones(group->getChild(i));
		}
	}

	//osg::Geode* geode = node->asGeode();
	//if(geode){
	//	for(int i=0;i<geode->getNumDrawables();++i)
	//	{
	//		//geode->getDrawable(i)->setUpdateCallback(NULL);
	//	}
	//}

}

void* gOsgSkin::findOsgAnimationManager(osg::Node* node)
{
	//osg::NodeCallback* nc = node->getUpdateCallback();
	//osgAnimation::BasicAnimationManager* man = dynamic_cast<osgAnimation::BasicAnimationManager*>(nc);
	
	osgAnimation::BasicAnimationManager* man = dynamic_cast<osgAnimation::BasicAnimationManager*>(node->getUpdateCallback());
	if(man)	return man;

	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if( !group ) return NULL;
	
	//else, traverse children	
	void* re;
	for(int i=0;i<group->getNumChildren();i++)
	{
		re = findOsgAnimationManager(group->getChild(i));
		if(re) return re;
	}
	return NULL;
}

//void gOsgSkin::testAnimManager(osg::Node* node)
void testAnimManager(osg::Node* node)
{
	std::cout << typeid(node).name() << std::endl;
	osgAnimation::AnimationManagerBase* pAM = dynamic_cast<osgAnimation::AnimationManagerBase*>(node->getUpdateCallback());
	if( pAM )
	{
		std::cout << "animManager / needToLink: " << pAM->needToLink() << std::endl;

		node->setUpdateCallback(NULL);
		//if( pAM->needToLink() ) pAM->link(node);
	}

	

	osgAnimation::UpdateBone* pUB = dynamic_cast<osgAnimation::UpdateBone*>(node->getUpdateCallback());
	if( pUB )
	{
		std::cout << "UpdateBone" << std::endl;

		node->setUpdateCallback(NULL);
	}

	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if( !group ) return;

	for(int i=0;i<group->getNumChildren();++i)
	{
		 testAnimManager(group->getChild(i));
	}
}

osgAnimation::Skeleton* gOsgSkin::findFirstOsgAnimationSkeletonNode(osg::Node* node)
{
	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if( !group ) return NULL;

	//see if node is Skeleton
	osgAnimation::Skeleton* re = dynamic_cast<osgAnimation::Skeleton*>(node);
	if( re )  return re; 
	
	//else, traverse children		
	for(int i=0;i<group->getNumChildren();++i)
	{
		 re = findFirstOsgAnimationSkeletonNode(group->getChild(i));
		 if(re) return re;
	}
	return NULL;
}

void gOsgSkin::updateSkin()
{
	if(!visState) return;

	std::vector<osg::Matrix> osMat;

	//
	// update osg Bones
	//
	for(std::vector <MBSToOsgBone>::iterator it=MBSBoneOsgBoneDisplacements.begin();it!=MBSBoneOsgBoneDisplacements.end();++it)
	{
		gLink* mb = it->mb;
		osg::MatrixTransform* ob = it->ob;
		gXMat disp = it->displacement;

		//std::string b_name("skullbase");
		//if (mb->name() == b_name)
		//{
		//	std::cout << "skullbase" << std::endl;
		//}

		gXMat mT = mb->frame();
		mT.setTrn( mT.trn() * invScale );

		gXMat oT = mT*disp;  
		osg::Matrix oT_;
		gXMatToOsgMatrix(oT_,oT); 
		
		osg::Matrix oG_; //osgBone's body matrix
		osgAnimation::Bone* obAsBone = dynamic_cast<osgAnimation::Bone*>(ob);

		if(!mb->parent()) //root
		{
			oG_ = oT_;
			ob->setMatrix(oG_); 
			if(obAsBone)	obAsBone->setMatrixInSkeletonSpace(oG_); 
		}
		else //Bone nodes		
		{			
			if(!obAsBone) assert(0);

			gLink* mpar = mb->parent();

			gXMat mp = mpar->frame();
			mp.setTrn( mp.trn() * invScale );

			gXMat p_oT = mp*MBSBoneOsgBoneDisplacements[mpar->id()].displacement;
			gXMat oG = p_oT.invMult(oT);

			gXMatToOsgMatrix(oG_,oG); 
			
			obAsBone->setMatrix(oG_); 

			osgAnimation::Bone* parAsBone = dynamic_cast<osgAnimation::Bone*>( MBSBoneOsgBoneDisplacements[mpar->id()].ob );

			obAsBone->setMatrixInSkeletonSpace( oG_ * parAsBone->getMatrixInSkeletonSpace() );

			//if(parAsBone) //parent is Bone
			//	obAsBone->setMatrixInSkeletonSpace( oG_ * parAsBone->getMatrixInSkeletonSpace() );
			//else //parent is root, defined as a MatrixTransform
			//{
			//	obAsBone->setMatrixInSkeletonSpace(oG_);
			//}
		}		
		
	}
	
	//
	// update endPointBones (theses are end points such as finger tips)
	//
	for(std::vector<osgAnimation::Bone*>::iterator it=endPointBones.begin();it!=endPointBones.end();++it)
	{
		osgAnimation::Bone* parent = (*it)->getBoneParent();
		if (parent)
			(*it)->setMatrixInSkeletonSpace((*it)->getMatrixInBoneSpace() * parent->getMatrixInSkeletonSpace());
		else
			(*it)->setMatrixInSkeletonSpace((*it)->getMatrixInBoneSpace());		
	}
}

void gOsgSkin::hideSkin()
{
	if(visState){
		visState = false;
		skinGroupNode->removeChild(nodeFile);		
	}
}

void gOsgSkin::unhideSkin()
{
	if(!visState){
		visState = true;
		skinGroupNode->addChild(nodeFile);
	}
}

void gOsgSkin::unloadSkin()
{
	if(visState) hideSkin();
	//dummy->removeChild(nodeFile); //as soon as reference count of nodeFile drops to 0, OSG automatically removes nodeFile
	//dummy = NULL;
	nodeFile = NULL;	
}

//---------------------------------------------
//20150117 Taeil Jin Create
//---------------------------------------------
class findNodeVisitor : public osg::NodeVisitor 
{
public: 
	//findNodeVisitor(); 
	findNodeVisitor(const std::string &searchName) :
	  osg::NodeVisitor(NODE_VISITOR,osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
		searchForName(searchName)
	{
	};
	
	virtual void apply(osg::Node &searchNode)
	{
		if( searchNode.getName() == searchForName)
		{
			foundNodeList.push_back(&searchNode);
		}
		else
		{
			traverse(searchNode);
		}
	};

	osg::Node* getFirst()
	{
		if( !foundNodeList.empty() )
			return foundNodeList[0];
		else 
			return NULL;
	}

// typedef a vector of node pointers for convenience
typedef std::vector<osg::Node*> nodeListType; 
private:

// the name we are looking for
std::string searchForName; 

// List of nodes with names that match the searchForName string
nodeListType foundNodeList;
};


//snap v to nearest axis (X,Y, or Z)
static gVec3 snap2XYZ(gVec3 v)
{
	v.normalize();
	double cx = fabs(v.x());
	double cy = fabs(v.y());
	double cz = fabs(v.z());

	if(cx>cy && cx>cz) //cx is max
	{
		if( v.x() > 0 ) return gVec3UnitX;
		else return -gVec3UnitX;
	}else if(cy>cx && cy>cz) //cy is max
	{
		if( v.y() > 0 ) return gVec3UnitY;
		else return -gVec3UnitY;
	}else //cz is max
	{
		if( v.z() > 0 ) return gVec3UnitZ;
		else return -gVec3UnitZ;
	}	
}

static gRotMat makeRotate(gVec3 from, gVec3 to)
{	
	from.normalize();
	to.normalize();

	double angle = acos( (from,to) );

	if(angle < gPI*0.5){
		gQuat R; R.makeRotate(from,to);
		return R.inRotMatrix();
	}
	else
	{ 
		//first, rotate about the principal axis
		gVec3 n = from%to;
		n = snap2XYZ(n);

		gVec3 fproj = from - (from,n)*n;
		fproj.normalize();
		gVec3 tproj = to - (to,n)*n;
		tproj.normalize();

		double c = (fproj,tproj);
		double s = ((fproj%tproj),n);
		double theta = atan2(s,c);
		gRotMat R1 = gRotMat::exp(n*theta);

		from = R1*from; // rotate from

		//second, fine tune
		gQuat R2_; R2_.makeRotate(from,to);
	
		return R2_.inRotMatrix()*R1;
	}
}

//make rotation matrix that rotates from1 --> to1, and from2 --> to2
static gRotMat makeRotate(gVec3 from1, gVec3 to1, gVec3 from2, gVec3 to2)
{
	from1.normalize();
	from2.normalize();
	to1.normalize();
	to2.normalize();

	gRotMat R1 = makeRotate(from1,to1); //first, rotate to match f1 to t1
	gVec3 Rf2 = R1*from2; //rotated f2

	//then, rotate about t1 to match Rf2 to t2
	Rf2 = Rf2 - (Rf2,to1)*to1; //project Rf2 to the normal plane to t1
	to2 = to2 - (to2,to1)*to1; //project t2 to the normal plane to t1
	
	Rf2.normalize();
	to2.normalize();
	double c = (Rf2,to2);
	double s = (to1, (Rf2%to2));
	double theta = atan2(s,c);
	gRotMat R2 = gRotMat::exp(to1*theta);
	return R2*R1;	//beware of the mult order!
}

/*
int gOsgSkin::loadSkin(osgViewer::ViewerBase* viewer, osg::ref_ptr<osg::Group> scene, const char* filename, const char* name_head, const char* name_right_shoulder)
{
	//
	//read fbx file
	//
	nodeFile = dynamic_cast<osg::Group*>(osgDB::readNodeFile(filename));

	if(!nodeFile){
		std::cout << "ERROR: Loading "<<filename<<" failed"<<std::endl;
		return 0; //error
	}
	dummy = new osg::Group;
	dummy->addChild(nodeFile);

	//skinGroupNode = new osg::Group;	
	//skinGroupNode->addChild(nodeFile);

	scene->addChild(nodeFile);

	//vh->addObject(skinGroupNode);
	//vh->getViewer()->updateTraversal(); //TODO: control verbosity to make it silent..
	viewer->updateTraversal();

	//osg::ref_ptr<osgUtil::UpdateVisitor> uv = new osgUtil::UpdateVisitor();
	//skinGroupNode->accept(*uv);

	nodeFile->setUpdateCallback(NULL);	
	//remove Bones' updateCallbacks to steal control from channels of original animation contained in fbx file	
	//traverseAndRemoveUpdateCallbacksFromBones(skinGroupNode);
	
	// find osgAnimation::Skeleton node to see if there's a skeleton 
	osgAnimation::Skeleton* osgSkeleton = findFirstOsgAnimationSkeletonNode(nodeFile);//return ptr
	if(!osgSkeleton){
		std::cout << "ERROR: Could not find osgAnimation::Skeleton node" <<std::endl;
		return 0; //error
	}
	
	//
	// initialize MBSBoneOsgBoneDisplacements
	//
	MBSBoneOsgBoneDisplacements.clear();
	for(int i=0;i<m_mbs->numLinks();++i)
	{
		gLink* link = m_mbs->link(i);
		findNodeVisitor visitor(link->name());
		nodeFile->accept(visitor);

		osg::ref_ptr<osg::MatrixTransform> oLink = dynamic_cast<osg::MatrixTransform*>(visitor.getFirst());
		if(!oLink)
		{
			std::cout << "ERROR: Cannot find bone " << link->name() << " from " << filename << std::endl;
			return 0; //error
		}

		//TODO: examine if there are multiple bones with the same name and output warning.
 	
		MBSToOsgBone m2o;
		m2o.mb = link;
		m2o.ob = oLink;
		//m2o.displacement = ; //will be added later
		MBSBoneOsgBoneDisplacements.push_back(m2o);
	}	

	//
	// find world to skeleton space
	//
	osg::Matrix w2s; 
	osg::ref_ptr<osgAnimation::Bone> rootAsBone = dynamic_cast<osgAnimation::Bone*>( MBSBoneOsgBoneDisplacements[0].ob);	
	if(rootAsBone){
		w2s.makeIdentity(); //if root is a Bone node, w2s is identity matrix
	}else{ 
		//if root is MatrixTransform node, w2s is root's matrixTranform
		w2s = MBSBoneOsgBoneDisplacements[0].ob->getMatrix(); 
	}

	//
	// pose mbs skeleton to match osg skeleton
	//
	for(int i=0;i<MBSBoneOsgBoneDisplacements.size();++i)
	{
		gLink* mLink = MBSBoneOsgBoneDisplacements[i].mb;
		osg::ref_ptr<osg::MatrixTransform> oLink = MBSBoneOsgBoneDisplacements[i].ob;
		osg::ref_ptr<osgAnimation::Bone> oLinkAsBone = 	dynamic_cast<osgAnimation::Bone*>(oLink.get());
				
		gXMat T; //mb's world transform

		if(!mLink->parent()) //root
		{
			//head
			gLink* mHead = m_mbs->findLink(name_head);		
			if(!mHead){
				std::cout << "ERROR: Cannot find bone " << name_head << " from MBS." << std::endl;
				return 0; //error
			}

			//rShoulder
			gLink* mRShoulder = m_mbs->findLink(name_right_shoulder);
			if(!mHead){
				std::cout << "ERROR: Cannot find bone " << name_right_shoulder << " from MBS." << std::endl;
				return 0; //error
			}

			//determine frontal & up axis of two roots
			gVec3 mFrontal = (mHead->pos()-mLink->pos())%(mRShoulder->pos()-mLink->pos());
			mFrontal.normalize();
			mFrontal = snap2XYZ(mFrontal);

			gVec3 mUp = mHead->pos()-mLink->pos();
			mUp.normalize();
			mUp = snap2XYZ(mUp);

			osg::Vec3 oHeadPos = ( dynamic_cast<osgAnimation::Bone*>(MBSBoneOsgBoneDisplacements[mHead->id()].ob)->getMatrixInSkeletonSpace() * w2s ).getTrans();
			osg::Vec3 oRSPos = ( dynamic_cast<osgAnimation::Bone*>(MBSBoneOsgBoneDisplacements[mRShoulder->id()].ob)->getMatrixInSkeletonSpace() * w2s ).getTrans();
			osg::Vec3 op2h_ = oHeadPos - oLink->getMatrix().getTrans();
			osg::Vec3 op2s_ = oRSPos - oLink->getMatrix().getTrans();
			gVec3 op2h, op2s; osgVec3TogVec3(op2h,op2h_); osgVec3TogVec3(op2s,op2s_); 
			gVec3 oFrontal = op2h%op2s; 
			oFrontal.normalize();
			oFrontal = snap2XYZ(oFrontal);
			gVec3 oUp = op2h;
			oUp.normalize();
			oUp = snap2XYZ(oUp);

			//compute R to match frontal and up vectors
			gRotMat R = makeRotate(mFrontal,oFrontal,mUp,oUp);

			//compute desired root xform			
			osg::Vec3 pos_ = oLink->getMatrix().getTrans();
			T.setTrn( pos_.x(), pos_.y(), pos_.z() ); //set position of root
			T.setRot( mLink->frame().rot() * R ); //set rotation of root
			
			//set coord
			mLink->setLocalFrame(T); //for root, it's ok to set its local frame directly instead of using ((gFreeLink*)mLink)->setCoord(T);

			m_mbs->updateKinematicsUptoPos();
		}
		else //non-root
		{
			if(!oLinkAsBone) assert(0);			
			osg::Matrix oT_ = oLinkAsBone->getMatrixInSkeletonSpace() * w2s; //world transform matrix
			
			int nChild = mLink->numChildren();						
			if(nChild == 1)
			{
				gLink* mc = mLink->child(0);
				osg::ref_ptr<osgAnimation::Bone> oc = dynamic_cast<osgAnimation::Bone*>( MBSBoneOsgBoneDisplacements[mc->id()].ob );
				
				osg::Vec3 oDir_ = (oc->getMatrixInSkeletonSpace() * w2s).getTrans() - oT_.getTrans(); //osg dir in world space
				gVec3 oDir;
				osgVec3TogVec3(oDir,oDir_);
				gVec3 mDir = mc->pos()-mLink->pos(); //mbs dir in world space

				//oDir.normalize();
				//mDir.normalize();
					
				gRotMat Rt = mLink->frame().rot(); //current rotation in world
				gRotMat R = makeRotate(mDir,oDir); //desired rotation in world				
				//R = (Rt.invMult(R))*Rt; //R = inv(Rt)*R*Rt //desired rotation in body frame
				R = R * ~Rt;

				R = ((gBallLink*)mLink)->coord() * R; //desired new coordinates

				((gBallLink*)mLink)->setCoord(R);
				m_mbs->updateKinematicsUptoPos();
			}
			else if(nChild > 1)
			{
				gLink* mc0 = mLink->child(0);
				osg::ref_ptr<osgAnimation::Bone> oc0 = dynamic_cast<osgAnimation::Bone*>( MBSBoneOsgBoneDisplacements[mc0->id()].ob );
				
				osg::Vec3 oDir0_ = (oc0->getMatrixInSkeletonSpace()*w2s).getTrans() - oT_.getTrans(); //dir in world space
				gVec3 oDir0;
				osgVec3TogVec3(oDir0,oDir0_);
				gVec3 mDir0 = mc0->pos()-mLink->pos();

				gLink* mc1 = mLink->child( mLink->numChildren()-1 ); //last children
				osg::ref_ptr<osgAnimation::Bone> oc1 = dynamic_cast<osgAnimation::Bone*>( MBSBoneOsgBoneDisplacements[mc1->id()].ob );
				osg::Vec3 oDir1_ = (oc1->getMatrixInSkeletonSpace()*w2s).getTrans() - oT_.getTrans(); //dir in world space
				gVec3 oDir1;
				osgVec3TogVec3(oDir1,oDir1_);
				gVec3 mDir1 = mc1->pos()-mLink->pos();

				gRotMat Rt = mLink->frame().rot(); //current rotation in world
				gRotMat R = makeRotate(mDir0,oDir0,mDir1,oDir1); //desired rotation in world				
				//R = (Rt.invMult(R))*Rt; //R = inv(Rt)*R*Rt //desired rotation in body frame
				
				R = R * ~Rt;
				R = ((gBallLink*)mLink)->coord() * R; //desired new coordinates

				((gBallLink*)mLink)->setCoord(R);
				m_mbs->updateKinematicsUptoPos(mLink); //traverse down from mLink and update kinematic variables
			}
			else				
			{	
				//TODO: we do not know how to pose terminal links to match their corresponding osgBones.
				//Note that a terminal osgBone has endPointBone at its tip, so we will be able to 
				//pose mbs bone if we also know its tip position. This information should be given
				//by a user, which is omitted in the current version.
			}			
		}		
	}

	//
	// create MBSBoneOsgBoneDisplacements
	//
	endPointBones.clear();
	for(int i=0;i<MBSBoneOsgBoneDisplacements.size();++i)
	{
		MBSToOsgBone& m2o = MBSBoneOsgBoneDisplacements[i];
		gLink* link = m2o.mb;
		osg::ref_ptr<osg::MatrixTransform> oLink = m2o.ob;
		osg::ref_ptr<osgAnimation::Bone> oLinkAsBone = dynamic_cast<osgAnimation::Bone*>(oLink.get());
	
		if(i==0) //root
		{
			gXMat disp;
			osg::Matrix oT_ = oLink->getMatrix();
			gXMat oT;
			osgMatrixTogXMat(oT,oT_);
			gXMat mT = link->frame();

			m2o.displacement = mT.invMult(oT);
		}
		else //non-root
		{
			if(!oLinkAsBone) assert(0);

			gXMat disp;
			osg::Matrix oT_ = oLinkAsBone->getMatrixInSkeletonSpace() * w2s ;
			gXMat oT;
			osgMatrixTogXMat(oT,oT_);
			gXMat mT = link->frame();

			m2o.displacement = mT.invMult(oT);

			//if has endPointBones as child, add to endPointBones
			for(int i=0;i<oLinkAsBone->getNumChildren();++i)
			{
				osgAnimation::Bone* c = dynamic_cast<osgAnimation::Bone*>(oLinkAsBone->getChild(i));
				if(c){
					if(c->getNumChildren()==0){
						//c->addChild(gOSGShape::createSphereShape(1.0, osg::Vec3(0,0,0), osg::Vec4(0,0,1,1)));
						endPointBones.push_back(c);
					}
				}
			}
		}
		
	}
	
	return 1;
}
*/

int gOsgSkin::loadSkin(osg::ref_ptr<osg::Group> scene, 
					   const char* filename, const char* name_head, const char* name_right_shoulder)
{
	//osgDB::Options objOpt;
	//objOpt.setOptionString("Embedded");
	//nodeFile = dynamic_cast<osg::Group*>(osgDB::readNodeFile(filename, &objOpt));
	nodeFile = dynamic_cast<osg::Group*>(osgDB::readNodeFile(filename));

	skinGroupNode = scene;

	if(!nodeFile){
		std::cout << "ERROR: Loading "<<filename<<" failed"<<std::endl;
		return 0; //error
	}

	osg::ref_ptr<osgUtil::UpdateVisitor> uv = new osgUtil::UpdateVisitor();
	osg::ref_ptr<osg::FrameStamp> stamp = new osg::FrameStamp;
	uv->setFrameStamp(stamp);
	nodeFile->accept(*uv);
	
	scene->addChild(nodeFile);
	visState = true;
	//viewer->updateTraversal();
	//nodeFile->setUpdateCallback(NULL);	
	//remove Bones' updateCallbacks to steal control from channels of original animation contained in fbx file	
	traverseAndRemoveUpdateCallbacksFromBones(nodeFile);
	
	// find osgAnimation::Skeleton node to see if there's a skeleton 
	osgAnimation::Skeleton* osgSkeleton = findFirstOsgAnimationSkeletonNode(nodeFile);//return ptr
	if(!osgSkeleton){
		std::cout << "ERROR: Could not find osgAnimation::Skeleton node" <<std::endl;
		return 0; //error
	}
	
	//
	// initialize MBSBoneOsgBoneDisplacements
	//
	MBSBoneOsgBoneDisplacements.clear();
	for(int i=0;i<m_mbs->numLinks();++i)
	{
		gLink* link = m_mbs->link(i);
		findNodeVisitor visitor(link->name());
		nodeFile->accept(visitor);

		osg::ref_ptr<osg::MatrixTransform> oLink = dynamic_cast<osg::MatrixTransform*>(visitor.getFirst());
		if(!oLink)
		{
			std::cout << "ERROR: Cannot find bone " << link->name() << " from " << filename << std::endl;
			return 0; //error
		}

		//TODO: examine if there are multiple bones with the same name and output warning.
		osg::Matrix olMat = oLink->getMatrix();
		olMat.setTrans( olMat.getTrans() * scale );
		oLink->setMatrix( olMat );

		osg::ref_ptr<osgAnimation::Bone> oBone = dynamic_cast<osgAnimation::Bone*>(visitor.getFirst());
		osg::Matrix olskelMat = oBone->getMatrixInSkeletonSpace();
		olskelMat.setTrans( olskelMat.getTrans() * scale );
		oBone->setMatrixInSkeletonSpace(olskelMat);

 	
		MBSToOsgBone m2o;
		m2o.mb = link;
		m2o.ob = oLink;
		//m2o.displacement = ; //will be added later
		MBSBoneOsgBoneDisplacements.push_back(m2o);
	}	

	//
	// find world to skeleton space
	//
	osg::Matrix w2s; 
	osg::ref_ptr<osgAnimation::Bone> rootAsBone = dynamic_cast<osgAnimation::Bone*>( MBSBoneOsgBoneDisplacements[0].ob);	
	if(rootAsBone){
		w2s.makeIdentity(); //if root is a Bone node, w2s is identity matrix
	}else{ 
		//if root is MatrixTransform node, w2s is root's matrixTranform
		w2s = MBSBoneOsgBoneDisplacements[0].ob->getMatrix(); 
	}

#if 1
	//
	// pose mbs skeleton to match osg skeleton
	//
	//for(int i=0;i<MBSBoneOsgBoneDisplacements.size();++i)
	{
		int i = 0;

		gLink* mLink = MBSBoneOsgBoneDisplacements[i].mb;
		osg::ref_ptr<osg::MatrixTransform> oLink = MBSBoneOsgBoneDisplacements[i].ob;
		osg::ref_ptr<osgAnimation::Bone> oLinkAsBone = 	dynamic_cast<osgAnimation::Bone*>(oLink.get());
				
		gXMat T; //mb's world transform

		if(!mLink->parent()) //root
		{
			//head
			gLink* mHead = m_mbs->findLink(name_head);		
			if(!mHead){
				std::cout << "ERROR: Cannot find bone " << name_head << " from MBS." << std::endl;
				return 0; //error
			}

			//rShoulder
			gLink* mRShoulder = m_mbs->findLink(name_right_shoulder);
			if(!mHead){
				std::cout << "ERROR: Cannot find bone " << name_right_shoulder << " from MBS." << std::endl;
				return 0; //error
			}

			//determine frontal & up axis of two roots
			gVec3 mFrontal = (mHead->pos()-mLink->pos())%(mRShoulder->pos()-mLink->pos());
			mFrontal.normalize();
			mFrontal = snap2XYZ(mFrontal);

			gVec3 mUp = mHead->pos()-mLink->pos();
			mUp.normalize();
			mUp = snap2XYZ(mUp);

			osg::Vec3 oHeadPos = ( dynamic_cast<osgAnimation::Bone*>(MBSBoneOsgBoneDisplacements[mHead->id()].ob)->getMatrixInSkeletonSpace() * w2s ).getTrans();
			osg::Vec3 oRSPos = ( dynamic_cast<osgAnimation::Bone*>(MBSBoneOsgBoneDisplacements[mRShoulder->id()].ob)->getMatrixInSkeletonSpace() * w2s ).getTrans();
			osg::Vec3 op2h_ = oHeadPos - oLink->getMatrix().getTrans();
			osg::Vec3 op2s_ = oRSPos - oLink->getMatrix().getTrans();
			gVec3 op2h, op2s; osgVec3TogVec3(op2h,op2h_); osgVec3TogVec3(op2s,op2s_); 
			gVec3 oFrontal = op2h%op2s; 
			oFrontal.normalize();
			oFrontal = snap2XYZ(oFrontal);
			gVec3 oUp = op2h;
			oUp.normalize();
			oUp = snap2XYZ(oUp);

			//compute R to match frontal and up vectors
			gRotMat R = makeRotate(mFrontal,oFrontal,mUp,oUp);

			//compute desired root xform			
			osg::Vec3 pos_ = oLink->getMatrix().getTrans();
			T.setTrn( pos_.x(), pos_.y(), pos_.z() ); //set position of root
			T.setRot( mLink->frame().rot() * R ); //set rotation of root
			
			//set coord
			mLink->setLocalFrame(T); //for root, it's ok to set its local frame directly instead of using ((gFreeLink*)mLink)->setCoord(T);

			m_mbs->updateKinematicsUptoPos();
		}
		else //non-root
		{
			//if (mLink->type() != TYPE_BALL_LINK) continue;

			if(!oLinkAsBone) assert(0);			
			osg::Matrix oT_ = oLinkAsBone->getMatrixInSkeletonSpace() * w2s; //world transform matrix
			
			int nChild = mLink->numChildren();						
			if(nChild == 1)
			{
				gLink* mc = mLink->child(0);
				osg::ref_ptr<osgAnimation::Bone> oc = dynamic_cast<osgAnimation::Bone*>( MBSBoneOsgBoneDisplacements[mc->id()].ob );
				
				osg::Vec3 oDir_ = (oc->getMatrixInSkeletonSpace() * w2s).getTrans() - oT_.getTrans(); //osg dir in world space
				gVec3 oDir;
				osgVec3TogVec3(oDir,oDir_);
				gVec3 mDir = mc->pos()-mLink->pos(); //mbs dir in world space

				//oDir.normalize();
				//mDir.normalize();
					
				gRotMat Rt = mLink->frame().rot(); //current rotation in world
				gRotMat R = makeRotate(mDir,oDir); //desired rotation in world				
				//R = (Rt.invMult(R))*Rt; //R = inv(Rt)*R*Rt //desired rotation in body frame
				R = R * ~Rt;

				R = ((gBallLink*)mLink)->coord() * R; //desired new coordinates

				((gBallLink*)mLink)->setCoord(R);
				m_mbs->updateKinematicsUptoPos();
			}
			else if(nChild > 1)
			{
				gLink* mc0 = mLink->child(0);
				osg::ref_ptr<osgAnimation::Bone> oc0 = dynamic_cast<osgAnimation::Bone*>( MBSBoneOsgBoneDisplacements[mc0->id()].ob );
				
				osg::Vec3 oDir0_ = (oc0->getMatrixInSkeletonSpace()*w2s).getTrans() - oT_.getTrans(); //dir in world space
				gVec3 oDir0;
				osgVec3TogVec3(oDir0,oDir0_);
				gVec3 mDir0 = mc0->pos()-mLink->pos();

				gLink* mc1 = mLink->child( mLink->numChildren()-1 ); //last children
				osg::ref_ptr<osgAnimation::Bone> oc1 = dynamic_cast<osgAnimation::Bone*>( MBSBoneOsgBoneDisplacements[mc1->id()].ob );
				osg::Vec3 oDir1_ = (oc1->getMatrixInSkeletonSpace()*w2s).getTrans() - oT_.getTrans(); //dir in world space
				gVec3 oDir1;
				osgVec3TogVec3(oDir1,oDir1_);
				gVec3 mDir1 = mc1->pos()-mLink->pos();

				gRotMat Rt = mLink->frame().rot(); //current rotation in world
				gRotMat R = makeRotate(mDir0,oDir0,mDir1,oDir1); //desired rotation in world				
				//R = (Rt.invMult(R))*Rt; //R = inv(Rt)*R*Rt //desired rotation in body frame
				
				R = R * ~Rt;
				R = ((gBallLink*)mLink)->coord() * R; //desired new coordinates

				((gBallLink*)mLink)->setCoord(R);
				m_mbs->updateKinematicsUptoPos(mLink); //traverse down from mLink and update kinematic variables
			}
			else				
			{	
				//TODO: we do not know how to pose terminal links to match their corresponding osgBones.
				//Note that a terminal osgBone has endPointBone at its tip, so we will be able to 
				//pose mbs bone if we also know its tip position. This information should be given
				//by a user, which is omitted in the current version.
			}			
		}		
	}
#endif

	//
	// create MBSBoneOsgBoneDisplacements
	//
	endPointBones.clear();
	for(int i=0;i<MBSBoneOsgBoneDisplacements.size();++i)
	{
		MBSToOsgBone& m2o = MBSBoneOsgBoneDisplacements[i];
		gLink* link = m2o.mb;
		osg::ref_ptr<osg::MatrixTransform> oLink = m2o.ob;
		osg::ref_ptr<osgAnimation::Bone> oLinkAsBone = dynamic_cast<osgAnimation::Bone*>(oLink.get());
	
		if(i==0) //root
		{
			gXMat disp;
			osg::Matrix oT_ = oLink->getMatrix();
			gXMat oT;
			osgMatrixTogXMat(oT,oT_);
			gXMat mT = link->frame();

			m2o.displacement = mT.invMult(oT);
		}
		else //non-root
		{
			if(!oLinkAsBone) assert(0);

			gXMat disp;
			osg::Matrix oT_ = oLinkAsBone->getMatrixInSkeletonSpace() * w2s ;
			gXMat oT;
			osgMatrixTogXMat(oT,oT_);
			gXMat mT = link->frame();

			m2o.displacement = mT.invMult(oT);

			//if has endPointBones as child, add to endPointBones
			for(int i=0;i<oLinkAsBone->getNumChildren();++i)
			{
				osgAnimation::Bone* c = dynamic_cast<osgAnimation::Bone*>(oLinkAsBone->getChild(i));
				if(c){
					if(c->getNumChildren()==0){
						//c->addChild(gOSGShape::createSphereShape(1.0, osg::Vec3(0,0,0), osg::Vec4(0,0,1,1)));
						endPointBones.push_back(c);
					}
				}
			}
		}
		
	}

	return 1;
}

int gOsgSkin::loadSkin(osg::ref_ptr<osg::Group> scene, const char* filename)
{
	nodeFile = dynamic_cast<osg::Group*>(osgDB::readNodeFile(filename));

	//ReaderWriterFBX rw;
	//osgDB::ReaderWriter::ReadResult res = rw.readNode(filename, new osgDB::Options());
	//nodeFile = res.getNode()->asGroup();

	skinGroupNode = scene;

	if (!nodeFile) {
		std::cout << "ERROR: Loading " << filename << " failed" << std::endl;
		return 0; //error
	}

	osg::ref_ptr<osgUtil::UpdateVisitor> uv = new osgUtil::UpdateVisitor();
	osg::ref_ptr<osg::FrameStamp> stamp = new osg::FrameStamp;
	uv->setFrameStamp(stamp);
	nodeFile->accept(*uv);

	scene->addChild(nodeFile);
	visState = true;
	//viewer->updateTraversal();
	//nodeFile->setUpdateCallback(NULL);	
	//remove Bones' updateCallbacks to steal control from channels of original animation contained in fbx file	
	traverseAndRemoveUpdateCallbacksFromBones(nodeFile);

	// find osgAnimation::Skeleton node to see if there's a skeleton 
	osgAnimation::Skeleton* osgSkeleton = findFirstOsgAnimationSkeletonNode(nodeFile);//return ptr
	if (!osgSkeleton) {
		std::cout << "ERROR: Could not find osgAnimation::Skeleton node" << std::endl;
		return 0; //error
	}

	//
	// initialize MBSBoneOsgBoneDisplacements
	//
	MBSBoneOsgBoneDisplacements.clear();
	for (int i = 0; i<m_mbs->numLinks(); ++i)
	{
		gLink* link = m_mbs->link(i);
		findNodeVisitor visitor(link->name());
		nodeFile->accept(visitor);

		osg::ref_ptr<osg::MatrixTransform> oLink = dynamic_cast<osg::MatrixTransform*>(visitor.getFirst());
		if (!oLink)
		{
			std::cout << "ERROR: Cannot find bone " << link->name() << " from " << filename << std::endl;
			return 0; //error
		}

		//TODO: examine if there are multiple bones with the same name and output warning.
		osg::Matrix olMat = oLink->getMatrix();
		olMat.setTrans(olMat.getTrans() * scale);
		oLink->setMatrix(olMat);

		osg::ref_ptr<osgAnimation::Bone> oBone = dynamic_cast<osgAnimation::Bone*>(visitor.getFirst());
		osg::Matrix olskelMat = oBone->getMatrixInSkeletonSpace();
		olskelMat.setTrans(olskelMat.getTrans() * scale);
		oBone->setMatrixInSkeletonSpace(olskelMat);


		MBSToOsgBone m2o;
		m2o.mb = link;
		m2o.ob = oLink;
		//m2o.displacement = ; //will be added later
		MBSBoneOsgBoneDisplacements.push_back(m2o);
	}

	//
	// find world to skeleton space
	//
	osg::Matrix w2s;
	osg::ref_ptr<osgAnimation::Bone> rootAsBone = dynamic_cast<osgAnimation::Bone*>(MBSBoneOsgBoneDisplacements[0].ob);
	if (rootAsBone) {
		w2s.makeIdentity(); //if root is a Bone node, w2s is identity matrix
	}
	else {
		//if root is MatrixTransform node, w2s is root's matrixTranform
		w2s = MBSBoneOsgBoneDisplacements[0].ob->getMatrix();
	}

	// root alignment
	{
		gLink* mLink = MBSBoneOsgBoneDisplacements[0].mb;
		osg::ref_ptr<osg::MatrixTransform> oLink = MBSBoneOsgBoneDisplacements[0].ob;
		osg::ref_ptr<osgAnimation::Bone> oLinkAsBone = dynamic_cast<osgAnimation::Bone*>(oLink.get());

		osg::Matrixd oMat = oLink->getMatrix();
		gXMat T(oMat.ptr());
		mLink->setLocalFrame(T); //for root, it's ok to set its local frame directly instead of using ((gFreeLink*)mLink)->setCoord(T);

		m_mbs->updateKinematicsUptoPos();
	}

	//
	// create MBSBoneOsgBoneDisplacements
	//
	endPointBones.clear();
	for (int i = 0; i<MBSBoneOsgBoneDisplacements.size(); ++i)
	{
		MBSToOsgBone& m2o = MBSBoneOsgBoneDisplacements[i];
		gLink* link = m2o.mb;
		osg::ref_ptr<osg::MatrixTransform> oLink = m2o.ob;
		osg::ref_ptr<osgAnimation::Bone> oLinkAsBone = dynamic_cast<osgAnimation::Bone*>(oLink.get());

		if (i == 0) //root
		{
			gXMat disp;
			osg::Matrix oT_ = oLink->getMatrix();
			gXMat oT;
			osgMatrixTogXMat(oT, oT_);
			gXMat mT = link->frame();

			m2o.displacement = mT.invMult(oT);
		}
		else //non-root
		{
			if (!oLinkAsBone) assert(0);

			gXMat disp;
			osg::Matrix oT_ = oLinkAsBone->getMatrixInSkeletonSpace() * w2s;
			gXMat oT;
			osgMatrixTogXMat(oT, oT_);
			gXMat mT = link->frame();

			m2o.displacement = mT.invMult(oT);

			//if has endPointBones as child, add to endPointBones
			for (int j = 0; j<oLinkAsBone->getNumChildren(); ++j)
			{
				osgAnimation::Bone* c = dynamic_cast<osgAnimation::Bone*>(oLinkAsBone->getChild(j));
				if (c) {
					if (c->getNumChildren() == 0) {
						//c->addChild(gOSGShape::createSphereShape(1.0, osg::Vec3(0,0,0), osg::Vec4(0,0,1,1)));
						endPointBones.push_back(c);
					}
				}
			}
		}
	}

	return 1;
}
