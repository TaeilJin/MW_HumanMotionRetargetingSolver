//#####################################################################
// Copyright 2010-2017, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#include "Loader/FBXLoader.h"
#include "MocapProcessor/mgUtility.h"
#include <algorithm>

#include <osg/Group>
#include <osgDB/ReadFile>
#include <osgUtil/UpdateVisitor>
#include <osgAnimation/UpdateBone>
#include <osgAnimation/Bone>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/AnimationUpdateCallback>

#include <osgAnimation/StackedRotateAxisElement>
#include <osgAnimation/StackedMatrixElement>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/StackedScaleElement>

//#include "ReaderWriterFBX.h"


int FBXLoader::loadMotion(const char* filename, mgSkeleton *skeleton, mgData *motion)
{
	assert( skeleton && motion );

	//if(open_file( filename )==0) return (int)RT_OPENERR;

	readMotion(skeleton, motion);

	close_file();

	return 0;
}

int FBXLoader::loadSkeleton(const char* filename, mgSkeleton *skeleton)
{
	_skeleton = skeleton;

	osg::ref_ptr<osg::Group> node = dynamic_cast<osg::Group*>(osgDB::readNodeFile(filename));

	//ReaderWriterFBX rw;
	//osgDB::ReaderWriter::ReadResult res = rw.readNode(filename, new osgDB::Options());
	//osg::ref_ptr<osg::Node> node = res.getNode();

	if (!node) {
		std::cout << "ERROR: Loading " << filename << " failed" << std::endl;
		return -1;
	}

	osg::ref_ptr<osgUtil::UpdateVisitor> uv = new osgUtil::UpdateVisitor();
	osg::ref_ptr<osg::FrameStamp> stamp = new osg::FrameStamp;
	uv->setFrameStamp(stamp);
	node->accept(*uv);

	//traverseAndRemoveUpdateCallbacksFromBones(nodeFile);

	_animationMgr = findFirstOsgAnimationManagerNode(node);

	const osgAnimation::AnimationList& list = _animationMgr->getAnimationList();
	
	// get First animation
	if (!list.empty())
	{
		_animation = list[0];
	}

	// find osgAnimation::Skeleton node to see if there's a skeleton 
	//osgSkeleton = findFirstOsgAnimationSkeletonNode(nodeFile);//return ptr
	osg::ref_ptr<osgAnimation::Skeleton> osgSkeleton = findFirstOsgAnimationSkeletonNode(node);//return ptr

	// get world translation
	//osg::ref_ptr<osg::Group> parentGroup = dynamic_cast<osg::Group*>( osgSkeleton.get() );
	osg::ref_ptr<osg::Group> parentGroup = dynamic_cast<osg::Group*>(osgSkeleton->getParent(0));

	while (parentGroup.valid())
	{
		osg::ref_ptr<osg::MatrixTransform> transformNode = dynamic_cast<osg::MatrixTransform*>(parentGroup.get());

		if (transformNode != NULL)
		{
			_parent_name_map[transformNode->getName()] = _parent_names.size();
			_parent_names.push_back(transformNode->getName());


			bool before = true;
			osg::Matrixd H, Hp;
			H.identity(); Hp.identity();

			osg::ref_ptr<osgAnimation::UpdateMatrixTransform> update = dynamic_cast<osgAnimation::UpdateMatrixTransform*>(parentGroup->getUpdateCallback());

			if (update)
			{
				osgAnimation::StackedTransform& trans = update->getStackedTransforms();
				for (int i = 0; i < trans.size(); i++)
				{
					// assume that there is no scaling channel in the bone
					osg::ref_ptr<osgAnimation::StackedRotateAxisElement> srae = dynamic_cast<osgAnimation::StackedRotateAxisElement*>(trans[i].get());
					if (srae) {
						before = false;
						continue;
					}

					osg::ref_ptr<osgAnimation::StackedTranslateElement> stae = dynamic_cast<osgAnimation::StackedTranslateElement*>(trans[i].get());
					if (stae)
						continue;

					if (before)
						trans[i]->applyToMatrix(H);
					else
						trans[i]->applyToMatrix(Hp);
				}

				_parent_H.push_back(H.ptr());
			}
			else _parent_H.push_back(gXMat()); //mBone->H.set(bone->getMatrixInBoneSpace().ptr());

			_parent_Hp.push_back(Hp.ptr());
		}

		if (parentGroup->getNumParents() > 0)
			parentGroup = parentGroup->getParent(0);
		else
			break;
	}

	if (!osgSkeleton) {
		std::cout << "ERROR: Could not find osgAnimation::Skeleton node" << std::endl;
		return -1;
	}

	// goto neutral pose
	takeZeroPose(osgSkeleton);

	skeleton->boneRoot = readBoneNode(osgSkeleton, skeleton);
	
	// check translation
	if (_animation)
	{
		const osgAnimation::ChannelList& cList = _animation->getChannels();

		for (int i = 0; i < cList.size(); i++)
		{
			if((skeleton->boneRoot->channel & mgBone::translateChannel) && (skeleton->boneRoot->channel & mgBone::rotationChannel))
				break;

			const osg::ref_ptr<osgAnimation::Channel> channel = cList[i];
			std::map<std::string, int>::iterator it = _parent_name_map.find(channel->getTargetName());
			
			if (it == _parent_name_map.end())
				continue;

			if (!(skeleton->boneRoot->channel & mgBone::translateChannel) 
				&&
				channel->getName() == "translate")
			{
				std::cout << "warning: the original skeleton has not a translation channel. now extended." << std::endl;

				skeleton->boneRoot->channel |= mgBone::translateChannel;
				skeleton->boneRoot->nChannel += 3;

				skeleton->nTotalChannel += 3;
				arma::ivec(skeleton->dataPos.data() + 1, skeleton->dataPos.size() - 1, false) += 3;
				skeleton->numT += 1;
			}

			if (!(skeleton->boneRoot->channel & mgBone::rotationChannel)
				&&
				channel->getName().find("rotate") != std::string::npos)
			{
				std::cout << "warning: the original skeleton has not a rotational channel. now extended." << std::endl;

				skeleton->boneRoot->channel |= mgBone::rotationChannel;
				skeleton->boneRoot->nChannel += 3;

				skeleton->nTotalChannel += 3;
				arma::ivec(skeleton->dataPos.data() + 1, skeleton->dataPos.size() - 1, false) += 3;
				skeleton->numR += 1;
			}
		}
	}
	
	return 0;
}
int	FBXLoader::readMotion(mgSkeleton *skeleton, mgData *motion)
{
	if (_animation)
	{
		//const osg::ref_ptr<osgAnimation::Animation> animation = _animationMgr->getAnimationList()[0];
		const osgAnimation::ChannelList& cList = _animation->getChannels();

		// get duration and key, frame.
		double endTime = 0;
		double startTime = DBL_MAX;
		unsigned int nKeys = 0;
		//_animation->computeDuration();

		for (int i = 0; i < cList.size(); i++)
		{
			const osg::ref_ptr<osgAnimation::Channel> channel = cList[i];
			const osgAnimation::Sampler* sampler = channel->getSampler();

			endTime = std::max(endTime, channel->getEndTime());
			startTime = std::min(startTime, channel->getStartTime());
			// assume that at least one channel might have keys in every frame.
			nKeys = std::max(nKeys, channel->getSampler()->getKeyframeContainer()->size());
			
			//motion->frameTime = keys->at(1).getTime() - keys->at(0).getTime();
		}
		//nKeys = 300;
		const double frameTime = (endTime - startTime) / (nKeys-1);
		//const double frameTime = 1. / 30.;

		motion->allocMemory(nKeys, skeleton->nTotalChannel);
		motion->frameTime = frameTime;

		arma::mat a_motion(motion->pMotions, motion->nChannel, motion->nMotion, false);
		a_motion.zeros();

		bool is_extended_translate = true;
		bool is_extended_rotation = true;

		
		// channel function
		auto readChannel = [&](arma::mat& m, const osg::ref_ptr<osgAnimation::Channel> channel, std::string& ch, const mgBone* bone)
		{
			std::string chName = channel->getName();

			if (chName == "translate" && (bone?(bone->channel & mgBone::translateChannel):true))
			//if (chName == "translate")
			{
				const osgAnimation::Vec3CubicBezierSampler* sampler =
					dynamic_cast<const osgAnimation::Vec3CubicBezierSampler*>(channel->getSampler());

				if (!sampler)
				{
					std::cout << "extracting key frame error" << std::endl;
					return -1;
				}

				ch = ch + "t";

				for (int f = 0; f < nKeys; f++)
				{
					double t = f * frameTime;

					osg::Vec3 data;
					sampler->getValueAt(t, data);

					m(0, f) = data.x();
					m(1, f) = data.y();
					m(2, f) = data.z();

					//motion->motions[f][dPos + 0] = data.x();
					//motion->motions[f][dPos + 1] = data.y();
					//motion->motions[f][dPos + 2] = data.z();
				}
				return 3;
				//dataPosDelta[it->second] += 3;
			}
			else if (chName.find("rotate") != std::string::npos)
			{
				const osgAnimation::FloatCubicBezierSampler* sampler =
					dynamic_cast<const osgAnimation::FloatCubicBezierSampler*>(channel->getSampler());

				if (!sampler)
				{
					std::cout << "extracting key frame error" << std::endl;
					return -1;
				}

				ch = ch + chName.substr(chName.find("rotate") + 6, 1);

				for (int f = 0; f < nKeys; f++)
				{
					double t = f * frameTime;

					float data;
					sampler->getValueAt(t, data);
					data = osg::RadiansToDegrees(data);

					m(0, f) = data;
					//motion->motions[f][dPos] = data;
				}
				//dataPosDelta[it->second] += 1;
				return 1;
			}
			return 0;
		};

		//std::vector<gXMat> transformOfRoot(nKeys);
		arma::mat parent_motions;
		int parent_data_pos = 0;
		//std::vector<int> parent_data_pos(_parent_names.size(), 0);
		std::vector<int> parent_nChannel(_parent_names.size(), 0);
		std::vector<std::string> parent_ch(_parent_names.size());

		std::vector<int> dataPosDelta(skeleton->bones.size(), 0);
		for (int i = 0; i < cList.size(); i++)
		{
			const osg::ref_ptr<osgAnimation::Channel> channel = cList[i];

			mgSkeleton::BONEMIt it = skeleton->getBoneMapFind(channel->getTargetName());
			if (it == skeleton->getBoneMapEnd())
			{
				//std::cout << "cannot find " << channel->getTargetName() << " bones" << std::endl;

				for (int p = 0; p < _parent_names.size(); p++)
				{
					arma::mat m(3, nKeys);
					
					if (_parent_names[p] == channel->getTargetName())
					{
						arma::mat m(3, nKeys);
						int readed = readChannel(m, channel, parent_ch[p], NULL);
						if (readed <= 0)
							continue;

						parent_motions = arma::join_vert(parent_motions, m.rows(0, readed - 1));
						parent_nChannel[p] += readed;
					}
				}

				continue;
			}

			int dPos = skeleton->dataPos[it->second];
			dPos += dataPosDelta[it->second];

			std::string ch; 

			arma::mat m(3, nKeys);
			int readed = readChannel(m, channel, ch, skeleton->bones[it->second] );
			if (readed <= 0)
				continue;

			a_motion.submat(dPos, 0, arma::SizeMat(readed, nKeys)) = m.rows(0, readed - 1);
			dataPosDelta[it->second] += readed;

			// check extended
			if (it->second == 0 && 
				ch.find('t') != std::string::npos &&
				(skeleton->boneRoot->channel & mgBone::translateChannel)
				)
			{
				is_extended_translate = false;
			}

			if (it->second == 0 &&
				ch.find("rotate") != std::string::npos &&
				(skeleton->boneRoot->channel & mgBone::rotationChannel)
				)
			{
				is_extended_rotation = false;
			}
		}

		// move data
		if (is_extended_translate && !is_extended_rotation)
		{
			a_motion.rows(3, 5) = a_motion.rows(0, 2);
			a_motion.rows(0, 2).zeros();
		}

		// apply root node to world origin
		for (int f = 0; f < nKeys; f++)
		{
			gVec3 trans;
			gRotMat rot;
			mgUtility::getTransAndRotMatFromData(motion->motions[f], skeleton->boneRoot, trans, rot);

			gXMat rootMat;
			parent_data_pos = 0;

			// aggregate root motion
			for (int p = 0; p < _parent_names.size(); p++)
			{
				gVec3 p_trans;

				int ch_pos = 0;
				if (parent_ch[p][ch_pos++] == 't')
				{
					arma::vec t = parent_motions.submat(parent_data_pos, f, arma::SizeMat(3, 1));
					p_trans.set(t.memptr());

					parent_data_pos += 3;
				}
				
				// convert transform matrix
				arma::vec r = parent_motions.submat(parent_data_pos, f, arma::SizeMat(3, 1));
				r *= gDTR;

				gVec3 p_rot(r.memptr());
				gRotMat rotMat;

				std::string rot_buf = parent_ch[p].substr(ch_pos, 3);
				std::transform(rot_buf.begin(), rot_buf.end(), rot_buf.begin(), ::tolower);

				mgUtility::getRotMatFromRotVec(rot_buf.c_str(), mgBone::EXTRINSIC, p_rot, rotMat);
				rootMat = gXMat(p_trans) * _parent_H[p] * gXMat(rotMat) * _parent_Hp[p] * rootMat;
			}

			gXMat trans_root(rot, trans);
			gXMat final_root = rootMat * trans_root;

			mgUtility::setTransAndRotMatToData(motion->motions[f], skeleton->boneRoot, final_root.trn(), final_root.rot());
		}
	}
	else
	{
		std::cout << "Nothing to read the motion in this file." << std::endl;
	}
	
	return 0;
}

void FBXLoader::traverseAndRemoveUpdateCallbacksFromBones(osg::Node* node)
{
	//node->setUpdateCallback(NULL);
	osgAnimation::UpdateBone* pUB = dynamic_cast<osgAnimation::UpdateBone*>(node->getUpdateCallback());
	if (pUB)
	{
		node->setUpdateCallback(NULL);
	}
	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if (group) {
		//traverse children		
		for (int i = 0; i<group->getNumChildren(); ++i)
		{
			traverseAndRemoveUpdateCallbacksFromBones(group->getChild(i));
		}
	}
}

osgAnimation::Skeleton* FBXLoader::findFirstOsgAnimationSkeletonNode(osg::Node* node)
{
	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if (!group) return NULL;

	//see if node is Skeleton
	osgAnimation::Skeleton* re = dynamic_cast<osgAnimation::Skeleton*>(node);
	if (re)  return re;

	//else, traverse children		
	for (int i = 0; i<group->getNumChildren(); ++i)
	{
		re = findFirstOsgAnimationSkeletonNode(group->getChild(i));
		if (re) return re;
	}
	return NULL;
}

osgAnimation::BasicAnimationManager* FBXLoader::findFirstOsgAnimationManagerNode(osg::Node* node)
{
	osgAnimation::BasicAnimationManager* manager = dynamic_cast<osgAnimation::BasicAnimationManager*>(node->getUpdateCallback());
	if (manager)
	{
		//std::cout << "name: " << node->getName() << std::endl;
		return manager;
	}

	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if (!group) return NULL;

	//else, traverse children		
	for (int i = 0; i<group->getNumChildren(); ++i)
	{
		manager = findFirstOsgAnimationManagerNode(group->getChild(i));
		if (manager) return manager;
	}
	return NULL;
}

mgBone* FBXLoader::readBoneNode(osg::Node* node, mgSkeleton *skeleton)
{
	osg::ref_ptr<osgAnimation::Bone> bone = dynamic_cast<osgAnimation::Bone*>(node);
	if (!bone)
	{
		osg::Group* group = node->asGroup();
		if (!group) return NULL;

		for (int i = 0; i < group->getNumChildren(); ++i)
		{
			mgBone* bone = readBoneNode(group->getChild(i), skeleton);

			if (bone)
				return bone;
		}

		return NULL;
	}

	// determines whether this link should be included or not
	unsigned int channel;
	unsigned int nChannel;
	std::string orderBuf;
	getChannelInfo(bone, channel, nChannel, orderBuf);

	// remove translation
	bool use_dummy = false;
	arma::vec translation_dummy(3, arma::fill::zeros);
	osg::ref_ptr<osgAnimation::Skeleton> osgSkeleton = dynamic_cast<osgAnimation::Skeleton*>(node->getParent(0));
	if (!osgSkeleton && (channel & mgBone::translateChannel))
	{
		std::cout << bone->getName() << " has translation!!" << std::endl;
		channel &= ~mgBone::translateChannel;
		nChannel -= 3;

		use_dummy = true;

		// FIXME!!

		//const osg::ref_ptr<osgAnimation::Animation> animation = _animationMgr->getAnimationList()[0];
		const osgAnimation::ChannelList& cList = _animation->getChannels();

		// get duration and key, frame.
		double endTime = 0;
		double startTime = DBL_MAX;
		unsigned int nKeys = 0;

		for (int i = 0; i < cList.size(); i++)
		{
			const osg::ref_ptr<osgAnimation::Channel> channel = cList[i];
			const osgAnimation::Sampler* sampler = channel->getSampler();

			endTime = std::max(endTime, channel->getEndTime());
			startTime = std::min(startTime, channel->getStartTime());
			nKeys = std::max(nKeys, channel->getSampler()->getKeyframeContainer()->size());
		}
		const double frameTime = (endTime - startTime) / (nKeys - 1);

		for (int i = 0; i < cList.size(); i++)
		{
			const osg::ref_ptr<osgAnimation::Channel> channel = cList[i];
			if (bone->getName() == channel->getTargetName() && channel->getName() == "translate")
			{
				const osgAnimation::Vec3CubicBezierSampler* sampler =
					dynamic_cast<const osgAnimation::Vec3CubicBezierSampler*>(channel->getSampler());

				if (!sampler)
				{
					std::cout << "extracting key frame error" << std::endl;
				}

				arma::mat m(3, nKeys);

				for (int f = 0; f < nKeys; f++)
				{
					double t = f * frameTime;

					osg::Vec3 data;
					sampler->getValueAt(t, data);

					m(0, f) = data.x();
					m(1, f) = data.y();
					m(2, f) = data.z();
				}

				translation_dummy = arma::mean(m, 1);
				break;
			}

		}
	}

	// Tip of end effector
	//if (nChannel == 0 && bone->getNumChildren() == 0)
	if (bone->getNumChildren() == 0)
	{
		// add end effectors box
		osg::Matrix mat = bone->getMatrixInBoneSpace();
		gXMat H(mat.ptr());
		skeleton->bones.back()->localEndPoints.push_back(H.trn());

		return NULL;
	}

	mgBone* mBone = skeleton->createBone(bone->getName());

	//alterChannel(bone, mBone);
	{
		mBone->channel = channel;
		mBone->nChannel = nChannel;

		mBone->eulerConv = mgBone::EXTRINSIC;

		if (orderBuf == "XYZ")		mBone->order = mgBone::XYZ;
		else if (orderBuf == "XZY") mBone->order = mgBone::XZY;
		else if (orderBuf == "YXZ") mBone->order = mgBone::YXZ;
		else if (orderBuf == "YZX") mBone->order = mgBone::YZX;
		else if (orderBuf == "ZXY") mBone->order = mgBone::ZXY;
		else if (orderBuf == "ZYX") mBone->order = mgBone::ZYX;
		else if (orderBuf == "XY")	mBone->order = mgBone::XY;
		else if (orderBuf == "XZ")	mBone->order = mgBone::XZ;
		else if (orderBuf == "YX")	mBone->order = mgBone::YX;
		else if (orderBuf == "YZ")	mBone->order = mgBone::YZ;
		else if (orderBuf == "ZX")	mBone->order = mgBone::ZX;
		else if (orderBuf == "ZY")	mBone->order = mgBone::ZY;
		else if (orderBuf == "X")	mBone->order = mgBone::X;
		else if (orderBuf == "Y")	mBone->order = mgBone::Y;
		else if (orderBuf == "Z")	mBone->order = mgBone::Z;
	}


	if (mBone->channel & mgBone::translateChannel)
		skeleton->numT++;
	if (mBone->channel & mgBone::rotationChannel)
		skeleton->numR++;

	skeleton->dataPos.push_back(skeleton->nTotalChannel);
	skeleton->nTotalChannel += mBone->nChannel;

	//osg::Matrix mat = bone->getMatrixInBoneSpace();
	bool before = true;
	osg::Matrixd H, Hp;
	H.identity(); Hp.identity();

	osg::ref_ptr<osgAnimation::UpdateMatrixTransform> update = dynamic_cast<osgAnimation::UpdateMatrixTransform*>(bone->getUpdateCallback());
	if (update)
	{
		osgAnimation::StackedTransform& trans = update->getStackedTransforms();
		for (int i = 0; i < trans.size(); i++)
		{
			// assume that there is no translation and scaling channel in the bone
			osg::ref_ptr<osgAnimation::StackedRotateAxisElement> srae = dynamic_cast<osgAnimation::StackedRotateAxisElement*>(trans[i].get());
			if (srae) {
				before = false;
				continue;
			}

			// translation_dummy
			osg::ref_ptr<osgAnimation::StackedTranslateElement> stae = dynamic_cast<osgAnimation::StackedTranslateElement*>(trans[i].get());
			if (stae && use_dummy) {
				H.setTrans(translation_dummy(0), translation_dummy(1), translation_dummy(2));
			}

			if (before)
				trans[i]->applyToMatrix(H);
			else
				trans[i]->applyToMatrix(Hp);
		}

		mBone->H.set(H.ptr());
	}
	else mBone->H.set(bone->getMatrixInBoneSpace().ptr());

	for (int i = 0; i<bone->getNumChildren(); ++i)
	{
		mgBone* child = readBoneNode(bone->getChild(i), skeleton);

		if (child)
		{
			child->H = gXMat(Hp.ptr()) * child->H;

			mBone->localEndPoints.push_back(child->H.trn());
			skeleton->addChild(mBone, child);
		}
	}

	return mBone;
}

void FBXLoader::alterChannel(osg::ref_ptr<osgAnimation::Bone> bone, mgBone* mBone)
{
	osg::ref_ptr<osgAnimation::UpdateMatrixTransform> pUpdate = dynamic_cast<osgAnimation::UpdateMatrixTransform*>(bone->getUpdateCallback());

	if (!pUpdate) return;

	const osgAnimation::StackedTransform& transforms = pUpdate->getStackedTransforms();

	bool fRotCh = false;
	bool fTrnCh = false;

	std::string orderBuf;

	for (osgAnimation::StackedTransform::const_iterator it = transforms.begin(); it != transforms.end(); ++it)
	{
		osgAnimation::StackedTransformElement* element = it->get();

		if (!element) continue;

		std::string eName = element->getName();

		if (eName.find("rotate") != std::string::npos)
		{
			mBone->nChannel++;
			fRotCh = true;
			orderBuf += eName.at(6);
		}
		else if (eName.find("translate") != std::string::npos)
		{
			mBone->nChannel += 3;
			fTrnCh = true;
		}
	}

	if (fRotCh)
		mBone->channel |= mgBone::rotationChannel;
	if (fTrnCh)
		mBone->channel |= mgBone::translateChannel;

	if (orderBuf == "XYZ")		mBone->order = mgBone::XYZ;
	else if (orderBuf == "XZY") mBone->order = mgBone::XZY;
	else if (orderBuf == "YXZ") mBone->order = mgBone::YXZ;
	else if (orderBuf == "YZX") mBone->order = mgBone::YZX;
	else if (orderBuf == "ZXY") mBone->order = mgBone::ZXY;
	else if (orderBuf == "ZYX") mBone->order = mgBone::ZYX;
	else if (orderBuf == "XY")	mBone->order = mgBone::XY;
	else if (orderBuf == "XZ")	mBone->order = mgBone::XZ;
	else if (orderBuf == "YX")	mBone->order = mgBone::YX;
	else if (orderBuf == "YZ")	mBone->order = mgBone::YZ;
	else if (orderBuf == "ZX")	mBone->order = mgBone::ZX;
	else if (orderBuf == "ZY")	mBone->order = mgBone::ZY;
	else if (orderBuf == "X")	mBone->order = mgBone::X;
	else if (orderBuf == "Y")	mBone->order = mgBone::Y;
	else if (orderBuf == "Z")	mBone->order = mgBone::Z;
}

void FBXLoader::getChannelInfo(osg::ref_ptr<osgAnimation::Bone> bone, unsigned int& channel, unsigned int& nChannel, std::string& orderBuf)
{
	bool fRotCh = false;
	bool fTrnCh = false;
	channel = 0;
	nChannel = 0;
	orderBuf = "";

	if (_animation)
	{
		const osgAnimation::ChannelList& cList = _animation->getChannels();

		for (int i = 0; i < cList.size(); i++)
		{
			const osg::ref_ptr<osgAnimation::Channel> channel = cList[i];

			if (channel->getTargetName() != bone->getName())
				continue;

			const osgAnimation::Sampler* sampler = channel->getSampler();

			std::string chName = channel->getName();
			if (chName == "translate")
			{
				nChannel += 3;
				fTrnCh = true;
				
			} 
			if (chName.find("rotate") != std::string::npos)
			{
				nChannel++;
				fRotCh = true;
				orderBuf += chName.at(6);
			}
		}
	}
	else
	{
		osg::ref_ptr<osgAnimation::UpdateMatrixTransform> pUpdate = dynamic_cast<osgAnimation::UpdateMatrixTransform*>(bone->getUpdateCallback());

		if (!pUpdate) return;

		const osgAnimation::StackedTransform& transforms = pUpdate->getStackedTransforms();

		//for (osgAnimation::StackedTransform::const_iterator it = transforms.begin(); it != transforms.end(); ++it)
		for (osgAnimation::StackedTransform::const_reverse_iterator it = transforms.rbegin();
			it != transforms.rend();
			++it)
		{
			osgAnimation::StackedTransformElement* element = it->get();

			if (!element) continue;

			std::string eName = element->getName();

			if (eName.find("rotate") != std::string::npos)
			{
				nChannel++;
				fRotCh = true;
				orderBuf += eName.at(6);
			}
			else if (eName.find("translate") != std::string::npos)
			{
				nChannel += 3;
				fTrnCh = true;
			}
		}
	}

	if (fRotCh)
		channel |= mgBone::rotationChannel;
	if (fTrnCh)
		channel |= mgBone::translateChannel;

	//test
	//std::cout << "n ch: " << nChannel << ", rot: " << orderBuf << ", rot: " << (channel | mgBone::rotationChannel) << ", trn: " << (channel | mgBone::translateChannel) << std::endl;

}

void FBXLoader::arrangeChannelPos(mgSkeleton *skeleton)
{
	if (_animation)
	{
		const osg::ref_ptr<osgAnimation::Animation> animation = _animationMgr->getAnimationList()[0];
		const osgAnimation::ChannelList& cList = animation->getChannels();

		skeleton->dataPos.resize(skeleton->bones.size());

		int dPos = 0;
		for (int i = 0; i < cList.size(); i++)
		{
			const osg::ref_ptr<osgAnimation::Channel> channel = cList[i];

			mgSkeleton::BONEMIt it = skeleton->getBoneMapFind(channel->getTargetName());
			if (it == skeleton->getBoneMapEnd())
			{
				//std::cout << "cannot find " << channel->getTargetName() << " bone." << std::endl;
				continue;
			}
			mgBone* bone = skeleton->bones[it->second];
			skeleton->dataPos[it->second] = dPos;

			dPos += bone->nChannel;
		}

	}
}

void FBXLoader::takeZeroPose(osg::ref_ptr<osg::Node> node)
{
	osg::ref_ptr<osgAnimation::Bone> b = dynamic_cast<osgAnimation::Bone*>(node.get());
	if (!b)
	{
		osg::ref_ptr<osg::Group> group = dynamic_cast<osg::Group*>(node.get());
		if (group)
		{
			for (int i = 0; i < group->getNumChildren(); ++i)
			{
				takeZeroPose(group->getChild(i));
			}
		}
		return;
	}

	// update
	osg::Matrixd t_matrix;
	t_matrix.makeIdentity();

	osg::ref_ptr<osgAnimation::UpdateMatrixTransform> update = dynamic_cast<osgAnimation::UpdateMatrixTransform*>(b->getUpdateCallback());
	osgAnimation::StackedTransform& trans = update->getStackedTransforms();
	for (int i = 0; i<trans.size(); i++)
	{
		osg::ref_ptr<osgAnimation::StackedTransformElement> elem = trans[i];
		if (elem->getTarget())
		{
			osg::ref_ptr<osgAnimation::StackedRotateAxisElement>	srae = dynamic_cast<osgAnimation::StackedRotateAxisElement*>(elem.get());
			if (srae)
			{
				srae->setAngle(0.);
				continue;
			}

			osg::ref_ptr<osgAnimation::StackedMatrixElement>		sme = dynamic_cast<osgAnimation::StackedMatrixElement*>(elem.get());
			if (sme)
			{
				sme->setMatrix(osg::Matrixd());
				continue;
			}

			osg::ref_ptr<osgAnimation::StackedTranslateElement>		ste = dynamic_cast<osgAnimation::StackedTranslateElement*>(elem.get());
			if (ste)
			{
				ste->setTranslate(osg::Vec3());
				continue;
			}

			osg::ref_ptr<osgAnimation::StackedQuaternionElement>	sqe = dynamic_cast<osgAnimation::StackedQuaternionElement*>(elem.get());
			if (sqe)
			{
				sqe->setQuaternion(osg::Quat());
				continue;
			}

			osg::ref_ptr<osgAnimation::StackedScaleElement>			sse = dynamic_cast<osgAnimation::StackedScaleElement*>(elem.get());
			if (sse)
			{
				sse->setScale(osg::Vec3(1., 1., 1.));
				continue;
			}
		}
		elem->applyToMatrix(t_matrix);
	}

	b->setMatrix(t_matrix);
	osgAnimation::Bone* parent = b->getBoneParent();
	if (parent)
		b->setMatrixInSkeletonSpace(t_matrix * parent->getMatrixInSkeletonSpace());
	else
		b->setMatrixInSkeletonSpace(t_matrix);

	for (int i = 0; i<b->getNumChildren(); ++i)
	{
		takeZeroPose(b->getChild(i));
	}
}
