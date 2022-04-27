//#####################################################################
// Copyright 2017, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#include "MocapProcessor/mgMBSUtil.h"
#include "MocapProcessor/mgUtility.h"
#include <bitset>
#include <sstream>

int mgMBSUtil::setPoseFromRawData(gMultibodySystem *mbs, mgSkeleton* skeleton, CoordinateType *data)
{
	assert( mbs->numLinks() == skeleton->bones.size() );

	gRotMat rot;
	gXMat worldMat;//, trans, rot, temp;
	gVec3 transVec;
//	int dataPos;
	mgBone* bone;
	gLink* link;

	for( unsigned int i=0; i<skeleton->bones.size(); i++ )
	{
		bone = skeleton->bones[i];

		link = mbs->link(i);

		gVec3 rotVec;
		gRotMat rotMat;

		if( bone->nChannel < 3 )//if( link->type() == TYPE_1D_LINK || link->type() == TYPE_2D_LINK )
			mgUtility::getTransAndRotVecFromData(data, bone, transVec, rotVec);
		else //ball link or free link
			mgUtility::getTransAndRotMatFromData(data, bone, transVec, rotMat);

		switch( link->type() )
		{
		case TYPE_1D_LINK:
			{
				g1dLink* link1d = static_cast<g1dLink*>(link);
				if( bone->channel == mgBone::Xrotation ) link1d->setCoord(rotVec.x());
				else if( bone->channel == mgBone::Yrotation ) link1d->setCoord(rotVec.y());
				else if( bone->channel == mgBone::Zrotation ) link1d->setCoord(rotVec.z());
				else if( bone->nChannel == 3 ){ 
					//this case is link is 1dLink but bone is ballLink
					link1d->setCoord( (gVec3::log(rotMat),link1d->screw().rot()) ); //this is quick yet sub-optimal solution.
				}
				else{ assert(0); } //not handled yet
			}
			break;
		case TYPE_2D_LINK: ///ONLY FOR UNIVERSAL JOINT
			{
				g2dLink* link2d = static_cast<g2dLink*>(link);
				switch( bone->order )
				{
				case mgBone::XY: link2d->setCoord(0,rotVec.x()); link2d->setCoord(1,rotVec.y()); break;
				case mgBone::XZ: link2d->setCoord(0,rotVec.x()); link2d->setCoord(1,rotVec.z()); break;
				case mgBone::YX: link2d->setCoord(0,rotVec.y()); link2d->setCoord(1,rotVec.x()); break;
				case mgBone::YZ: link2d->setCoord(0,rotVec.y()); link2d->setCoord(1,rotVec.z()); break;
				case mgBone::ZX: link2d->setCoord(0,rotVec.z()); link2d->setCoord(1,rotVec.x()); break;
				case mgBone::ZY: link2d->setCoord(0,rotVec.z()); link2d->setCoord(1,rotVec.y()); break;
				default: break;
				}
			}
			break;
		case TYPE_BALL_LINK:
			{
				gBallLink* ball = static_cast<gBallLink*>(link);				
				ball->setCoord(rotMat);
			}
			break;
		case TYPE_WELDED_LINK:
			break;
		case TYPE_FREE_LINK:
			{
				gFreeLink* free = static_cast<gFreeLink*>(link);				
				gXMat mat(rotMat, transVec);
				free->setCoord(mat);
			}
			break;
		}

	}
	
	return 0;
}

int mgMBSUtil::setPoseFromRawData(gMultibodySystem *mbs, double* data)
{
	for (int i=0;i<mbs->numLinks();i++)
	{
		gLink* link = mbs->link(i);
		if (i==0)
		{			
			gRotMat rot;
			rot.setIdentity();
			//rot.makeRotateZXY(data[3],data[4],data[5]);
			//gXMat T(rot,gVec3(data[0],data[1],data[2]));

			rot.makeRotateZXY(data[3],data[4],data[5]);
			gXMat T(rot,gVec3(data[0],data[1],data[2]));

			((gFreeLink*)link)->setCoord(T);	
			
		}
		else
		{
			gRotMat rot;
			rot.makeRotateZXY(data[3*(i+1)], data[3*(i+1)+1], data[3*(i+1)+2]);
			if(link->dof()==6)link->setLocalFrame(rot);
			else if(link->dof()==3)((gBallLink*)link)->setCoord(rot);
			else if(link->dof()==1)
			{
				gVec3 s = gVec3::log(rot); s.normalize(); //SHL: FIXED...
				gReal theta = acos( (s,link->screwBody(0).rot()) );
				((g1dLink*)link)->setCoord(theta);
			}
		}
	}

	return 0;
}

// test
static inline int gSgn(gReal x)
{
	return (x>=0) ? 1:-1;
} 
static int gRound(gReal x)
{
	return (int)(gSgn(x)*(gFabs(x)+gREAL(0.5)));
}
gVec3 log(const gRotMat& R, const gVec3 nbor)
{
	gReal x = gMin(1 , gREAL(0.5)*R.trace()-gREAL(0.5));
	x = gMax(-1, x);
	gReal angle = acos( x );	// 0 to pi
	
	if( angle < gEPSILON )//if R=eye
	{
		if( IsZero(nbor.x()) && IsZero(nbor.y()) && IsZero(nbor.z()) )
		{
			return gVec3(0,0,0);
		}
		else
		{
			gReal len = nbor.magnitude();
			gVec3 w(nbor); w/=len;
			return w*(2*gPI*gRound(len/(2*gPI)));
		}
	}
	else
	{
		gReal s = gSin(angle);
		gVec3 w;
		w.setFromSkewSymMatrix(R.cptr());
		w*=gInv(s);

		int n = gRound( ( (nbor,w)/(w,w) - angle ) / (2*gPI) );
		gVec3 w1 = w * (2*gPI*n+angle);

		int n2 = gRound( ( (nbor,w)/(w,w) + angle ) / (2*gPI) );
		gVec3 w2 = w * (2*gPI*(n2+1)-angle);

		gVec3 wr;
		if( (nbor-w1).magnitudeSquared() > (nbor-w2).magnitudeSquared() )
		{
			wr = w2;
		} else {
			wr = w1;
		}

		return wr;
	}
}

int mgMBSUtil::getCoordArrayFromRawData(arma::vec& coord, const gMultibodySystem *mbs, mgSkeleton* skeleton, const CoordinateType *data, const arma::vec* ref)
{
	assert( mbs->numLinks() == skeleton->bones.size() );

	gRotMat rot;
	gXMat worldMat;//, trans, rot, temp;
	gVec3 transVec;
	const gLink* link;
	mgBone* bone;

	//unsigned int pos = 0;
	if( coord.n_elem != mbs->dof() ) coord.resize(mbs->dof());
	coord.zeros();

	//int i = skeleton->bones.size() - 1;
	for( unsigned int i=0; i<skeleton->bones.size(); i++ )
	{
		bone = skeleton->bones[i];
		link = mbs->link(i);

		int idx = link->genCoordIndex();

		gVec3 rotVec;
		gRotMat rotMat;

		mgUtility::getTransAndRotVecFromData(data, bone, transVec, rotVec);
		mgUtility::getRotMatFromRotVec(bone, rotVec, rotMat);

		//if( bone->nChannel < 3 )//if( link->type() == TYPE_1D_LINK || link->type() == TYPE_2D_LINK )
		//	mgUtility::getTransAndRotVecFromData(data, bone, transVec, rotVec);
		//else //ball link or free link
		//	mgUtility::getTransAndRotMatFromData(data, bone, transVec, rotMat);

		switch( link->type() )
		{
		case TYPE_1D_LINK:
			{
				//g1dLink* link1d = static_cast<g1dLink*>(link);

				if( bone->channel == mgBone::Xrotation ) coord(idx) = rotVec.x();
				else if( bone->channel == mgBone::Yrotation ) coord(idx) = rotVec.y();
				else if( bone->channel == mgBone::Zrotation ) coord(idx) = rotVec.z();
				//else if( bone->nChannel == 3 ){ 
				//	//this case is link is 1dLink but bone is ballLink
				//	link1d->setCoord( (gVec3::log(rotMat),link1d->screw().rot()) ); //this is quick yet sub-optimal solution.
				//}
				else{ assert(0); } //not handled yet
			}
			break;
		case TYPE_2D_LINK: ///ONLY FOR UNIVERSAL JOINT
			{
				//g2dLink* link2d = static_cast<g2dLink*>(link);
				switch( bone->order )
				{
				case mgBone::XY: coord(idx+0)=rotVec.x();coord(idx+1)=rotVec.y(); break;
				case mgBone::XZ: coord(idx+0)=rotVec.x();coord(idx+1)=rotVec.z(); break;
				case mgBone::YX: coord(idx+0)=rotVec.y();coord(idx+1)=rotVec.x(); break;
				case mgBone::YZ: coord(idx+0)=rotVec.y();coord(idx+1)=rotVec.z(); break;
				case mgBone::ZX: coord(idx+0)=rotVec.z();coord(idx+1)=rotVec.x(); break;
				case mgBone::ZY: coord(idx+0)=rotVec.z();coord(idx+1)=rotVec.y(); break;
				default: break;
				}
			}
			break;
		case TYPE_BALL_LINK:
			{
				gVec3 rot;
				// Possible error because singularity.
				if( ref )
				{
					//gVec3 nbor( ref->at(idx), ref->at(idx+1), ref->at(idx+2) );
					gVec3 nbor( ref->memptr() + idx ); // ref->at(idx), ref->at(idx+1), ref->at(idx+2) );
					//rot = gVec3::log(rotMat, nbor);
					rot = log(rotMat, nbor);
				} else rot = gVec3::log(rotMat);

				coord(idx+0) = rot.x();
				coord(idx+1) = rot.y();
				coord(idx+2) = rot.z();
			}
			break;
		case TYPE_WELDED_LINK:
			break;
		case TYPE_FREE_LINK:
			{
				//trn, rot. why??!?
				
				gVec3 rot;
				// Possible error because singularity.
				if( ref )
				{
					gVec3 nbor( ref->memptr() + idx );
					//rot = gVec3::log(rotMat, nbor);
					rot = log(rotMat, nbor);
				} else rot = gVec3::log(rotMat);

				//coord(idx+0) = transVec.x();
				//coord(idx+1) = transVec.y();
				//coord(idx+2) = transVec.z();

				//coord(idx+3) = rot.x();
				//coord(idx+4) = rot.y();
				//coord(idx+5) = rot.z();

				coord(idx+0) = rot.x();
				coord(idx+1) = rot.y();
				coord(idx+2) = rot.z();
				coord(idx+3) = transVec.x();
				coord(idx+4) = transVec.y();
				coord(idx+5) = transVec.z();
			}
			break;
		}

	}
	
	return 0;
}

int mgMBSUtil::getCoordArrayFromRawData2(arma::vec& coord, gMultibodySystem *mbs, mgSkeleton* skeleton, const CoordinateType *data, const arma::vec* ref)
{
	//assert( mbs->numLinks() == skeleton->bones.size() );

	gRotMat rot;
	gXMat worldMat;//, trans, rot, temp;
	gVec3 transVec;
	gLink* link;
	mgBone* bone;

	if( coord.n_elem != mbs->dof() ) coord.resize(mbs->dof());
	coord.zeros();

	for( unsigned int i=0; i<skeleton->bones.size(); i++ )
	{
		bone = skeleton->bones[i];
		//link = mbs->link(i);
		link = mbs->findLink(bone->name.c_str());

		if( link == NULL ) continue;

		int idx = link->genCoordIndex();

		gVec3 rotVec;
		gRotMat rotMat;

		mgUtility::getTransAndRotVecFromData(data, bone, transVec, rotVec);
		mgUtility::getRotMatFromRotVec(bone, rotVec, rotMat);

		unsigned int nRotChannel = bone->channel & bone->rotationChannel;
		std::bitset<32> nRotChBit(nRotChannel);
		int nRot_Data = nRotChBit.count();

		switch( link->type() )
		{
		case TYPE_1D_LINK:
			{
				g1dLink* link1d = static_cast<g1dLink*>(link);

				if( nRot_Data == 1 ) {
					if( bone->channel == mgBone::Xrotation ) coord(idx) = rotVec.x();
					else if( bone->channel == mgBone::Yrotation ) coord(idx) = rotVec.y();
					else if( bone->channel == mgBone::Zrotation ) coord(idx) = rotVec.z();
				} else {
					coord(idx) = (gVec3::log(rotMat), link1d->screw().rot());
				}
			}
			break;
		case TYPE_2D_LINK: ///ONLY FOR UNIVERSAL JOINT
			{
				assert( nRot_Data == 2 );

				//g2dLink* link2d = static_cast<g2dLink*>(link);
				switch( bone->order )
				{
					case mgBone::XY: coord(idx+0)=rotVec.x();coord(idx+1)=rotVec.y(); break;
					case mgBone::XZ: coord(idx+0)=rotVec.x();coord(idx+1)=rotVec.z(); break;
					case mgBone::YX: coord(idx+0)=rotVec.y();coord(idx+1)=rotVec.x(); break;
					case mgBone::YZ: coord(idx+0)=rotVec.y();coord(idx+1)=rotVec.z(); break;
					case mgBone::ZX: coord(idx+0)=rotVec.z();coord(idx+1)=rotVec.x(); break;
					case mgBone::ZY: coord(idx+0)=rotVec.z();coord(idx+1)=rotVec.y(); break;
					default: break;
				}

				
			}
			break;
		case TYPE_BALL_LINK:
			{
				assert( nRot_Data == 3 );

				gVec3 rot;
				// Possible error because singularity.
				if( ref )
				{
					//gVec3 nbor( ref->at(idx), ref->at(idx+1), ref->at(idx+2) );
					gVec3 nbor( ref->memptr() + idx ); // ref->at(idx), ref->at(idx+1), ref->at(idx+2) );
					//rot = gVec3::log(rotMat, nbor);
					rot = log(rotMat, nbor);
				} else rot = gVec3::log(rotMat);

				coord(idx+0) = rot.x();
				coord(idx+1) = rot.y();
				coord(idx+2) = rot.z();
			}
			break;
		case TYPE_WELDED_LINK:
			break;
		case TYPE_FREE_LINK:
			{
				gVec3 rot;
				// Possible error because singularity.
				if( ref )
				{
					gVec3 nbor( ref->memptr() + idx );
					//rot = gVec3::log(rotMat, nbor);
					rot = log(rotMat, nbor);
				} else rot = gVec3::log(rotMat);

				//coord(idx+0) = transVec.x();
				//coord(idx+1) = transVec.y();
				//coord(idx+2) = transVec.z();

				//coord(idx+3) = rot.x();
				//coord(idx+4) = rot.y();
				//coord(idx+5) = rot.z();

				coord(idx+0) = rot.x();
				coord(idx+1) = rot.y();
				coord(idx+2) = rot.z();
				coord(idx+3) = transVec.x();
				coord(idx+4) = transVec.y();
				coord(idx+5) = transVec.z();
			}
			break;
		}

	}
	
	return 0;
}

int mgMBSUtil::getCompactCoordFromPose(arma::vec& coord, gMultibodySystem *mbs, const arma::vec& ref)
{
	for( int i=0; i<mbs->numLinks(); i++ )
	{
		gLink* link = mbs->link(i);
		int jIdx = link->genCoordIndex();
		gVec3 nbor(ref.memptr() + jIdx);

		switch( link->type() )
		{
		case TYPE_1D_LINK:
			{
				g1dLink* link1d = static_cast<g1dLink*>(link);
				coord(jIdx) = link1d->coord();
			}
			break;
		case TYPE_2D_LINK: ///ONLY FOR UNIVERSAL JOINT
			{
				g2dLink* link2d = static_cast<g2dLink*>(link);
				coord(jIdx) = link2d->coord(0);
				coord(jIdx+1) = link2d->coord(1);
			}
			break;
		case TYPE_BALL_LINK:
			{
				gBallLink* linkBall = static_cast<gBallLink*>(link);
				gVec3 rot = log(linkBall->coord(), nbor);

				coord(jIdx+0) = rot.x();
				coord(jIdx+1) = rot.y();
				coord(jIdx+2) = rot.z();
			}
			break;
		case TYPE_WELDED_LINK:
			break;
		case TYPE_FREE_LINK:
			{
				gFreeLink* linkFree = static_cast<gFreeLink*>(link);

				gVec3 rot = log(linkFree->coord().rot(), nbor);
				gVec3 trans = linkFree->coord().trn();

				coord(jIdx+0) = rot.x();
				coord(jIdx+1) = rot.y();
				coord(jIdx+2) = rot.z();
				coord(jIdx+3) = trans.x();
				coord(jIdx+4) = trans.y();
				coord(jIdx+5) = trans.z();
			}
			break;
		}
	}

	return 0;
}

int mgMBSUtil::setPoseFromRawExpData(gMultibodySystem *mbs, mgSkeleton* skeleton, CoordinateType *data)
{
	gRotMat rot;
	gXMat worldMat;//, trans, rot, temp;
	gVec3 transVec;
	gLink* link;
	mgBone* bone;

	unsigned int pos = 0;

	for( unsigned int i=0; i<mbs->numLinks(); i++ )
	{
		bone = skeleton->bones[i];
		link = mbs->link(i);

		gVec3 rotVec;
		gRotMat rotMat;

		switch( link->type() )
		{
		case TYPE_1D_LINK:
			{
				rotVec.setX( data[pos++] );
				rotVec.setY( data[pos++] );
				rotVec.setZ( data[pos++] );
				rotMat = gRotMat::exp(rotVec);

				g1dLink* link1d = static_cast<g1dLink*>(link);
				if( bone->channel == mgBone::Xrotation ) link1d->setCoord(rotVec.x());
				else if( bone->channel == mgBone::Yrotation ) link1d->setCoord(rotVec.y());
				else if( bone->channel == mgBone::Zrotation ) link1d->setCoord(rotVec.z());
				else if( bone->nChannel == 3 ){ 
					//this case is link is 1dLink but bone is ballLink
					link1d->setCoord( (gVec3::log(rotMat),link1d->screw().rot()) ); //this is quick yet sub-optimal solution.
				}
				else{ assert(0); } //not handled yet
			}
			break;
		case TYPE_2D_LINK: ///ONLY FOR UNIVERSAL JOINT
			{
				rotVec.setX( data[pos++] );
				rotVec.setY( data[pos++] );
				rotVec.setZ( data[pos++] );
				rotMat = gRotMat::exp(rotVec);

				g2dLink* link2d = static_cast<g2dLink*>(link);
				switch( bone->order )
				{
				case mgBone::XY:
					{
						rotVec = mgUtility::getXYZFromRotation(rotMat);
						link2d->setCoord(0,rotVec.x()); link2d->setCoord(1,rotVec.y()); 
					}
					break;
				case mgBone::XZ:
					{
						rotVec = mgUtility::getXZYFromRotation(rotMat);
						link2d->setCoord(0,rotVec.x()); link2d->setCoord(1,rotVec.z());
					}
					break;
				case mgBone::YX:
					{
						rotVec = mgUtility::getYXZFromRotation(rotMat);
						link2d->setCoord(0,rotVec.y()); link2d->setCoord(1,rotVec.x());
					}
					break;
				case mgBone::YZ:
					{
						rotVec = mgUtility::getYZXFromRotation(rotMat);
						link2d->setCoord(0,rotVec.y()); link2d->setCoord(1,rotVec.z());
					}
					break;
				case mgBone::ZX:
					{
						rotVec = mgUtility::getZXYFromRotation(rotMat);
						link2d->setCoord(0,rotVec.z()); link2d->setCoord(1,rotVec.x());
					}
					break;
				case mgBone::ZY:
					{
						rotVec = mgUtility::getZYXFromRotation(rotMat);
						link2d->setCoord(0,rotVec.z()); link2d->setCoord(1,rotVec.y());
					}
					break;
				default: break;
				}
			}
			break;
		case TYPE_BALL_LINK:
			{
				rotVec.setX( data[pos++] );
				rotVec.setY( data[pos++] );
				rotVec.setZ( data[pos++] );
				rotMat = gRotMat::exp(rotVec);

				gBallLink* ball = static_cast<gBallLink*>(link);				
				ball->setCoord(rotMat);
			}
			break;
		case TYPE_WELDED_LINK:
			break;
		case TYPE_FREE_LINK:
			{
				rotVec.setX( data[pos++] );
				rotVec.setY( data[pos++] );
				rotVec.setZ( data[pos++] );

				transVec.setX( data[pos++] );
				transVec.setY( data[pos++] );
				transVec.setZ( data[pos++] );
				rotMat = gRotMat::exp(rotVec);

				gFreeLink* free = static_cast<gFreeLink*>(link);				
				gXMat mat(rotMat, transVec);
				free->setCoord(mat);

			}
			break;
		}
	}
	
	return 0;
}

int mgMBSUtil::getQuatCoordArrayFromRawData(arma::vec& coord, const gMultibodySystem *mbs, const mgSkeleton* skeleton, const CoordinateType *data)
{
	assert(mbs->numLinks() == skeleton->bones.size());

	gRotMat rot;
	gXMat worldMat;
	gVec3 transVec;
	mgBone* bone;
	
	if (coord.n_elem != mbs->sizeSafeCoordArray()) coord.resize(mbs->sizeSafeCoordArray());
	coord.zeros();

	int idx = 0;
	for (unsigned int i = 0; i<skeleton->bones.size(); i++)
	{
		bone = skeleton->bones[i];
		const gLink* link = mbs->link(i);

		gVec3 rotVec;
		gRotMat rotMat;

		mgUtility::getTransAndRotVecFromData(data, bone, transVec, rotVec);
		mgUtility::getRotMatFromRotVec(bone, rotVec, rotMat);

		switch (link->type())
		{
		case TYPE_1D_LINK:
		{
			if (bone->channel == mgBone::Xrotation) coord(idx++) = rotVec.x();
			else if (bone->channel == mgBone::Yrotation) coord(idx++) = rotVec.y();
			else if (bone->channel == mgBone::Zrotation) coord(idx++) = rotVec.z();
		}
		break;
		case TYPE_2D_LINK: ///ONLY FOR UNIVERSAL JOINT
		{
			switch (bone->order)
			{
			case mgBone::XY: coord(idx++) = rotVec.x(); coord(idx++) = rotVec.y(); break;
			case mgBone::XZ: coord(idx++) = rotVec.x(); coord(idx++) = rotVec.z(); break;
			case mgBone::YX: coord(idx++) = rotVec.y(); coord(idx++) = rotVec.x(); break;
			case mgBone::YZ: coord(idx++) = rotVec.y(); coord(idx++) = rotVec.z(); break;
			case mgBone::ZX: coord(idx++) = rotVec.z(); coord(idx++) = rotVec.x(); break;
			case mgBone::ZY: coord(idx++) = rotVec.z(); coord(idx++) = rotVec.y(); break;
			default: break;
			}
		}
		break;
		case TYPE_BALL_LINK:
		{
			gQuat rot = rotMat.inQuat();
			
			coord(idx++) = rot.x();
			coord(idx++) = rot.y();
			coord(idx++) = rot.z();
			coord(idx++) = rot.w();
		}
		break;
		case TYPE_WELDED_LINK:
			break;
		case TYPE_FREE_LINK:
		{
			gQuat rot = rotMat.inQuat();

			coord(idx++) = rot.x();
			coord(idx++) = rot.y();
			coord(idx++) = rot.z();
			coord(idx++) = rot.w();

			coord(idx++) = transVec.x();
			coord(idx++) = transVec.y();
			coord(idx++) = transVec.z();
		}
		break;
		}

	}

	return 0;
}