//#####################################################################
// Copyright 2010-2015, Sukwon Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#include "Loader/mgSkeletonToBCharacter.h"

int mgSkeletonToBCharacter::saveToBCharacter(const mgSkeleton *skeleton, const char* filename, double totalMass, double minBoneLength, double defaultLinkAspectRatio)
{
	FILE *fp = fopen(filename, "w");
	if( !fp )
	{
		printf("error: cannot open a file(%s) to write.\n", filename);
		return -1;
	}

	fprintf(fp, "HIERARCHY\n\n");

	mgBone* bone;
	gVec3 pos;
	gQuat quat;
	for( int i=0; i<skeleton->bones.size(); i++ )
	{
		bone = skeleton->bones[i];
		pos = bone->H.trn();
		quat = bone->H.rotInQuat();

		fprintf(fp, "LINK\n");
		fprintf(fp, "NAME %s\n", bone->name.c_str());
		
		// Warning: Assume that the root bone index is zero as like getRootPose
		if( i == 0 )
		{
			fprintf(fp, "REF WORLD\n");

		} else {
			fprintf(fp, "PARENT %s\n", bone->parent->name.c_str());
			fprintf(fp, "REF LOCAL\n");
		}

		fprintf(fp, "POS %g %g %g\n", pos.x(), pos.y(), pos.z());
		fprintf(fp, "ROT QUAT %g %g %g %g\n", quat.x(), quat.y(), quat.z(), quat.w()); 

		// free
		if((bone->channel & mgBone::translateChannel) != 0)
		{
			if( ((bone->channel & mgBone::rotationChannel) ^ mgBone::rotationChannel ) == 0 ||
				((bone->channel & mgBone::expChannel) ^ mgBone::expChannel ) == 0 )
			{
				fprintf(fp, "JOINT ACC FREE\n");
			} else {
				fprintf(fp, "JOINT ACC WELDED\n");
			}
		} else if( ((bone->channel & mgBone::rotationChannel) ^ mgBone::rotationChannel ) == 0 ||
					((bone->channel & mgBone::expChannel) ^ mgBone::expChannel ) == 0 ) {
			fprintf(fp, "JOINT ACC BALL\n");
			//FIXME: MBS supports the revol limit only.
		} else if( (bone->channel ^ (mgBone::Xrotation)) == 0 ) {
			fprintf(fp, "JOINT ACC REVOLUTE 1 0 0\n");
		} else if( (bone->channel ^ (mgBone::Yrotation)) == 0 ) {
			fprintf(fp, "JOINT ACC REVOLUTE 0 1 0\n");
		} else if( (bone->channel ^ (mgBone::Zrotation)) == 0 ) {
			fprintf(fp, "JOINT ACC REVOLUTE 0 0 1\n");
		} else if( (bone->channel ^ (mgBone::Xrotation | mgBone::Zrotation)) == 0 ) {
			fprintf(fp, "JOINT ACC UNIVERSAL X Z\n");
		} else if( (bone->channel ^ (mgBone::Xrotation | mgBone::Yrotation)) == 0 ) {
			fprintf(fp, "JOINT ACC UNIVERSAL X Y\n");
		} else if( (bone->channel ^ (mgBone::Yrotation | mgBone::Zrotation)) == 0 ) {
			fprintf(fp, "JOINT ACC UNIVERSAL Y Z\n");
		} else {
			// WELD
			fprintf(fp, "JOINT ACC WELDED\n");
		}

		// range
		if (bone->limit.size() == bone->nChannel && bone->nChannel > 0)
		{
			fprintf(fp, "RANGE ");
			for (int d = 0; d < bone->nChannel; d++)
			{
				fprintf(fp, "(%d:%g,%g) ", d, bone->limit[d].first, bone->limit[d].second);
			}
			fprintf(fp, "\n");
		}
		
		fprintf(fp, "END_LINK\n\n");

	}

	fprintf(fp, "END_HIERARCHY\n\nINERTIAL_PROPERTIES\n\n");

//	mgBone *child;
	gVec3 AA, BB, posVec;
	std::vector<gVec3> bbs; //bounding box
	std::vector<gXMat> localCoMFrames;
	double totalVolume = 0;	
	double mass, volume;
	

	for( int i=0; i<skeleton->bones.size(); i++ )
	{
		bone = skeleton->bones[i];

		if(bone->localEndPoints.size()==1)
		{
			gVec3 lep = bone->localEndPoints[0];

			//calc total volume
			double linkHeight = std::max(lep.magnitude(),minBoneLength);
			double linkWidth = defaultLinkAspectRatio*linkHeight;
			gVec3 bb(linkWidth,linkHeight,linkWidth);
			bbs.push_back(bb);
			volume = linkHeight*linkWidth*linkWidth;
			totalVolume += volume;
	
			//localCoMFrames
			gXMat CoMFrame; CoMFrame.setTrn(lep*0.5);
		
			if(lep.magnitude()>0.001){
				lep.normalize();
				gQuat q; q.makeRotate(gVec3(0,1,0),lep);
				CoMFrame.setRot(q.inRotMatrix());
			}else{
				CoMFrame.setRot(gRotMatOne); //just identity
			}
			localCoMFrames.push_back(CoMFrame);		
		}else{

			//compute AABB
			double halfMinBoneLength = 0.5*minBoneLength;
			gVec3 bbL(-halfMinBoneLength,-halfMinBoneLength,-halfMinBoneLength);
			gVec3 bbH(halfMinBoneLength,halfMinBoneLength,halfMinBoneLength);
			for(int i=0;i<bone->localEndPoints.size();++i)
			{
				double x = bone->localEndPoints[i].x();
				double y = bone->localEndPoints[i].y();
				double z = bone->localEndPoints[i].z();
				if(x < bbL.x()) bbL.setX(x);
				if(y < bbL.y()) bbL.setY(y);
				if(z < bbL.z()) bbL.setZ(z);
				if(x > bbH.x()) bbH.setX(x);
				if(y > bbH.y()) bbH.setY(y);
				if(z > bbH.z()) bbH.setZ(z);
			}

			gVec3 bb = bbH-bbL;
						
			if(bb.x()>=bb.y() && bb.x()>=bb.z()){
				bb.setY(std::max(bb.y(),defaultLinkAspectRatio*bb.x()));
				bb.setZ(std::max(bb.z(),defaultLinkAspectRatio*bb.x()));
			}else if(bb.y()>=bb.z() && bb.y()>=bb.x()){
				bb.setX(std::max(bb.x(),defaultLinkAspectRatio*bb.y()));
				bb.setZ(std::max(bb.z(),defaultLinkAspectRatio*bb.y()));
			}else if(bb.z()>=bb.x() && bb.z()>=bb.y()){
				bb.setX(std::max(bb.x(),defaultLinkAspectRatio*bb.z()));
				bb.setY(std::max(bb.y(),defaultLinkAspectRatio*bb.z()));
			}

			bbs.push_back(bb);

			volume = bb.x()*bb.y()*bb.z();
			totalVolume += volume;

			//localCoMFrames
			gVec3 com = 0.5*(bbL+bbH);
			gXMat CoMFrame; CoMFrame.setTrn(com);
			localCoMFrames.push_back(CoMFrame);		

		}
	}

	if( bbs.size() != skeleton->bones.size() )
	{
		printf("Warning: mismatch occurs when calculate body size.\n");
	}

	double minMass = 0.001;

	for( int i=0; i<skeleton->bones.size(); i++ )
	{
		bone = skeleton->bones[i];

		gVec3 bb = bbs[i];

		//double linkHeight = std::max(sizeVec.magnitude(),minBoneLength);
		//double linkWidth = defaultLinkAspectRatio*linkHeight;
		volume = bb.x()*bb.y()*bb.z();
		
		mass = totalMass * ( volume / totalVolume );
		if( mass < minMass ) mass = minMass; // ADD SWL

		gVec3 inertia; 
		inertia.set(
			mass/12. * ( bb.y()*bb.y() + bb.z()*bb.z() ),
			mass/12. * ( bb.x()*bb.x() + bb.z()*bb.z() ),
			mass/12. * ( bb.y()*bb.y() + bb.x()*bb.x() )
			);

		gReal I[6];
		I[0] = inertia.x(); I[3]=inertia.y(); I[5]=inertia.z(); I[1]=I[2]=I[4]=0.0f;
		gInertia inCoM(mass,gVec3Zero,I,gInertia::INERTIA_REF_BODY);

		gInertia inBody = inCoM.xform( localCoMFrames[i] );
		
		fprintf(fp, "%s\t%g\t%g %g %g\tWRT_BODY_FRAME %g %g %g %g %g %g\n", bone->name.c_str(), mass, inBody.com(0), inBody.com(1), inBody.com(2),
			inBody.rotInertia(0), inBody.rotInertia(1), inBody.rotInertia(2), inBody.rotInertia(3), inBody.rotInertia(4), inBody.rotInertia(5) );
	}

	fprintf(fp, "END_INERTIAL_PROPERTIES\n\n");
	fprintf(fp, "GEOM_DATA_PATH\n.\\model_data\nGEOM_FILE\nEND_OF_GEOM_FILE\n\n");
	fprintf(fp, "BULLET_GEOMETRY_DEFINITION\n\n");

	for( int i=0; i<skeleton->bones.size(); i++ )
	{
		bone = skeleton->bones[i];		
		gVec3 halfbb = bbs[i]*0.5;
		gQuat q = localCoMFrames[i].rot().inQuat();
		gVec3 com = localCoMFrames[i].trn();
		
		fprintf(fp, "GEOM %s\t ROT_QUAT %g %g %g %g POS %g %g %g BOX %g %g %g\n", bone->name.c_str(), q.e(0), q.e(1), q.e(2), q.e(3), com.e(0), com.e(1), com.e(2),
			halfbb.x(), halfbb.y(), halfbb.z() );
	}

	fprintf(fp, "END_OF_BULLET_GEOMETRY_DEFINITION\n\n");

	fclose(fp);

	return 0;
}

//
//int mgSkeletonToBCharacter::saveToBCharacter(mgSkeleton *skeleton, const char* filename)
//{
//	FILE *fp = fopen(filename, "w");
//	if( !fp )
//	{
//		printf("error: cannot open a file(%s) to write.\n", filename);
//		return -1;
//	}
//
//	fprintf(fp, "HIERARCHY\n\n");
//
//	mgBone* bone;
//	gVec3 pos;
//	gQuat quat;
//	for( int i=0; i<skeleton->bones.size(); i++ )
//	{
//		bone = skeleton->bones[i];
//		pos = bone->H.trn();
//		quat = bone->H.rotInQuat();
//
//		fprintf(fp, "LINK\n");
//		fprintf(fp, "NAME %s\n", bone->name.c_str());
//		
//		// Warning: Assume that the root bone index is zero as like getRootPose
//		if( i == 0 )
//		{
//			fprintf(fp, "REF WORLD\n");
//
//		} else {
//			fprintf(fp, "PARENT %s\n", bone->parent->name.c_str());
//			fprintf(fp, "REF LOCAL\n");
//		}
//
//		fprintf(fp, "POS %g %g %g\n", pos.x(), pos.y(), pos.z());
//		fprintf(fp, "ROT QUAT %g %g %g %g\n", quat.x(), quat.y(), quat.z(), quat.w()); 
//
//		// free
//		if((bone->channel ^ (mgBone::translateChannel | mgBone::rotationChannel)) == 0)
//		{
//			fprintf(fp, "JOINT ACC FREE\n");
//		} else if( (bone->channel ^ (mgBone::rotationChannel)) == 0 ) {
//			fprintf(fp, "JOINT ACC BALL\n");
//			//FIXME: MBS does support only the revol limit.
//		} else if( (bone->channel ^ (mgBone::Xrotation)) == 0 ) {
//			fprintf(fp, "JOINT ACC REVOLUTE 1 0 0 RANGE %g %g\n", bone->limit[0].first, bone->limit[0].second);
//		} else if( (bone->channel ^ (mgBone::Yrotation)) == 0 ) {
//			fprintf(fp, "JOINT ACC REVOLUTE 0 1 0 RANGE %g %g\n", bone->limit[0].first, bone->limit[0].second);
//		} else if( (bone->channel ^ (mgBone::Zrotation)) == 0 ) {
//			fprintf(fp, "JOINT ACC REVOLUTE 0 0 1 RANGE %g %g\n", bone->limit[0].first, bone->limit[0].second);
//		} else if( (bone->channel ^ (mgBone::Xrotation | mgBone::Zrotation)) == 0 ) {
//			//fprintf(fp, "JOINT ACC BALL\n");
//			fprintf(fp, "JOINT ACC UNIVERSAL X Z\n");
//		} else if( (bone->channel ^ (mgBone::Xrotation | mgBone::Yrotation)) == 0 ) {
//			//fprintf(fp, "JOINT ACC BALL\n");
//			fprintf(fp, "JOINT ACC UNIVERSAL X Y\n");
//		} else if( (bone->channel ^ (mgBone::Yrotation | mgBone::Zrotation)) == 0 ) {
//			//fprintf(fp, "JOINT ACC BALL\n");
//			fprintf(fp, "JOINT ACC UNIVERSAL Y Z\n");
//		} else {
//			// WELD
//			//fprintf(fp, "JOINT ACC WELDED\n");
//			fprintf(fp, "JOINT ACC BALL\n");
//			//printf("Warning: un supported joint type. channel: %u\n", bone->channel);
//		}
//		
//		fprintf(fp, "END_LINK\n\n");
//
//	}
//
//	fprintf(fp, "END_HIERARCHY\n\nINERTIAL_PROPERTIES\n\n");
//
//	mgBone *child;
//	gVec3 AA, BB, posVec, sizeVec, com;
//	std::vector<gVec3> sizeVecs;
//	std::vector<gVec3> COMs;
//	const double minSize = 1.0;
//	const double minVol = 0.1; //1g
//	double totalVolume = 0;
//	double totalMass = 65.;
//	double mass, volume;
//
//	for( int i=0; i<skeleton->bones.size(); i++ )
//	{
//		bone = skeleton->bones[i];
//
//		sizeVec = bone->localEndPoint;
//		COMs.push_back( sizeVec * 0.5 );
//		
//		sizeVec.setX(
//				(abs(sizeVec.x()) > minSize ) ? abs(sizeVec.x()) : minSize
//			);
//		sizeVec.setY(
//				(abs(sizeVec.y()) > minSize ) ? abs(sizeVec.y()) : minSize
//			);
//		sizeVec.setZ(
//				(abs(sizeVec.z()) > minSize ) ? abs(sizeVec.z()) : minSize
//			);
//
//		sizeVecs.push_back(sizeVec);
//		volume = abs(sizeVec.x() * sizeVec.y() * sizeVec.z());
//		totalVolume += (volume < minVol)?minVol:volume;
//	}
//
//	if( sizeVecs.size() != skeleton->bones.size() )
//	{
//		printf("Warning: mismatch occurs when calculate body size.\n");
//	}
//
//	for( int i=0; i<skeleton->bones.size(); i++ )
//	{
//		bone = skeleton->bones[i];
//
//		sizeVec = sizeVecs[i];
//		com = COMs[i];
//
//		volume = abs(sizeVec.x() * sizeVec.y() * sizeVec.z());
//		volume = (volume < minVol)?minVol:volume;
//
//		mass = totalMass * ( volume / totalVolume );
//
//		gVec3 inertia;
//		inertia.set(
//			mass/12. * ( sizeVec.y()*sizeVec.y() + sizeVec.z() * sizeVec.z()),
//			mass/12. * ( sizeVec.x()*sizeVec.x() + sizeVec.z() * sizeVec.z()),
//			mass/12. * ( sizeVec.x()*sizeVec.x() + sizeVec.y() * sizeVec.y())
//			);
//
//		gReal I[6];
//		I[0] = inertia.x(); I[3]=inertia.y(); I[5]=inertia.z(); I[1]=I[2]=I[4]=0.0f;
//		gInertia in(mass,gVec3Zero,I,gInertia::INERTIA_REF_BODY);
//		gInertia newIn = in.xform( com );
//		newIn.rotInertia(0);
//
//		//fprintf(fp, "%s\t%g\t%g %g %g\tWRT_COM %g %g %g %g %g %g\n", bone->name.c_str(), mass, com.x(), com.y(), com.z(), 
//		//	newIn.rotInertia(0), newIn.rotInertia(1), newIn.rotInertia(2), newIn.rotInertia(3), newIn.rotInertia(4), newIn.rotInertia(5) );//inertia.x(), inertia.y(), inertia.z());
//		fprintf(fp, "%s\t%g\t%g %g %g\tWRT_COM %g 0 0 %g 0 %g\n", bone->name.c_str(), mass, com.x(), com.y(), com.z(), inertia.x(), inertia.y(), inertia.z());
//	}
//
//	fprintf(fp, "END_INERTIAL_PROPERTIES\n\n");
//	fprintf(fp, "GEOM_DATA_PATH\n.\\model_data\nGEOM_FILE\nEND_OF_GEOM_FILE\n\n");
//	fprintf(fp, "BULLET_GEOMETRY_DEFINITION\n\n");
//
//	for( int i=0; i<skeleton->bones.size(); i++ )
//	{
//		bone = skeleton->bones[i];
//		sizeVec = sizeVecs[i] * 0.5;
//		
//		fprintf(fp, "GEOM %s\tBOX %g %g %g\n", bone->name.c_str(), sizeVec.x(), sizeVec.y(), sizeVec.z() );
//	}
//
//	fprintf(fp, "END_OF_BULLET_GEOMETRY_DEFINITION\n\n");
//
//	fclose(fp);
//
//	return 0;
//}