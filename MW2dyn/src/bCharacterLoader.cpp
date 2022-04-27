//#####################################################################
// Copyright 2010-2015, Hynchul Choi, Sukwon Lee, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

//#include "Character/bHeader.h"
#include "Character/bCharacterLoader.h"
#include "mbs/gArmaUtil.h"



gRotMat bCharacterLoader::readRot()
{
	gRotMat R;
	char buf[64];
	char buf1[64];
	gReal r[9];
	read_word(buf);

	if(!strncmp(buf,"X",1) || !strncmp(buf,"Y",1) || !strncmp(buf,"Z",1) )
	{
		read_word(buf1);  r[0] = gDTR*atof(buf1);
		read_word(buf1);  r[1] = gDTR*atof(buf1);
		read_word(buf1);  r[2] = gDTR*atof(buf1);

		for(int i=0;i<3;++i)
		{
			if(!strncmp(buf+i,"X",1))
			{
				gRotMat Rx; Rx.makeRotateX(r[i]); R*=Rx;
			}
			else if(!strncmp(buf+i,"Y",1))
			{
				gRotMat Ry; Ry.makeRotateY(r[i]); R*=Ry;
			}
			else if(!strncmp(buf+i,"Z",1))
			{
				gRotMat Rz; Rz.makeRotateZ(r[i]); R*=Rz;
			}
			else
			{
				assert(0);
			}
		}	
	}	
	else if(!strcmp(buf,"QUAT"))
	{
		read_word(buf);	r[0] = atof(buf);	//x
		read_word(buf);	r[1] = atof(buf);	//y
		read_word(buf);	r[2] = atof(buf);	//z
		read_word(buf); r[3] = atof(buf);	//w
		gQuat quat(r[0],r[1],r[2],r[3]);
		quat.normalize();
		R = quat.inRotMatrix();
	}
	else //ROT
	{		
		for(int i=0;i<9;++i){ read_word(buf); r[i]=atof(buf); }
		R.set(r);
		R.rectify(0); //2012-02-16: added to ensure SO(3) structure of R
	}


	//if(!strcmp(buf,"XYZ"))
	//{
	//	read_word(buf);  r[0] = gDTR*atof(buf);//rotx
	//	read_word(buf);  r[1] = gDTR*atof(buf);//roty
	//	read_word(buf);  r[2] = gDTR*atof(buf);//rotz
	//	R.makeRotateXYZ(r[0],r[1],r[2]);		
	//}
	//else if(!strcmp(buf,"ZYX"))
	//{
	//	read_word(buf);  r[0] = gDTR*atof(buf);//rotz
	//	read_word(buf);  r[1] = gDTR*atof(buf);//roty
	//	read_word(buf);  r[2] = gDTR*atof(buf);//rotx
	//	R.makeRotateZYX(r[0],r[1],r[2]);
	//}
	//else if(!strcmp(buf,"ZXY"))
	//{
	//	read_word(buf);  r[0] = gDTR*atof(buf);//rotz
	//	read_word(buf);  r[1] = gDTR*atof(buf);//rotx
	//	read_word(buf);  r[2] = gDTR*atof(buf);//roty
	//	R.makeRotateZXY(r[0],r[1],r[2]);
	//}
	//else if(!strcmp(buf,"QUAT"))
	//{
	//	read_word(buf);	r[0] = atof(buf);	//x
	//	read_word(buf);	r[1] = atof(buf);	//y
	//	read_word(buf);	r[2] = atof(buf);	//z
	//	read_word(buf); r[3] = atof(buf);	//w
	//	gQuat quat(r[0],r[1],r[2],r[3]);
	//	quat.normalize();
	//	R = quat.inRotMatrix();
	//}
	//else //ROT
	//{		
	//	for(int i=0;i<9;++i){ read_word(buf); r[i]=atof(buf); }
	//	R.set(r);
	//	R.rectify(0); //2012-02-16: added to ensure SO(3) structure of R
	//}
	return R;
}

gInertia bCharacterLoader::readInertia()
{		
	gReal mass;
	gReal I[6];
	gReal c[3];
	char buf[128];

	read_word(buf); mass  = atof(buf);

	read_word(buf); c[0] = atof(buf); 
	read_word(buf); c[1] = atof(buf);
	read_word(buf); c[2] = atof(buf);

	read_word(buf);
	int refFrame;
	if(!strcmp(buf,"WRT_COM"))
	{
		refFrame = 1;
	}
	else if(!strcmp(buf,"WRT_BODY_FRAME"))
	{
		refFrame = 0;
	}
	else
	{
		exit(-1); //error
	}

	read_word(buf); I[0] = atof(buf);
	read_word(buf); I[1] = atof(buf);
	read_word(buf); I[2] = atof(buf);
	read_word(buf); I[3] = atof(buf);
	read_word(buf); I[4] = atof(buf);
	read_word(buf); I[5] = atof(buf);

	return gInertia(mass,c,I,refFrame);
}

gInertia bCharacterLoader::readInertiaParam()
{
	gReal mass;
	gReal I[6];
	gReal c[3];
	char buf[128];

	read_word(buf); mass  = atof(buf);

	read_word(buf); c[0] = atof(buf); 
	read_word(buf); c[1] = atof(buf);
	read_word(buf); c[2] = atof(buf);

	read_word(buf);
	int refFrame;
	if(!strcmp(buf,"WRT_COM"))
	{
		refFrame = 1;
	}
	else if(!strcmp(buf,"WRT_BODY_FRAME"))
	{
		refFrame = 0;
	}
	else
	{
		exit(-1); //error
	}

	read_word(buf); I[0] = atof(buf);
	read_word(buf); I[1] = atof(buf);
	read_word(buf); I[2] = atof(buf);
	read_word(buf); I[3] = atof(buf);
	read_word(buf); I[4] = atof(buf);
	read_word(buf); I[5] = atof(buf);

	return gInertia(mass,c,I,refFrame);
}

int bCharacterLoader::readFile(const char* fileName, bCharacter* system, gReal scale)
{
	if(open_file(fileName)==0) return 0;
	
	// we can open the file individually to remove a sequential issue, but it's not an efficient way neither.
	// so instead, we put the file pointer to a starting point in every read function..
	readHierarchy(system);	
	readInertia(system);

	//scale the size
	scaleSize(system, scale);

	readGeometry(system, scale);
	//readFootGeometry(system, scale); //2015.4.28. Deprecated - shl
	readChains(system);

	close_file();
	
	return 1;
}

void bCharacterLoader::scaleSize(bCharacter* system, gReal scale)
{

	//cannot use for(int i=0;i<system->numLinks();++i) because system::_nLinks not initialized yet.
	for(unsigned int i=0;i<system->links().size();++i)
	{
		gLink* link = system->link(i);
		
		//scale length
		link->localFrameDefault().setTrn( scale*link->localFrameDefault().trn() ); //scale _H
		link->localFrame().setTrn( scale*link->localFrame().trn() ); //scale _G (this is necessary only for the baselink)

		//scale CoM
		gVec3 com = scale*link->inertia().comInVec3();
		link->inertia().setCOM(com.x(),com.y(),com.z());

		//don't need to scale mass, but must scale rotational inertia		
		gReal in[6];
		gReal scaleSqr = scale*scale;
		in[0] = scaleSqr*link->inertia().rotInertia(0);
		in[1] = scaleSqr*link->inertia().rotInertia(1);
		in[2] = scaleSqr*link->inertia().rotInertia(2);
		in[3] = scaleSqr*link->inertia().rotInertia(3);
		in[4] = scaleSqr*link->inertia().rotInertia(4);
		in[5] = scaleSqr*link->inertia().rotInertia(5);
		link->inertia().setRotInertia(in);
	}
}

int bCharacterLoader::readHierarchy(bCharacter* system)
{
	// set file pointer to starting point
	fseek(_file, 0, SEEK_SET);

	char buf[128];
	go_after("HIERARCHY");
	while(1)
	{
		read_word(buf);
		if(!strcmp(buf, "END_HIERARCHY")) break;
		else if(!strcmp(buf,"LINK")) readLink(system); //read link
	}

	system->initializeHierarchy();

	return 1;
}

int bCharacterLoader::readInertia(bCharacter* system)
{
	// set file pointer to starting point
	fseek(_file, 0, SEEK_SET);

	char buf[128];
	
	go_after("INERTIAL_PROPERTIES");
	while(1)
	{
		gRigidBody* body;
		read_word(buf);
		if(!strcmp(buf, "END_INERTIAL_PROPERTIES")) break;
		else
		{
			body = system->findLink(buf);
			gInertia inertia = readInertiaParam();
			if(body)
			{
				body->setInertia(inertia);
				if( !check_positive_definiteness(body->inertia()) )
				{
					std::cout <<"ERROR: " << body->name() <<"'s gInertia is not positive definite!" <<std::endl;
				}
			}
		}
	}

	return 1;
}

int bCharacterLoader::readGeometry(bCharacter* system, gReal scale)
{
	// set file pointer to starting point
	fseek(_file, 0, SEEK_SET);

	char buf[128];
	gVec3 shapeInfo;
	gLink* link;

	btCollisionShape* shape;
	btVector3 length;

	bShapeSet* shapeSet = system->getOrCreateVisShapeSet();

	if(go_after("GEOM_DATA_PATH") == -1) 
	{
		printf("Cannot find GEOM_DATA_PATH in model file.\n");
		return -1;
	}

	read_word(buf);
	std::string model_path(buf);

	if(go_after("GEOM_FILE") == -1) 
	{
		printf("Cannot find GEOM_FILE in model file.\n");
		return -1;
	}
	while(1) {
		read_word(buf);
		if(!strcmp(buf,"END_OF_GEOM_FILE")) {
			break;
		}
		else if(!strcmp(buf, "OBJ")) {
			read_word(buf);
			link = system->findLink(buf);

			assert( link );
			read_word(buf);

			std::string path = buf;
			path = model_path + path;

			shapeSet->setVisShape(link->name(), path);
		}
		else{
			assert(0);
		}
	}
	

	if(go_after("BULLET_GEOMETRY_DEFINITION") == -1)
	{
		printf("Cannot find BULLET_GEOMETRY_DEFINITION in model file.\n");
		return -1;
	}
	while(1) {
		read_word(buf);
		if(!strcmp(buf,"END_OF_BULLET_GEOMETRY_DEFINITION")) {
			break;
		}
		else if(!strcmp(buf, "GEOM")) {
			
			read_word(buf);
			link = system->findLink(buf);

			
			//SHL 2014-05-29
			if(go_after("ROT_QUAT") == -1) 
			{
				printf("Cannot find GEOM: ROT_QUAT in model file.\n");
				return -1;
			}

			gQuat q;
			q.setX(readReal(buf)); q.setY(readReal(buf)); q.setZ(readReal(buf)); q.setW(readReal(buf));

			if(go_after("POS") == -1) 
			{
				printf("Cannot find GEOM: POS in model file.\n");
				return -1;
			}

			gVec3 p; p.setX(readReal(buf)); p.setY(readReal(buf)); p.setZ(readReal(buf));
			
			gXMat shapeXform(q.inRotMatrix(),p * scale);

			read_word(buf);

			if(!strcmp(buf, "BOX")) {
				gReal x = readReal(buf);
				gReal y = readReal(buf);
				gReal z = readReal(buf);

				length.setValue(x, y, z);
				shape = new btBoxShape(length * scale);
			}
			else if(!strcmp(buf, "CAPSULEX")) {
				gReal x = readReal(buf);
				gReal y = readReal(buf);
				
				shape = new btCapsuleShapeX(x * scale, y * scale);
			}
			else if(!strcmp(buf, "CAPSULEY")) {
				gReal x = readReal(buf);
				gReal y = readReal(buf);

				shape = new btCapsuleShape(x * scale, y * scale);
			}
			else if(!strcmp(buf, "CAPSULEZ")) {
				gReal x = readReal(buf);
				gReal y = readReal(buf);

				shape = new btCapsuleShapeZ(x * scale, y * scale);
			}
			else if(!strcmp(buf, "SPHERE")) {
				gReal x = readReal(buf);

				shape = new btSphereShape(x * scale);
			}

			if(link){
				shapeSet->setShape(link->name(), shape);
				shapeSet->setShapeXform(link->name(), shapeXform); //SHL 2014-05-29
			}
		}
	}

	return 1;
}

int bCharacterLoader::readHierarchy(const char* fileHiearchy, bCharacter* system)
{
	char buf[128];
	if(open_file(fileHiearchy)==0) return 0;
	
	go_after("HIERARCHY");
	while(1)
	{
		read_word(buf);
		if(!strcmp(buf, "END_HIERARCHY")) break;
		else if(!strcmp(buf,"LINK")) readLink(system); //read link
	}

	go_after("INERTIAL_PROPERTIES");
	while(1)
	{
		gRigidBody* body;
		read_word(buf);
		if(!strcmp(buf, "END_INERTIAL_PROPERTIES")) break;
		else
		{
			body = system->findLink(buf);			
			gInertia inertia = readInertia();
			if(body)
			{
				body->setInertia(inertia);
				if( !check_positive_definiteness(body->inertia()) )
				{
					std::cout <<"ERROR: " << body->name() <<"'s gInertia is not positive definite!" <<std::endl; 
				}
			}
		}
	}

	close_file();
	return 1;
}

void bCharacterLoader::readLink(bCharacter* system)
{
	char buf[128];
	gXMat M;
	char name[128];
	bool refWorld = false;
	gLink* parent = NULL;
	int jointType = 0;
	gVec3 axis[3];
	gTwist sc;
	gReal jointLimitHi[6] = { HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL, HUGE_VAL };
	gReal jointLimitLo[6] = { -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL };
	bool accDriven = false;
	gCoordSpring* spring = NULL;

	arma::vec limitHi(jointLimitHi, 6, false);
	arma::vec limitLo(jointLimitLo, 6, false);
	
	while(1)
	{
		read_word(buf);
		if(!strcmp(buf,"END_LINK"))
		{
			//create a link and set type-specific variables
			gLink* link;
			switch(jointType)
			{
			case TYPE_1D_LINK:
				{
					g1dLink* rlink = new g1dLink();
					rlink->setScrew(sc);
					link = rlink;
				}
				break;
			case TYPE_2D_LINK: ///ONLY FOR UNIVERSAL JOINT
				{
					g2dLink* ulink = new g2dLink();
					ulink->setScrew(0,gTwist(axis[0],gVec3Zero));  
					ulink->setScrew(1,gTwist(axis[1],gVec3Zero));
					link = ulink;
				}
				break;
			case TYPE_BALL_LINK:
				link = new gBallLink();
				break;
			case TYPE_WELDED_LINK:
				link = new gWeldedLink();
				break;
			case TYPE_FREE_LINK:
				link = new gFreeLink();
				break;
			}

			if (link->dof() > 0)
			{
				link->setJointLimitLo(limitLo.subvec(0, link->dof() - 1));
				link->setJointLimitHi(limitHi.subvec(0, link->dof() - 1));
			}

			system->addLink(link);
			if(parent) system->connect(parent,link);
			link->setName(name);
			link->setAccDriven(accDriven);			
			
			if(refWorld)
			{
				link->setFrame(M);
				if(parent) link->setLocalFrameDefault(parent->frame().invMult(M));
				else link->setLocalFrame(M); //this works for gFreeLink/gWeldedLink
			}
			else
			{
				link->setLocalFrameDefault(M);
				if(parent) link->setFrame(parent->frame()*M);
				else link->setLocalFrame(M);
			}

			if(spring!=NULL){
				spring->setLink(link);
				system->addJointSpring(spring);
			}
			
			return;
		}
		else if(!strcmp(buf,"NAME"))
		{
			read_word(buf);
			strcpy(name,buf);
		}
		else if(!strcmp(buf,"PARENT"))
		{
			read_word(buf);
			parent = system->findLink(buf);
		}
		else if(!strcmp(buf,"REF")){
			read_word(buf);
			if(!strcmp(buf,"WORLD")) refWorld = true;
			else refWorld = false;
		}
		else if(!strcmp(buf,"ROT"))
		{
			M.setRot(readRot());
		}
		else if(!strcmp(buf,"POS"))
		{
			M.setTrn(readVec3());
		}
		else if(!strcmp(buf,"JOINT"))
		{
			read_word(buf);

			//ACC/TORQUE driven
			if (!strcmp(buf, "ACC")) {
				accDriven = true;
				read_word(buf);
			}
			else accDriven = false;

			if(!strcmp(buf,"REVOLUTE")){
				jointType = TYPE_1D_LINK;
				sc.set(0,readReal(buf));
				sc.set(1,readReal(buf));
				sc.set(2,readReal(buf));
				sc.set(3,0);
				sc.set(4,0);
				sc.set(5,0);

				//read_word(buf);
				//assert( !strcmp(buf,"RANGE") );
				//jointLimitLo[0] = readReal(buf)*gDTR;
				//jointLimitHi[0] = readReal(buf)*gDTR;
			}
			else if(!strcmp(buf,"PRISMATIC")){
				jointType = TYPE_1D_LINK;
				sc.set(0,0);
				sc.set(1,0);
				sc.set(2,0);
				sc.set(3,readReal(buf));
				sc.set(4,readReal(buf));
				sc.set(5,readReal(buf));
			}
			else if(!strcmp(buf,"GENERAL_1D")){
				jointType = TYPE_1D_LINK;
				sc.set(0,readReal(buf));
				sc.set(1,readReal(buf));
				sc.set(2,readReal(buf));
				sc.set(3,readReal(buf));
				sc.set(4,readReal(buf));
				sc.set(5,readReal(buf));
			}
			else if(!strcmp(buf,"UNIVERSAL")){
				jointType = TYPE_2D_LINK;
				for(int i=0; i<2; i++){
					read_word(buf);
					if(!strcmp(buf,"X")) axis[i] = gVec3UnitX;
					else if(!strcmp(buf,"Y")) axis[i] = gVec3UnitY;
					else if(!strcmp(buf,"Z")) axis[i] = gVec3UnitZ;
					else std::cout <<"error" <<std::endl;
				}
				// add limit
			}
			else if(!strcmp(buf,"BALL")){
				jointType = TYPE_BALL_LINK;
			}
			else if(!strcmp(buf,"WELDED")){
				jointType = TYPE_WELDED_LINK;
			}
			else if(!strcmp(buf,"FREE")) {
				jointType = TYPE_FREE_LINK;
			}
			else{
				printf("UNSUPPORTED JOINT TYPE!\n");
				break;
			}

		}
		else if (!strcmp(buf, "RANGE"))
		{
			memset(buf, 0, 128);

			//read_word(buf);
			read_line(buf);

			int pos = 0;
			std::string str = buf;

			while (1)
			{
				int sPos = str.find_first_of('(', pos);

				if (std::string::npos == sPos)
					break;

				sPos++;

				int coPos = str.find_first_of(':', sPos);
				std::string idxStr = str.substr(sPos, coPos - sPos);
				coPos++;

				int cPos = str.find_first_of(',', coPos);
				std::string lowStr = str.substr(coPos, cPos - coPos);
				cPos++;

				int ePos = str.find_first_of(')', cPos);
				std::string hiStr = str.substr(cPos, ePos - cPos);


				int idx = std::stoi(idxStr);
				double lo = std::stod(lowStr);
				double hi = std::stod(hiStr);

				jointLimitHi[idx] = hi * gDTR;
				jointLimitLo[idx] = lo * gDTR;

				pos = ePos + 1;
			}

			//jointLimitLo[0] = readReal(buf)*gDTR;
			//jointLimitHi[0] = readReal(buf)*gDTR;
		}
		else if(!strcmp(buf,"LINEAR_SPRING"))
		{
			switch(jointType)
			{
			case TYPE_1D_LINK:
				{
					g1dLinkLinearSpring* s = new g1dLinkLinearSpring();
					s->setStiffness(readReal(buf));
					s->setDamping(readReal(buf));
					s->setRestCoord(gZero);
					spring = s;
				}
				break;
			default:
				{
					std::cout <<"not supported!!" <<std::endl;
				}
				break;
			}
		}
		else if(!strcmp(buf,"LIMIT_SPRING"))
		{
			switch(jointType)
			{
			case TYPE_1D_LINK:
				{
					g1dLinkLimitSpring* lig = new g1dLinkLimitSpring();
					lig->setJointLimitPositive(readReal(buf)*gDTR);
					lig->setJointLimitNegative(readReal(buf)*gDTR);
					lig->setJointLimitStiffness(readReal(buf));					
					lig->setDamping(readReal(buf));
					spring = lig;
				}
				break;
			case TYPE_2D_LINK:
				{
					g2dLinkLimitSpring* lig = new g2dLinkLimitSpring();
					for(int i=0; i<2; i++)
					{
						lig->setJointLimitPositive(i,readReal(buf)*gDTR);
						lig->setJointLimitNegative(i,readReal(buf)*gDTR);
						lig->setJointLimitStiffness(i,readReal(buf));
						lig->setDamping(i,readReal(buf));
					}
					spring = lig;
				}
				break;
			case TYPE_BALL_LINK:
				{
					gBallLinkLimitSpring* lig = new gBallLinkLimitSpring();					
					for(int i=0; i<3; i++)
					{
						int axis;
						read_word(buf);
						if(!strcmp(buf,"X")) axis = 0;
						else if(!strcmp(buf,"Y")) axis =1;
						else axis = 2;
						lig->setJointLimitPositive(axis,readReal(buf)*gDTR);
						lig->setJointLimitNegative(axis,readReal(buf)*gDTR);
						lig->setJointLimitStiffness(axis,readReal(buf));
						lig->setDamping(axis,readReal(buf));						
					}
					spring = lig;
				}
				break;
			}
			
		}
	}
}

int	bCharacterLoader::loadModel(const char* filename, bCharacter* system, gReal scale)
{
	//read Hierarchy file
	// 2012-02-01 :	updated - readFile, and inside there are readHierarchy, readInertia, and readGeometry. threes are sequential.
	//if( readHierarchy(filename, system) == 0 ) return 0;
	if( readFile(filename, system, scale) == 0 ) return 0;

	//initialize
	system->initialize();
	system->updateKinematicsUptoPos();

	return 1;
}

int bCharacterLoader::readChains(bCharacter* system)
{
	// set file pointer to starting point
	fseek(_file, 0, SEEK_SET);

	char buf[100];
	bool eof = false;

	// find foot geometry tag
	if(go_after("CHAIN_DEFINITION") == -1)
	{
		eof = true;
	}

	while(!eof)
	{
		if(read_word(buf) == -1) break;
		double* data;
		char chainName[100];
			
		if(!strcmp(buf, "END_OF_CHAIN_DEFINITION")) break;
		else if(!strcmp(buf, "CHAIN"))
		{
			read_word(chainName); //read chain name
			std::vector<int>* indices;
			if(!strcmp(chainName,"TRUNK"))	indices = &system->m_trunkIdx;
			else if(!strcmp(chainName,"LEFT_LEG")) indices = &system->m_lLegIdx;
			else if(!strcmp(chainName,"RIGHT_LEG")) indices = &system->m_rLegIdx;
			else if(!strcmp(chainName,"LEFT_ARM"))indices = &system->m_lArmIdx;
			else if(!strcmp(chainName,"RIGHT_ARM"))indices = &system->m_rArmIdx;
			else if(!strcmp(chainName,"HEAD"))indices = &system->m_headIdx;
			else assert(0);
			
			std::vector<std::string> linkNames; // read link list
			while(1)
			{
				read_word(buf);
				if(!strcmp(buf,"END_OF_CHAIN")) break;
				else linkNames.push_back(buf);
			}

			indices->clear();
			for(int i=0; i<linkNames.size();++i)
			{
				gLink* link = system->findLink(linkNames[i].c_str());
				assert(link);
				int id = link->id();
				assert(id>=0);
				indices->push_back( id );
			}
		}
		else{ assert(0); } // it should not happen

	}

	return 1;
}