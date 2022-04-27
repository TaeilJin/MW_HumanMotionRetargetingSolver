//#####################################################################
// Copyright 2010-2015, Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#include "mgPoseTransfer.h"

/**
*this method computes objective function
*/
//double *p,         /* I/O: initial parameter estimates. On output contains the estimated solution */
//double *x,         /* I: measurement vector. NULL implies a zero vector */
//int m,             /* I: parameter vector dimension (i.e. #unknowns) */
//int n,             /* I: measurement vector dimension */
void objFuncLevMar(double *p, double *hx, int m, int n, void *adata)
{		
	mgPoseTransfer* pt = (mgPoseTransfer*)adata;

	assert( n == (pt->desiredPoints.size()*3 + pt->positions.size()*3+pt->directions.size()*3+pt->tar->dof()-6) );

	//p: coordinate vector
	arma::vec lca(p,m,false);
	pt->tar->storeCoord();
	pt->tar->setFromCompactCoordArray(lca); //apply p to tar
	pt->tar->updateKinematicsUptoPos();
	for(int i=0;i<pt->tarPoints.size();++i){
		pt->tarPoints[i].updateKinematicsUptoPos();
	}	

	int cnt = 0;
	for (int i = 0; i < pt->desiredPoints.size(); ++i)
	{
		int idx0 = pt->desiredPoints[i].idx;
		gVec3 posDes = pt->desiredPoints[i].pos_desired;
		gVec3 posTar = pt->tarPoints[idx0].posWorld();
		gVec3 del = posDes - posTar;
		hx[cnt++] = del.x() * pt->desiredPoints[i].weight;
		hx[cnt++] = del.y() * pt->desiredPoints[i].weight;
		hx[cnt++] = del.z() * pt->desiredPoints[i].weight;

	}
	for(int i=0;i<pt->directions.size();++i)
	{
		int idx0 = pt->directions[i].idx0;
		int idx1 = pt->directions[i].idx1;
		gVec3 dirSrc = pt->srcPoints[idx1].posWorld()-pt->srcPoints[idx0].posWorld();
		gVec3 dirTar = pt->tarPoints[idx1].posWorld()-pt->tarPoints[idx0].posWorld();
		dirSrc.normalize();
		dirTar.normalize();
		gVec3 del = dirSrc-dirTar;
		hx[cnt++] = del.x()*pt->directions[i].weight;
		hx[cnt++] = del.y()*pt->directions[i].weight;
		hx[cnt++] = del.z()*pt->directions[i].weight;
	}
	
	for(int i=0;i<pt->positions.size();++i)
	{
		int idx0 = pt->positions[i].idx;
		gVec3 posSrc = pt->srcPoints[idx0].posWorld();
		gVec3 posTar = pt->tarPoints[idx0].posWorld();
		gVec3 del = posSrc-posTar;
		hx[cnt++] = del.x()*pt->positions[i].weight;
		hx[cnt++] = del.y()*pt->positions[i].weight;
		hx[cnt++] = del.z()*pt->positions[i].weight;
	}
		
	for(int i=6;i<pt->tar->dof();++i)
	{
		hx[cnt++] = (lca[i]-pt->preferredTarCoords[i])*pt->preferredTarCoordsWeight;
	}

	pt->tar->restoreCoord();
	pt->tar->updateKinematicsUptoPos();

	return;
}
void objFuncLevMarEEPos(double *p, double *hx, int m, int n, void *adata)
{
	mgPoseTransfer* pt = (mgPoseTransfer*)adata;

	assert(n == (pt->tar->dof() - 6 + pt->desiredPoints.size() * 3));

	//p: coordinate vector
	arma::vec lca(p, m, false);
	pt->tar->storeCoord();
	pt->tar->setFromCompactCoordArray(lca); //apply p to tar
	pt->tar->updateKinematicsUptoPos();
	for (int i = 0; i<pt->tarPoints.size(); ++i) {
		pt->tarPoints[i].updateKinematicsUptoPos();
	}

	int cnt = 0;
	for (int i = 0; i<pt->desiredPoints.size(); ++i)
	{
		int idx0 = pt->desiredPoints[i].idx;
		gVec3 posDes = pt->desiredPoints[i].pos_desired;
		gVec3 posTar = pt->tarPoints[idx0].posWorld();
		gVec3 del = posDes - posTar;
		hx[cnt++] = del.x()*pt->desiredPoints[i].weight;
		hx[cnt++] = del.y()*pt->desiredPoints[i].weight;
		hx[cnt++] = del.z()*pt->desiredPoints[i].weight;
	
	}

	for (int i = 6; i<pt->tar->dof(); ++i)
	{
		hx[cnt++] = (lca[i] - pt->preferredTarCoords[i])*pt->preferredTarCoordsWeight;
	}

	pt->tar->restoreCoord();
	pt->tar->updateKinematicsUptoPos();

	return;
}

/**
*this method transfers pose from src to tar
*/
int		mgPoseTransfer::transferPoseLevMar(gXMat& offset)
{
	if(tarPoints.size()!=srcPoints.size()) return -1;
		
	//apply offset to srcPoints
	src->storeCoord();
	src->baseLink()->setLocalFrame( offset*src->baseLink()->localFrame() );
	src->updateKinematicsUptoPos();
	for(int i=0;i<srcPoints.size();++i){
		srcPoints[i].updateKinematicsUptoPos();
	}	

	//update tarPoints
	for(int i=0;i<tarPoints.size();++i) 
		tarPoints[i].updateKinematicsUptoPos();
	
	//desired direction vectors from srcCharacter
	int sz = directions.size()*3 + positions.size()*3 + tar->dof() - 6 + desiredPoints.size()*3;	
	
	//double *p,         /* I/O: initial parameter estimates. On output contains the estimated solution */
	//double *x,         /* I: measurement vector. NULL implies a zero vector */
	//int m,             /* I: parameter vector dimension (i.e. #unknowns) */
	//int n,             /* I: measurement vector dimension */
	//int itmax,         /* I: maximum number of iterations */
	//
	//double *work,      /* I: working memory, allocated internally if NULL. If !=NULL, it is assumed to point to
	//                    * a memory chunk at least LM_DIF_WORKSZ(m, n)*sizeof(double) bytes long
	//                    */
	//double *covar,     /* O: Covariance matrix corresponding to LS solution; Assumed to point to a mxm matrix.
	//                    * Set to NULL if not needed.
	//                    */
	//void *adata)       /* I: pointer to possibly needed additional data, passed uninterpreted to func.
	//                    * Set to NULL if not needed
	//                    */
	int m = tar->dof();
	double* p = new double[m];
	arma::vec lca(p,m,false);
	tar->getCompactCoordArray(lca); //initialize p as current coordinates of tar
	dlevmar_dif(objFuncLevMar, p, NULL, m, sz, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	
	//printf("nIter=%d, reason=%d, e=%g to %g\n",int(LevMarInfo[5]), int(LevMarInfo[6]), LevMarInfo[0],LevMarInfo[1]);

	//apply x (delta of joint coordinates)	
	tar->setFromCompactCoordArray(lca);	
	tar->updateKinematicsUptoPos();
	src->restoreCoord();
	src->updateKinematicsUptoPos();
	for(int i=0;i<srcPoints.size();++i){
		srcPoints[i].updateKinematicsUptoPos();
	}	

	delete[] p;

	return 1;
}

int mgPoseTransfer::addDesiredPair(std::vector<gVec3> desiredPoses)
{
	addDesiredObjective("t0", 1.0, desiredPoses[0]);
	addDesiredObjective("t1", 1.0, desiredPoses[1]);
	addDesiredObjective("t2", 1.0, desiredPoses[2]);
	addDesiredObjective("t3", 1.0, desiredPoses[3]);
	addDesiredObjective("t4", 1.0, desiredPoses[4]);
	addDesiredObjective("t5", 1.0, desiredPoses[5]);

	addDesiredObjective("t6", 1.0, desiredPoses[6]);
	addDesiredObjective("t7", 1.0, desiredPoses[7]);
	addDesiredObjective("t8", 1.0, desiredPoses[8]);
	addDesiredObjective("t9", 1.0, desiredPoses[9]);
	addDesiredObjective("t10", 1.0, desiredPoses[10]);

	addDesiredObjective("t11", 1.0, desiredPoses[11]);
	addDesiredObjective("t12", 1.0, desiredPoses[12]);
	addDesiredObjective("t13", 1.0, desiredPoses[13]);
	addDesiredObjective("t14", 1.0, desiredPoses[14]);
	addDesiredObjective("t15", 1.0, desiredPoses[15]);

	addDesiredObjective("t16", 1.0, desiredPoses[16]);
	addDesiredObjective("t17", 1.0, desiredPoses[17]);
	addDesiredObjective("t18", 1.0, desiredPoses[18]);
	addDesiredObjective("t19", 1.0, desiredPoses[19]);
	addDesiredObjective("t20", 1.0, desiredPoses[20]);
	return 0;
}

int mgPoseTransfer::transferDesiredPoseLevMar(gXMat& offset, std::vector<gVec3> desiredJointPos)
{
	//apply desired positions
	addDesiredPair(desiredJointPos);
	//update tarPoints
	for (int i = 0; i < tarPoints.size(); ++i)
		tarPoints[i].updateKinematicsUptoPos();

	//desired direction vectors from srcCharacter
	int sz = tar->dof() - 6 + desiredPoints.size() * 3;

	//double *p,         /* I/O: initial parameter estimates. On output contains the estimated solution */
	//double *x,         /* I: measurement vector. NULL implies a zero vector */
	//int m,             /* I: parameter vector dimension (i.e. #unknowns) */
	//int n,             /* I: measurement vector dimension */
	//int itmax,         /* I: maximum number of iterations */
	//
	//double *work,      /* I: working memory, allocated internally if NULL. If !=NULL, it is assumed to point to
	//                    * a memory chunk at least LM_DIF_WORKSZ(m, n)*sizeof(double) bytes long
	//                    */
	//double *covar,     /* O: Covariance matrix corresponding to LS solution; Assumed to point to a mxm matrix.
	//                    * Set to NULL if not needed.
	//                    */
	//void *adata)       /* I: pointer to possibly needed additional data, passed uninterpreted to func.
	//                    * Set to NULL if not needed
	//                    */
	int m = tar->dof();
	double* p = new double[m];
	arma::vec lca(p, m, false);
	tar->getCompactCoordArray(lca); //initialize p as current coordinates of tar
	dlevmar_dif(objFuncLevMarEEPos, p, NULL, m, sz, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);

	//printf("nIter=%d, reason=%d, e=%g to %g\n",int(LevMarInfo[5]), int(LevMarInfo[6]), LevMarInfo[0],LevMarInfo[1]);

	//apply x (delta of joint coordinates)	
	tar->setFromCompactCoordArray(lca);
	tar->updateKinematicsUptoPos();
	desiredPoints.clear();
	/*src->restoreCoord();
	src->updateKinematicsUptoPos();
	for (int i = 0; i < srcPoints.size(); ++i) {
		srcPoints[i].updateKinematicsUptoPos();
	}*/

	delete[] p;

	return 1;
}

/**
add a direction objective for pose transfer
direction is defined by a vector from pointName0 to pointName1
weight: importance of this objective
*/
bool mgPoseTransfer::addDirectionObjective(const char* pointName0, const char* pointName1, double weight)
{
	int pointIndex0 = -1;
	int pointIndex1 = -1;
		
	for(int i=0;i<namePoints.size();++i)
	{
		if( namePoints[i].compare(pointName0) == 0 ){ 
			pointIndex0 = i;			
			break;
		}
	}

	for(int i=0;i<namePoints.size();++i)
	{
		if( namePoints[i].compare(pointName1) == 0 ){ 
			pointIndex1 = i;			
			break;
		}
	}
		
	if( pointIndex0==-1 ){
		printf("error: cannot find point %s\n",pointName0);
		return false;
	}
	if( pointIndex1==-1 ){
		printf("error: cannot find point %s\n",pointName1);
		return false;
	}

	idxDirTransfer* dir = new idxDirTransfer();
	dir->idx0 = pointIndex0;
	dir->idx1 = pointIndex1;
	dir->weight = weight;
	directions.push_back(*dir);

	return true;
}

/**
add a position objective for pose transfer
this tries to match the absolute position of the point of target character to that of source character.
weight: importance of this objective
*/
bool mgPoseTransfer::addPositionObjective(const char* pointName, double weight)
{
	int pointIndex = -1;
	
	for(int i=0;i<namePoints.size();++i)
	{
		if( namePoints[i].compare(pointName) == 0 ) pointIndex = i;
	}

	if( pointIndex == -1 ){ 
		printf("error: cannot find point %s\n",pointName);
		return false;
	}
		
	idxPosTransfer* pos = new idxPosTransfer();
	pos->idx = pointIndex;
	pos->weight = weight;
	pos->scale = NULL;
	positions.push_back(*pos);

	return true;
}
bool mgPoseTransfer::addPositionObjective(const char* pointName, double weight, double scale)
{
	int pointIndex = -1;

	for (int i = 0; i<namePoints.size(); ++i)
	{
		if (namePoints[i].compare(pointName) == 0) pointIndex = i;
	}

	if (pointIndex == -1) {
		printf("error: cannot find point %s\n", pointName);
		return false;
	}

	idxPosTransfer* pos = new idxPosTransfer();
	pos->idx = pointIndex;
	pos->weight = weight;
	pos->scale = scale;
	positions.push_back(*pos);

	return true;
}
bool mgPoseTransfer::addDesiredObjective(const char* pointName, double weight, gVec3 desiredpos)
{
	int pointIndex = -1;

	for (int i = 0; i<namePoints.size(); ++i)
	{
		if (namePoints[i].compare(pointName) == 0) pointIndex = i;
	}

	if (pointIndex == -1) {
		printf("error: cannot find point %s\n", pointName);
		return false;
	}

	idxDesired* pos = new idxDesired();
	pos->idx = pointIndex;
	pos->weight = weight;
	pos->pos_desired = desiredpos;
	desiredPoints.push_back(*pos);

	return true;
}