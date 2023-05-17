#ifndef _MBS_LOADER_H_
#define _MBS_LOADER_H_

#include "MBS/gMultibodySystem.h"
#include "Loader/Parser.h"

class MBSLoader :public Parser
{
public:
	MBSLoader() {};
	virtual			~MBSLoader() {};
	//virtual int		 loadModel(const char* filename, gMultibodySystem* system, gVisSystem* visSys=NULL);

	//2012-02-11: scale: scaling the size of the model
	virtual int		 loadModel(const char* filename, gMultibodySystem* system, gReal scale = 1.0);
	virtual int		 loadModelUnity(const char* filename, gMultibodySystem* system, gReal scale);
private:
	gRotMat			 readRot();

	void			 readLink(gMultibodySystem* system);

	// 2012-02-01 : added
	// 2012-02-11 : added scale parameter. See scaleSize() for details.
	//TODO: joint limits of prismatic joint need to be scaled as well.
	int				readFile(const char* fileName, gMultibodySystem* system, gReal scale);
	int				readFileUnity(const char* fileName, gMultibodySystem* system, gReal scale);

	// 2012-02-01 : added.
	int				readHierarchy(gMultibodySystem* system);
	int				readInertia(gMultibodySystem* system);



	// 2014-01-03 : chain info added
	int				readChains(gMultibodySystem* system);

	// 2012-05-13 : read foot and arm index
	//int				readLimbsIdx(bCharacter* system);

	//2012-02-11: scale the size of the multibody system. 	
	void			scaleSize(gMultibodySystem* system, gReal scale);

	//gInertia		readInertia();
	gInertia		readInertiaParam();	// 2012-02-01 : added. it does same function as readInertia
	int				readHierarchy(const char* file, gMultibodySystem* system);
};

#endif // _MBS_LOADER_H_