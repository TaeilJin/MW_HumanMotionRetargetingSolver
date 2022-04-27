#ifndef _BULLET_SKIN_H
#define _BULLET_SKIN_H


// for Skinning
#define BONECOUNT 4

// for Skinning
typedef struct
{
	btVector3 vertex;
	int boneCount;
	btScalar weight[BONECOUNT];
	int boneIndex[BONECOUNT];
	btTransform transformInv0[BONECOUNT];
	
} BoneVertex;


class bSkin
{
protected:
	int m_numMeshTriangle;
	int m_numMeshVertices;	
	float* m_meshVertices;
	BoneVertex* m_boneVertex;

	//btGImpactShapeInterface  *m_triMeshSkinShape;
	//bCharacter				 *m_humanoid;

	bCharacter *_bdmbs;

public:
	bSkin(){};
	~bSkin(){};

	void initSkinMesh(float* vertexMesh, const int numVertex, const int numTriangle, float scale=1.0);

	void setWeightBoneVertex(string fileIndex, string fileWeight);
	void setTransBoneVertex(btRigidBody** bodies);
	void setTransBoneVertex();
	
	void setAutoWeightBoneVertex();
	void saveIndexWeightBoneToFile();

	void update(void);
	//void updateMeshToGImpact(void);
	void updateMeshVertex(btVector3& v, int i);

	//inline void setCharacter(bCharacter *character) { m_humanoid = character; };
	void setMBS(bCharacter *bdmbs) { _bdmbs = bdmbs; };

	inline BoneVertex* getBoneVertex() { return m_boneVertex; };
	inline int getNumMeshVertices(){ return m_numMeshVertices; };
	inline float* getMeshVertices() { return m_meshVertices; };

};

#endif /* _GSKIN_H */