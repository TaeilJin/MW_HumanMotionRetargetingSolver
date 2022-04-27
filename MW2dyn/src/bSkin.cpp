#include "Character/bHeader.h"

#include <fstream>
#include <sstream>

const int num_bone_inf = 4;

void bSkin::initSkinMesh( float* vertexMesh, const int numVertex, const int numTriangle, float scale )
{
	m_numMeshTriangle = numTriangle;
	m_numMeshVertices = numVertex;	
	m_meshVertices = vertexMesh;
	m_boneVertex = new BoneVertex[ m_numMeshVertices ];

	btTransform trans;
	trans.setIdentity();
	trans.setOrigin( btVector3( 0, -2.25 , 0));

	// copy Jeremy Mesh Vertices into local variable
	for(int i = 0, j = 0; j < m_numMeshVertices; ++j, i+=3)
	{
		btVector3 v = scale * btVector3(vertexMesh[i], vertexMesh[i+1] , vertexMesh[i+2] ) ;

		m_boneVertex[j].vertex = trans * v;
	}
}

void getBoneIndex(string fileIndex, int* index)
{
	string line1;
	ifstream myfile1(fileIndex);
	
	int i = 0;
	if (myfile1.is_open())
	{
		while ( myfile1.good() )
		{
			getline(myfile1, line1);
			istringstream inputString( line1 );	

			unsigned int idx1, idx2, idx3, idx4;
			inputString >> idx1 >> idx2 >> idx3 >> idx4;

			index[i] = idx1 - 1;
			index[i+1] = idx2 - 1;
			index[i+2] = idx3 - 1;
			index[i+3] = idx4 - 1;

			i+=4;	  			
		}		
		myfile1.close();
	}
	else printf("Unable to open file"); 
}

void getBoneWeight(string fileWeight, float* weight)
{
	string line2;
	ifstream myfile2(fileWeight);
	double idx1, idx2, idx3, idx4;
	float total;

	//FILE *fp = fopen(fileWeight.c_str(), "r");

	int j = 0;
	if (myfile2.is_open())
	//if( fp )
	{
		while ( myfile2.good() )
		//while( !feof(fp) )
		{
			getline(myfile2, line2);
			istringstream inputString( line2 );			

			//float idx1, idx2, idx3, idx4;
			//
			//fscanf(fp, "%f %f %f %f", &idx1, &idx2, &idx3, &idx4);
			inputString >> idx1 >> idx2 >> idx3 >> idx4;
			total = idx1 + idx2 + idx3 + idx4;

			weight[j] = idx1/total;
			weight[j+1] = idx2/total;
			weight[j+2] = idx3/total;
			weight[j+3] = idx4/total;

			j+=4;	  			
		}
		myfile2.close();
		//fclose(fp);
	}
	else printf("Unable to open file");
}

btScalar getDistanceVertexToBone(btVector3 &P, btVector3 &v1, btVector3 &v2)
{
	btVector3 line = v2 - v1;
	btVector3 w = P - v1;

	btScalar c1 = w.dot(line);
	if(c1 <= 0)
	{
		return P.distance(v1);
	}

	btScalar c2 = line.dot(line);
	if(c2 <= c1)
	{
		return P.distance(v2);
	}

	btScalar b = c1/c2;
	btVector3 Pb = v1 + b*line;

	return P.distance(Pb);
}

btScalar getBoneWeight(btScalar dist)
{
	btScalar d_b_sim = 0.1f; // if dist is closer to 10, equal 1
	btScalar d_c_sim = 1.f; // it dist is far from 50, equal 0
	btScalar d_min_sim = 0;
	btScalar d_max_sim = 1;

	btScalar superscript = -((dist-d_b_sim) * (dist-d_b_sim)) / (2*(d_c_sim*d_c_sim));
	btScalar weight = d_min_sim + (d_max_sim - d_min_sim) * exp( superscript ) ;

	return weight;
}

typedef std::pair<btScalar, int> Dist; //<distance vertex to bone, index of bone>

bool comparator( const Dist& l, const Dist& r)
{ 
	return l.first < r.first; 
}

void bSkin::setAutoWeightBoneVertex()
{	
	// Number of bone influence	

	list< Dist > distVertexToBones; 

	for(int i = 0; i < m_numMeshVertices; ++i)
	{		
		// Check for all vertex
		btVector3 v = m_boneVertex[i].vertex;
		
		const int numOfBone = _bdmbs->getMBS()->numLinks();
				
		for(int indexBone = 0; indexBone  < numOfBone; ++indexBone)
		{
			//Check for all bones
			gLink* bone;
			gLink* boneChild;
			
			bone = _bdmbs->getMBS()->link(indexBone);				

			btVector3 startOfBone(0,0,0);
			btVector3 endOfBone(0,0,0);

			gVec3 b = bone->pos();
			startOfBone = btVector3(b.x(), b.y(), b.z()) ;
			
			if(bone->numChildren() > 0)
			{
				gVec3 a = bone->child(0)->pos();
				endOfBone = btVector3(a.x(), a.y(), a.z()) ;
			}
			else
			{			
				gVec3 a = bone->posCenterOfMassWorld();
				endOfBone = btVector3(a.x(), a.y(), a.z());
			}
							
			btScalar distance = getDistanceVertexToBone(v, startOfBone, endOfBone);
			printf("[%d] %f \n", indexBone, distance);
			
			distVertexToBones.push_back( make_pair(distance, indexBone) );
		}

		printf("\n");

		distVertexToBones.sort(comparator);

		list<Dist>::iterator it;
		it = distVertexToBones.begin();

		// Get 4 sortest bone influence
		for ( int k = 0; k < num_bone_inf; ++k)
		{									
			if(k==0)
			{
				m_boneVertex[i].weight[0] = getBoneWeight(it->first);
				m_boneVertex[i].weight[1] = 1 - m_boneVertex[i].weight[0];

				if(m_boneVertex[i].weight[1] < 0.00001)
				{
					m_boneVertex[i].weight[0] = 1;
					m_boneVertex[i].weight[1] = 0;				
				}
					
				m_boneVertex[i].weight[2] = 0;
				m_boneVertex[i].weight[3] = 0;
			}

			m_boneVertex[i].boneIndex[k] = it->second;

			++it;
		}
		distVertexToBones.clear();
	}	
}

void bSkin::saveIndexWeightBoneToFile()
{
	ofstream weightBoneFile( "weight.bone", ios::out );
	ofstream indexBoneFile( "index.bone", ios::out );

	for(int i = 0; i < m_numMeshVertices; ++i)
	{
		weightBoneFile << m_boneVertex[i].weight[0] << " ";
		weightBoneFile << m_boneVertex[i].weight[1] << " ";
		weightBoneFile << m_boneVertex[i].weight[2] << " ";
		weightBoneFile << m_boneVertex[i].weight[3] << endl;

		indexBoneFile << m_boneVertex[i].boneIndex[0]+1 << " ";
		indexBoneFile << m_boneVertex[i].boneIndex[1]+1 << " ";
		indexBoneFile << m_boneVertex[i].boneIndex[2]+1 << " ";
		indexBoneFile << m_boneVertex[i].boneIndex[3]+1 << endl;
	}
}

void bSkin::setWeightBoneVertex(string fileIndex, string fileWeight)
{
	int j = 0;

	int* index = new int[ m_numMeshVertices*4 ];
	btScalar* weight = new btScalar[ m_numMeshVertices*4 ];

	getBoneIndex( fileIndex, index );
	getBoneWeight( fileWeight, weight );

	for(int i = 0; i < m_numMeshVertices; ++i)
	{
		int w1 = index[j];
		int w2 = index[j+1];
		int w3 = index[j+2];
		int w4 = index[j+3];

		m_boneVertex[i].weight[0] = weight[j];
		m_boneVertex[i].weight[1] = weight[j+1];
		m_boneVertex[i].weight[2] = weight[j+2];
		m_boneVertex[i].weight[3] = weight[j+3];

		m_boneVertex[i].boneIndex[0] = w1;
		m_boneVertex[i].boneIndex[1] = w2;
		m_boneVertex[i].boneIndex[2] = w3;
		m_boneVertex[i].boneIndex[3] = w4;

		j+=4;
	}
}

void bSkin::setTransBoneVertex()
{
	if( m_numMeshVertices > 0 ) 
	{	
		for( int i = 0; i < m_numMeshVertices; ++i )
		{			
			btVector3 v = m_boneVertex[i].vertex;
			btScalar totalWeight = 0;
			m_boneVertex[i].boneCount = 0;

			for( int j = 0; j < 4; ++j )
			{
				int indexBone = m_boneVertex[i].boneIndex[j];
				//btRigidBody* body = (m_humanoid->getBodies())[indexBone];
				btRigidBody* body = _bdmbs->getBody(indexBone);
				btTransform currentTransform = body->getWorldTransform();				

				m_boneVertex[i].transformInv0[j] = currentTransform.inverse();
				m_boneVertex[i].boneCount++;
			}
		}
	}
}

void bSkin::setTransBoneVertex(btRigidBody** bodies)
{
	if( m_numMeshVertices > 0 ) 
	{
		for( int i = 0; i < m_numMeshVertices; ++i )
		{			
			btVector3 v = m_boneVertex[i].vertex;
			btScalar totalWeight = 0;
			m_boneVertex[i].boneCount = 0;

			for( int j = 0; j < 4; ++j )
			{
				int indexBone = m_boneVertex[i].boneIndex[j];
				btRigidBody* body = bodies[indexBone];
				btTransform currentTransform = body->getWorldTransform();				

				m_boneVertex[i].transformInv0[j] = currentTransform.inverse();
				m_boneVertex[i].boneCount++;
			}
		}
	}
}

void bSkin::updateMeshVertex(btVector3& v, int i)
{
	m_meshVertices[i] = v.x();
	m_meshVertices[i+1] = v.y();
	m_meshVertices[i+2] = v.z();	
}
/*
void bSkin::updateMeshToGImpact()
{
	//#### Update the Skin Mesh
	m_triMeshSkinShape->setMargin(0.0f); //#### Set collision margin
	//m_triMeshSkinShape->postUpdate();
	m_triMeshSkinShape->updateBound(); //#### Revit all the meshes
}
*/
void bSkin::update()
{
	btVector3 vertexNew;

	for(int i=0, k=0; i < m_numMeshVertices; ++i, k+=3) 
	{		
		vertexNew.setZero();

		//#### Apply skinning for 4 Bone Influence		
		for(int j=0; j < num_bone_inf ; ++j)
		{
			int indexBone = m_boneVertex[i].boneIndex[j];			

			//btRigidBody* body = (_bdmbs->getBodies())[indexBone];
			btRigidBody* body = _bdmbs->getBody(indexBone);
			
			//btTransform boneTransform = body->getWorldTransform();
			btTransform boneTransform;
			//_bdmbs->getBody(indexBone)->getMotionState()->getWorldTransform(boneTransform);
			body->getMotionState()->getWorldTransform(boneTransform);

			btScalar weight  = m_boneVertex[i].weight[j];

			btTransform boneTransformOld =  m_boneVertex[i].transformInv0[j];
			btVector3 localVertex = boneTransformOld( m_boneVertex[i].vertex );

			vertexNew = vertexNew + (weight * boneTransform(localVertex));
		}		

		updateMeshVertex(vertexNew, k);
	}

	//#### Update the skin mesh in GImpact
	//updateMeshToGImpact();
}
