/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/


#include "osgNewtonStdAfx.h"
#include "osgNewtonMesh.h"
#include "osgNewtonWorld.h"
#include "osgNewtonSceneBody.h"

namespace osg
{

newtonSceneBody::newtonSceneBody (newtonWorld* const ogreWorld, dLong collisionMask)
	:dNewtonDynamicBody (NULL)
{
	dMatrix matrix(GetIdentityMatrix());
	dNewtonCollisionScene collision (ogreWorld, collisionMask);
	SetBody (NewtonCreateDynamicBody (ogreWorld->GetNewton (), collision.GetShape(), &matrix[0][0]));
}

newtonSceneBody::~newtonSceneBody()
{
}

void newtonSceneBody::BeginAddRemoveCollision()
{
	dNewtonCollisionScene* const scene = (dNewtonCollisionScene*) GetCollision();
	scene->BeginAddRemoveCollision();
}

void* newtonSceneBody::AddCollision(const dNewtonCollision* const collision)
{
	dNewtonCollisionScene* const scene = (dNewtonCollisionScene*) GetCollision();
	return scene->AddCollision(collision);
}

void newtonSceneBody::RemoveCollision (void* const handle)
{
	dNewtonCollisionScene* const scene = (dNewtonCollisionScene*) GetCollision();
	scene->RemoveCollision(handle);
}

void newtonSceneBody::EndAddRemoveCollision()
{
	dNewtonCollisionScene* const scene = (dNewtonCollisionScene*) GetCollision();
	scene->EndAddRemoveCollision();

	// need to update the aabb in the broad phase, for this we call set matrix
	dMatrix matrix;
	NewtonBody* const body = GetNewtonBody();
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	NewtonBodySetMatrix(body, &matrix[0][0]);
}


void* newtonSceneBody::AddCollisionTree (osg::Node* const treeNode)
{
	newtonWorld* const world = (newtonWorld*) GetNewton();

	// convert the nod and all its children to a newton mesh
	newtonMesh mesh (world, treeNode);

	// convert the mesh to clean convex faces
	mesh.Polygonize();

	// create a collision tree mesh
	dNewtonCollisionMesh meshCollision (world, mesh, 0);

	// add this collision to the scene body
	return AddCollision (&meshCollision);
}


/*
void* newtonSceneBody::AddTerrain (Terrain* const terrain)
{
	newtonWorld* const world = (newtonWorld*) GetNewton();

	int width = terrain->getSize() - 1;
	int height = terrain->getSize() - 1;
	int size = width * height;
	Real horizontalScale = (terrain->getWorldSize() / (terrain->getSize() - 1));

	dNewtonScopeBuffer<dFloat> elevations(size);
	dNewtonScopeBuffer<char> attributes(size);
	
	for (int i = 0; i < width; i++) {	
		int index = i * height;
		for (int k = 0; k < height; k++) {
			// for now make collsionID zero, until we can get material information from the terrain tile
			attributes[index] = 0;
			elevations[index] = terrain->getHeightAtPoint(i, k);
			index ++;
		}
	}

	// build the height field collision
	dNewtonCollisionHeightField terrainCollision (world, width, height, 5, 0, 1.0f, horizontalScale, &elevations[0], &attributes[0], 0);

	// set the offset matrix for this collision shape
	Vector3 posit (-(width / 2.0f) * horizontalScale, 0.0f, (height / 2.0f) * horizontalScale);
	Quaternion rot(Ogre::Degree(90.0f), Ogre::Vector3(0.0f, 1.0f, 0.0f));

	Matrix4 matrix;
	matrix.makeTransform (posit, Vector3(1.0f, 1.0f, 1.0f), rot);
	matrix = matrix.transpose();

	terrainCollision.SetMatrix (&matrix[0][0]);
	return AddCollision (&terrainCollision);
}
*/
};