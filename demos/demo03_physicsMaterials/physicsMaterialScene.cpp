
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


#include <HelpersStdAfx.h>
#include <Utils.h>
#include <DemoExample.h>
#include "physicsMaterialScene.h"


void PhysicsMaterialScene (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, int count, const Vec3& origin, const dNewtonCollision& shape)
{
	// get the root node
	dAssert (viewer->getSceneData());
	Group* const rootGroup = viewer->getSceneData()->asGroup();
	dAssert (rootGroup);

	// create a texture and apply uv to this mesh
	ref_ptr<Texture2D> texture = new Texture2D;
	ref_ptr<Image> image = osgDB::readImageFile("images\\smilli.tga");
	texture->setImage (image.get());
	texture->setWrap(Texture::WRAP_S, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_R, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_T, Texture::REPEAT);

	// create a visual for visual representation
	newtonMesh boxMesh (&shape);
	boxMesh.Triangulate();
	int materialId = boxMesh.AddMaterial(texture);

	// apply uv to this mesh
	boxMesh.ApplyBoxMapping (materialId, materialId, materialId);

	// create a manual object for rendering 
	ref_ptr<Geode> geometryNode = boxMesh.CreateGeodeNode();

	dFloat mass = 20.0f;
	dFloat scaleStep = (2.0f - 0.5f) / count;
	dFloat scaleY = 0.5f;
	for (int i = 0; i < count; i ++) {
		Matrix matrix;

		// make a ogre node
		matrix.setTrans (origin + Vec3 (0.0f, i * 4.0f, 0.0f));
		ref_ptr<MatrixTransform> transformNode = new MatrixTransform(matrix);	
		transformNode->addChild(geometryNode.get());
		rootGroup->addChild(transformNode.get());

		// make a dynamic body
		newtonDynamicBody* const body = new newtonDynamicBody (world, mass, &shape, transformNode.get(), matrix);

		// apply non uniform scale to both 
		dNewtonCollision* const collision = body->GetCollision();
//		node->setScale(1.0f, scaleY, 1.0f);
//		collision->SetScale(1.0f, scaleY, 1.0f);

		scaleY += scaleStep;
	}
}	



