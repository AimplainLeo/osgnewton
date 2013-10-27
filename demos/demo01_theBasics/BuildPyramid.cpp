
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
#include "BuildPyramid.h"


void BuildPyramid (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& location, dFloat mass, int base, int high)
{
	dAssert (viewer->getSceneData());
	Group* const rootGroup = viewer->getSceneData()->asGroup();
	dAssert (rootGroup);

	// please do not ask why, I just like golden ratios dimensions, 
	Vec3 blockBoxSize (1.62f/2.0f, 0.5f, 0.25f);

	// find the floor position
	Vec4 start(Vec4(location, 0.0f) + Vec4 (0.0f, 0.0f, 10.0f, 1.0f));
	Vec4 end (start - Vec4 (0.0f, 0.0f, 20.0f, 1.0f));
	newtonRayCast raycaster(world, DemoExample::m_rayCast); 

	raycaster.CastRay (start, end);
	Vec4 position (raycaster.m_contact + Vec4 (0.0f, 0.0f, blockBoxSize.z() * 0.5f, 1.0f));

	// build the visual mesh out of a collision box,  make a box collision shape
	dNewtonCollisionBox boxShape (world, blockBoxSize.x(), blockBoxSize.y(), blockBoxSize.z(), DemoExample::m_all);

	// create a visual for visual representation
	newtonMesh boxMesh (&boxShape);
	boxMesh.Triangulate();

	// create a texture and apply uv to this mesh
	ref_ptr<Texture2D> texture = new Texture2D;
	ref_ptr<Image> image = osgDB::readImageFile("images\\crate.tga");
	texture->setImage (image.get());
	texture->setWrap(Texture::WRAP_S, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_R, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_T, Texture::REPEAT);

	int materialId = boxMesh.AddMaterial(texture);

	// apply uv to this mesh
	boxMesh.ApplyBoxMapping (materialId, materialId, materialId);

	// create a manual object for rendering 
	ref_ptr<Geode> geometryNode = boxMesh.CreateGeodeNode();

	// get the initial matrix
	Matrix matrix;
	matrix.setTrans (position.x(), position.y(), position.z());

	float z0 = matrix.getTrans().x() + blockBoxSize.z() / 2.0f;
	float x0 = matrix.getTrans().x() - (blockBoxSize.x() + 0.01f) * base / 2;

	dFloat collisionPenetration = 1.0f / 256.0f;

	matrix.setTrans(matrix.getTrans() + Vec3 (0.0f, 0.0f, z0));

	for (int j = 0; j < high; j ++) {
		Vec3 posit (matrix.getTrans());
		posit.x() = x0;
		matrix.setTrans(posit);
		for (int i = 0; i < (base - j) ; i ++) {
			// add a new node tranform and a link to the mesh
			ref_ptr<MatrixTransform> transformNode = new MatrixTransform(matrix);	
			transformNode->addChild(geometryNode.get());
			rootGroup->addChild(transformNode.get());

			// add a new body
			new newtonDynamicBody (world, mass, &boxShape, transformNode.get(), matrix);

			Vec3 posit (matrix.getTrans());
			posit.x() += blockBoxSize.x();
			matrix.setTrans(posit);
		}
		x0 += (blockBoxSize.x() + 0.01f) * 0.5f;

		posit = matrix.getTrans();
		posit.z() += blockBoxSize.z() - collisionPenetration; 
		matrix.setTrans(posit);
	}
}	



