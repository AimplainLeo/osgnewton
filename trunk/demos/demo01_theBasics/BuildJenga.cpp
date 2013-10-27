
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
#include "BuildJenga.h"

using namespace osg;

void BuildJenga (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& location, int high)
{
	dAssert (viewer->getSceneData());
	Group* const rootGroup = viewer->getSceneData()->asGroup();
	dAssert (rootGroup);

	Vec3 blockBoxSize (0.4f, 0.4f * 3.0f, 0.2f);

	// find the floor position
	Vec4 start(Vec4(location, 0.0f) + Vec4 (0.0f, 0.0f, 10.0f, 1.0f));
	Vec4 end (start - Vec4 (0.0f, 0.0f, 20.0f, 1.0f));
	newtonRayCast raycaster(world, DemoExample::m_rayCast); 

	raycaster.CastRay (start, end);
	Vec4 position (raycaster.m_contact + Vec4 (0.0f, 0.0f, blockBoxSize.z() * 0.5f, 1.0f));

	Matrix baseMatrix;
	baseMatrix.setTrans (position.x(), position.y(), position.z());

	// set realistic mass and inertia matrix for each block
	dFloat mass = 5.0f;

	// create a 90 degree rotation matrix
	Matrix rotMatrix (Quat (90.0f * 3.141592f / 180.0f, Vec3 (0.0f, 0.0f, 1.0f)));

	dFloat collisionPenetration = 1.0f / 256.0f;

	// make a box collision shape
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

	for (int i = 0; i < high; i ++) { 
		Matrix matrix(baseMatrix);
		Vec3 step_x (Matrix::transform3x3 (Vec3(1.0f, 0.0f, 0.0f), matrix)); 

		step_x = step_x * blockBoxSize.x();
		matrix.setTrans (matrix.getTrans() - step_x);

		for (int j = 0; j < 3; j ++) { 
			ref_ptr<MatrixTransform> transformNode = new MatrixTransform(matrix);	
			transformNode->addChild(geometryNode.get());
			rootGroup->addChild(transformNode.get());
			new newtonDynamicBody (world, mass, &boxShape, transformNode.get(), matrix);
			matrix.setTrans (matrix.getTrans() + step_x);
		}

		baseMatrix = rotMatrix * baseMatrix;
		Vec3 step_y (Matrix::transform3x3 (Vec3(0.0f, 0.0f, 1.0f), matrix)); 
		step_y = step_y * (blockBoxSize.z() - collisionPenetration);
		baseMatrix.setTrans (baseMatrix.getTrans() + step_y);
	}
}	



