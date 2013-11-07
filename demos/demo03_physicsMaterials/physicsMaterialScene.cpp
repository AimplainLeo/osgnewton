
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


static void MakeStaticRamp (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& location, int rampMaterialID)
{
	dAssert (viewer->getSceneData());
	Group* const rootGroup = viewer->getSceneData()->asGroup();
	dAssert (rootGroup);

	// create a texture and apply uv to this mesh
	ref_ptr<Texture2D> texture = new Texture2D;
	ref_ptr<Image> image = osgDB::readImageFile("images\\wood_2.tga");
	texture->setImage (image.get());
	texture->setWrap(Texture::WRAP_S, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_R, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_T, Texture::REPEAT);

	Vec3 blockBoxSize (20.0f, 40.0f, 0.25f);
	dNewtonCollisionBox shape (world, blockBoxSize.x(), blockBoxSize.y(), blockBoxSize.z(), DemoExample::m_all);
	

	// create a visual for visual representation
	newtonMesh boxMesh (&shape);
	boxMesh.Triangulate();
	int materialId = boxMesh.AddMaterial(texture);

	// apply uv to this mesh
	boxMesh.ApplyBoxMapping (materialId, materialId, materialId);

	// create a manual object for rendering 
	ref_ptr<Geode> geometryNode = boxMesh.CreateGeodeNode();

	// make a osg transform node
	Matrix matrix (Quat (30.0f * 3.141592f / 180.0f, Vec3 (0.0f, 1.0f, 0.0f)));
	matrix.setTrans (location + Vec3 (0.0f, 20.0f, 0.0f));
	ref_ptr<MatrixTransform> transformNode = new MatrixTransform(matrix);	
	rootGroup->addChild(transformNode.get());

	// attach geometry to transform node 
	transformNode->addChild(geometryNode.get());

	// make a dynamic body
	shape.SetMaterialId(rampMaterialID);
	new newtonDynamicBody (world, 0.0f, &shape, transformNode.get(), matrix);
}


static void AddFrictionBodies (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& location, int frictionMaterialIDStart)
{
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

	Vec3 blockBoxSize (0.75f, 0.5f, 0.25f);
	dNewtonCollisionBox shape (world, blockBoxSize.x(), blockBoxSize.y(), blockBoxSize.z(), DemoExample::m_all);

	// create a visual for visual representation
	newtonMesh boxMesh (&shape);
	boxMesh.Triangulate();
	int materialId = boxMesh.AddMaterial(texture);

	// apply uv to this mesh
	boxMesh.ApplyBoxMapping (materialId, materialId, materialId);

	// create a manual object for rendering 
	ref_ptr<Geode> geometryNode = boxMesh.CreateGeodeNode();
	
	Matrix matrix (Quat (30.0f * 3.141592f / 180.0f, Vec3 (0.0f, 1.0f, 0.0f)));
	Vec3 origin (location.x() - 6.0f, location.y() + 4.0f, location.z() + 4.0f);

	float mass = 10.0f;
	for (int i = 0; i < 10; i ++) {
		// make a osg transform node
		matrix.setTrans (origin);
		ref_ptr<MatrixTransform> transformNode = new MatrixTransform(matrix);	
		rootGroup->addChild(transformNode.get());

		origin.y() += 3.0f;

		// attach geometry to transform node 
		transformNode->addChild(geometryNode.get());

		// make a dynamic body
		shape.SetMaterialId(frictionMaterialIDStart + i);
		newtonDynamicBody* const body = new newtonDynamicBody (world, mass, &shape, transformNode.get(), matrix);

		// set the linear and angular drag do zero
		body->SetLinearDrag (0.0f);
		body->SetAngularDrag(Vec4 (0.0f, 0.0f, 0.0f, 0.0f));
	}
}


static void AddRestitutionBodies (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& location, int restitutionMaterialIDStart)
{
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

	dNewtonCollisionSphere shape (world, 0.5f, DemoExample::m_all);

	// create a visual for visual representation
	newtonMesh boxMesh (&shape);
	boxMesh.Triangulate();
	int materialId = boxMesh.AddMaterial(texture);

	// apply uv to this mesh
	boxMesh.ApplySphericalMapping(materialId);

	// create a manual object for rendering 
	ref_ptr<Geode> geometryNode = boxMesh.CreateGeodeNode();

	Matrix matrix;
	Vec3 origin (location.x() + 10.0f, location.y() + 20.0f, location.z() + 10.0f);

	float mass = 10.0f;
	for (int i = 0; i < 10; i ++) {
		// make a osg transform node
		matrix.setTrans (origin);
		ref_ptr<MatrixTransform> transformNode = new MatrixTransform(matrix);	
		rootGroup->addChild(transformNode.get());

		origin.x() += 2.0f;

		// attach geometry to transform node 
		transformNode->addChild(geometryNode.get());

		// make a dynamic body
		shape.SetMaterialId(restitutionMaterialIDStart + i);
		newtonDynamicBody* const body = new newtonDynamicBody (world, mass, &shape, transformNode.get(), matrix);

		// set the linear and angular drag do zero
		body->SetLinearDrag (0.0f);
		body->SetAngularDrag(Vec4 (0.0f, 0.0f, 0.0f, 0.0f));
	}
}


void PhysicsMaterialScene (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, int count, const Vec3& origin)
{
	int rampMaterialId = 0;
	int frictionMaterialIDStart = rampMaterialId + 1;
	int restitutionMaterialIDStart = frictionMaterialIDStart + 10;

	// create 10 diffErent friCtion material 
	dFloat friction = 0.0f;
	for (int i = 0; i < 10; i ++) {
		dMaterialPairManager::dMaterialPair materialInterAction;
		materialInterAction.m_staticFriction0 = friction;
		materialInterAction.m_staticFriction1 = friction;
		materialInterAction.m_kineticFriction0 = friction;
		materialInterAction.m_kineticFriction1 = friction;
		world->AddMaterialPair (frictionMaterialIDStart + i, rampMaterialId, materialInterAction);
		friction += 0.065f;
	}

	// create 10 restitution materials
	dFloat restitution = 0.1f;
	for (int i = 0; i < 10; i ++) {
		dMaterialPairManager::dMaterialPair materialInterAction;
		materialInterAction.m_restitution = restitution;
		world->AddMaterialPair (restitutionMaterialIDStart + i, rampMaterialId, materialInterAction);
		restitution += 0.1f;
	}

	MakeStaticRamp (viewer, world, origin, rampMaterialId);
	AddFrictionBodies (viewer, world, origin, frictionMaterialIDStart);
	AddRestitutionBodies (viewer, world, origin, restitutionMaterialIDStart);
}	



