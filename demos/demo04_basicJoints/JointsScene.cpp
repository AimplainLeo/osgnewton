
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
#include "JointsScene.h"


newtonDynamicBody* CreateBox (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& location, const Vec3& size)
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


	dNewtonCollisionBox shape (world, size.x(), size.y(), size.z(), DemoExample::m_all);

	// create a visual for visual representation
	newtonMesh boxMesh (&shape);
	boxMesh.Triangulate();
	int materialId = boxMesh.AddMaterial(texture);

	// apply uv to this mesh
	boxMesh.ApplyBoxMapping (materialId, materialId, materialId);

	// create a manual object for rendering 
	ref_ptr<Geode> geometryNode = boxMesh.CreateGeodeNode();

	// make a osg transform node
	Matrix matrix;
	matrix.setTrans (location + Vec3 (0.0f, 20.0f, 0.0f));
	ref_ptr<MatrixTransform> transformNode = new MatrixTransform(matrix);	
	rootGroup->addChild(transformNode.get());

	// attach geometry to transform node 
	transformNode->addChild(geometryNode.get());

	// make a dynamic body
	return new newtonDynamicBody (world, 10.0f, &shape, transformNode.get(), matrix);
}


void AddBallAndSockect (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& origin)
{
/*
	Vec3 size (1.0f, 1.0f, 1.0f);
	newtonDynamicBody* const box0 = CreateBox (viewer, world, origin + Vec3 (0.0f, 0.0f, 5.0f), size);
	newtonDynamicBody* const box1 = CreateBox (viewer, world, origin + Vec3 (1.0f, 1.0f, 4.0f), size);

	// connect first box to the world
	Matrix matrix (box0->GetMatrix());
	matrix.setTrans (matrix.getTrans() + Vec3 (-size.x() * 0.5f, -size.y() * 0.5f, size.z() * 0.5f));
	new dNewtonBallAndSocketJoint (&dMatrix (matrix.ptr())[0][0], box0);

	// link the two boxes
	matrix = box1->GetMatrix();
	matrix.setTrans (matrix.getTrans() + Vec3 (-size.x() * 0.5f, -size.y() * 0.5f, size.z() * 0.5f));
	new dNewtonBallAndSocketJoint (&dMatrix (matrix.ptr())[0][0], box0, box1);
*/
}


void AddHinges (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& origin)
{
	Vec3 size (1.0f, 0.125f, 2.0f);
	newtonDynamicBody* const box0 = CreateBox (viewer, world, origin + Vec3 (0.0f, 0.0f, 2.0f), size);
//	newtonDynamicBody* const box1 = CreateBox (viewer, world, origin + Vec3 (1.0f, 0.0f, 2.0f), size);

	Matrix localPin (Quat (90.0f * 3.141592f / 180.0f, Vec3 (0.0f, 1.0f, 0.0f)));
	// connect first box to the world
	Matrix matrix (localPin * box0->GetMatrix());
	matrix.setTrans (matrix.getTrans() + Vec3 (-size.x() * 0.5f, 0.0f, 0.0f));
	dNewtonHingeJoint* const hinge0 = new dNewtonHingeJoint (&dMatrix (matrix.ptr())[0][0], box0);

	// link the two boxes
//	matrix = localPin * box1->GetMatrix();
//	matrix.setTrans (matrix.getTrans() + Vec3 (-size.x() * 0.5f, 0.0f, 0.0f));
//	dNewtonHingeJoint* const hinge1 = new dNewtonHingeJoint (&dMatrix (matrix.ptr())[0][0], box1);
}