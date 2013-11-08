
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

newtonDynamicBody* CreateCylinder (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& location, dFloat radius, dFloat height)
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

    dNewtonCollisionCylinder shape (world, radius, height, DemoExample::m_all);

    // create a visual for visual representation
    newtonMesh boxMesh (&shape);
    boxMesh.Triangulate();
    int materialId = boxMesh.AddMaterial(texture);

    // apply uv to this mesh
    boxMesh.ApplyCylindricalMapping(materialId, materialId);

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


newtonDynamicBody* CreateWheel (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& location, dFloat radius, dFloat height)
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

    dNewtonCollisionChamferedCylinder shape (world, radius, height, DemoExample::m_all);

    // create a visual for visual representation
    newtonMesh boxMesh (&shape);
    boxMesh.Triangulate();
    int materialId = boxMesh.AddMaterial(texture);

    // apply uv to this mesh
    boxMesh.ApplyCylindricalMapping(materialId, materialId);

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

static dNewtonHingeJoint* AddHingeWheel (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& origin, dFloat radius, dFloat height, newtonDynamicBody* const parent)
{
    newtonDynamicBody* const wheel = CreateWheel (viewer, world, origin, height, radius);

    // the joint pin is the first row of the matrix
    Matrix localPin (Quat (0.0f * 3.141592f / 180.0f, Vec3 (0.0f, 1.0f, 0.0f)));

    // connect first box to the world
    Matrix matrix (localPin * wheel->GetMatrix());
    return new dNewtonHingeJoint (&dMatrix (matrix.ptr())[0][0], wheel, parent);
}




void AddBallAndSockect (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& origin)
{
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
}


void AddHinge (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& origin)
{
	Vec3 size (1.5f, 0.125f, 4.0f);
	newtonDynamicBody* const box0 = CreateBox (viewer, world, origin + Vec3 (0.0f, 0.0f, 4.0f), size);
	newtonDynamicBody* const box1 = CreateBox (viewer, world, origin + Vec3 (1.5f, 0.0f, 4.0f), size);
    newtonDynamicBody* const box2 = CreateBox (viewer, world, origin + Vec3 (3.0f, 0.0f, 4.0f), size);

    // the joint pin is the first row of the matrix, to make a upright pin we
    // take the x axis and rotate by 90 degree around the y axis
    Matrix localPin (Quat (90.0f * 3.141592f / 180.0f, Vec3 (0.0f, 1.0f, 0.0f)));
	
	// connect first box to the world
	Matrix matrix (localPin * box0->GetMatrix());
	matrix.setTrans (matrix.getTrans() + Vec3 (-size.x() * 0.5f, 0.0f, 0.0f));
	dNewtonHingeJoint* const hinge0 = new dNewtonHingeJoint (&dMatrix (matrix.ptr())[0][0], box0);
    hinge0->EnableLimits (true);
    hinge0->SetLimis(-45.0f * 3.141592f / 180.0f, 45.0f * 3.141592f / 180.0f);

	// link the two boxes
	matrix = localPin * box1->GetMatrix();
	matrix.setTrans (matrix.getTrans() + Vec3 (-size.x() * 0.5f, 0.0f, 0.0f));
	dNewtonHingeJoint* const hinge1 = new dNewtonHingeJoint (&dMatrix (matrix.ptr())[0][0], box0, box1);
    hinge1->EnableLimits (true);
    hinge1->SetLimis (-45.0f * 3.141592f / 180.0f, 45.0f * 3.141592f / 180.0f);

	// link the two boxes
    matrix = localPin * box2->GetMatrix();
    matrix.setTrans (matrix.getTrans() + Vec3 (-size.x() * 0.5f, 0.0f, 0.0f));
    dNewtonHingeJoint* const hinge2 = new dNewtonHingeJoint (&dMatrix (matrix.ptr())[0][0], box1, box2);
    hinge2->EnableLimits (true);
    hinge2->SetLimis (-45.0f * 3.141592f / 180.0f, 45.0f * 3.141592f / 180.0f);
}


void AddUniversal (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& origin)
{
    newtonDynamicBody* const wheel = CreateWheel (viewer, world, origin + Vec3 (0.0f, 0.0f, 4.0f), 1.0f, 0.5f);

    Matrix localPin(Quat (90.0f * 3.141592f / 180.0f, Vec3 (1.0f, 0.0f, 0.0f)));
    Matrix matrix (localPin * wheel->GetMatrix());
    dNewtonUniversalJoint* const universal = new dNewtonUniversalJoint (&dMatrix (matrix.ptr())[0][0], wheel);

    // disable limit of first axis
    universal->EnableLimit_0(false);

    // set limit on second axis
    universal->SetLimis_1 (-500.0f * 3.141592f / 180.0f, 500.0f * 3.141592f / 180.0f);
}


void AddSlider (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& origin)
{
    // make a reel static
    newtonDynamicBody* const reel = CreateBox (viewer, world, origin + Vec3 (0.0f, 0.0f, 4.0f), Vec3 (8.0f, 0.25f, 0.25f));
    reel->SetMassAndInertia (0.0f, 0.0f, 0.0f, 0.0f);

    newtonDynamicBody* const wheel = CreateWheel (viewer, world, origin + Vec3 (0.0f, 0.0f, 4.0f), 1.0f, 0.5f);

    Matrix matrix (wheel->GetMatrix());
    dNewtonSliderJoint* const slider = new dNewtonSliderJoint (&dMatrix (matrix.ptr())[0][0], wheel, reel);

    // enable limit of first axis
    slider->EnableLimits(true);

    // set limit on second axis
    slider->SetLimis (-4.0f, 4.0f);
}


void AddCylindrical (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& origin)
{
    // make a reel static
    newtonDynamicBody* const reel = CreateCylinder(viewer, world, origin + Vec3 (0.0f, 0.0f, 4.0f), 0.25f, 8.0f);
    reel->SetMassAndInertia (0.0f, 0.0f, 0.0f, 0.0f);

    newtonDynamicBody* const wheel = CreateWheel (viewer, world, origin + Vec3 (0.0f, 0.0f, 4.0f), 1.0f, 0.5f);

    Matrix matrix (wheel->GetMatrix());
    dNewtonCylindricalJoint* const slider = new dNewtonCylindricalJoint (&dMatrix (matrix.ptr())[0][0], wheel, reel);

    // enable limit of first axis
    slider->EnableLimit_0(true);

    // set limit on second axis
    slider->SetLimis_0 (-4.0f, 4.0f);
}


void AddGear (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& origin)
{
    newtonDynamicBody* const reel = CreateCylinder(viewer, world, origin + Vec3 (0.0f, 0.0f, 4.0f), 0.25f, 4.0f);
    reel->SetMassAndInertia (0.0f, 0.0f, 0.0f, 0.0f);

    dNewtonHingeJoint* const hinge0 = AddHingeWheel (viewer, world, origin + Vec3 (-1.0f, 0.0f, 4.0f), 0.5f, 1.0f, reel);
    dNewtonHingeJoint* const hinge1 = AddHingeWheel (viewer, world, origin + Vec3 ( 1.0f, 0.0f, 4.0f), 0.5f, 1.0f, reel);

    newtonDynamicBody* const body0 = (newtonDynamicBody*)hinge0->GetBody0();
    newtonDynamicBody* const body1 = (newtonDynamicBody*)hinge1->GetBody0();

    Matrix matrix0 (body0->GetMatrix());
    Matrix matrix1 (body1->GetMatrix());
    matrix0.setTrans (Vec3 (0.0f, 0.0f, 0.0f));
    matrix1.setTrans (Vec3 (0.0f, 0.0f, 0.0f));

    Vec3 pin0 (matrix0.preMult(Vec3( 1.0f, 0.0f, 0.0f)));
    Vec3 pin1 (matrix1.preMult(Vec3( 1.0f, 0.0f, 0.0f)));
    new dNewtonGearJoint (4.0f, pin0.ptr(), body0, pin1.ptr(), body1);
}