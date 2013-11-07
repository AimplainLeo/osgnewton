
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
#include "SpawnSingleShapeArray.h"



class PhysicsWorld: public DemoExample
{
	public:
	PhysicsWorld(osgViewer::Viewer* const viewer)
		:DemoExample(viewer)
	{
		// load the the static physic and visual world
		LoadStaticScene();

		// load the the dynamics physic and visual world
		LoadDynamicsScene();


		// initialize the Camera position after the scene was loaded
		Camera* const camera = m_viewer->getCamera();
		ResetCamera (camera->getViewMatrix());
	}

	~PhysicsWorld()
	{
	}

	void LoadStaticScene()
	{
		dAssert (m_viewer->getSceneData());

		Group* const rootGroup = m_viewer->getSceneData()->asGroup();
		dAssert (rootGroup);

		// load a flat mane as static floor
		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("flatplane.osg");
		rootGroup->addChild(node.get());

		// create a scene body and add all static meshes
		osg::newtonSceneBody* const sceneBody = new osg::newtonSceneBody (this, m_allExcludingMousePick);

		// start adding collision shape to the scene body
		sceneBody->BeginAddRemoveCollision();

		// add this collision to the scene body
		sceneBody->AddCollisionTree (node.get());

		// done adding collision shape to the scene body, now optimize the scene
		sceneBody->EndAddRemoveCollision();
	}

	void LoadDynamicsScene()
	{
		osg::Matrix camMatrix;
		camMatrix.setTrans (osg::Vec3(0.0f, 0.0f, 3.0f));
		ResetCamera (camMatrix);


		osg::Vec3 origin (0.0, 10.0f, 4.0f);

		const int compoundCount = 1;

		// add some compound collision shapes
//		SpawnManualCompoundCollisionShapes (m_viewer, this, compoundCount, origin + Vec3 (-20.0f, 0.0f, 0.0f));

//		SpawnAutomaticCompoundCollisionShapes (m_viewer, this, compoundCount, origin + Vec3 (20.0f, 1.0f, 0.0f), "cow.osg");
		SpawnAutomaticCompoundCollisionShapes (m_viewer, this, compoundCount, origin + Vec3 (20.0f, 1.0f, 0.0f), "cessna.osg");
		

		// make a convex hull from 200 random points
		dNewtonScopeBuffer<dVector> points(200);
		for (int i = 0; i < points.GetElementsCount(); i ++) {
			points[i] = dVector (Rand (0.5f), Rand (0.5f), Rand (0.5f), 0.0f);
		}
		// spawn 20 of each of the regular solid supported by the engine
		const int spawnCount = 20;
//		SpawnSingleShapeArray (m_viewer, this, spawnCount, origin + osg::Vec3 (-16.0f, 0.0f, 0.0f), dNewtonCollisionSphere (this, 0.5f, m_all));
//		SpawnSingleShapeArray (m_viewer, this, spawnCount, origin + osg::Vec3 (-12.0f, 0.0f, 0.0f), dNewtonCollisionBox (this, 0.5f, 0.75f, 0.6f, m_all));
//		SpawnSingleShapeArray (m_viewer, this, spawnCount, origin + osg::Vec3 (- 8.0f, 0.0f, 0.0f), dNewtonCollisionCapsule (this, 0.25f, 0.5f, m_all));
//		SpawnSingleShapeArray (m_viewer, this, spawnCount, origin + osg::Vec3 (- 4.0f, 0.0f, 0.0f), dNewtonCollisionTaperedCapsule (this, 0.25f, 0.35f, 0.5f, m_all));
//		SpawnSingleShapeArray (m_viewer, this, spawnCount, origin + osg::Vec3 (  0.0f, 0.0f, 0.0f), dNewtonCollisionCone (this, 0.25f, 0.75f, m_all));
//		SpawnSingleShapeArray (m_viewer, this, spawnCount, origin + osg::Vec3 (  4.0f, 0.0f, 0.0f), dNewtonCollisionCylinder (this, 0.25f, 0.5f, m_all));
//		SpawnSingleShapeArray (m_viewer, this, spawnCount, origin + osg::Vec3 (  8.0f, 0.0f, 0.0f), dNewtonCollisionTaperedCylinder (this, 0.25f, 0.35f, 0.5f, m_all));
//		SpawnSingleShapeArray (m_viewer, this, spawnCount, origin + osg::Vec3 ( 12.0f, 0.0f, 0.0f), dNewtonCollisionChamferedCylinder (this, 0.25f, 0.5f, m_all));
//		SpawnSingleShapeArray (m_viewer, this, spawnCount, origin + osg::Vec3 ( 16.0f, 0.0f, 0.0f), dNewtonCollisionConvexHull (this, points.GetElementsCount(), &points[0].m_x, sizeof (dVector), 1.0e-3f, m_all));
	}
};


int main (int argc, char* argv[])
{
#if defined(_DEBUG) && defined(_MSC_VER)
	// Track all memory leaks at the operating system level.
	// make sure no Newton tool or utility leaves leaks behind.
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF|_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF));
#endif

	// osg main viewer class
	osgViewer::Viewer viewer;

	// add a group node to serve a the root of all node
	viewer.setSceneData(new osg::Group);

	// set window mode default parameters
	InitWindowsSystem (viewer, "Newton 3.12 and OpenSceneGraph integration: demo02 standard collision shapes", 100, 100, 1024, 768);

	// create and instance of a physics world
	PhysicsWorld world(&viewer);

	// main simulation loop
	while (!viewer.done()) {
		// advance the physical world
		world.Update();

		// render the visual world
		viewer.frame();
	}

	// wait for last simulation frame to complete before shutting down
	world.WaitForUpdateToFinish();
	return 0;
}


