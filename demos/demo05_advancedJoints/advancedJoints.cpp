
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
#include "LumberYard.h"
#include "Forklift.h"

#define ARTICULATED_VEHICLE_CAMERA_DISTANCE			10.0f
#define ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD	2.5f

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
		sceneBody->AddCollisionTree (node.get(), DemoExample::m_all);

		// done adding collision shape to the scene body, now optimize the scene
		sceneBody->EndAddRemoveCollision();
	}

	void LoadDynamicsScene()
	{
		osg::Matrix camMatrix;
		camMatrix.setTrans (osg::Vec3(0.0f, -10.0f, 3.0f));
		ResetCamera (camMatrix);

		// get a start position close to the ground
		Vec4 start(0.0f, 0.0f, 1000.0f, 0.0f);
		Vec4 end(0.0f, 0.0f, -1000.0f, 0.0f);
		newtonRayCast raycaster (this, DemoExample::m_rayCast); 
		raycaster.CastRay (start, end);

		ForkliftPhysicsModel* const forkLift = new ForkliftPhysicsModel(m_viewer, this, "forklift.osg", Vec3 (raycaster.m_contact.x(), raycaster.m_contact.y(), raycaster.m_contact.z() + 1.0f));

        // set this object as the player
        newtonDynamicBody* const playerRootBody = (newtonDynamicBody*) forkLift->GetBoneBody (forkLift->GetBone(0));
        GetInputManager()->SetPlayer (playerRootBody, forkLift);
	}

    virtual void OnBeginUpdate (dFloat timestepInSecunds)
    {
		DemoExample::OnBeginUpdate (timestepInSecunds);

		const newtonInputManager::osgPlayerUserDataPair& playerData = GetInputManager()->GetPlayer();

		newtonDynamicBody* const playerBody = (newtonDynamicBody*)playerData.m_player;
		ForkliftPhysicsModel* const playerController = (ForkliftPhysicsModel*)playerData.m_userData;

		// set all of the player inputs
		ForkliftPhysicsModel::InputRecored inputs;
		inputs.m_throtler = int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_W)) - int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_S));
		inputs.m_steering = int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_A)) - int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_D));
        
		inputs.m_tilt = int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_X)) - int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_Z));
		inputs.m_lift = int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_Q)) - int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_E));
		inputs.m_palette = int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_F)) - int (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_G));

        // check if there are some vehicle input, if there is, then wakeup the vehicle
        if (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_W) || 
            m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_S) || 
            m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_A) || 
            m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_D) ||	
            m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_F) ||	
            m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_G) ||	
            m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_Z) ||	
            m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_C) ||	
            m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_Q) ||	
            m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_E)) {
                playerBody->SetSleepState(false);
        }

        playerController->ApplyInputs (inputs);
    }


    void OnEndUpdate(dFloat timestepInSecunds)
    {
		DemoExample::OnEndUpdate (timestepInSecunds);

		const newtonInputManager::osgPlayerUserDataPair& playerData = GetInputManager()->GetPlayer();
		newtonDynamicBody* const playerBody = (newtonDynamicBody*)playerData.m_player;

		// reposition the camera origin to point to the player
		Matrix playerMatrix(playerBody->GetMatrix());

		Matrix camMatrix(GetCameraTransform());
		camMatrix.setTrans (Vec3 (0.0f, 0.0f, 0.0f));
		Vec3 frontDir (camMatrix.preMult(Vec3 (0.0f, 1.0f, 0.0f)));
		Vec3 camOrigin (playerMatrix.preMult (Vec3(0.0f, 0.0f, ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD)));
        camOrigin -= camMatrix.preMult(Vec3 (0.0f, ARTICULATED_VEHICLE_CAMERA_DISTANCE, 0.0f));
		camMatrix.setTrans(camOrigin);
		SeCameraTransform (camMatrix);
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


