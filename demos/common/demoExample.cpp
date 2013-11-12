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

#include "HelpersStdAfx.h"
#include "DemoExample.h"
#include "ShootRigidBody.h"

#define FREE_CAMERA_SPEED		(1.0f / 16.0f)
#define FREE_CAMERA_YAW_SPEED	(1.0f * 3.141592f / 180.0f)
#define FREE_CAMERA_PITCH_SPEED (1.0f * 3.141592f / 180.0f)
 

DemoExample::SmoothCamera::SmoothCamera()
	:dNewtonTransformLerp()
	,m_cameraYawAngle(0.0f)
	,m_cameraPitchAngle(0.0f)
	,m_cameraTranslation(0.0f, 0.0f, 0.0f)
{
}

void DemoExample::SmoothCamera::Move (dFloat deltaTranslation, dFloat deltaStrafe, dFloat pitchAngleStep, dFloat yawAngleStep)
{
	// here we update the camera movement at simulation rate
	m_cameraYawAngle = dMod (m_cameraYawAngle + yawAngleStep, 3.141592f * 2.0f);
	m_cameraPitchAngle = dClamp (m_cameraPitchAngle + pitchAngleStep, - 80.0f * 3.141592f / 180.0f, 80.0f * 3.141592f / 180.0f);

	Matrix yawMatrix;
	Matrix pitchMatrix;
	yawMatrix.makeRotate (m_cameraYawAngle, Vec3f (0.0f, 1.0f, 0.0f));
	pitchMatrix.makeRotate (m_cameraPitchAngle, Vec3f (1.0f, 0.0f, 0.0f));
	Matrix matrix (pitchMatrix * yawMatrix);
	dMatrix tmpMatrix (matrix.ptr());

	m_cameraTranslation += tmpMatrix[0].Scale(deltaStrafe);
	m_cameraTranslation += tmpMatrix[2].Scale(deltaTranslation);
	tmpMatrix.m_posit = m_cameraTranslation;	
	Update (&tmpMatrix[0][0]);
}


void DemoExample::SmoothCamera::Reset (const Matrix& viewMatrix)
{
	dMatrix floatMatrix (viewMatrix.ptr());
	m_cameraYawAngle = dAtan2(-floatMatrix[0][2], floatMatrix[0][0]);
	m_cameraPitchAngle = dAtan2(floatMatrix[2][1], floatMatrix[1][1]);
	m_cameraTranslation = floatMatrix.m_posit;

	ResetMatrix (&floatMatrix[0][0]);
}

Matrix DemoExample::SmoothCamera::GetCameraTransform () const
{
	dMatrix tmp;
	GetTargetMatrix(&tmp[0][0]);
	return Matrix (&tmp[0][0]);
}

void DemoExample::SmoothCamera::SeCameraTransform (const Matrix& matrix)
{
	SetTargetMatrix(&dMatrix(matrix.ptr())[0][0]);
}


Matrix DemoExample::SmoothCamera::CalculateIntepolatedMatrix (dFloat param) const 
{
	static Matrix rotation (Quat (-90.0f * 3.14159265f / 180.0f, Vec3f (1.0f, 0.0f, 0.0f)));

	dMatrix tmpMatrix;
	InterplateMatrix (param, &tmpMatrix[0][0]);
	return Matrix::inverse(Matrix (&tmpMatrix[0][0])) * rotation;
}

DemoExample::DemoExample (osgViewer::Viewer* const viewer)
	:newtonWorld()
	,m_debugDisplay()
	,m_cameraSmoothing()
	,m_stats()
	,m_viewer(viewer)
	,m_pickParam(0.0f)
	,m_mousePickMemory(false)
	,m_debugDisplayKey(false)
	,m_asyncronousUpdateKey(true)
{
	// add and input event handler to the viewer for collection information
	m_inputHandler = new InputEventHandler;
	m_viewer->addEventHandler(m_inputHandler.get());

	// initialize the stats object
	m_stats.Init(m_viewer, this);

	// set the ray picked collision mask
	GetRayPickManager()->SetCollisionMask(m_mousePick);

	// initialize the debug display object
	m_debugDisplay.Init (viewer, this);

	// init random shooting class
	m_shooting.Init (viewer, this);


	// add a directional light for day light
	ref_ptr<Light> light = new Light;
	light->setLightNum(1);
	light->setDiffuse(Vec4 (0.3f, 0.3f, 0.3f, 0.3f));
	light->setPosition(Vec4 (0.0f, 0.0f, 50.0, 0.0f));
	light->setDirection (Vec3 (0.0f, 0.0f, -1.0f));

	ref_ptr<LightSource> lightSrc = new LightSource;
	lightSrc->setLight(light.get());

	dAssert (viewer->getSceneData());
	Group* const rootGroup = viewer->getSceneData()->asGroup();
	dAssert (rootGroup);
	rootGroup->addChild(lightSrc.get());
	rootGroup->getOrCreateStateSet()->setMode(GL_LIGHT1, StateAttribute::ON);
}

DemoExample::~DemoExample ()
{
}

InputEventHandler* DemoExample::GetInputSystem() const
{
	return m_inputHandler.get();
}

Matrix DemoExample::GetCameraTransform () const
{
	return m_cameraSmoothing.GetCameraTransform();
}

void DemoExample::SeCameraTransform (const Matrix& matrix)
{
	m_cameraSmoothing.SeCameraTransform(matrix);
}

void DemoExample::ResetCamera (const Matrix& matrix)
{
	m_cameraSmoothing.Reset(matrix);
}

void DemoExample::Update ()
{
	if (!m_inputHandler->AppTerminated()) {
		// update simulation stats
		m_stats.Update();

		// update the physics state 
		newtonWorld::Update();
	}
}


// called asynchronous at the beginning of a physics update. 
void DemoExample::OnBeginUpdate (dFloat timestepInSecunds)
{
	// update mouse picking system
	UpdateMousePick ();
}

// called asynchronous at the beginning end a physics update. 
void DemoExample::OnEndUpdate (dFloat timestepInSecunds)
{
	// update free Camera
	UpdateFreeCamera ();
}

// called synchronous with the render update loop, at the beginning of setting all node transform after a physics update  
void DemoExample::OnNodesTransformBegin(dFloat interpolationParam)
{
	//see if we are hiding the help menu
	m_stats.SetVilibleFlag (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_F1) ? true : false);

	// check debug display mode
	m_debugDisplayKey.Update (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_F3) ? true : false);
	if (m_debugDisplayKey.TriggerUp()) {
		m_debugDisplay.SetMode (!m_debugDisplay.GetMode());
	}

	// check if the player want to run the physics concurrent of not 
	m_asyncronousUpdateKey.Update (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_F2) ? true : false);
	if (m_asyncronousUpdateKey.TriggerUp()) {
		WaitForUpdateToFinish();
		SetConcurrentUpdateMode(!GetConcurrentUpdateMode());
	}

	// update random shooting system
	m_shooting.Update();
}

// called synchronous with the render update loop, at the end of setting all node transform after a physics update  
void DemoExample::OnNodesTransformEnd(dFloat interpolationParam)
{
	Camera* const camera = m_viewer->getCamera();
	camera->setViewMatrix (m_cameraSmoothing.CalculateIntepolatedMatrix(interpolationParam));
}



void DemoExample::UpdateFreeCamera ()
{
	dFloat moveScale = FREE_CAMERA_SPEED;
	
	dFloat yaw = 0.0f;
	dFloat pitch = 0.0f;
	dFloat strafe = 0.0f;
	dFloat translation = 0.0f;

	if (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_W)) {
		translation = -moveScale;	
	}

	if (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_S)) {
		translation = moveScale;	
	}

	if (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_A)) {
		strafe = -moveScale;	
	}

	if (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_D)) {
		strafe = moveScale;	
	}

	if (!(m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_Control_L) || m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_Control_R))) {
		m_inputHandler->HideCursor (m_viewer);
		int dx = m_inputHandler->MouseRelX();
		if (dx > 2) {
			yaw = -FREE_CAMERA_YAW_SPEED;
		} else if (dx < -2) {
			yaw = FREE_CAMERA_YAW_SPEED;
		}

		int dy = m_inputHandler->MouseRelY();
		if (dy > 2) {
			pitch = -FREE_CAMERA_PITCH_SPEED;
		} else if (dy < -2) {
			pitch = FREE_CAMERA_PITCH_SPEED;
		}
	}

strafe = 0.0f;
translation = 0.0f;
//yaw = 0.0f;
pitch = 0.0f;

	m_cameraSmoothing.Move (translation, strafe, pitch, yaw);
}


void DemoExample::UpdateMousePick ()
{
	bool mouseKey1 = m_inputHandler->IsMouseLeftButtonDown();

	rayPickerManager* const rayPicker = GetRayPickManager();
	if (m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_Control_L) || m_inputHandler->IsKeyDown(osgGA::GUIEventAdapter::KEY_Control_R)) {
		m_inputHandler->ShowCursor (m_viewer);
		if (mouseKey1) {
			Camera* const camera = m_viewer->getCamera(); 

			Matrix matrix;
			matrix.invert(camera->getViewMatrix() * camera->getProjectionMatrix() * camera->getViewport()->computeWindowMatrix());

			int x = m_inputHandler->MouseX();
			int y = m_inputHandler->MouseY();

			Vec3 winP0 (x, y,  0.0f);
			Vec3 winP1 (x, y,  1.0f);

			Vec4 q0 (winP0 * matrix, 1.0f);
			Vec4 q1 (winP1 * matrix, 1.0f);

			if (!m_mousePickMemory) {
				rayPicker->SetPickedBody (NULL);
				
				dNewtonBody* const body = rayPicker->RayCast (q0, q1, m_pickParam);
				if (body) {
					Vec4 p (q0 + (q1 - q0) * m_pickParam);
					rayPicker->SetPickedBody (body, p);
				}
			} else {
				Vec4 p (q0 + (q1 - q0) * m_pickParam);
				rayPicker->SetTarget (p);
			}

		} else {
			rayPicker->SetPickedBody (NULL);
		}
	} else {
		rayPicker->SetPickedBody (NULL);
	}

	m_mousePickMemory = mouseKey1;
}