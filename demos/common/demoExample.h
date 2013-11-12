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

#include "StatsDisplay.h"
#include "ShootRigidBody.h"
#include "InputEventHandler.h"

using namespace osg;

class DemoExample: public newtonWorld
{
	public:
	enum demoCollisionTypes
	{
		m_rayCast = 1<<0,
		m_mousePick = 1<<1,
		m_gravidyBody = 1<<2,

		m_all = m_rayCast | m_gravidyBody | m_mousePick,
//		m_allExcludingMousePick = m_all & ~ m_mousePick,
m_allExcludingMousePick = m_all | 256,
	};


	class SmoothCamera: public dNewtonTransformLerp
	{
		public:
		SmoothCamera();
		void Reset (const Matrix& matrix);
		void Move (dFloat deltaTranslation, dFloat deltaStrafe, dFloat pitchAngleStep, dFloat yawAngleStep);

		Matrix CalculateIntepolatedMatrix (dFloat param) const ;

		Matrix GetCameraTransform () const;
		void SeCameraTransform (const Matrix& matrix);

		private:
		dFloat m_cameraYawAngle;
		dFloat m_cameraPitchAngle;
		dVector m_cameraTranslation;
	};

	DemoExample (osgViewer::Viewer* const viewer);
	~DemoExample ();
	virtual void Update ();

    Matrix GetCameraTransform () const;
    void SeCameraTransform (const Matrix& matrix);

	void ResetCamera (const Matrix& matrix);

	virtual void LoadStaticScene() {}
	virtual void LoadDynamicsScene() {} 

	// called asynchronous at the beginning of a physics update. 
	virtual void OnBeginUpdate (dFloat timestepInSecunds);

	// called asynchronous at the beginning end a physics update. 
	virtual void OnEndUpdate (dFloat timestepInSecunds);

	// called synchronous with the render update loop, at the beginning of setting all node transform after a physics update  
	virtual void OnNodesTransformBegin(dFloat interpolationParam);

	// called synchronous with the render update loop, at the end of setting all node transform after a physics update  
	virtual void OnNodesTransformEnd(dFloat interpolationParam);

	// move the camera
	void UpdateFreeCamera ();

	// pick object with mouse cursor 
	void UpdateMousePick ();

	InputEventHandler* GetInputSystem() const;

	protected:
	newtonDebugger m_debugDisplay;
	SmoothCamera m_cameraSmoothing;
	StatsDisplay m_stats;
	ShootRigidBody m_shooting;
	osgViewer::Viewer* m_viewer;
	osg::ref_ptr<InputEventHandler> m_inputHandler;
	dFloat m_pickParam;
	bool m_mousePickMemory;
	KeyTrigger m_debugDisplayKey;
	KeyTrigger m_asyncronousUpdateKey;
};
