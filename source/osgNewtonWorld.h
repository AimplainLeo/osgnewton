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


#ifndef _OSG_NEWTON_WORLD_H_
#define _OSG_NEWTON_WORLD_H_

#include "osgNewtonStdAfx.h"




namespace osg
{
	class rayPickerManager;
	class newtonInputManager;
	class newtonPlayerManager;
	class newtonTriggerManager;
	class newtonArticulationManager;

	class OSG_NEWTON_API newtonWorld: public dNewton 
	{
		public:
		newtonWorld (int updateFramerate = 120.0f);
		virtual ~newtonWorld();

		const osg::Vec4& GetGravity() const;
		void SetGravity(const osg::Vec4& gravity);
	
		// call each frame update to advance the physical world
		virtual void Update ();

		// get the system time in microseconds 
		dLong GetTimeInMicroSeconds() const;

		// get the physic time step time in microseconds 
		dLong GetPhysicsTimestepInMicroSeconds() const;

		// set the nominal frame per second update rate
		void SetUpdateFPS(dFloat desiredFps, int maxUpdatesPerFrames = 3);

		// called synchronous with the render update loop, at the beginning of setting all node transform after a physics update  
		virtual void OnNodesTransformBegin(dFloat interpolationParam){}

		// called synchronous with the render update loop, at the end of setting all node transform after a physics update  
		virtual void OnNodesTransformEnd(dFloat interpolationParam){}

		// called asynchronous at the beginning of a physics update. 
		virtual void OnBeginUpdate (dFloat timestepInSecunds){}

		// called asynchronous at the beginning end a physics update. 
		virtual void OnEndUpdate (dFloat timestepInSecunds){}


		newtonInputManager* GetInputManager() const;
		rayPickerManager* GetRayPickManager() const; 
		newtonPlayerManager* GetPlayerManager() const; 
		newtonTriggerManager* GetTriggerManager() const; 
		newtonArticulationManager* GetHierarchyTransformManager() const; 

		// set the asynchronous update mode
		void SetConcurrentUpdateMode (bool mode);
		bool GetConcurrentUpdateMode () const; 

		protected:
		newtonInputManager* m_inputManager;
		newtonPlayerManager* m_playerManager;
		newtonTriggerManager* m_triggerManager;
		rayPickerManager* m_rayPickerManager;
		newtonArticulationManager* m_localTransformManager;
		
		osg::Vec4 m_gravity;
		dFloat m_timestep;
		dLong m_lastPhysicTimeInMicroseconds;
		dLong m_physicUpdateTimestepInMicroseconds;
		bool m_concurrentUpdateMode;
	};
};



#endif
