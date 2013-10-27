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


#ifndef _OSG_NEWTON_RAY_PICKING_MANAGER_H_
#define _OSG_NEWTON_RAY_PICKING_MANAGER_H_

#include "osgNewtonStdAfx.h"

#define OSG_NEWTON_RAY_PICKER_PLUGIN_NAME	"__rayPickManager__"

namespace osg
{
	class newtonWorld;

	class OSG_NEWTON_API rayPickerController: public CustomControllerBase
	{
		public:
		virtual void PreUpdate(dFloat timestep, int threadIndex)
		{
			//do nothing;
		}

		virtual void PostUpdate(dFloat timestep, int threadIndex)
		{
			//do nothing;
		}
	};


	class OSG_NEWTON_API rayPickerManager: public CustomControllerManager<rayPickerController> 
	{
		public:
		class RayPicker;

		rayPickerManager (newtonWorld* const world, dLong collisionMask);
		virtual ~rayPickerManager();

		void PreUpdate(dFloat timestep);
		void PostUpdate (dFloat timestep);

		void SetCollisionMask(dLong mask);
		dLong GetCollisionMask() const;

		dNewtonBody* RayCast (const Vec4& lineP0, const Vec4& lineP1, dFloat& pickParam) const;

		void SetTarget (const Vec4& targetPoint);
		void SetPickedBody (dNewtonBody* const body, const Vec4& handle = Vec4 (0.0f, 0.0f, 0.0f, 1.0f));

		virtual void Debug () const {};

		protected:
		Vec4 m_globalTarget;
		Vec4 m_localpHandlePoint;
		newtonWorld* m_world;
		dNewtonBody* m_pickedBody;
		dLong m_collisionMask;
		dFloat m_stiffness;
		unsigned m_lock;
	};
};

#endif
