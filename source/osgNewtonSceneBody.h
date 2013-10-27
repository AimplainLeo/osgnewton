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


#ifndef _OSG_NEWTON_SCENE_BODY_H_
#define _OSG_NEWTON_SCENE_BODY_H_

#include "osgNewtonStdAfx.h"
//#include "osgNewtonRayPickManager.h"


namespace osg
{
	class OSG_NEWTON_API newtonSceneBody: public dNewtonDynamicBody
	{
		public:
		newtonSceneBody (newtonWorld* const world, dLong collisionMask);
		~newtonSceneBody();

		virtual void OnForceAndTorque (dFloat timestep, int threadIndex) {};

		virtual void BeginAddRemoveCollision();
		virtual void* AddCollision(const dNewtonCollision* const collision);

		//virtual void* AddTerrain (Terrain* const terrain);
		virtual void* AddCollisionTree (osg::Node* const treeNode);
		
		virtual void RemoveCollision (void* const handle);
		virtual void EndAddRemoveCollision();
	};
};


#endif
