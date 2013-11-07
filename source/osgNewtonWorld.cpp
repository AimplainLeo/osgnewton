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


#include "osgNewtonStdAfx.h"
#include "osgNewtonWorld.h"
#include "osgNewtonDynamicBody.h"
#include "osgNewtonInputManager.h"
#include "osgNewtonRayPickManager.h"
#include "osgNewtonPlayerManager.h"
#include "osgNewtonTriggerManager.h"
#include "osgNewtonArticulationManager.h"

namespace osg
{


newtonWorld::newtonWorld (int updateFramerate)
	:dNewton()
	,m_materialMap()
	,m_gravity(0.0f, 0.0f, -9.8f, 0.0f)
	,m_timestep(0.0f)
	,m_lastPhysicTimeInMicroseconds(GetTimeInMicrosenconds ())
	,m_physicUpdateTimestepInMicroseconds(0)
	,m_concurrentUpdateMode(true)
{
	SetUpdateFPS (dFloat (updateFramerate), 3);

	// add some of the essential managers by order of execution
//	m_triggerManager = new newtonTriggerManager(this);
//	m_localTransformManager = new newtonArticulationManager (this);
//	m_playerManager = new newtonPlayerManager (this);
	m_rayPickerManager = new rayPickerManager (this, 0);
	m_inputManager = new newtonInputManager(this);
}

newtonWorld::~newtonWorld()
{
}

void newtonWorld::SetUpdateFPS(dFloat desiredFps, int maxUpdatesPerFrames)
{
	m_timestep = 1.0f / desiredFps;
	SetMaxUpdatesPerIterations (maxUpdatesPerFrames);
}


const osg::Vec4& newtonWorld::GetGravity() const 
{
	return m_gravity;
}

void newtonWorld::SetGravity(const osg::Vec4& gravity)
{
	m_gravity = gravity;
}

dLong newtonWorld::GetPhysicsTimestepInMicroSeconds() const
{
	return m_physicUpdateTimestepInMicroseconds;
}

dLong newtonWorld::GetTimeInMicroSeconds() const
{
	return dNewton::GetTimeInMicrosenconds();
}


newtonInputManager* newtonWorld::GetInputManager() const 
{
	return m_inputManager;
}

newtonArticulationManager* newtonWorld::GetHierarchyTransformManager() const 
{
	return m_localTransformManager;
}

newtonTriggerManager* newtonWorld::GetTriggerManager() const 
{
	return m_triggerManager;
}

newtonPlayerManager* newtonWorld::GetPlayerManager() const 
{
	return m_playerManager;
}


rayPickerManager* newtonWorld::GetRayPickManager() const 
{
	return m_rayPickerManager;
}


void newtonWorld::SetConcurrentUpdateMode (bool mode)
{
	m_concurrentUpdateMode = mode;
}

bool newtonWorld::GetConcurrentUpdateMode () const
{
	return m_concurrentUpdateMode;
}


bool newtonWorld::OnBodiesAABBOverlap (const dNewtonBody* const body0, const dNewtonBody* const body1, int threadIndex) const
{
	dNewtonCollision* const collision0 = body0->GetCollision();
	dNewtonCollision* const collision1 = body1->GetCollision();

	// check if these two collision shape are part of a hierarchical model
	void* const node0 = collision0->GetUserData();
	void* const node1 = collision1->GetUserData();
	if (node0 && node1) {
		//both collision are child nodes, check if there are self colliding
		return GetHierarchyTransformManager()->SelfCollisionTest (node0, node1);
	}

	// check all other collision using the bitfield mask, 
	//for now simple return true
	return (collision0->GetCollisionMask() & collision0->GetCollisionMask()) ? true : false;
}

bool newtonWorld::OnCompoundSubCollisionAABBOverlap (const dNewtonBody* const body0, const dNewtonCollision* const subShape0, const dNewtonBody* const body1, const dNewtonCollision* const subShape1, int threadIndex) const
{
	//	return (subShape0->m_collisionMask & subShape1->m_collisionMask) ? true : false;
	return true;
}

void newtonWorld::OnContactProcess (dNewtonContactMaterial* const contactMaterial, dFloat timestep, int threadIndex) const
{
	for (void* contact = contactMaterial->GetFirstContact(); contact; contact = contactMaterial->GetNextContact(contact)) {
		dNewtonCollision* const shape0 = contactMaterial->GetShape0(contact);
		dNewtonCollision* const shape1 = contactMaterial->GetShape1(contact);
		const dMaterialPairManager::dMaterialPair* const materialPair = m_materialMap.GetPair (shape0->GetMaterialId(), shape1->GetMaterialId(), threadIndex);

		contactMaterial->SetContactRestitution(contact, materialPair->m_restitution);
		contactMaterial->SetContactFrictionCoef (contact, materialPair->m_staticFriction0, materialPair->m_kineticFriction0, 0);
		contactMaterial->SetContactFrictionCoef (contact, materialPair->m_staticFriction1, materialPair->m_kineticFriction1, 1);
	}

	dNewton::OnContactProcess (contactMaterial, timestep, threadIndex);
}


dMaterialPairManager::dMaterialPair* newtonWorld::GetDefualtMaterialPair ()
{
	return m_materialMap.GetDefualtPair ();
}

void newtonWorld::AddMaterialPair (int materialId0, int materialId1, const dMaterialPairManager::dMaterialPair& pair)
{
	m_materialMap.AddPair (materialId0, materialId1, pair);
}


const dMaterialPairManager::dMaterialPair* newtonWorld::GetMaterialPair (int materialId0, int materialId1, int threadIndex) const
{
	return m_materialMap.GetPair (materialId0, materialId1, threadIndex);
}




void newtonWorld::Update ()
{
	dLong lastTime = m_lastPhysicTimeInMicroseconds;
	m_lastPhysicTimeInMicroseconds = GetTimeInMicrosenconds ();
	dFloat applicationTime = dFloat (m_lastPhysicTimeInMicroseconds - lastTime) * 1.0e-6f;
	if (m_concurrentUpdateMode) {
		dNewton::UpdateAsync (m_timestep);
	} else {
		dNewton::Update (m_timestep);
	}


	dFloat param = GetInterpolationParam(m_timestep);
	dAssert (applicationTime > 0.0f);
	OnNodesTransformBegin (param);

	// iterate over all physics bodies and get the tranformtaion matrix;
	for (dNewtonBody* body = GetFirstBody(); body; body = GetNextBody(body)) {
		if (!body->GetSleepState()) {
			MatrixTransform* const node = (MatrixTransform*) body->GetUserData();
			if (node) {

				dMatrix physMatrix;
				body->GetVisualMatrix (param, &physMatrix[0][0]);
				Matrix visualMatrix(&physMatrix[0][0]);
				node->setMatrix (visualMatrix);

				// update the application user data (that need to be update at rendering time, ex animations, particles emitions, etc)
				body->OnApplicationPostTransform (applicationTime);
			}
		}
	}
	OnNodesTransformEnd (param);

	m_physicUpdateTimestepInMicroseconds = GetTimeInMicrosenconds () - m_lastPhysicTimeInMicroseconds;
}

};