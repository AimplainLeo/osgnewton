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
#include "osgNewtonRayCast.h"
#include "osgNewtonDynamicBody.h"
#include "osgNewtonRayPickManager.h"

namespace osg
{

class rayPickerManager::RayPicker: public newtonRayCast
{
	public:
	RayPicker(dNewton* const world, long collisionMask)
		:newtonRayCast(world, collisionMask)
	{
	}

	dFloat OnRayHit (const dNewtonBody* const body, const dNewtonCollision* const shape, const dFloat* const contact, const dFloat* const normal, dLong collisionID, dFloat intersectParam)
	{
		if (intersectParam < m_param) {
			// skip static bodies
			dFloat mass = 0.0f;
			if (body->GetType() == dNewtonBody::m_dynamic) {
				mass = ((newtonDynamicBody*) body)->GetMass();
			} else {
				dAssert (0);
			}
			
			if (mass > 0.0f) {
				m_bodyHit = (dNewtonBody*) body;
				m_param = intersectParam;
				m_normal = Vec4 (normal[0], normal[1], normal[2], 0.0f); 
				m_contact = Vec4 (contact[0], contact[1], contact[2], 0.0f); 
			}
		}
		return intersectParam;
	}
};




rayPickerManager::rayPickerManager (newtonWorld* const world, dLong collisionMask)
	:CustomControllerManager<rayPickerController>(world->GetNewton(), OSG_NEWTON_RAY_PICKER_PLUGIN_NAME)
	,m_globalTarget (0.0f, 0.0f, 0.0f, 0.0f) 
	,m_localpHandlePoint (0.0f, 0.0f, 0.0f, 0.0f) 
	,m_world(world)
	,m_pickedBody(NULL)
	,m_collisionMask(collisionMask)
	,m_stiffness(0.25f)
	,m_lock(0)
{
}

rayPickerManager::~rayPickerManager()
{
}


void rayPickerManager::PostUpdate (dFloat timestep)
{
	//do nothing;
}


dNewtonBody* rayPickerManager::RayCast (const Vec4& lineP0, const Vec4& lineP1, dFloat& pickParam) const
{
	RayPicker rayPicker (m_world, m_collisionMask);

	rayPicker.CastRay(lineP0, lineP1, 0);
	pickParam = rayPicker.m_param;
	return rayPicker.m_bodyHit;
}	

void rayPickerManager::SetPickedBody (dNewtonBody* const body, const Vec4& handle)
{
	dNewton::ScopeLock scopelock (&m_lock);

	m_pickedBody = body;
	if (m_pickedBody) {
		Matrix matrix;
		if (m_pickedBody->GetType() == dNewtonBody::m_dynamic) {
			matrix.invert(((newtonDynamicBody*) body)->GetMatrix());
		} else {
			dAssert (0);
		}
		m_localpHandlePoint = handle * matrix;
		m_globalTarget = handle;
	}
}

void rayPickerManager::SetTarget (const Vec4& targetPoint)
{
	dNewton::ScopeLock scopelock (&m_lock);
	m_globalTarget = targetPoint;
}

void rayPickerManager::SetCollisionMask(dLong mask)
{
	m_collisionMask = mask;
}

dLong rayPickerManager::GetCollisionMask() const
{
	return m_collisionMask;
}


void rayPickerManager::PreUpdate(dFloat timestep)
{
	// all of the work will be done here;
	dNewton::ScopeLock scopelock (&m_lock);
	if (m_pickedBody) {
		if (m_pickedBody->GetType() == dNewtonBody::m_dynamic) {
			newtonDynamicBody* const body = (newtonDynamicBody*)m_pickedBody;

			dFloat invTimeStep = 1.0f / timestep;
			Matrix matrix (body->GetMatrix());
			Vec4 omega0 (body->GetOmega());
			Vec4 veloc0 (body->GetVeloc());

			Vec4 peekPosit (m_localpHandlePoint * matrix);
			Vec4 peekStep (m_globalTarget - peekPosit);

			Vec4 pointVeloc (body->GetPointVeloc (peekPosit));
			Vec4 deltaVeloc (peekStep * (m_stiffness * invTimeStep) - pointVeloc);

			for (int i = 0; i < 3; i ++) {
				Vec4 veloc (0.0f, 0.0f, 0.0f, 0.0f);
				veloc[i] = deltaVeloc[i];
				body->ApplyImpulseToDesiredPointVeloc (peekPosit, veloc);
			}


            // damp angular velocity
            //NewtonBodyGetOmega (body, &omega1[0]);
            //NewtonBodyGetVelocity (body, &veloc1[0]);
            Vec4 omega1 (body->GetOmega());
            Vec4 veloc1 (body->GetVeloc());
            omega1 = omega1 * (0.9f);

            // restore body velocity and angular velocity
            body->SetVeloc(veloc0);
            body->SetOmega(omega0);

            // convert the delta velocity change to a external force and torque
            dFloat Ixx;
            dFloat Iyy;
            dFloat Izz;
            dFloat mass;
            body->GetMassAndInertia (mass, Ixx, Iyy, Izz);

            matrix.setTrans(Vec3(0.0f, 0.0f, 0.0f));

            Vec4 relOmega (omega1 - omega0);
            relOmega = matrix.preMult(relOmega);
            Vec4 angularMomentum (Ixx, Iyy, Izz, 0.0f);
//          angularMomentum = matrix.RotateVector (angularMomentum.CompProduct(matrix.UnrotateVector(omega1 - omega0)));
            angularMomentum = componentMultiply (relOmega, angularMomentum);
            angularMomentum = matrix.postMult(angularMomentum);
            Vec4 torque (angularMomentum * invTimeStep);
            body->AddTorque(torque);

            Vec4 relVeloc (veloc1 - veloc0);
            Vec4 force (relVeloc * (mass * invTimeStep));
            body->AddForce (force);

		} else {
			dAssert (0);
		}
	}
}

};