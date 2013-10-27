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

namespace osg
{

newtonRayCast::newtonRayCast(dNewton* const world, dLong collisionMask)
	:dNewtonRayCast(world, collisionMask)
	,m_param(1.0f)
{
}

void newtonRayCast::CastRay (const Vec4& p0, const Vec4& p1, int threadIndex)
{
	m_param = 1.2f;
	m_bodyHit = NULL;
	dNewtonRayCast::CastRay(p0.ptr(), p1.ptr());
}

dFloat newtonRayCast::OnRayHit (const dNewtonBody* const body, const dNewtonCollision* const shape, const dFloat* const contact, const dFloat* const normal, dLong collisionID, dFloat intersectParam)
{
	if (intersectParam < m_param) {
		m_bodyHit = (dNewtonBody*) body;
		m_param = intersectParam;
		m_shapeId = collisionID;
		m_normal = Vec4 (normal[0], normal[1], normal[2], 0.0f); 
		m_contact = Vec4 (contact[0], contact[1], contact[2], 0.0f); 
	}
	return intersectParam;
}
};
