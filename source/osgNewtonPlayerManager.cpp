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
#include "osgNewtonPlayerManager.h"

namespace osg
{

Vec3 newtonPlayerManager::netwonPlayer::m_upDir (0.0f, 1.0f, 0.0f);
Vec3 newtonPlayerManager::netwonPlayer::m_frontDir (0.0f, 0.0f, -1.0f);


newtonPlayerManager::newtonPlayerManager (newtonWorld* const world)
	:dNewtonPlayerManager (world)
{
}

newtonPlayerManager::~newtonPlayerManager ()
{
}


newtonPlayerManager::netwonPlayer::netwonPlayer (newtonPlayerManager* const manager, MatrixTransform* const node, dFloat mass, dFloat outerRadius, dFloat innerRadius, dFloat height, dFloat stairStep, dLong collisionMask)
	:dNewtonPlayer (manager, node, mass, outerRadius, innerRadius, height, stairStep, &m_upDir.x(), &m_frontDir.x(), collisionMask)
{
	dAssert(0);
}

newtonPlayerManager::netwonPlayer::~netwonPlayer()
{
}


void newtonPlayerManager::netwonPlayer::OnPlayerMove (dFloat timestep)
{
	const newtonWorld* const world = (newtonWorld*) GetNewton();
	dVector gravity (world->GetGravity().ptr());
//	SetPlayerVelocity (dFloat forwardSpeed, dFloat lateralSpeed, dFloat verticalSpeed, dFloat headingAngle, const dFloat* const gravity, dFloat timestep);
	SetPlayerVelocity (0.0f, 0.0f, 0.0f, 0.0f, &gravity.m_x, timestep);
}

};
