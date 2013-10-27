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

#ifndef __SHOOT_ROGID_BODY_H__
#define __SHOOT_ROGID_BODY_H__

#include <HelpersStdAfx.h>



using namespace osg;

#define _SHELLS_COUNT 4

class ShootRigidBody
{
	public:
	ShootRigidBody ();

	void Init (osgViewer::Viewer* const viewer, osg::newtonWorld* const world);
	~ShootRigidBody();

	void Update();

	dFloat m_shootingTimer;
	dLong m_lastTimeInMicroseconds;
	osg::ref_ptr<Geode> m_shells[_SHELLS_COUNT];
	dNewtonCollision* m_shootingCollisions[_SHELLS_COUNT];
	osgViewer::Viewer* m_viewer;
	osg::newtonWorld* m_world;
};


#endif