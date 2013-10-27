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

#ifndef _OSG_NEWTON_STDAFX_H_
#define _OSG_NEWTON_STDAFX_H_


#include <osg/Vec3>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Matrix>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>

#include <dNewton.h>
#include <dNewtonMesh.h>
#include <dNewtonJoint.h>
#include <dNewtonRayCast.h>
#include <dNewtonCollision.h>
#include <dNewtonDynamicBody.h>
#include <dNewtonScopeBuffer.h>
#include <dNewtonInputManager.h>
#include <dNewtonTransformLerp.h>
#include <dNewtonPlayerManager.h>
#include <dNewtonTriggerManager.h>
#include <dNewtonActuatorJoints.h>
#include <dNewtonArticulationManager.h>


#ifdef _OSG_NEWTON_STATIC_LIB
	#define OSG_NEWTON_API
#else 
	#ifdef _OSG_NEWTON_BUILD_DLL
		#ifdef _WIN32
			#define OSG_NEWTON_API __declspec (dllexport)
		#else
			#define OSG_NEWTON_API __attribute__ ((visibility("default")))
		#endif
	#else
		#ifdef _WIN32
			#define OSG_NEWTON_API __declspec (dllimport)
		#else
			#define OSG_NEWTON_API
		#endif
	#endif
#endif

#endif



