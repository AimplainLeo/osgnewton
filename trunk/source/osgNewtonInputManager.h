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

#ifndef _OSG_NEWTON_INPUT_MANAGER_H_
#define _OSG_NEWTON_INPUT_MANAGER_H_

#include "osgNewtonStdAfx.h"


namespace osg
{
	class newtonWorld;

	class OSG_NEWTON_API newtonInputManager: public dNewtonInputManager
	{
		public:
		class osgPlayerUserDataPair
		{
			public:
			osgPlayerUserDataPair()
				:m_userData(NULL)
				,m_player(NULL)
			{
			}
			
			void* m_userData; 
			dNewtonDynamicBody* m_player;
		};

		newtonInputManager (newtonWorld* const world);
		virtual ~newtonInputManager();

		void SetPlayer(dNewtonDynamicBody* const player, void* const userData)
		{
			m_playerInfo.m_player = player;
			m_playerInfo.m_userData = userData;
		}

		const osgPlayerUserDataPair& GetPlayer() const
		{
			return m_playerInfo;
		}

		virtual void OnBeginUpdate (dFloat timestepInSecunds);
		virtual void OnEndUpdate (dFloat timestepInSecunds);

		newtonWorld* m_world;
		osgPlayerUserDataPair m_playerInfo;
	};
};

#endif
