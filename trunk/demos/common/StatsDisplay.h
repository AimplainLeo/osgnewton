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




#ifndef _STAT_DISPLAY_H_
#define _STAT_DISPLAY_H_

#include "Utils.h"

using namespace osg;

class StatsDisplay
{
	public:
	StatsDisplay();
	virtual void Init(osgViewer::Viewer* const viewer, newtonWorld* const physicsWorld);
	virtual ~StatsDisplay(void);

	virtual bool Update();
	osgText::Text* CreateText (int x, int y, const char* const title, int characterSize);

	void SetVilibleFlag(bool flags);

	private:
	osgViewer::Viewer* m_viewer;
	newtonWorld* m_physicsWorld;
	ref_ptr<Geode> m_textNode;
	ref_ptr<Camera> m_textCamera;
	ref_ptr<osgText::Font> m_font;

	dLong m_currTime;
	dLong m_physicsTimeAcc;
	int m_frameCount0;
	int m_frameCount1;
	KeyTrigger m_visilityFlag;

	protected:
	osgText::Text* m_frameRate;
	osgText::Text* m_physicsTime;
	osgText::Text* m_rigigBodyCount;
	osgText::Text* m_hideStatDisplay;
	osgText::Text* m_debugDisplay;
	osgText::Text* m_freeCameraNavigation;
	osgText::Text* m_extraControlKeys;
	osgText::Text* m_pikingBodyFromeScreen;
	osgText::Text* m_exitApplication;
	osgText::Text* m_throwBody;
	osgText::Text* m_asyncronousPhysicsSimulation;
};

#endif