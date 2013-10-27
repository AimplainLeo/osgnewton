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

#include "HelpersStdAfx.h"
#include "StatsDisplay.h"

#define FONT_CHARACTER_SIZE				18
#define UPDATE_INTERVAL_IN_MICROSECONDS	100000

StatsDisplay::StatsDisplay()
	:m_viewer(NULL)
	,m_physicsWorld(NULL)
	,m_textNode(NULL)
	,m_textCamera(NULL)
	,m_font(NULL)
	,m_currTime(0)
	,m_physicsTimeAcc(0)
	,m_frameCount0(0)
	,m_frameCount1(0)
	,m_visilityFlag(true)
	,m_frameRate(NULL)
	,m_physicsTime(NULL)
{
}

void StatsDisplay::Init(osgViewer::Viewer* const viewer, newtonWorld* const physicsWorld)
{
	m_viewer = viewer;
	m_physicsWorld = physicsWorld;

	m_textNode = new Geode;
	m_textCamera = new Camera;

	//m_font = osgText::readFontFile("fonts\\calibri.ttf");
	m_font = osgText::readFontFile("fonts\\arial.ttf");

	dAssert (m_viewer->getSceneData());
	Group* const rootGroup = m_viewer->getSceneData()->asGroup();
	dAssert (rootGroup);
	
	// add a 2d camera for rendering the text
	const Camera* const camera = m_viewer->getCamera();
	const Viewport& viewport = *camera->getViewport(); 

	m_textCamera->setReferenceFrame(Transform::ABSOLUTE_RF);
	m_textCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	m_textCamera->setRenderOrder(Camera::POST_RENDER);
	m_textCamera->setAllowEventFocus(false);
	m_textCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, StateAttribute::OFF);

	Matrix matrix (Matrix::ortho2D (viewport.x(), viewport.width(), viewport.y(), viewport.height()));
	m_textCamera->setProjectionMatrix(matrix);

	rootGroup->addChild(m_textCamera);
	m_textCamera->addChild(m_textNode);

	m_frameRate = CreateText(10, 20, "frame rate(fps): 0.0f", FONT_CHARACTER_SIZE);
	m_physicsTime = CreateText(10, 40, "physics time(ms): 0.0f", FONT_CHARACTER_SIZE);
	m_rigigBodyCount = CreateText(10, 60, "rigid bodies count: 0.0f", FONT_CHARACTER_SIZE);
	m_hideStatDisplay = CreateText(10, 80, "F1: hide stats info", FONT_CHARACTER_SIZE);
	m_asyncronousPhysicsSimulation = CreateText(10, 100, "F2: toggle asynchronous simulation update", FONT_CHARACTER_SIZE);
	m_debugDisplay = CreateText(10, 120, "F3: toggle display physic debug", FONT_CHARACTER_SIZE);
	m_freeCameraNavigation = CreateText (10, 150, "W, S, A, D:  free camera navigation", FONT_CHARACTER_SIZE);
	m_throwBody = CreateText (10, 170, "space: throw random rigid body", FONT_CHARACTER_SIZE);
	m_pikingBodyFromeScreen = CreateText (10, 190, "hold CTRL and left mouse key: show mouse cursor and pick objects from the screen", FONT_CHARACTER_SIZE);
	m_exitApplication = CreateText (10, 210, "ESC: exit application", FONT_CHARACTER_SIZE);

	// set a mask for invisibility
	m_textCamera->setNodeMask(1);
}


StatsDisplay::~StatsDisplay(void)
{

}

void StatsDisplay::SetVilibleFlag(bool flags)
{
	m_visilityFlag.Update (flags);
	m_textCamera->setNodeMask(m_visilityFlag.m_state ? 1 : 0);
}


osgText::Text* StatsDisplay::CreateText (int x, int y, const char* const title, int characterSize)
{
	osgText::Text* const text = new osgText::Text;
	const Camera* const camera = m_viewer->getCamera();
	const Viewport& viewport = *camera->getViewport(); 
	
	text->setFont(m_font.get());
	text->setCharacterSize(characterSize);
	text->setAxisAlignment(osgText::TextBase::SCREEN);
	text->setPosition(Vec3 (x, viewport.height() - y, 0.0f));
	text->setText(title);
	m_textNode->addDrawable(text);

	return text;
}

bool StatsDisplay::Update()
{
	bool state = false; 
	m_frameCount1 ++;
	dLong time = m_physicsWorld->GetTimeInMicroSeconds();
	m_physicsTimeAcc += m_physicsWorld->GetPhysicsTimestepInMicroSeconds();
	
	if (time > (m_currTime + UPDATE_INTERVAL_IN_MICROSECONDS)) {
		char text[1024];

		int frameCount = m_frameCount1 - m_frameCount0;
		dAssert (frameCount);
		dFloat timeStep = dFloat (time - m_currTime) * 1.0e-6f / frameCount;
		dFloat fps = 1.0f / timeStep;

		dFloat phsyicsStep = dFloat (m_physicsTimeAcc) * 1.0e-3f / frameCount;

		sprintf (text, "frame rate(fps): %5.2f", fps);
		m_frameRate->setText(text);

		sprintf (text, "physics time(ms): %5.2f", phsyicsStep);
		m_physicsTime->setText(text);

		sprintf (text, "rigid bodies count: %d", m_physicsWorld->GetBodyCount());
		m_rigigBodyCount->setText(text);

		sprintf (text, "F2: toggle %s simulation update", m_physicsWorld->GetConcurrentUpdateMode() ? "asynchronous" : "synchronous");
		m_asyncronousPhysicsSimulation->setText(text);
		
		m_physicsTimeAcc = 0;
		m_frameCount0 = m_frameCount1;
		m_currTime = time;
		state = true;
	}

	return state;
}