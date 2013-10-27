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

using namespace osg;

class InputEventHandler: public osgGA::GUIEventHandler
{
	public:
	InputEventHandler();
	virtual ~InputEventHandler();

	int MouseX() const { return m_mouseX0;}
	int MouseY() const { return m_mouseY0;}

	int SetMouseX();
	int SetMouseY();

	int MouseRelX() const;
	int MouseRelY() const;

	void HideCursor(osgViewer::Viewer* const viewer);
	void ShowCursor(osgViewer::Viewer* const viewer);

	bool AppTerminated() const {return m_applicationTerminated;}

	bool IsKeyDown(osgGA::GUIEventAdapter::KeySymbol key) const;

	bool IsMouseLeftButtonDown() const {return m_leftMouseButton;}
	bool IsMouseRightButtonDown() const {return m_rightMouseButton;}

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*);


	private:
	void KeyUp(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void KeyDown(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void MouseMove(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void MouseButtonDown(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void MouseButtonRelease(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	bool m_keyBuffer[512];
	int m_mouseX0;
	int m_mouseY0;
	int m_mouseX1;
	int m_mouseY1;
	int m_frameCount;
	int m_mouseMoveCount;
	bool m_cursorOn;
	bool m_leftMouseButton;
	bool m_rightMouseButton;
	bool m_applicationTerminated;
	
};
