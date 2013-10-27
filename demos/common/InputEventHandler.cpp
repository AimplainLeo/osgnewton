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
#include "InputEventHandler.h"

InputEventHandler::InputEventHandler()
	:osgGA::GUIEventHandler()
	,m_mouseX0(0)
	,m_mouseY0(0)
	,m_mouseX1(0)
	,m_mouseY1(0)
	,m_frameCount(0)
	,m_mouseMoveCount(0)
	,m_cursorOn(true)
	,m_leftMouseButton(false)
	,m_rightMouseButton(false)
	,m_applicationTerminated(false)
{
	memset (m_keyBuffer, 0, sizeof (m_keyBuffer));
}

InputEventHandler::~InputEventHandler()
{
}


bool InputEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*) 
{ 
	osgViewer::View* const view = dynamic_cast<osgViewer::View*>(&aa);
	if (view && view->getViewerBase()) {
		osgGA::GUIEventAdapter::EventType eventType = ea.getEventType();
		switch (eventType) 
		{
			case osgGA::GUIEventAdapter::KEYDOWN:
			{
				KeyDown(ea, aa);
				break;
			}
			case osgGA::GUIEventAdapter::KEYUP:
			{
				KeyUp(ea, aa);
				break;
			}

			case osgGA::GUIEventAdapter::DRAG:
			case osgGA::GUIEventAdapter::MOVE:
			{
				MouseMove(ea, aa);
				break;
			}

			case osgGA::GUIEventAdapter::PUSH:
			{
				MouseButtonDown(ea, aa);
				break;
			}

			case osgGA::GUIEventAdapter::RELEASE:
			{
				MouseButtonRelease(ea, aa);
				break;
			}

			case osgGA::GUIEventAdapter::FRAME:
			{
				m_frameCount ++;
				break;
			}

			case osgGA::GUIEventAdapter::CLOSE_WINDOW:
			case osgGA::GUIEventAdapter::QUIT_APPLICATION:
			{
				m_applicationTerminated = true;
				break;
			}

			default:
				break;
		}
	}

	return false;
}

void InputEventHandler::KeyUp(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	unsigned keySymbol = ea.getKey();
	dAssert ((keySymbol & 0x1ff) < sizeof (m_keyBuffer) / sizeof (m_keyBuffer[0]));
	m_keyBuffer[keySymbol & 0x1ff] = false;
}

void InputEventHandler::KeyDown(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	unsigned keySymbol = ea.getKey();
	dAssert ((keySymbol & 0x1ff) < sizeof (m_keyBuffer) / sizeof (m_keyBuffer[0]));
	m_keyBuffer[keySymbol & 0x1ff] = true;;
}


bool InputEventHandler::IsKeyDown(osgGA::GUIEventAdapter::KeySymbol keySymbol) const
{
	dAssert ((keySymbol & 0x1ff) < sizeof (m_keyBuffer) / sizeof (m_keyBuffer[0]));
	return m_keyBuffer[keySymbol & 0x1ff];
}


void InputEventHandler::MouseMove(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	int x0 = ea.getXmin();
	int y0 = ea.getYmin();
	int x1 = ea.getXmax();
	int y1 = ea.getYmax();

	int centerX = (x0 + x1) / 2;
	int centerY = (y0 + y1) / 2;

	if (!m_cursorOn) {
		if (!((ea.getX() == centerX) && (ea.getY() == centerY))) {
			m_mouseX0 = centerX;
			m_mouseY0 = centerY;
			m_mouseX1 = ea.getX();
			m_mouseY1 = ea.getY();
			aa.requestWarpPointer (centerX, centerY);
		}
	} else {
		if (m_mouseMoveCount > (m_frameCount - 64)) {
			// track mouse inside frame window
			m_mouseX0 = m_mouseX1;
			m_mouseY0 = m_mouseY1;
			m_mouseX1 = ea.getX();
			m_mouseY1 = ea.getY();
		}
	}
	m_mouseMoveCount = m_frameCount;
}

void InputEventHandler::MouseButtonDown(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_leftMouseButton |= (ea.getButton() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) ? true : false;
	m_rightMouseButton |= (ea.getButton() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) ? true : false;
}

void InputEventHandler::MouseButtonRelease(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_leftMouseButton &= (ea.getButton() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) ? false : false;
	m_rightMouseButton &= (ea.getButton() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) ? false : false;
}


int InputEventHandler::MouseRelX() const 
{ 
	return (m_mouseMoveCount > (m_frameCount - 64)) ? m_mouseX1 - m_mouseX0 : 0;
}

int InputEventHandler::MouseRelY() const 
{ 
	return (m_mouseMoveCount > (m_frameCount - 64)) ? m_mouseY1 - m_mouseY0 : 0;
}


void InputEventHandler::HideCursor(osgViewer::Viewer* const viewer)
{
	if (m_cursorOn) {
		m_cursorOn = false;
		osg::GraphicsContext* const context = viewer->getCamera()->getGraphicsContext();
		osgViewer::GraphicsWindow* const gw = dynamic_cast<osgViewer::GraphicsWindow*>(context);
		gw->setCursor(osgViewer::GraphicsWindow::NoCursor);
	}
}

void InputEventHandler::ShowCursor(osgViewer::Viewer* const viewer)
{
	if (!m_cursorOn) {
		m_cursorOn = true;
		osg::GraphicsContext* const context = viewer->getCamera()->getGraphicsContext();
		osgViewer::GraphicsWindow* const gw = dynamic_cast<osgViewer::GraphicsWindow*>(context);
		gw->setCursor(osgViewer::GraphicsWindow::LeftArrowCursor);
	}
}
