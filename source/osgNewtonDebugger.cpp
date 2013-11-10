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
#include "osgNewtonDebugger.h"

namespace osg
{
class DebuggerCollisionMeshBuilder: public dNewtonCollision::dDebugRenderer
{
	public:
	DebuggerCollisionMeshBuilder (dNewtonCollision* const collision)
		:dNewtonCollision::dDebugRenderer(collision)
	{
	}

	void OnDrawFace (int vertexCount, const dFloat* const faceVertex, int id)
	{
		int i = vertexCount - 1;
		dVector p0 (faceVertex[i * 3 + 0], faceVertex[i * 3 + 1], faceVertex[i * 3 + 2]);
		for (int i = 0; i < vertexCount; i ++) {
			dVector p1 (faceVertex[i * 3 + 0], faceVertex[i * 3 + 1], faceVertex[i * 3 + 2]);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);
			p0 = p1;
		}
	}
};


class DebugMeshDrawble: public Drawable
{
	public: 
	DebugMeshDrawble ()
		:m_collision(NULL)
	{
	}


	DebugMeshDrawble (dNewtonCollision* const collision)
		:m_collision(collision)
	{
	}

	DebugMeshDrawble (const DebugMeshDrawble& copy, const CopyOp& copyop = CopyOp::SHALLOW_COPY)
		:Drawable (copy, copyop)
		,m_collision(copy.m_collision)
	{
	}

	~DebugMeshDrawble ()
	{
	}

	META_Object (osg, DebugMeshDrawble);

	virtual BoundingBox computeBound() const
	{
		dVector minBox;
		dVector maxBox;
		dMatrix matrix (GetIdentityMatrix());
		m_collision->CalculateAABB (&matrix[0][0], &minBox[0], &maxBox[0]);
		return BoundingBox(Vec3 (minBox.m_x, minBox.m_y, minBox.m_z), Vec3 (maxBox.m_x, maxBox.m_y, maxBox.m_z));
	}

	virtual void drawImplementation(RenderInfo& renderInfo) const
	{
		DebuggerCollisionMeshBuilder meshBuider (m_collision);

		glDisable (GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);

		glBegin(GL_LINES);
		glColor3f(1.0f, 1.0f, 1.0f);

		dMatrix localMatrix (GetIdentityMatrix());
		m_collision->DebugRender (&localMatrix[0][0], &meshBuider);

		glEnd();
	}

	dNewtonCollision* m_collision;
};


class newtonDebugger::DebugTransformNode: public MatrixTransform
{
	public:
	DebugTransformNode (dNewtonBody* const body)
		:MatrixTransform()
		,m_body(body)
	{
		dNewtonCollision* const collision = body->GetCollision();

		ref_ptr<Geode> geoNode = new Geode;
		addChild (geoNode);
		geoNode->addDrawable (new DebugMeshDrawble(collision));
	}
	~DebugTransformNode()
	{
	}
	dNewtonBody* m_body;
};


class newtonDebugger::UpdateCallback: public NodeCallback
{
	public:
	UpdateCallback()
		:NodeCallback()
	{
	}

	virtual void operator() (Node* node, NodeVisitor* visitor)
	{
		Group* const rootNode = static_cast <Group*> (node);

		unsigned chidrenCount =  rootNode->getNumChildren();

		for (unsigned int i = 0; i < chidrenCount; i ++) {
			dMatrix matrix;
			DebugTransformNode* const child = static_cast <DebugTransformNode*> (rootNode->getChild(i));
			child->m_body->GetMatrix (&matrix[0][0]);
			Matrix visualMatrix(&matrix[0][0]);
			child->setMatrix (visualMatrix);
		}
		traverse(node, visitor);
	}
};


newtonDebugger::newtonDebugger ()
	:m_world(NULL)
	,m_viewer(NULL)
	,m_debugRoot(NULL)
	,m_mode(false)
{
}

void newtonDebugger::Init (osgViewer::Viewer* const viewer, newtonWorld* const world)
{
	m_world = world;
	m_viewer = viewer;
}

newtonDebugger::~newtonDebugger()
{
}

bool newtonDebugger::GetMode() const
{
	return m_mode;
}

void newtonDebugger::SetMode(bool onOff)
{
	m_mode = onOff;
	if (onOff) {
		ShowDebugInformation();
	} else {
		HideDebugInformation();
	}
}

void newtonDebugger::HideDebugInformation()
{
	if (m_debugRoot.get()) {
		dAssert (m_viewer->getSceneData());
		Group* const rootNode = m_viewer->getSceneData()->asGroup();
		rootNode->removeChild(m_debugRoot);
	}
	m_debugRoot = NULL;
}


void newtonDebugger::ShowDebugInformation()
{
	HideDebugInformation();

	dAssert (m_viewer->getSceneData());
	Group* const rootNode = m_viewer->getSceneData()->asGroup();

	m_debugRoot = new Group();
	m_debugRoot->setUpdateCallback(new UpdateCallback());

	rootNode->addChild(m_debugRoot.get());
	for (dNewtonBody* body = m_world->GetFirstBody(); body; body = m_world->GetNextBody(body)) {
		ref_ptr<DebugTransformNode> transformNode = new DebugTransformNode(body);	
		m_debugRoot->addChild(transformNode.get());
	}
}


};