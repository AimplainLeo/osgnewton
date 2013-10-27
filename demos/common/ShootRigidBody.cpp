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


#include <HelpersStdAfx.h>
#include "Utils.h"
#include "DemoExample.h"
#include "ShootRigidBody.h"

using namespace osg;

#define SHOOTING_RATE_TIME_IN_SECONDS	0.1f
#define SHOOTING_SPEED					40.0f

ShootRigidBody::ShootRigidBody ()
	:m_viewer(NULL)
	,m_world(NULL)
	,m_shootingTimer (0)
{
}

void ShootRigidBody::Init (osgViewer::Viewer* const viewer, osg::newtonWorld* const myWorld)
{
	m_world = myWorld;
	m_viewer = viewer; 
	ref_ptr<Texture2D> texture = new Texture2D;
	ref_ptr<Image> image = osgDB::readImageFile("images\\frowny.tga");
	texture->setImage (image.get());
	texture->setWrap(Texture::WRAP_S, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_R, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_T, Texture::REPEAT);

	{
		// make sphere;
		m_shootingCollisions[0] = new dNewtonCollisionSphere (m_world, 0.25f, DemoExample::m_all);

		newtonMesh mesh (m_shootingCollisions[0]);
		mesh.Triangulate();
		int materialId = mesh.AddMaterial(texture.get());
		mesh.ApplySphericalMapping (materialId);

		ref_ptr<Geode> geometryNode = mesh.CreateGeodeNode();
		m_shells[0] = geometryNode;
	}

	{
		// make capsule;
		m_shootingCollisions[1] = new dNewtonCollisionCapsule (m_world, 0.25f, 1.0f, DemoExample::m_all);

		newtonMesh mesh (m_shootingCollisions[1]);
		mesh.Triangulate();
		int materialId = mesh.AddMaterial(texture.get());
		mesh.ApplyCylindricalMapping (materialId, materialId);

		ref_ptr<Geode> geometryNode = mesh.CreateGeodeNode();
		m_shells[1] = geometryNode;
	}


	{
		// make chamhered cylinder;
		m_shootingCollisions[2] = new dNewtonCollisionChamferedCylinder (m_world, 0.25f, 1.0f, DemoExample::m_all);

		newtonMesh mesh (m_shootingCollisions[2]);
		mesh.Triangulate();
		int materialId = mesh.AddMaterial(texture.get());
		mesh.ApplyCylindricalMapping (materialId, materialId);

		ref_ptr<Geode> geometryNode = mesh.CreateGeodeNode();
		m_shells[2] = geometryNode;
	}


	{
		// make cylinder cylinder;
		m_shootingCollisions[3] = new dNewtonCollisionCylinder (m_world, 0.25f, 1.0f, DemoExample::m_all);

		newtonMesh mesh (m_shootingCollisions[3]);
		mesh.Triangulate();
		int materialId = mesh.AddMaterial(texture.get());
		mesh.ApplyCylindricalMapping (materialId, materialId);

		ref_ptr<Geode> geometryNode = mesh.CreateGeodeNode();
		m_shells[3] = geometryNode;
	}

	DemoExample* const world = (DemoExample*) m_world;
	m_lastTimeInMicroseconds = world->GetTimeInMicrosenconds () + dLong (SHOOTING_RATE_TIME_IN_SECONDS * 1000000);
}


ShootRigidBody::~ShootRigidBody()
{
	for (int i = 0; i < int (sizeof (m_shootingCollisions) / sizeof (m_shootingCollisions[0])); i ++) {
		delete m_shootingCollisions[i];
	}
}


void ShootRigidBody::Update()
{
	DemoExample* const world = (DemoExample*) m_world;
	InputEventHandler* const inputSystem = world->GetInputSystem();

	if (inputSystem->IsKeyDown(osgGA::GUIEventAdapter::KEY_Space)) {
		dLong lastPhysicTimeInMicroseconds = world->GetTimeInMicrosenconds ();

		if (lastPhysicTimeInMicroseconds > m_lastTimeInMicroseconds) {
			m_lastTimeInMicroseconds = lastPhysicTimeInMicroseconds + dLong(SHOOTING_RATE_TIME_IN_SECONDS * 1000000);

			// make sure we do no create bodies when the physics is running an update.
			world->WaitForUpdateToFinish();

			// create new projectile rigid body
			Camera* const camera = m_viewer->getCamera();
			Matrix matrix;
			matrix.invert(camera->getViewMatrix());
			matrix.orthoNormalize(matrix);

			int index = (rand() >> 3) % int (sizeof (m_shootingCollisions) / sizeof (m_shootingCollisions[0]));

			Group* const rootGroup = m_viewer->getSceneData()->asGroup();
			ref_ptr<MatrixTransform> transformNode = new MatrixTransform(matrix);	

			transformNode->addChild(m_shells[index].get());
			rootGroup->addChild(transformNode.get());

			newtonDynamicBody* const body = new newtonDynamicBody (world, 30.0f, m_shootingCollisions[index], transformNode.get(), matrix);

			const dFloat speed = SHOOTING_SPEED;
			Vec4 veloc (Matrix::transform3x3 (Vec3(0.0f, 0.0f, -speed), matrix), 1.0);   
			body->SetVeloc(veloc);
		}
	}
}
