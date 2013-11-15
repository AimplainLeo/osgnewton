
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
#include <Utils.h>
#include <DemoExample.h>
#include "LumberYard.h"


using namespace osg;

static Geode* CreateBoxMesh (osg::newtonWorld* const world, const Vec3& size)
{
	// create a texture and apply uv to this mesh
	ref_ptr<Texture2D> texture = new Texture2D;
	ref_ptr<Image> image = osgDB::readImageFile("images\\wood_2.tga");
	texture->setImage (image.get());
	texture->setWrap(Texture::WRAP_S, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_R, Texture::REPEAT);
	texture->setWrap(Texture::WRAP_T, Texture::REPEAT);

	dNewtonCollisionBox shape (world, size.x(), size.y(), size.z(), 0);

	// create a visual for visual representation
	newtonMesh boxMesh (&shape);
	boxMesh.Triangulate();
	int materialId = boxMesh.AddMaterial(texture);

	// apply uv to this mesh
	boxMesh.ApplyBoxMapping (materialId, materialId, materialId);

	// create a manual object for rendering 
	return boxMesh.CreateGeodeNode();
}


static void AddPart (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, ref_ptr<Geode> part, const Vec3& location, const Vec3& size, dFloat mass)
{
	Group* const rootGroup = viewer->getSceneData()->asGroup();

	Matrix matrix;
	matrix.setTrans (location);
	ref_ptr<MatrixTransform> transformNode = new MatrixTransform(matrix);	
	rootGroup->addChild(transformNode.get());

	// attach geometry to transform node 
	transformNode->addChild(part.get());

	// make a dynamic body
	dNewtonCollisionBox box (world, size.x(), size.y(), size.z(), DemoExample::m_all);
	new newtonDynamicBody (world, mass, &box, transformNode.get(), matrix);
}

void LumberYard (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const Vec3& location, dFloat mass, int high)
{
	dAssert (viewer->getSceneData());
	Group* const rootGroup = viewer->getSceneData()->asGroup();
	dAssert (rootGroup);

	// create the mesh parts
	Vec3 postSize(0.2f, 0.2f, 1.0f);
	ref_ptr<Geode> post = CreateBoxMesh (world, postSize);

	Vec3 plankSize(4.0f, 1.0f, 0.1f);
	ref_ptr<Geode> plank = CreateBoxMesh (world, plankSize);

	Vec3 lumberSize(plankSize.x() + 0.2, 0.2f, 0.2f);
	ref_ptr<Geode> lumber = CreateBoxMesh (world, lumberSize);

	Vec3 slackSize(0.1f, plankSize.y() + 0.1f, 0.1f);
	ref_ptr<Geode> slack = CreateBoxMesh (world, slackSize);

	dFloat width = postSize.x();
	// place four post
	AddPart (viewer, world, post, location + Vec3(-plankSize.x() + width, -plankSize.y() + width, postSize.z()) * 0.5f, postSize, mass * 2.0f);
	AddPart (viewer, world, post, location + Vec3(-plankSize.x() + width,  plankSize.y() - width, postSize.z()) * 0.5f, postSize, mass * 2.0f);
	AddPart (viewer, world, post, location + Vec3( plankSize.x() - width, -plankSize.y() + width, postSize.z()) * 0.5f, postSize, mass * 2.0f);
	AddPart (viewer, world, post, location + Vec3( plankSize.x() - width,  plankSize.y() - width, postSize.z()) * 0.5f, postSize, mass * 2.0f);

	// place the plank on top
	AddPart (viewer, world, plank, location + Vec3 (0.0f,  0.0f, postSize.z() + width * 0.5f), plankSize, mass);

	// stack the lumber
	dFloat z0 = postSize.z() + plankSize.z();
	for (int i = 0; i < high; i ++) {
		AddPart (viewer, world, lumber, location + Vec3 (0.0f, (- plankSize.y() + lumberSize.y()) * 0.5f, z0 + lumberSize.z() * 0.5f), lumberSize, mass * 0.5f);
		AddPart (viewer, world, lumber, location + Vec3 (0.0f, (  plankSize.y() - lumberSize.y()) * 0.5f, z0 + lumberSize.z() * 0.5f), lumberSize, mass * 0.5f);

		z0 += lumberSize.z();
		dFloat x0 = (-lumberSize.x() + slackSize.x()) * 0.5f;
		dFloat dx = (lumberSize.x() - slackSize.x()) / 4.0f;
		for (int j = 0; j <= 4; j ++) {
			AddPart (viewer, world, slack, location + Vec3 (x0, 0.0f, z0 + slackSize.z() * 0.5f), slackSize, mass * 0.25);
			x0 += dx;
		}
		z0 += slackSize.z();
	}
}



