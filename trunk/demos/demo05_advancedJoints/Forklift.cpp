
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
#include "Forklift.h"

using namespace osg;

class ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION
{
	public:
	char m_boneName[32];
	char m_shapeTypeName[32];
	dFloat m_mass;
	dFloat m_shapeAligmentAngle;
};


static ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION forkliftDefinition[] =
{
	{"body",		"convexHull",			900.0f,  0.0f},
	{"fr_tire",		"tireShape",			 50.0f, 90.0f},
	{"fl_tire",		"tireShape",			 50.0f, 90.0f},
	{"rr_tire",		"tireShape",			 50.0f, 90.0f},
	{"rl_tire",		"tireShape",			 50.0f, 90.0f},
	{"lift_1",		"convexHull",			 50.0f, 90.0f},
	{"lift_2",		"convexHull",			 40.0f, 90.0f},
	{"lift_3",		"convexHull",			 30.0f, 90.0f},
	{"lift_4",		"convexHull",			 20.0f, 90.0f},
	{"right_teeth", "convexHullAggregate",	 10.0f, 90.0f},
	{"left_teeth",  "convexHullAggregate",	 10.0f, 90.0f},
};													


class ForkliftPhysicsModel::ForkliftTireBody: public newtonDynamicBody
{
	public:
	ForkliftTireBody (newtonWorld* const world, dFloat mass, const dNewtonCollision* const collision, MatrixTransform* const node, const Matrix& location)
		:newtonDynamicBody (world, mass, collision, node, location)
	{
	}

	virtual void OnContactProcess (dNewtonContactMaterial* const contactMaterial, dFloat timestep, int threadIndex) const
	{
		newtonDynamicBody* const rootBody = (newtonDynamicBody*)GetParent();

		Matrix tireMatrix (GetMatrix());
		Matrix chassis (rootBody->GetMatrix());

		Vec3 upPin (Matrix::transform3x3 (Vec3(0.0f, 0.0f, 1.0f), chassis));
		Vec3 axisPin (Matrix::transform3x3 (Vec3(1.0f, 0.0f, 0.0f), tireMatrix));
		Vec3 dir (axisPin ^ upPin);
		for (void* contact = contactMaterial->GetFirstContact(); contact; contact = contactMaterial->GetNextContact(contact)) {
			contactMaterial->RotateTangentDirections (contact, dir.ptr());
		}
	}
};


class ForkliftPhysicsModel::TraverseNode: public NodeVisitor 
{
	public:
	TraverseNode(ForkliftPhysicsModel* const me, newtonWorld* const world)
		:NodeVisitor (NodeVisitor::TRAVERSE_ALL_CHILDREN)
		,m_world(world)
		,m_model (me)
		,m_parentBone(NULL)
		,m_parentNode(NULL)
		,m_currentNode(NULL)
		,m_currentBody(NULL)
	{
	}

	ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION* FindDefintionRecord (const char* const name)
	{
		int count = sizeof (forkliftDefinition) / sizeof (forkliftDefinition[0]);
		for (int i = 0; i < count; i ++) {
			if (!strcmp (forkliftDefinition[i].m_boneName, name)) {
				return &forkliftDefinition[i];
			}
		}
		return NULL;
	}

	newtonDynamicBody* CreateBodyPart (Transform* const node, const ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION* const definition)
	{
		dAssert (node->asMatrixTransform());
		const NodePath& path = getNodePath();
		Matrix matrix (computeLocalToWorld(path));

		newtonDynamicBody* body = NULL;
		dNewtonCollisionNull placeHolderShape(m_world);
		if (strcmp (definition->m_shapeTypeName, "tireShape")) {
			body = new newtonDynamicBody (m_world, definition->m_mass, &placeHolderShape, node->asMatrixTransform(), matrix);
		} else {
			body = new ForkliftTireBody (m_world, definition->m_mass, &placeHolderShape, node->asMatrixTransform(), matrix);
		}
		return body;
	}

	virtual void apply(Geode& node)
	{
		const std::string name (node.getName().substr(0, node.getName().find("-GEODE")));
		ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION* const definition = FindDefintionRecord (name.c_str());
		if (definition) {
			const NodePath& path = getNodePath();

			if (!strcmp (definition->m_shapeTypeName, "tireShape")) {
				const BoundingBox& box = node.getBoundingBox();
				Vec3 size (box._max - box._min);
				dFloat height = size.y();
				dFloat radius = size.x() * 0.5f - height * 0.5f;
				dNewtonCollisionChamferedCylinder wheel (m_world, radius, height, DemoExample::m_allExcludingMousePick);

				Matrix matrix (Quat (90.0f * 3.141592f / 180.0f, Vec3 (0.0f, 0.0f, 1.0f)));
				wheel.SetMatrix(&dMatrix(matrix.ptr())[0][0]);
				m_currentBody->SetCollision (&wheel);

			} else if (!strcmp (definition->m_shapeTypeName, "convexHull")) {
				Matrix matrix;
				for (int i = path.size() - 1; (i >= 0) && (path[i] != m_currentNode); i --) {
					if (path[i]->asTransform()) {
						MatrixTransform* const transformNode = path[i]->asTransform()->asMatrixTransform();
						matrix = matrix * transformNode->getMatrix();
					}
				}
				newtonMesh convexMesh (m_world, &node);
				convexMesh.ApplyTransform(&dMatrix(matrix.ptr())[0][0]);

				dNewtonCollisionConvexHull convexHull (m_world, convexMesh, DemoExample::m_allExcludingMousePick);
				//dMatrix localMatrix;
				//convexHull.GetMatrix(&localMatrix[0][0]);
				//matrix = Matrix (&localMatrix[0][0]) * matrix;
				//convexHull.SetMatrix(&dMatrix(matrix.ptr())[0][0]);
				m_currentBody->SetCollision (&convexHull);

			} else if (!strcmp (definition->m_shapeTypeName, "convexHullAggregate")) {
				Matrix matrix;
				for (int i = path.size() - 1; (i >= 0) && (path[i] != m_currentNode); i --) {
					if (path[i]->asTransform()) {
						MatrixTransform* const transformNode = path[i]->asTransform()->asMatrixTransform();
						matrix = matrix * transformNode->getMatrix();
					}
				}
				newtonMesh mesh (m_world, &node);
				mesh.ApplyTransform(&dMatrix(matrix.ptr())[0][0]);

				newtonMesh convexAproximation (m_world);
				convexAproximation.CreateApproximateConvexDecomposition(mesh, 0.01f, 0.2f, 32, 100);
				dNewtonCollisionCompound compoundShape (m_world, convexAproximation, DemoExample::m_allExcludingMousePick);
				m_currentBody->SetCollision (&compoundShape);

			} else {
				dAssert (0);
			}

			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;
			m_currentBody->GetMassAndInertia(mass, Ixx, Iyy, Izz);
			m_currentBody->SetMassAndInertia(mass, m_currentBody->GetCollision());
		}
		traverse (node);
	}

	virtual void apply(Transform& node)
	{
		const std::string& name = node.getName();
		ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION* const definition = FindDefintionRecord (name.c_str());
		if (definition) {
			Matrix matrix;
			if (m_parentNode) {
				const NodePath& path = getNodePath();
				for (int i = path.size() - 1; (i >= 0) && (path[i] != m_parentNode); i --) {
					if (path[i]->asTransform()) {
						MatrixTransform* const transformNode = path[i]->asTransform()->asMatrixTransform();
						matrix = matrix * transformNode->getMatrix();
					}
				}
			}

			m_currentNode = node.asMatrixTransform();
			newtonDynamicBody* const body = CreateBodyPart (&node, definition);
			m_currentBody = body;

			void* const bone = m_model->AddBone (body, &dMatrix (matrix.ptr())[0][0], m_parentBone);

			void* const saveParent = m_parentBone;
			Transform* const saveParentNode = m_parentNode;
			m_parentBone = bone;
			m_parentNode = &node;
			traverse (node);
			m_parentBone = saveParent;
			m_parentNode = saveParentNode;
		} else {
			traverse (node);
		}
	}

	newtonWorld* m_world;
	ForkliftPhysicsModel* m_model;
	void* m_parentBone;
	Transform* m_parentNode;
	MatrixTransform* m_currentNode;
	newtonDynamicBody* m_currentBody;
};


ForkliftPhysicsModel::ForkliftPhysicsModel (osgViewer::Viewer* const viewer, newtonWorld* const world, const char* const fileName, const Vec3& origin)
	:newtonArticulationManager::articulatedTransformController (world->GetHierarchyTransformManager(), true)
	,m_liftPosit(0.0f)
	,m_openPosit(0.0f)
	,m_tiltAngle(0.0f)
	,m_maxEngineTorque(0.0f)
	,m_omegaResistance(0.0f)
{
	dAssert (viewer->getSceneData());
	Group* const rootGroup = viewer->getSceneData()->asGroup();
	dAssert (rootGroup);

	// load the mesh
	ref_ptr<Node> vehicleNode = osgDB::readNodeFile(fileName);
	osg::Node* const rootNode = FindNodeByName (std::string (forkliftDefinition[0].m_boneName), vehicleNode.get());
	dAssert (rootNode);

	// place vehicle in the scene
	Matrix matrix;
	matrix.setTrans(origin);
	rootNode->asTransform()->asMatrixTransform()->setMatrix(matrix);

	// add the node to the scene root node
	rootGroup->addChild(vehicleNode.get());

	// traverse the hierarchy and create all bodies
	TraverseNode traverse (this, world);
	rootNode->accept(traverse);

	int boneCount = GetBoneCount();
	for (int i = 0; i < boneCount; i ++) {
		void* const bone = GetBone(i);
		newtonDynamicBody* const body = (newtonDynamicBody*) GetBoneBody (bone);
		MatrixTransform* const transformNode = (MatrixTransform*)body->GetUserData();

		if (transformNode->getName() == "body") {
			// move the center of mass a little to the back	of the chassis
			Vec4 cog(body->GetCOG());
			//com.z += 0.5f;
			cog.z() -= 0.2f;
			body->SetCOG (cog);
		} else if (transformNode->getName() == "fr_tire") {
			m_frontTire[0] = LinkFrontTire (body);
		} else if (transformNode->getName() == "fl_tire") {
			m_frontTire[1] = LinkFrontTire (body);
		} else if (transformNode->getName() == "rr_tire") {
			m_rearTire[0] = LinkRearTire (body);
		} else if (transformNode->getName() == "rl_tire") {
			m_rearTire[1] = LinkRearTire (body);
		} else if (transformNode->getName() == "lift_1") {
			m_revolvePlatform = LinkBasePlatform (body);
		} else if (transformNode->getName() == "lift_2") {
			m_slidePlaforms[0] = LinkLiftPlatform (body);
		} else if (transformNode->getName() == "lift_3") {
			m_slidePlaforms[1] = LinkLiftPlatform (body);
		} else if (transformNode->getName() == "lift_4") {
			m_slidePlaforms[2] = LinkLiftPlatform (body);
		} else if (transformNode->getName() == "right_teeth") {
			m_slideTooth[0] = LinkTooth (body, 1.0f);
		} else if (transformNode->getName() == "left_teeth") {
			m_slideTooth[1] = LinkTooth (body, 0.0f);
		}
	}

	// links the two front tire with a relational joint to add as a differential to regulate
	// the angular velocity
	//newtonDynamicBody* const rootBody = (newtonDynamicBody*) GetBoneBody (GetBone(0));
	//newtonDynamicBody* const leftTire = (newtonDynamicBody*) m_frontTire[0]->GetBody0();
	//newtonDynamicBody* const rightTire = (newtonDynamicBody*) m_frontTire[1]->GetBody0();
	//Vec3 pin0 (Matrix::transform3x3 (Vec3(1.0f, 0.0f, 0.0f), rootBody->GetMatrix()));
	//Vec3 pin1 (pin0 * (-1.0f));
	//new dNewtonGearJoint (1.0f, pin0.ptr(), leftTire, pin1.ptr(), rightTire);

	// calculate a fake engine 
	CalculateEngine ((newtonDynamicBody*)m_frontTire[0]->GetBody0());

	// disable self collision between all body parts
	DisableAllSelfCollision();
}

ForkliftPhysicsModel::~ForkliftPhysicsModel()
{
}

dNewtonHingeJoint* ForkliftPhysicsModel::LinkFrontTire (newtonDynamicBody* const tire)  
{
	newtonDynamicBody* const parent = (newtonDynamicBody*)tire->GetParent();

	Matrix tireMatrix(tire->GetMatrix());
	Matrix axisMatrix(parent->GetMatrix());
	axisMatrix.setTrans(tireMatrix.getTrans());

	dNewtonHingeJoint* const joint = new dNewtonHingeJoint (&dMatrix(axisMatrix.ptr())[0][0], tire, parent);
	return joint;
}

dNewtonUniversalActuator* ForkliftPhysicsModel::LinkRearTire (newtonDynamicBody* const tire)  
{
	newtonDynamicBody* const parent = (newtonDynamicBody*)tire->GetParent();

	Matrix tireMatrix(tire->GetMatrix());
	Matrix axisMatrix(Matrix (Quat (90.0f * 3.141592f / 180.0f, Vec3 (1.0f, 0.0f, 0.0f))) * parent->GetMatrix());
	axisMatrix.setTrans(tireMatrix.getTrans());

	dFloat angleLimit = 30.0f * 3.141592f / 180.0f;
	dFloat angularRate = 60.0f * 3.141592f / 180.0f;
	dNewtonUniversalActuator* const joint = new dNewtonUniversalActuator (&dMatrix(axisMatrix.ptr())[0][0], angularRate, -angleLimit, angleLimit, angularRate, -angleLimit, angleLimit, tire, parent); 

	// disable the limits of the first row, so that it can spin free
	joint->SetEnableFlag0 (false);
	return joint;
}


dNewtonHingeActuator* ForkliftPhysicsModel::LinkBasePlatform (newtonDynamicBody* const platform)  
{
	newtonDynamicBody* const parent = (newtonDynamicBody*)platform->GetParent();

	Matrix platformMatrix(platform->GetMatrix());
	Matrix axisMatrix(parent->GetMatrix());
	axisMatrix.setTrans(platformMatrix.getTrans());

	dFloat minAngleLimit = -20.0f * 3.141592f / 180.0f;
	dFloat maxAngleLimit =  30.0f * 3.141592f / 180.0f;
	dFloat angularRate = 10.0f * 3.141592f / 180.0f;
	return new dNewtonHingeActuator (&dMatrix(axisMatrix.ptr())[0][0], angularRate, minAngleLimit, maxAngleLimit, platform, parent);
}

dNewtonSliderActuator* ForkliftPhysicsModel::LinkLiftPlatform (newtonDynamicBody* const platform)  
{
	newtonDynamicBody* rootBody = platform;
	while (rootBody->GetParent()) {
		rootBody = (newtonDynamicBody*)rootBody->GetParent();
	}
	newtonDynamicBody* const parent = (newtonDynamicBody*)platform->GetParent();

	Matrix aligmentMatrix (Quat (-90.0f * 3.141592f / 180.0f, Vec3 (0.0f, 1.0f, 0.0f)));
	Matrix platformMatrix(platform->GetMatrix());

	Matrix axisMatrix(aligmentMatrix * rootBody->GetMatrix());
	axisMatrix.setTrans(platformMatrix.getTrans());

	return new dNewtonSliderActuator (&dMatrix(axisMatrix.ptr())[0][0], 0.1f, -0.25f, 1.5f, platform, parent);
}

dNewtonSliderActuator* ForkliftPhysicsModel::LinkTooth (newtonDynamicBody* const platform, dFloat dir)
{
	newtonDynamicBody* rootBody = platform;
	while (rootBody->GetParent()) {
		rootBody = (newtonDynamicBody*)rootBody->GetParent();
	}
	newtonDynamicBody* const parent = (newtonDynamicBody*)platform->GetParent();

	Matrix aligmentMatrix (Quat (dir * 180.0f * 3.141592f / 180.0f, Vec3 (0.0f, 0.0f, 1.0f)));
	Matrix axisMatrix(aligmentMatrix * rootBody->GetMatrix());
	Matrix platformMatrix (platform->GetMatrix());

	axisMatrix.setTrans(platformMatrix.getTrans());
	return new dNewtonSliderActuator (&dMatrix(axisMatrix.ptr())[0][0], 0.2f, -0.25f, 0.25f, platform, parent);
}


void ForkliftPhysicsModel::ApplyInputs(const InputRecored& inputs)
{
	m_inputRecored = inputs;
}


void ForkliftPhysicsModel::CalculateEngine(newtonDynamicBody* const tire)
{
	// calculate the maximum torque that the engine will produce
	dNewtonCollision* const tireShape = tire->GetCollision();
	newtonDynamicBody* const rootBody = (newtonDynamicBody*)tire->GetParent();
	newtonWorld* const world = (newtonWorld*) rootBody->GetNewton();

	dAssert (tireShape->GetType() == dNewtonCollision::m_chamferedCylinder);

	Vec3 p0;
	Vec3 p1;
	Matrix matrix;
	tireShape->CalculateAABB (&dMatrix(matrix.ptr())[0][0], &p0.x(), &p1.x());

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	rootBody->GetMassAndInertia (mass, Ixx, Iyy, Izz);

	const Vec4 gravity (world->GetGravity());

	dFloat radius = (p1.z() - p0.z()) * 0.5f;

	// calculate a torque the will produce a 0.5f of the force of gravity
	m_maxEngineTorque = 0.25f * mass * radius * gravity.length();

	// calculate the coefficient of drag for top speed of 20 m/s
	dFloat maxOmega = 200.0f / radius;
	m_omegaResistance = 1.0f / maxOmega;
}


void ForkliftPhysicsModel::OnUpdateBoneTransform (dNewtonBody* const bone, const dFloat* const localMatrix)
{
	// do nothing for now 
}


void ForkliftPhysicsModel::OnPreUpdate (dFloat timestep)
{
	// apply engine torque
	dFloat brakeTorque = 0.0f;
	dFloat engineTorque = 0.0f;
	if (m_inputRecored.m_throtler > 0) {
		engineTorque = -m_maxEngineTorque; 
	} else if (m_inputRecored.m_throtler < 0) {
		engineTorque = m_maxEngineTorque; 
	} else {
		brakeTorque = 1000.0f;
	}
	m_frontTire[0]->SetFriction(brakeTorque);
	m_frontTire[1]->SetFriction(brakeTorque);

	void* const bone = GetBone(0);
	newtonDynamicBody* const rootBody = (newtonDynamicBody*) GetBoneBody (bone);
	Vec3 tirePin (Matrix::transform3x3 (Vec3(1.0f, 0.0f, 0.0f), rootBody->GetMatrix()));
	if (engineTorque != 0.0f) {
		Vec4 torque (tirePin * engineTorque, 0.0f);
		((newtonDynamicBody*)m_frontTire[0]->GetBody0())->AddTorque (torque);
		((newtonDynamicBody*)m_frontTire[1]->GetBody0())->AddTorque (torque);
	}

	for (int i = 0; i < 2; i ++) {
		newtonDynamicBody* const body = (newtonDynamicBody*)m_frontTire[i]->GetBody0();
		Vec4 omega(body->GetOmega());
		dFloat omegaMag = omega * tirePin;
		dFloat sign = (omegaMag >= 0.0f) ? 1.0 : -1.0f;
		omega -= Vec4(tirePin * (sign * omegaMag * omegaMag * m_omegaResistance), 0.0f);
		body->SetOmega(omega);
	}

	// apply steering control
	dFloat steeringAngle = m_rearTire[0]->GetActuatorAngle1();
	if (m_inputRecored.m_steering < 0) {
		steeringAngle = m_rearTire[0]->GetMinAngularLimit1(); 
	} else if (m_inputRecored.m_steering > 0) {
		steeringAngle = m_rearTire[0]->GetMaxAngularLimit1(); 
	}
	m_rearTire[0]->SetTargetAngle1(steeringAngle);
	m_rearTire[1]->SetTargetAngle1(steeringAngle);

	// control tilt angle
	dFloat tiltAngle = m_tiltAngle;
	if (m_inputRecored.m_tilt > 0) {
		tiltAngle = m_revolvePlatform->GetMinAngularLimit();
		m_tiltAngle = m_revolvePlatform->GetActuatorAngle();
	} else if (m_inputRecored.m_tilt < 0) {
		tiltAngle = m_revolvePlatform->GetMaxAngularLimit();
		m_tiltAngle = m_revolvePlatform->GetActuatorAngle();
	}
	m_revolvePlatform->SetTargetAngle (tiltAngle);

	// control lift position
	dFloat liftPosit= m_liftPosit;
	if (m_inputRecored.m_lift > 0) {
		liftPosit = m_slidePlaforms[0]->GetMinPositLimit();
		m_liftPosit = m_slidePlaforms[0]->GetActuatorPosit();
	} else if (m_inputRecored.m_lift < 0) {
		liftPosit = m_slidePlaforms[0]->GetMaxPositLimit();
		m_liftPosit = m_slidePlaforms[0]->GetActuatorPosit();
	}
	for (int i = 0; i < sizeof (m_slidePlaforms) / sizeof (m_slidePlaforms[0]); i ++) {
		m_slidePlaforms[i]->SetTargetPosit(liftPosit);
	}

	// control teeth position
	dFloat toothPosit = m_slideTooth[0]->GetActuatorPosit();
	if (m_inputRecored.m_palette > 0) {
		toothPosit = m_slideTooth[0]->GetMinPositLimit();
		m_openPosit = m_slideTooth[0]->GetActuatorPosit();
	} else if (m_inputRecored.m_palette < 0) {
		toothPosit = m_slideTooth[0]->GetMaxPositLimit();
		m_openPosit = m_slideTooth[0]->GetActuatorPosit();
	}
	for (int i = 0; i < sizeof (m_slideTooth) / sizeof (m_slideTooth[0]); i ++) {
		m_slideTooth[i]->SetTargetPosit(toothPosit);
	}

}
