
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
//	{"lift_2",		"convexHull",			 40.0f, 90.0f},
//	{"lift_3",		"convexHull",			 30.0f, 90.0f},
//	{"lift_4",		"convexHull",			 20.0f, 90.0f},
//	{"left_teeth",  "convexHullAggregate",	 10.0f, 90.0f},
//	{"right_teeth", "convexHullAggregate",	 10.0f, 90.0f},
};													


class ForkliftPhysicsModel::ForkliftTireBody: public newtonDynamicBody
{
	public:
	ForkliftTireBody (newtonWorld* const world, dFloat mass, const dNewtonCollision* const collision, MatrixTransform* const node, const Matrix& location)
		:newtonDynamicBody (world, mass, collision, node, location)
		,m_rootBody(NULL)
	{
	}

	virtual void OnContactProcess (dNewtonContactMaterial* const contactMaterial, dFloat timestep, int threadIndex) const
	{
/*
		Matrix4 tireMatrix (GetMatrix());
		Matrix4 chassis (m_rootBody->GetMatrix());

		chassis.setTrans (Vec3 (0.0f, 0.0f, 0.0f));
		tireMatrix.setTrans (Vec3 (0.0f, 0.0f, 0.0f));
		Vec3 upPin (chassis * Vec3 (0.0f, 1.0f, 0.0f));
		Vec3 axisPin (chassis * Vec3 (1.0f, 0.0f, 0.0f));

		Vec3 dir (axisPin.crossProduct (upPin));
		for (void* contact = contactMaterial->GetFirstContact(); contact; contact = contactMaterial->GetNextContact(contact)) {
			contactMaterial->RotateTangentDirections (contact, &dir.x);
		}
*/
	}
	newtonDynamicBody* m_rootBody;
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

		// move the center of mass a little to the back	of the chassis
		//Vec3 com;
		//body->GetCenterOfMass (&com.x);
		//com.z += 0.5f;
		//com.y -= 0.2f;
		//body->SetCenterOfMass (&com.x);

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
				dNewtonCollisionConvexHull convexHull (m_world, convexMesh, DemoExample::m_allExcludingMousePick);
				dMatrix localMatrix;
				convexHull.GetMatrix(&localMatrix[0][0]);
				matrix = Matrix (&localMatrix[0][0]) * matrix;
				convexHull.SetMatrix(&dMatrix(matrix.ptr())[0][0]);
				m_currentBody->SetCollision (&convexHull);
			} else if (!strcmp (definition->m_shapeTypeName, "convexHullAggregate")) {
				dAssert (0);
//				shape = MakeConvexHullAggregate(bodyPart);
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
//	,m_application(application)
//	,m_liftPosit(0.0f)
//	,m_openPosit(0.0f)
//	,m_tiltAngle(0.0f)
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
	for (int i = 1; i < boneCount; i ++) {
		void* const bone = GetBone(i);
		newtonDynamicBody* const body = (newtonDynamicBody*) GetBoneBody (bone);
		MatrixTransform* const transformNode = (MatrixTransform*)body->GetUserData();
		if (transformNode->getName() == "fr_tire") {
			((ForkliftTireBody*)body)->m_rootBody = (newtonDynamicBody*)body->GetParent();
			m_frontTire[0] = LinkFrontTire (body);
		} else if (transformNode->getName() == "fl_tire") {
			((ForkliftTireBody*)body)->m_rootBody = (newtonDynamicBody*)body->GetParent();
			m_frontTire[1] = LinkFrontTire (body);
		} else if (transformNode->getName() == "rr_tire") {
			((ForkliftTireBody*)body)->m_rootBody = (newtonDynamicBody*)body->GetParent();
			m_rearTire[0] = LinkRearTire (body);
		} else if (transformNode->getName() == "rl_tire") {
			((ForkliftTireBody*)body)->m_rootBody = (newtonDynamicBody*)body->GetParent();
			m_rearTire[1] = LinkRearTire (body);
		} else if (transformNode->getName() == "lift_1") {
			m_revolvePlatform = LinkBasePlatform (body);
		}
	}
/*
	// find all vehicle components
	SceneNode* const base1Node = (SceneNode*) bodyNode->getChild (rootName + "lift_1");
	SceneNode* const base2Node = (SceneNode*) base1Node->getChild (rootName + "lift_2");
	SceneNode* const base3Node = (SceneNode*) base2Node->getChild (rootName + "lift_3");
	SceneNode* const base4Node = (SceneNode*) base3Node->getChild (rootName + "lift_4");
	SceneNode* const leftTeethNode = (SceneNode*) base4Node->getChild (rootName + "left_teeth");
	SceneNode* const rightTeethNode = (SceneNode*) base4Node->getChild (rootName + "right_teeth");
	dAssert (leftTeethNode);
	dAssert (leftTeethNode);

	// make the lift base
	newtonDynamicBody* const base1 = CreateBasePlatform (base1Node, origin);
	newtonDynamicBody* const base2 = CreateBasePlatform (base2Node, origin);
	newtonDynamicBody* const base3 = CreateBasePlatform (base3Node, origin);
	newtonDynamicBody* const base4 = CreateBasePlatform (base4Node, origin);

	// make the left and right palette teeth
	newtonDynamicBody* const leftTooth = CreateTooth (leftTeethNode, origin);
	newtonDynamicBody* const rightTooth = CreateTooth (rightTeethNode, origin);

	// add the lifter apparatus bones 
	void* const base1Bone = AddBone (base1, &bindMatrix[0][0], parentBone);
	void* const base2Bone = AddBone (base2, &bindMatrix[0][0], base1Bone);
	void* const base3Bone = AddBone (base3, &bindMatrix[0][0], base2Bone);
	void* const base4Bone = AddBone (base4, &bindMatrix[0][0], base3Bone);
	// connect the forklift base

	m_slidePlaforms[0] = LinkBasePlatform (base1, base2);
	m_slidePlaforms[1] = LinkBasePlatform (base2, base3);
	m_slidePlaforms[2] = LinkBasePlatform (base3, base4);

	// add the teeth bode
	AddBone (leftTooth, &bindMatrix[0][0], base4Bone);
	AddBone (rightTooth, &bindMatrix[0][0], base4Bone);
	// connect the teeth
	m_slideTooth[0] = LinkTooth (base4, leftTooth, 1.0f);
	m_slideTooth[1] = LinkTooth (base4, rightTooth, -1.0f);
*/
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
	Matrix axisMatrix(parent->GetMatrix());
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
//	Matrix aligmentMatrix (Quaternion (Radian (3.141592f * 0.5f), Vec3 (0.0f, 1.0f, 0.0f)));
//	Matrix baseMatrix((platform->GetMatrix() * aligmentMatrix).transpose());

	newtonDynamicBody* const parent = (newtonDynamicBody*)platform->GetParent();

	Matrix tireMatrix(platform->GetMatrix());
	Matrix axisMatrix(parent->GetMatrix());
	axisMatrix.setTrans(tireMatrix.getTrans());

	dFloat minAngleLimit = -30.0f * 3.141592f / 180.0f;
	dFloat maxAngleLimit =  20.0f * 3.141592f / 180.0f;
	dFloat angularRate = 10.0f * 3.141592f / 180.0f;
	return new dNewtonHingeActuator (&dMatrix(axisMatrix.ptr())[0][0], angularRate, minAngleLimit, maxAngleLimit, platform, parent);
}


/*
dNewtonSliderActuator* ForkliftPhysicsModel::LinkTooth(newtonDynamicBody* const parent, newtonDynamicBody* const child, dFloat dir)
{
	Matrix4 aligmentMatrix (Quaternion (Radian (dir * 3.141592f * 0.5f), Vec3 (0.0f, 1.0f, 0.0f)));
	Matrix4 baseMatrix((child->GetMatrix() * aligmentMatrix).transpose());
	return new dNewtonSliderActuator (&baseMatrix[0][0], 0.25f, -0.25f, 0.25f, child, parent);
}

void ForkliftPhysicsModel::ApplyInputs(const InputRecored& inputs)
{
	m_inputRecored = inputs;
}
*/

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
/*
	// apply steering control
	dFloat steeringAngle = m_rearTire[0]->GetActuatorAngle1();
	if (m_inputRecored.m_steering < 0) {
		steeringAngle = m_rearTire[0]->GetMinAngularLimit1(); 
	} else if (m_inputRecored.m_steering > 0) {
		steeringAngle = m_rearTire[0]->GetMaxAngularLimit1(); 
	}
	m_rearTire[0]->SetTargetAngle1(steeringAngle);
	m_rearTire[1]->SetTargetAngle1(steeringAngle);

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

	Matrix4 matrix(m_rootBody->GetMatrix());
	matrix.setTrans(Vec3::ZERO);

	Vec3 tirePing (matrix * Vec3(1.0f, 0.0f, 0.0f));
	if (engineTorque != 0.0f) {
		Vec3 torque (tirePing * engineTorque);
		m_frontTireBody[0]->AddTorque (torque);
		m_frontTireBody[1]->AddTorque (torque);
	}

	for (int i = 0; i < 2; i ++) {
		Vec3 omega(m_frontTireBody[i]->GetOmega());
		dFloat omegaMag = omega.dotProduct(tirePing);
		dFloat sign = (omegaMag >= 0.0f) ? 1.0 : -1.0f;
		omega -= tirePing * (sign * omegaMag * omegaMag * m_omegaResistance);
		m_frontTireBody[i]->SetOmega(omega);
	}


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
*/
}
