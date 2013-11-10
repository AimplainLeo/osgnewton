
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
	enum
	{
		m_tireID = 1<<0,
		m_bodyPart = 2<<0,
	};

	char m_boneName[32];
	char m_shapeTypeName[32];
	dFloat m_mass;
	dFloat m_algimentAngle;
	int m_bodyPartID;
	char m_articulationName[32];
};


static ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION forkliftDefinition[] =
{
	{"body",		"convexHull",			900.0f,  0.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "mainBody"},
	{"fr_tire",		"tireShape",			 50.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_tireID, "frontTire"},
	{"fl_tire",		"tireShape",			 50.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_tireID, "frontTire"},
	{"rr_tire",		"tireShape",			 50.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_tireID, "rearTire"},
	{"rl_tire",		"tireShape",			 50.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_tireID, "rearTire"},
//	{"lift_1",		"convexHull",			 50.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "hingeActuator"},
//	{"lift_2",		"convexHull",			 40.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
//	{"lift_3",		"convexHull",			 30.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
//	{"lift_4",		"convexHull",			 20.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
//	{"left_teeth",  "convexHullAggregate",	 10.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "paletteActuator"},
//	{"right_teeth", "convexHullAggregate",	 10.0f, 90.0f, ForkliftPhysicsModel::ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "paletteActuator"},
};													


class ForkliftPhysicsModel::ForkliftTireBody: public newtonDynamicBody
{
	public:
	ForkliftTireBody (newtonWorld* const world, dFloat mass, const dNewtonCollision* const collision, MatrixTransform* const node, const Matrix& location)
		:newtonDynamicBody (world, mass, collision, node, location)
//		,m_rootBody(rootBody)
	{
	}

	virtual void OnContactProcess (dNewtonContactMaterial* const contactMaterial, dFloat timestep, int threadIndex) const
	{
/*
		Matrix4 tireMatrix (GetMatrix());
		Matrix4 chassis (m_rootBody->GetMatrix());

		chassis.setTrans (Vector3 (0.0f, 0.0f, 0.0f));
		tireMatrix.setTrans (Vector3 (0.0f, 0.0f, 0.0f));
		Vector3 upPin (chassis * Vector3 (0.0f, 1.0f, 0.0f));
		Vector3 axisPin (chassis * Vector3 (1.0f, 0.0f, 0.0f));

		Vector3 dir (axisPin.crossProduct (upPin));
		for (void* contact = contactMaterial->GetFirstContact(); contact; contact = contactMaterial->GetNextContact(contact)) {
			contactMaterial->RotateTangentDirections (contact, &dir.x);
		}
*/
	}
//	newtonDynamicBody* m_rootBody;
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
		//Vector3 com;
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


ForkliftPhysicsModel::ForkliftPhysicsModel (osgViewer::Viewer* const viewer, osg::newtonWorld* const world, const char* const fileName, const Vec3& origin)
	:newtonArticulationManager::articulatedTransformController (world->GetHierarchyTransformManager(), true)
//	,m_application(application)
//	,m_liftPosit(0.0f)
//	,m_openPosit(0.0f)
//	,m_tiltAngle(0.0f)
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
			m_frontTire[0] = LinkFrontTire (body);
		} else if (transformNode->getName() == "fl_tire") {
			m_frontTire[1] = LinkFrontTire (body);
		} else if (transformNode->getName() == "rr_tire") {
			m_rearTire[0] = LinkRearTire (body);
		} else if (transformNode->getName() == "rl_tire") {
			m_rearTire[1] = LinkRearTire (body);
		}
	}
/*
	// find all vehicle components
	SceneNode* const base1Node = (SceneNode*) bodyNode->getChild (rootName + "lift_1");
	SceneNode* const base2Node = (SceneNode*) base1Node->getChild (rootName + "lift_2");
	SceneNode* const base3Node = (SceneNode*) base2Node->getChild (rootName + "lift_3");
	SceneNode* const base4Node = (SceneNode*) base3Node->getChild (rootName + "lift_4");
	dAssert (base1Node);
	dAssert (base2Node);
	dAssert (base3Node);
	dAssert (base4Node);

	SceneNode* const leftTeethNode = (SceneNode*) base4Node->getChild (rootName + "left_teeth");
	SceneNode* const rightTeethNode = (SceneNode*) base4Node->getChild (rootName + "right_teeth");
	dAssert (leftTeethNode);
	dAssert (leftTeethNode);

	//convert the body part to rigid bodies
	Matrix4 bindMatrix (Matrix4::IDENTITY);
	m_rootBody = CreateRootBody (bodyNode, origin);
	void* const parentBone = AddBone (m_rootBody, &bindMatrix[0][0], NULL);

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
	m_revolvePlatform = LinkBasePlatform (base1);
	m_slidePlaforms[0] = LinkBasePlatform (base1, base2);
	m_slidePlaforms[1] = LinkBasePlatform (base2, base3);
	m_slidePlaforms[2] = LinkBasePlatform (base3, base4);

	// add the teeth bode
	AddBone (leftTooth, &bindMatrix[0][0], base4Bone);
	AddBone (rightTooth, &bindMatrix[0][0], base4Bone);
	// connect the teeth
	m_slideTooth[0] = LinkTooth (base4, leftTooth, 1.0f);
	m_slideTooth[1] = LinkTooth (base4, rightTooth, -1.0f);

	// calculate a fake engine 
	CalculateEngine (m_frontTireBody[0]);
*/
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

//	Matrix tireMatrix;
//	Matrix matrixOffset;
//	tire->GetCollision()->GetMatrix(&matrixOffset[0][0]);
//	tireMatrix = (tire->GetMatrix() * matrixOffset.transpose()).transpose();
	dFloat angleLimit = 30.0f * 3.141592f / 180.0f;
	dFloat angularRate = 60.0f * 3.141592f / 180.0f;
//	dNewtonUniversalActuator* const joint = new dNewtonUniversalActuator (&tireMatrix[0][0], angularRate, -angleLimit, angleLimit, angularRate, -angleLimit, angleLimit, tire, m_rootBody);

	Matrix tireMatrix(tire->GetMatrix());
	Matrix axisMatrix(parent->GetMatrix());
	axisMatrix.setTrans(tireMatrix.getTrans());

	dNewtonUniversalActuator* const joint = new dNewtonUniversalActuator (&dMatrix(axisMatrix.ptr())[0][0], angularRate, -angleLimit, angleLimit, angularRate, -angleLimit, angleLimit, tire, parent); 

	// disable the limits of the first row, so that it can spin free
	joint->SetEnableFlag0 (false);
	return joint;
}



/*
void* ForkliftPhysicsModel::AddBone (dNewtonBody* const bone, const dFloat* const bindMatrix, void* const parentBodne)
{	
	// add the bode to the controller
	void* const boneNode = OgreNewtonArticulationManager::OgreNewtonArticulatedTransformController::AddBone (bone, bindMatrix, parentBodne);

	// save the body handle as the used data pf the body collision for using in later in eh collision callback
	dNewtonCollision* const collision = bone->GetCollision();
	dAssert (!collision->GetUserData());
	collision->SetUserData (boneNode);

	return boneNode;
}




newtonDynamicBody* ForkliftPhysicsModel::CreateRootBody (SceneNode* const node, const Vector3& origin)
{
	Entity* const ent = (Entity*) node->getAttachedObject (0);
	Vector3 scale (node->getScale());
	OgreNewtonMesh bodyMesh (m_application->GetPhysics(), ent);
	bodyMesh.ApplyTransform (Vector3::ZERO, scale, Quaternion::IDENTITY);
	dNewtonCollisionConvexHull bodyCollision (m_application->GetPhysics(), bodyMesh, m_allExcludingMousePick);
	Matrix4 bodyMatrix;
	bodyMatrix.makeTransform (node->_getDerivedPosition() + origin, Vector3 (1.0f, 1.0f, 1.0f), node->_getDerivedOrientation());
	newtonDynamicBody* const body = new newtonDynamicBody (m_application->GetPhysics(), BODY_MASS, &bodyCollision, node, bodyMatrix);

	// move the center of mass a little to the back	of the chassis
	Vector3 com;
	body->GetCenterOfMass (&com.x);
	com.z += 0.5f;
	com.y -= 0.2f;
	body->SetCenterOfMass (&com.x);
	
	return body;
}


newtonDynamicBody* ForkliftPhysicsModel::CreateTireBody (SceneNode* const tireNode, const Vector3& origin)
{
	Entity* const ent = (Entity*) tireNode->getAttachedObject (0);
	Vector3 scale (tireNode->getScale());
	AxisAlignedBox box (ent->getBoundingBox());
	dFloat height = scale.z * (box.getMaximum().z - box.getMinimum().z);
	dFloat radius = scale.x * (box.getMaximum().x - box.getMinimum().x) * 0.5f - height * 0.5f;
	dNewtonCollisionChamferedCylinder shape (m_application->GetPhysics(), radius, height, m_allExcludingMousePick);

	Matrix4 aligmentMatrix (Quaternion (Radian (3.141592f * 0.5f), Vector3 (0.0f, 1.0f, 0.0f)));
	aligmentMatrix = aligmentMatrix.transpose();
	shape.SetMatrix (&aligmentMatrix[0][0]);

	Matrix4 matrix;
	matrix.makeTransform(tireNode->_getDerivedPosition() + origin, Vector3 (1.0f, 1.0f, 1.0f), tireNode->_getDerivedOrientation());
	return new ForkliftTireBody (m_application->GetPhysics(), TIRE_MASS, &shape, tireNode, matrix, m_rootBody);
}

newtonDynamicBody* ForkliftPhysicsModel::CreateBasePlatform (SceneNode* const baseNode, const Vector3& origin)
{
	Entity* const ent = (Entity*) baseNode->getAttachedObject (0);
	Vector3 scale (baseNode->getScale());
	OgreNewtonMesh bodyMesh (m_application->GetPhysics(), ent);
	bodyMesh.ApplyTransform (Vector3::ZERO, scale, Quaternion::IDENTITY);
	dNewtonCollisionConvexHull collision (m_application->GetPhysics(), bodyMesh, m_allExcludingMousePick);

	Matrix4 matrix;
	matrix.makeTransform(baseNode->_getDerivedPosition() + origin, Vector3 (1.0f, 1.0f, 1.0f), baseNode->_getDerivedOrientation());
	return new newtonDynamicBody (m_application->GetPhysics(), RACK_MASS, &collision, baseNode, matrix);
}


newtonDynamicBody* ForkliftPhysicsModel::CreateTooth (SceneNode* const baseNode, const Vector3& origin)
{
	Entity* const ent = (Entity*) baseNode->getAttachedObject (0);
	Vector3 scale (baseNode->getScale());
	OgreNewtonMesh mesh (m_application->GetPhysics(), ent);
	mesh.ApplyTransform (Vector3::ZERO, scale, Quaternion::IDENTITY);

	OgreNewtonMesh convexAproximation (m_application->GetPhysics());
	convexAproximation.CreateApproximateConvexDecomposition(mesh, 0.01f, 0.2f, 32, 100);
	dNewtonCollisionCompound compoundShape (m_application->GetPhysics(), convexAproximation, m_allExcludingMousePick);

	Matrix4 matrix;
	matrix.makeTransform(baseNode->_getDerivedPosition() + origin, Vector3 (1.0f, 1.0f, 1.0f), baseNode->_getDerivedOrientation());
	return new newtonDynamicBody (m_application->GetPhysics(), TOOTH_MASS, &compoundShape, baseNode, matrix);
}

dNewtonHingeJoint* ForkliftPhysicsModel::LinkFrontTire (newtonDynamicBody* const tire)  
{
	Matrix4 tireMatrix(tire->GetMatrix());
	Matrix4 axisMatrix(m_rootBody->GetMatrix());

	axisMatrix.setTrans(tireMatrix.getTrans());
	axisMatrix = axisMatrix.transpose();
	dNewtonHingeJoint* const hinge = new dNewtonHingeJoint (&axisMatrix[0][0], tire, m_rootBody);
	return hinge;
}


dNewtonHingeActuator* ForkliftPhysicsModel::LinkBasePlatform (newtonDynamicBody* const platform)  
{
	Matrix4 aligmentMatrix (Quaternion (Radian (3.141592f * 0.5f), Vector3 (0.0f, 1.0f, 0.0f)));
	Matrix4 baseMatrix((platform->GetMatrix() * aligmentMatrix).transpose());

	dFloat minAngleLimit = -30.0f * 3.141592f / 180.0f;
	dFloat maxAngleLimit =  20.0f * 3.141592f / 180.0f;
	dFloat angularRate = 10.0f * 3.141592f / 180.0f;
	return new dNewtonHingeActuator (&baseMatrix[0][0], angularRate, minAngleLimit, maxAngleLimit, platform, m_rootBody);
}

dNewtonSliderActuator* ForkliftPhysicsModel::LinkBasePlatform (newtonDynamicBody* const parent, newtonDynamicBody* const platform)
{
	Matrix4 aligmentMatrix (Quaternion (Radian (3.141592f * 0.5f), Vector3 (0.0f, 0.0f, 1.0f)));
	Matrix4 baseMatrix((platform->GetMatrix() * aligmentMatrix).transpose());
	return new dNewtonSliderActuator (&baseMatrix[0][0], 0.1f, -0.25f, 1.5f, platform, parent);
}


dNewtonSliderActuator* ForkliftPhysicsModel::LinkTooth(newtonDynamicBody* const parent, newtonDynamicBody* const child, dFloat dir)
{
	Matrix4 aligmentMatrix (Quaternion (Radian (dir * 3.141592f * 0.5f), Vector3 (0.0f, 1.0f, 0.0f)));
	Matrix4 baseMatrix((child->GetMatrix() * aligmentMatrix).transpose());
	return new dNewtonSliderActuator (&baseMatrix[0][0], 0.25f, -0.25f, 0.25f, child, parent);
}

void ForkliftPhysicsModel::CalculateEngine(newtonDynamicBody* const tire)
{
	// calculate the maximum torque that the engine will produce
	dNewtonCollision* const tireShape = tire->GetCollision();
	dAssert (tireShape->GetType() == dNewtonCollision::m_chamferedCylinder);

	Vector3 p0;
	Vector3 p1;
	Matrix4 matrix (Matrix4::IDENTITY);
	tireShape->CalculateAABB (&matrix[0][0], &p0.x, &p1.x);

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	m_rootBody->GetMassAndInertia (mass, Ixx, Iyy, Izz);

	const Vector3& gravity = m_application->GetPhysics()->GetGravity();

	dFloat radius = (p1.y - p0.y) * 0.5f;

	// calculate a torque the will produce a 0.5f of the force of gravity
	m_maxEngineTorque = 0.25f * mass * radius * gravity.length();

	// calculate the coefficient of drag for top speed of 20 m/s
	dFloat maxOmega = 200.0f / radius;
	m_omegaResistance = 1.0f / maxOmega;
}


void ForkliftPhysicsModel::ApplyInputs(const InputRecored& inputs)
{
	m_inputRecored = inputs;
}
*/

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
	matrix.setTrans(Vector3::ZERO);

	Vector3 tirePing (matrix * Vector3(1.0f, 0.0f, 0.0f));
	if (engineTorque != 0.0f) {
		Vector3 torque (tirePing * engineTorque);
		m_frontTireBody[0]->AddTorque (torque);
		m_frontTireBody[1]->AddTorque (torque);
	}

	for (int i = 0; i < 2; i ++) {
		Vector3 omega(m_frontTireBody[i]->GetOmega());
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
