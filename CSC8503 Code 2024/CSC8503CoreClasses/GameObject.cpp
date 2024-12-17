#include "GameObject.h"
#include "CollisionDetection.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "NetworkObject.h"
#include "Camera.h"
#include "Window.h"


using namespace NCL::CSC8503;

GameObject::GameObject(const std::string& objectName)	{
	name			= objectName;
	worldID			= -1;
	isActive		= true;
	boundingVolume	= nullptr;
	physicsObject	= nullptr;
	renderObject	= nullptr;
	networkObject	= nullptr;
}

GameObject::~GameObject()	{
	delete boundingVolume;
	delete physicsObject;
	delete renderObject;
	delete networkObject;
}

bool GameObject::GetBroadphaseAABB(Vector3&outSize) const {
	if (!boundingVolume) {
		return false;
	}
	outSize = broadphaseAABB;
	return true;
}

void GameObject::UpdateBroadphaseAABB() {
	if (!boundingVolume) {
		return;
	}
	if (boundingVolume->type == VolumeType::AABB) {
		broadphaseAABB = ((AABBVolume&)*boundingVolume).GetHalfDimensions();
	}
	else if (boundingVolume->type == VolumeType::Sphere) {
		float r = ((SphereVolume&)*boundingVolume).GetRadius();
		broadphaseAABB = Vector3(r, r, r);
	}
	else if (boundingVolume->type == VolumeType::OBB) {
		Matrix3 mat = Quaternion::RotationMatrix<Matrix3>(transform.GetOrientation());
		mat = Matrix::Absolute(mat);
		Vector3 halfSizes = ((OBBVolume&)*boundingVolume).GetHalfDimensions();
		broadphaseAABB = mat * halfSizes;
	}
}


PlayerObject::~PlayerObject() {
}

void PlayerObject::UpdateMovement(float dt) {
	if (!playerController) {
		return;
	}
	pYaw -= playerController->GetNamedAxis("XLook");

	if (pYaw < 0) {
		pYaw += 360.0f;
	}
	if (pYaw > 360.0f) {
		pYaw -= 360.0f;
	}
	Quaternion yawRotation = Quaternion::Quaternion(Matrix::Rotation(pYaw, Vector3(0, 1, 0)));
	
	/**
	Matrix3 yawRotation = Matrix::RotationMatrix3x3(pYaw, Vector3(0, 1, 0));
	*/
	this->GetTransform().SetOrientation(yawRotation);
	
	Ray r = Ray(this->GetTransform().GetPosition(), Vector3(0.0f, -1.0f, 0.0f));
	RayCollision collision;
	bool collides = rayHit(r, collision, true, this);
	bool noJump = false;
	if (collision.rayDistance > 5.0f + FLT_EPSILON) { noJump = true; }

	Vector3 jump = Vector3(0.0f, 12.0f, 0.0f);
	Vector3 movement = yawRotation * Vector3(0, 0, playerController->GetNamedAxis("Forward")) * speed*dt;
	movement += yawRotation * Vector3(-playerController->GetNamedAxis("Sidestep"), 0, 0) * speed*dt;
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::SPACE) && noJump==false) {
		movement += jump;
	}
	this->GetPhysicsObject()->ApplyLinearImpulse(movement);
}