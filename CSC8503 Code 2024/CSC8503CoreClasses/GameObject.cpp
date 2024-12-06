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

void PlayerObject::UpdateMovement() {
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
	this->GetTransform().SetOrientation(yawRotation); //think this is wrong // Correction: This line breaks everything*/
}