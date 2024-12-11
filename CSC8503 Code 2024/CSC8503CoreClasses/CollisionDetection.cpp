#include "CollisionDetection.h"
#include "CollisionVolume.h"
#include "AABBVolume.h"
#include "OBBVolume.h"
#include "SphereVolume.h"
#include "Window.h"
#include "Maths.h"
#include "Debug.h"

using namespace NCL;

bool CollisionDetection::RayPlaneIntersection(const Ray& r, const Plane& p, RayCollision& collisions) {
	float ln = Vector::Dot(p.GetNormal(), r.GetDirection());

	if (ln == 0.0f) {
		return false; //direction vectors are perpendicular!
	}

	Vector3 planePoint = p.GetPointOnPlane();

	Vector3 pointDir = planePoint - r.GetPosition();

	float d = Vector::Dot(pointDir, p.GetNormal()) / ln;

	collisions.collidedAt = r.GetPosition() + (r.GetDirection() * d);

	return true;
}

bool CollisionDetection::RayIntersection(const Ray& r, GameObject& object, RayCollision& collision) {
	bool hasCollided = false;

	const Transform& worldTransform = object.GetTransform();
	const CollisionVolume* volume = object.GetBoundingVolume();

	if (!volume) {
		return false;
	}

	switch (volume->type) {
	case VolumeType::AABB:		hasCollided = RayAABBIntersection(r, worldTransform, (const AABBVolume&)*volume, collision); break;
	case VolumeType::OBB:		hasCollided = RayOBBIntersection(r, worldTransform, (const OBBVolume&)*volume, collision); break;
	case VolumeType::Sphere:	hasCollided = RaySphereIntersection(r, worldTransform, (const SphereVolume&)*volume, collision); break;

	case VolumeType::Capsule:	hasCollided = RayCapsuleIntersection(r, worldTransform, (const CapsuleVolume&)*volume, collision); break;
	}

	return hasCollided;
}

bool CollisionDetection::RayBoxIntersection(const Ray& r, const Vector3& boxPos, const Vector3& boxSize, RayCollision& collision) {
	Vector3 boxMin = boxPos - boxSize;
	Vector3 boxMax = boxPos + boxSize;

	Vector3 rayPos = r.GetPosition();
	Vector3 rayDir = r.GetDirection();

	Vector3 tVals(-1, -1, -1);

	for (int i = 0; i < 3; ++i) {
		if (rayDir[i] > 0) {
			tVals[i] = (boxMin[i] - rayPos[i]) / rayDir[i];
		}
		else if (rayDir[i] < 0) {
			tVals[i] = (boxMax[i] - rayPos[i]) / rayDir[i];
		}
	}

	float bestT = Vector::GetMaxElement(tVals);

	if (bestT < 0.0f) {
		return false;
	}

	Vector3 intersection = rayPos + (rayDir * bestT);
	const float epsilon = 0.0001f;
	for (int i = 0; i < 3; ++i) {
		if (intersection[i] + epsilon < boxMin[i] || intersection[i] - epsilon > boxMax[i]){
			return false;
		}
	}
	collision.collidedAt = intersection;
	collision.rayDistance = bestT;
	return true;
}

bool CollisionDetection::RayAABBIntersection(const Ray&r, const Transform& worldTransform, const AABBVolume& volume, RayCollision& collision) {
	Vector3 boxPos = worldTransform.GetPosition();
	Vector3 boxSize = volume.GetHalfDimensions();
	return RayBoxIntersection(r, boxPos, boxSize, collision);
}

bool CollisionDetection::RayOBBIntersection(const Ray&r, const Transform& worldTransform, const OBBVolume& volume, RayCollision& collision) {
	Quaternion orientation = worldTransform.GetOrientation();
	Vector3 position = worldTransform.GetPosition();

	Matrix3 transform = Quaternion::RotationMatrix<Matrix3>(orientation);
	Matrix3 invTransform = Quaternion::RotationMatrix<Matrix3>(orientation.Conjugate());

	Vector3 localRayPos = r.GetPosition() - position;
	
	Ray tempRay(invTransform * localRayPos, invTransform * r.GetDirection());

	bool collided = RayBoxIntersection(tempRay, Vector3(), volume.GetHalfDimensions(), collision);
	if (collided) {
		collision.collidedAt = transform * collision.collidedAt + position;
	}
	return collided;
}

bool CollisionDetection::RaySphereIntersection(const Ray&r, const Transform& worldTransform, const SphereVolume& volume, RayCollision& collision) {
	Vector3 spherePos = worldTransform.GetPosition();
	float sphereRadius = volume.GetRadius();

	Vector3 dir = (spherePos - r.GetPosition());

	float sphereProj = Vector::Dot(dir, r.GetDirection());

	if (sphereProj < 0.0f) {
		return false;
	}

	Vector3 point = r.GetPosition() + (r.GetDirection() * sphereProj);

	float sphereDist = Vector::Length(point - spherePos);

	if (sphereDist > sphereRadius) {
		return false;
	}

	float offset = sqrt((sphereRadius * sphereRadius) - (sphereDist * sphereDist));

	collision.rayDistance = sphereProj - (offset);
	collision.collidedAt = r.GetPosition() + (r.GetDirection() * collision.rayDistance);
	return true;
}

bool CollisionDetection::RayCapsuleIntersection(const Ray& r, const Transform& worldTransform, const CapsuleVolume& volume, RayCollision& collision) {


	return false;
}

bool CollisionDetection::ObjectIntersection(GameObject* a, GameObject* b, CollisionInfo& collisionInfo) {
	const CollisionVolume* volA = a->GetBoundingVolume();
	const CollisionVolume* volB = b->GetBoundingVolume();

	if (!volA || !volB) {
		return false;
	}

	collisionInfo.a = a;
	collisionInfo.b = b;

	Transform& transformA = a->GetTransform();
	Transform& transformB = b->GetTransform();

	VolumeType pairType = (VolumeType)((int)volA->type | (int)volB->type);

	//Two AABBs
	if (pairType == VolumeType::AABB) {
		return AABBIntersection((AABBVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}
	//Two Spheres
	if (pairType == VolumeType::Sphere) {
		return SphereIntersection((SphereVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	//Two OBBs
	if (pairType == VolumeType::OBB) {
		return OBBIntersection((OBBVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);
	}
	//Two Capsules

	//AABB vs Sphere pairs
	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Sphere) {
		return AABBSphereIntersection((AABBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBSphereIntersection((AABBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	//OBB vs sphere pairs
	if (volA->type == VolumeType::OBB && volB->type == VolumeType::Sphere) {
		return OBBSphereIntersection((OBBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBSphereIntersection((OBBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	//Capsule vs other interactions
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::Sphere) {
		return SphereCapsuleIntersection((CapsuleVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::Capsule) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return SphereCapsuleIntersection((CapsuleVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::AABB) {
		return AABBCapsuleIntersection((CapsuleVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}
	if (volB->type == VolumeType::Capsule && volA->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBCapsuleIntersection((CapsuleVolume&)*volB, transformB, (AABBVolume&)*volA, transformA, collisionInfo);
	}

	return false;
}

bool CollisionDetection::AABBTest(const Vector3& posA, const Vector3& posB, const Vector3& halfSizeA, const Vector3& halfSizeB) {
	Vector3 delta = posB - posA;
	Vector3 totalSize = halfSizeA + halfSizeB; 

	if (abs(delta.x) < totalSize.x &&
		abs(delta.y) < totalSize.y &&
		abs(delta.z) < totalSize.z) {
		return true;
	}
	return false;
}

//AABB/AABB Collisions
bool CollisionDetection::AABBIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxAPos = worldTransformA.GetPosition();
	Vector3 boxBPos = worldTransformB.GetPosition();

	Vector3 boxASize = volumeA.GetHalfDimensions();
	Vector3 boxBSize = volumeB.GetHalfDimensions();

	bool overlap = AABBTest(boxAPos, boxBPos, boxASize, boxBSize);

	if (overlap) {
		static const Vector3 faces[6] =
		{
			Vector3(-1,0,0),Vector3(1,0,0),
			Vector3(0,-1,0),Vector3(0,1,0),
			Vector3(0,0,-1),Vector3(0,0,1)
		};

		Vector3 maxA = boxAPos + boxASize;
		Vector3 minA = boxAPos - boxASize;

		Vector3 maxB = boxBPos + boxBSize;
		Vector3 minB = boxBPos - boxBSize;

		float distances[6] =
		{
			(maxB.x - minA.x),
			(maxA.x - minB.x),
			(maxB.y - minA.y),
			(maxA.y - minB.y),
			(maxB.z - minA.z),
			(maxA.z - minB.z)
		};
		float penetration = FLT_MAX;
		Vector3 bestAxis;

		for (int i = 0; i < 6; i++) {
			if (distances[i] < penetration) {
				penetration = distances[i];
				bestAxis = faces[i];
			}
		}
		collisionInfo.AddContactPoint(Vector3(), Vector3(), bestAxis, penetration);
		return true;
	}
	return false;
}

//Sphere / Sphere Collision
bool CollisionDetection::SphereIntersection(const SphereVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	float radii = volumeA.GetRadius() + volumeB.GetRadius();
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	float deltaLength = Vector::Length(delta);

	if (deltaLength < radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal = Vector::Normalise(delta);
		Vector3 localA = normal * volumeA.GetRadius();
		Vector3 localB = -normal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		return true;
	}
	return false;
}

//AABB - Sphere Collision
bool CollisionDetection::AABBSphereIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxSize = volumeA.GetHalfDimensions();
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	Vector3 closestPointOnBox = Vector::Clamp(delta, -boxSize, boxSize);
	Vector3 localPoint = delta - closestPointOnBox;
	float distance = Vector::Length(localPoint);

	if (distance < volumeB.GetRadius()) {
		Vector3 collisionNormal = Vector::Normalise(localPoint);
		float penentration = (volumeB.GetRadius() - distance);

		Vector3 localA = Vector3();
		Vector3 localB = -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penentration);
		return true;
	}
	return false;
}

bool  CollisionDetection::OBBSphereIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxSize = volumeA.GetHalfDimensions();
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	Quaternion qa = worldTransformA.GetOrientation();
	Quaternion iq = qa.Conjugate();

	Matrix3 rotation = Quaternion::RotationMatrix<Matrix3>(qa);
	Matrix3 invRotation = Quaternion::RotationMatrix<Matrix3>(iq);



	Vector3 invDelta = invRotation * delta;


	Vector3 closestPointOnBox = Vector::Clamp(invDelta, -boxSize, boxSize);
	Vector3 localPoint = invDelta - closestPointOnBox;
	float distance = Vector::Length(localPoint);

	if (distance < volumeB.GetRadius()) {

		//Make a black whole between spheres and cubes so maybe normal error?

		Vector3 localCollisionNormal = Vector::Normalise(localPoint);
		float penentration = (volumeB.GetRadius() - distance);

		Vector3 collisionNormal = rotation * localCollisionNormal;

		Vector3 localA = Vector3();
		Vector3 localB = -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penentration);
		return true;
	}
	/*
	* Apply inverse rotation to Box to make axis line up, then rotate sphere relative to box origin by first - box position in world to work out intersection with AABB/Sphere system
	* After finding localpoint, rotate back to original position using reverese of rotation on sphere and transformation of cube
	*/

	return false;
}

bool CollisionDetection::AABBCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	return false;
}

bool CollisionDetection::SphereCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	return false;
}

void ResolveFaceToFaceCollision(Transform transformA, Transform transformB, const Vector3& displacement) {
	// Move object A in the opposite direction of the displacement (to resolve overlap)
	transformA.SetPosition(transformA.GetPosition()- displacement);

	// Move object B in the direction of the displacement (to resolve overlap)
	transformB.SetPosition(transformB.GetPosition() + displacement);

	// Optionally, you might want to adjust the velocities or any other dynamic properties
	// if you're doing a physics simulation (e.g., bouncing back after collision).
}
/**
float CalculatePenetrationDepth(const Vector3& normal, const Vector3& boxSizeA, const Transform& transformA, const Vector3& boxSizeB, const Transform& transformB, const Matrix3& rotationA, const Matrix3& rotationB) {
	// Ensure the distance is between the centers of A and B
	float distance = abs(Vector::Dot(normal, transformA.GetPosition() - transformB.GetPosition()));

	// Calculate projections
	float projectionA = boxSizeA.x * abs(Vector::Dot(normal, rotationA.GetColumn(0))) +
		boxSizeA.y * abs(Vector::Dot(normal, rotationA.GetColumn(1))) +
		boxSizeA.z * abs(Vector::Dot(normal, rotationA.GetColumn(2)));

	float projectionB = boxSizeB.x * abs(Vector::Dot(normal, rotationB.GetColumn(0))) +
		boxSizeB.y * abs(Vector::Dot(normal, rotationB.GetColumn(1))) +
		boxSizeB.z * abs(Vector::Dot(normal, rotationB.GetColumn(2)));

	// Calculate the penetration depth
	float penetration = projectionA + projectionB - distance;
	return penetration > 0 ? penetration : 0;
}

bool CollisionDetection::OBBIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Vector3 boxSizeA = volumeA.GetHalfDimensions();
	Vector3 boxSizeB = volumeB.GetHalfDimensions();
	Quaternion qa = worldTransformA.GetOrientation();
	Matrix3 rotationA = Quaternion::RotationMatrix<Matrix3>(qa);
	Quaternion qb = worldTransformB.GetOrientation();
	Matrix3 rotationB = Quaternion::RotationMatrix<Matrix3>(qb);

	float minPenetration = FLT_MAX;
	Vector3 collisionAxis;
	bool collisionDetected = true;

	// Project on axis helper function
	auto ProjectOnAxis = [](const OBBVolume& volume, const Transform& transform, Matrix3 rotation, const Vector3& axis) -> float {
		Vector3 xAxis = rotation.GetColumn(0);
		Vector3 yAxis = rotation.GetColumn(1);
		Vector3 zAxis = rotation.GetColumn(2);
		return abs(Vector::Dot(axis, xAxis) * volume.GetHalfDimensions().x) +
			abs(Vector::Dot(axis, yAxis) * volume.GetHalfDimensions().y) +
			abs(Vector::Dot(axis, zAxis) * volume.GetHalfDimensions().z);
		};

	// Build list of axes to test
	Vector3 testAxes[15];
	int axisIndex = 0;

	// Add box A axes
	for (int i = 0; i < 3; ++i) {
		testAxes[axisIndex++] = rotationA.GetColumn(i);
	}

	// Add box B axes
	for (int i = 0; i < 3; ++i) {
		testAxes[axisIndex++] = rotationB.GetColumn(i);
	}

	// Add cross products of axes A and B
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Vector3 crossAxis = Vector::Cross(rotationA.GetColumn(i), rotationB.GetColumn(j));
			if (Vector::LengthSquared(crossAxis) > FLT_EPSILON) {
				testAxes[axisIndex++] = Vector::Normalise(crossAxis);
			}
		}
	}

	// Perform the Separating Axis Theorem (SAT) test
	for (int i = 0; i < axisIndex; ++i) {
		Vector3 axis = testAxes[i];

		// Project each box onto the axis
		float projectionA = ProjectOnAxis(volumeA, worldTransformA, rotationA, axis);
		float projectionB = ProjectOnAxis(volumeB, worldTransformB, rotationB, axis);

		// Calculate the center distance
		float centerDistance = abs(Vector::Dot(axis, worldTransformB.GetPosition() - worldTransformA.GetPosition()));
		float overlap = projectionA + projectionB - centerDistance;

		if (overlap <= 0.0f) {
			return false; // No overlap
		}

		if (overlap < minPenetration) {
			minPenetration = overlap;
			collisionAxis = axis;
		}
	}

	// If we get here, there's a collision
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();
	Vector3 localPointA;
	localPointA.x = Vector::Dot(delta, rotationA.GetColumn(0));
	localPointA.y = Vector::Dot(delta, rotationA.GetColumn(1));
	localPointA.z = Vector::Dot(delta, rotationA.GetColumn(2));

	Vector3 localPointB;
	localPointB.x = Vector::Dot(-delta, rotationB.GetColumn(0));
	localPointB.y = Vector::Dot(-delta, rotationB.GetColumn(1));
	localPointB.z = Vector::Dot(-delta, rotationB.GetColumn(2));

	// Clamp points to the box extents
	Vector3 localA = Vector::Clamp(localPointA, -boxSizeA, boxSizeA);
	Vector3 localB = Vector::Clamp(localPointB, -boxSizeB, boxSizeB);

	// Calculate collision normal
	Vector3 localColPoint = delta - localA;
	Vector3 localCollisionNormal = Vector::Normalise(localColPoint);
	Vector3 collisionNormal = rotationA * localCollisionNormal;

	collisionInfo.AddContactPoint(localA, localB, collisionNormal, minPenetration);
	return true;
}*/





/**/
float CalculatePenetrationDepth(const Vector3& normal, Vector3& boxSizeA, const Transform& transformA, Vector3& boxSizeB, const Transform& transformB, Matrix3 rotationA, Matrix3 rotationB ) {
	Vector3 axesA[3] = { rotationA.GetColumn(0), rotationA.GetColumn(1), rotationA.GetColumn(2) };
	Vector3 axesB[3] = { rotationB.GetColumn(0), rotationB.GetColumn(1), rotationB.GetColumn(2) };

	float projectionA = boxSizeA.x * abs(Vector::Dot(normal, axesA[0])) + boxSizeA.y * abs(Vector::Dot(normal, axesA[1])) + boxSizeA.z * abs(Vector::Dot(normal, axesA[2]));
	float projectionB = boxSizeB.x * abs(Vector::Dot(normal, axesB[0])) + boxSizeB.y * abs(Vector::Dot(normal, axesB[1])) + boxSizeB.z * abs(Vector::Dot(normal, axesB[2]));

	float distance = abs(Vector::Dot(normal, transformB.GetPosition() - transformA.GetPosition()));

	float penetration = projectionA + projectionB - distance;

	return penetration > 0 ? penetration : 0;
}

bool CollisionDetection::OBBIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxSizeA = volumeA.GetHalfDimensions();
	Vector3 boxSizeB = volumeB.GetHalfDimensions();
	Quaternion qa = worldTransformA.GetOrientation();
	Matrix3 rotationA = Quaternion::RotationMatrix<Matrix3>(qa);
	Quaternion qb = worldTransformB.GetOrientation();
	Matrix3 rotationB = Quaternion::RotationMatrix<Matrix3>(qb);

	float minPenetration = FLT_MAX;
	Vector3 collisionAxis;
	bool collisionDetected = true;

	auto ProjectOnAxis = [](const OBBVolume& volume, const Transform& transform, Matrix3 rotation, const Vector3& axis)->float {
		float projection = 0.0;
		Vector3 xAxis = rotation.GetColumn(0);
		Vector3 yAxis = rotation.GetColumn(1);
		Vector3 zAxis = rotation.GetColumn(2);
		projection += abs(Vector::Dot(axis, xAxis) * volume.GetHalfDimensions().x);
		projection += abs(Vector::Dot(axis, yAxis) * volume.GetHalfDimensions().y);
		projection += abs(Vector::Dot(axis, zAxis) * volume.GetHalfDimensions().z);
		return projection;
		};

	Vector3 testAxes[15];
	int axisIndex = 0;

	Vector3 axesA[3] = { rotationA.GetColumn(0), rotationA.GetColumn(1), rotationA.GetColumn(2) };
	for (int i = 0; i < 3; ++i) {
		testAxes[axisIndex++] = axesA[i];
	}

	Vector3 axesB[3] = { rotationB.GetColumn(0), rotationB.GetColumn(1), rotationB.GetColumn(2) };
	for (int i = 0; i < 3; ++i) {
		testAxes[axisIndex++] = axesB[i];
	}


	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Vector3 crossAxis = Vector::Cross(axesA[i], axesB[j]);
			bool check = true;
			if (abs(crossAxis.x) < FLT_EPSILON && abs(crossAxis.y) < FLT_EPSILON && abs(crossAxis.z) < FLT_EPSILON) {
				check = false;
			}
			if (check == true) {
				testAxes[axisIndex++] = Vector::Normalise(crossAxis);
			}
			else {
				

				Vector3 normalA = axesA[i];
				Vector3 normalB = axesB[j];

				
				//float penDepth = CalculatePenetrationDepth(normalA, boxSizeA, worldTransformA, boxSizeB, worldTransformB, rotationA, rotationB);
				testAxes[axisIndex++] = Vector::Normalise(normalA);
			}
		
		}
	}

	for (int i = 0; i < 15; ++i) {
		Vector3 axis = testAxes[i];

		float projectionA = ProjectOnAxis(volumeA, worldTransformA, rotationA, axis);
		float projectionB = ProjectOnAxis(volumeB, worldTransformB, rotationB, axis);

		float centerDistance = abs(Vector::Dot(axis, worldTransformB.GetPosition() - worldTransformA.GetPosition()));
		float overlap = projectionA + projectionB - centerDistance;

		if (overlap <= -FLT_EPSILON) { return false; }

		if (overlap < minPenetration) {
			minPenetration = overlap;
			collisionAxis = axis;
		}
	}
	

	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();
	Vector3 centredPointA = delta - worldTransformA.GetPosition();
	Vector3 localPointA;
	localPointA.x = Vector::Dot(centredPointA, axesA[0]);
	localPointA.y = Vector::Dot(centredPointA, axesA[1]);
	localPointA.z = Vector::Dot(centredPointA, axesA[2]);
	
	Vector3 centredPointB = delta - worldTransformB.GetPosition();
	Vector3 localPointB;
	localPointB.x = Vector::Dot(centredPointB, axesB[0]);
	localPointB.y = Vector::Dot(centredPointB, axesB[1]);
	localPointB.z = Vector::Dot(centredPointB, axesB[2]);


	Vector3 localA = Vector::Clamp(localPointA, -boxSizeA, boxSizeA);
	Vector3 localB = Vector::Clamp(localPointB, -boxSizeB, boxSizeB);

	Vector3 localColPoint = delta - localA;
	Vector3 localCollisionNormal = Vector::Normalise(localColPoint);
	Vector3 collisionNormal = rotationA * localCollisionNormal;


	collisionInfo.AddContactPoint(localA, localB, collisionNormal /*Vector::Normalise(collisionAxis)*/, minPenetration);
	return true;

	
}
//*/

Matrix4 GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix::Translation(position) *
		Matrix::Rotation(-yaw, Vector3(0, -1, 0)) *
		Matrix::Rotation(-pitch, Vector3(-1, 0, 0));

	return iview;
}

Matrix4 GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane) {
	float negDepth = nearPlane - farPlane;

	float invNegDepth = negDepth / (2 * (farPlane * nearPlane));

	Matrix4 m;

	float h = 1.0f / tan(fov*PI_OVER_360);

	m.array[0][0] = aspect / h;
	m.array[1][1] = tan(fov * PI_OVER_360);
	m.array[2][2] = 0.0f;

	m.array[2][3] = invNegDepth;//// +PI_OVER_360;
	m.array[3][2] = -1.0f;
	m.array[3][3] = (0.5f / nearPlane) + (0.5f / farPlane);

	return m;
}

Vector3 CollisionDetection::Unproject(const Vector3& screenPos, const PerspectiveCamera& cam) {
	Vector2i screenSize = Window::GetWindow()->GetScreenSize();

	float aspect = Window::GetWindow()->GetScreenAspect();
	float fov		= cam.GetFieldOfVision();
	float nearPlane = cam.GetNearPlane();
	float farPlane  = cam.GetFarPlane();

	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(cam) * GenerateInverseProjection(aspect, fov, nearPlane, farPlane);

	Matrix4 proj  = cam.BuildProjectionMatrix(aspect);

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(screenPos.x / (float)screenSize.x) * 2.0f - 1.0f,
		(screenPos.y / (float)screenSize.y) * 2.0f - 1.0f,
		(screenPos.z),
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

Ray CollisionDetection::BuildRayFromMouse(const PerspectiveCamera& cam) {
	Vector2 screenMouse = Window::GetMouse()->GetAbsolutePosition();
	Vector2i screenSize	= Window::GetWindow()->GetScreenSize();

	//We remove the y axis mouse position from height as OpenGL is 'upside down',
	//and thinks the bottom left is the origin, instead of the top left!
	Vector3 nearPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		-0.99999f
	);

	//We also don't use exactly 1.0 (the normalised 'end' of the far plane) as this
	//causes the unproject function to go a bit weird. 
	Vector3 farPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		0.99999f
	);

	Vector3 a = Unproject(nearPos, cam);
	Vector3 b = Unproject(farPos, cam);
	Vector3 c = b - a;

	c = Vector::Normalise(c);

	return Ray(cam.GetPosition(), c);
}

//http://bookofhook.com/mousepick.pdf
Matrix4 CollisionDetection::GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane) {
	Matrix4 m;

	float t = tan(fov*PI_OVER_360);

	float neg_depth = nearPlane - farPlane;

	const float h = 1.0f / t;

	float c = (farPlane + nearPlane) / neg_depth;
	float e = -1.0f;
	float d = 2.0f*(nearPlane*farPlane) / neg_depth;

	m.array[0][0] = aspect / h;
	m.array[1][1] = tan(fov * PI_OVER_360);
	m.array[2][2] = 0.0f;

	m.array[2][3] = 1.0f / d;

	m.array[3][2] = 1.0f / e;
	m.array[3][3] = -c / (d * e);

	return m;
}

/*
And here's how we generate an inverse view matrix. It's pretty much
an exact inversion of the BuildViewMatrix function of the Camera class!
*/
Matrix4 CollisionDetection::GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix::Translation(position) *
		Matrix::Rotation(yaw, Vector3(0, 1, 0)) *
		Matrix::Rotation(pitch, Vector3(1, 0, 0));

	return iview;
}


/*
If you've read through the Deferred Rendering tutorial you should have a pretty
good idea what this function does. It takes a 2D position, such as the mouse
position, and 'unprojects' it, to generate a 3D world space position for it.

Just as we turn a world space position into a clip space position by multiplying
it by the model, view, and projection matrices, we can turn a clip space
position back to a 3D position by multiply it by the INVERSE of the
view projection matrix (the model matrix has already been assumed to have
'transformed' the 2D point). As has been mentioned a few times, inverting a
matrix is not a nice operation, either to understand or code. But! We can cheat
the inversion process again, just like we do when we create a view matrix using
the camera.

So, to form the inverted matrix, we need the aspect and fov used to create the
projection matrix of our scene, and the camera used to form the view matrix.

*/
Vector3	CollisionDetection::UnprojectScreenPosition(Vector3 position, float aspect, float fov, const PerspectiveCamera& c) {
	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(c) * GenerateInverseProjection(aspect, fov, c.GetNearPlane(), c.GetFarPlane());


	Vector2i screenSize = Window::GetWindow()->GetScreenSize();

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(position.x / (float)screenSize.x) * 2.0f - 1.0f,
		(position.y / (float)screenSize.y) * 2.0f - 1.0f,
		(position.z) - 1.0f,
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

