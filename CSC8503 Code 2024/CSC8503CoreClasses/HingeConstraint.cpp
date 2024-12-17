#include "C:/Users/c1009080/source/CSC8503/CSC8503-NetworkCodebase/CSC8503 Code 2024/CSC8503CoreClasses/CMakeFiles/CSC8503CoreClasses.dir/Debug/cmake_pch.hxx"
#include "HingeConstraint.h"
#include "GameObject.h"
#include "PhysicsObject.h"

using namespace NCL;
using namespace Maths;
using namespace CSC8503;

HingeConstraint::HingeConstraint(GameObject* a, GameObject* b){
	objectA = a;
	objectB = b;
}

HingeConstraint::~HingeConstraint(){}

void HingeConstraint::UpdateConstraint(float dt) {
	Vector3 delta = (objectA->GetTransform().GetPosition() - Vector::Normalise(objectB->GetTransform().GetPosition()));
	float nAngle = RadiansToDegrees(atan2f(-delta.z, delta.x));

	objectA->GetTransform().SetOrientation(Quaternion::EulerAnglesToQuaternion(0, nAngle, 0));
	objectB->GetTransform().SetOrientation(objectA->GetTransform().GetOrientation());

	Vector3 tempPos = objectA->GetTransform().GetPosition();
	tempPos.y = objectB->GetTransform().GetPosition().y;
	objectA->GetTransform().SetPosition(tempPos);
}
