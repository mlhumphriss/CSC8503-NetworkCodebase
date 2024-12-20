#include "GameObject.h"
#include "CollisionDetection.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "NetworkObject.h"
#include "Camera.h"
#include "Window.h"
#include "StateTransition.h"
#include "StateMachine.h"
#include "State.h"
#include "Debug.h"


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

	canChase = InMaze();
	//std::cout << canChase << std::endl;
	
	Vector3 movement = yawRotation * Vector3(0, 0, playerController->GetNamedAxis("Forward")) * speed*dt;
	movement += yawRotation * Vector3(-playerController->GetNamedAxis("Sidestep"), 0, 0) * speed*dt;
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::SPACE) && noJump==false) {
		movement += jump;
	}
	this->GetPhysicsObject()->ApplyLinearImpulse(movement);
}

bool PlayerObject::InMaze() {
	Vector3 currentPos = this->GetTransform().GetPosition();
	if ((currentPos.x >= 0 && currentPos.x <= 160) && (currentPos.z >= 0 && currentPos.z <= 160) && (currentPos.y > -5)) { return true; }
	return false;
}


EnemyObject::EnemyObject() :GameObject() {
	tag = 3;
	speed = 2.0f;
	routeDisrupted = true;
	routePoint = 0;
	enemyStateMachine = new StateMachine();

	//state stuff add below
	State* Idle = new State([&](float dt)-> void
		{
			routeDisrupted = true;
			this->GetRenderObject()->SetColour(Vector4(1, 1, 0, 1));
		}
	);
	State* Respawn = new State([&](float dt)-> void
		{
			this->GetTransform().SetPosition(respawn);
		}
	);
	State* Chase = new State([&](float dt)-> void
		{
			this->UpdateChaseMovement(dt);
		}
	);
	State* Path = new State([&](float dt)-> void
		{
			this->CreatePath();
			if (!routeDisrupted) { this->UpdatePathMovement(dt); }
			//DisplayPathfinding();
		}
	);

	enemyStateMachine->AddState(Idle);
	enemyStateMachine->AddState(Respawn);
	enemyStateMachine->AddState(Chase);
	enemyStateMachine->AddState(Path);

	enemyStateMachine->AddTransition(new StateTransition(Idle, Respawn, [&]()->bool
		{
			return this->GetTransform().GetPosition().y < -1.0f;
		}
	));
	enemyStateMachine->AddTransition(new StateTransition(Chase, Respawn, [&]()->bool
		{
			return this->GetTransform().GetPosition().y < -1.0f;
		}
	));
	enemyStateMachine->AddTransition(new StateTransition(Path, Respawn, [&]()->bool
		{
			return this->GetTransform().GetPosition().y < -1.0f;
		}
	));
	enemyStateMachine->AddTransition(new StateTransition(Respawn, Idle, [&]()->bool
		{
			return this->GetTransform().GetPosition().y > -1.0f;
		}
	));
	enemyStateMachine->AddTransition(new StateTransition(Idle, Chase, [&]()->bool
		{
			RayCollision colCheck;
			Ray c = Ray(this->GetTransform().GetPosition(), player->GetTransform().GetPosition() - this->GetTransform().GetPosition());
			bool collides = rayHit(c, colCheck, true, this);
			if (Vector::Length(player->GetTransform().GetPosition() - colCheck.collidedAt) < 10.0f) {
				return true;
			}
			return false;
		}
	));
	enemyStateMachine->AddTransition(new StateTransition(Path, Chase, [&]()->bool
		{
			RayCollision colCheck;
			Ray c = Ray(this->GetTransform().GetPosition(), player->GetTransform().GetPosition() - this->GetTransform().GetPosition());
			bool collides = rayHit(c, colCheck, true, this);
			if (Vector::Length(player->GetTransform().GetPosition() - colCheck.collidedAt) < 10.0f) {
				return true;
			}
			return false;
		}
	));
	enemyStateMachine->AddTransition(new StateTransition(Chase, Idle, [&]()->bool
		{
			RayCollision colCheck;
			Ray c = Ray(this->GetTransform().GetPosition(), player->GetTransform().GetPosition() - this->GetTransform().GetPosition());
			bool collides = rayHit(c, colCheck, true, this);
			if (Vector::Length(player->GetTransform().GetPosition() - colCheck.collidedAt) > 10.0f) {
				return true;
			}
			return false;
		}
	));
	enemyStateMachine->AddTransition(new StateTransition(Path, Idle, [&]()->bool
		{
			return !(player->GetChase());
		}
	));
	enemyStateMachine->AddTransition(new StateTransition(Idle, Path, [&]()->bool
		{
			return (player->GetChase());
		}
	));


}
EnemyObject::~EnemyObject() {
	delete enemyStateMachine;
}
void EnemyObject::Update(float dt) {
	enemyStateMachine->Update(dt);
}

void EnemyObject::UpdateChaseMovement(float dt) {
	Vector3 direction = player->GetTransform().GetPosition() - this->GetTransform().GetPosition();
	this->GetRenderObject()->SetColour(Vector4(1, 0, 0, 1));
	
	Vector3 movement = Vector3(direction.x, -1.0f, direction.z) * speed * dt;
	this->GetPhysicsObject()->ApplyLinearImpulse(movement);
}

void EnemyObject::UpdatePathMovement(float dt) {
	this->GetRenderObject()->SetColour(Vector4(1.0f, 0.5f, 0.0f, 1.0f));
	if (pathNodes.size() <= 0) { routeDisrupted = true; return; }
	
	Vector3 nextNode = pathNodes[routePoint];
	Vector3 tempPos = Vector3(this->GetTransform().GetPosition().x, 0, this->GetTransform().GetPosition().z);
	Vector3 tempNext = Vector3(nextNode.x, 0, nextNode.z);
	if (Vector::Length(tempPos - tempNext) < 3.5) {
		if (routePoint < pathNodes.size()) {
			routePoint++;
			nextNode = pathNodes[routePoint];
		}
		else { routeDisrupted = true;}
	}
	Vector3 direction = nextNode - this->GetTransform().GetPosition();
	Vector3 movement = Vector3(direction.x, -1.0f, direction.z) * speed * dt;
	this->GetPhysicsObject()->ApplyLinearImpulse(movement);
}

void EnemyObject::CreatePath() {
	if (CheckNewPathNeeded() == false) { return; }
	NavigationGrid grid("TestGrid2.txt");
	NavigationPath outPath;
	destination = player->GetTransform().GetPosition();
	bool found = grid.FindPath(this->GetTransform().GetPosition(), destination, outPath);

	if (!found) { routeDisrupted = true; return; }

	routePoint = 0;
	pathNodes.clear();
	Vector3 pos;
	while (outPath.PopWaypoint(pos)) {
		pathNodes.push_back(pos);
	}
}

bool EnemyObject::CheckNewPathNeeded() {
	if (Vector::Length(player->GetTransform().GetPosition() - destination) > 30.0f || routeDisrupted==true) {
		routeDisrupted = false;
		return true;
	}
	return false;
}

void EnemyObject::DisplayPathfinding() {
	for (int i = 1; i < pathNodes.size(); ++i) {
		Vector3 a = pathNodes[i - 1];
		Vector3 b = pathNodes[i];

		Debug::DrawLine(a, b, Vector4(0, 1, 1, 1));
	}
}