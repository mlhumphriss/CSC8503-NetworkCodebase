#pragma once
#include "Transform.h"
#include "CollisionVolume.h"
#include "Controller.h"
#include "Camera.h"
#include "Ray.h"
//#include "GameWorld.h"

using std::vector;

namespace NCL::CSC8503 {
	class NetworkObject;
	class RenderObject;
	class PhysicsObject;

	class GameObject	{
	public:
		GameObject(const std::string& name = "");
		~GameObject();

		void SetBoundingVolume(CollisionVolume* vol) {
			boundingVolume = vol;
		}

		const CollisionVolume* GetBoundingVolume() const {
			return boundingVolume;
		}

		bool IsActive() const {
			return isActive;
		}

		Transform& GetTransform() {
			return transform;
		}

		RenderObject* GetRenderObject() const {
			return renderObject;
		}

		PhysicsObject* GetPhysicsObject() const {
			return physicsObject;
		}

		NetworkObject* GetNetworkObject() const {
			return networkObject;
		}

		void SetRenderObject(RenderObject* newObject) {
			renderObject = newObject;
		}

		void SetPhysicsObject(PhysicsObject* newObject) {
			physicsObject = newObject;
		}

		const std::string& GetName() const {
			return name;
		}

		virtual void OnCollisionBegin(GameObject* otherObject) {
			//std::cout << "OnCollisionBegin event occured!\n";
		}

		virtual void OnCollisionEnd(GameObject* otherObject) {
			//std::cout << "OnCollisionEnd event occured!\n";
		}

		bool GetBroadphaseAABB(Vector3&outsize) const;

		void UpdateBroadphaseAABB();

		void SetWorldID(int newID) {
			worldID = newID;
		}

		int		GetWorldID() const {
			return worldID;
		}

		void SetAsFloor(bool t) {
			isFloor = t;
		}

		bool GetFloor() const {
			return isFloor;
		}

		void SetGrounded(bool g) {
			grounded = g;
		}

		bool GetGrounded() const {
			return grounded;
		}

		void SetDim(Vector3 d) {
			dim = d;
		}

		Vector3 GetDim() const {
			return dim;
		}
		int GetTag() const {
			return tag;
		}

	protected:
		Transform			transform;

		CollisionVolume*	boundingVolume;
		PhysicsObject*		physicsObject;
		RenderObject*		renderObject;
		NetworkObject*		networkObject;

		Vector3		dim;
		int			tag; //1 player, 2 kitten, 3 enemy, 4 bonus, 5 key

		bool		grounded;
		bool		isFloor;
		bool		isActive;
		int			worldID;
		std::string	name;

		Vector3 broadphaseAABB;
	};

	class PlayerObject : public GameObject {
	public:
		PlayerObject() : GameObject() {// When called need to pass in active controller somehow
			pYaw = 0.0f;
			speed = 25.0f;
			tag = 1;
		}
		~PlayerObject();
		void UpdateMovement(float dt);

		void SetController(const Controller& c) {
			playerController = &c;
		}

		typedef std::function<bool(Ray& r, RayCollision& closestCollision, bool closestObject, GameObject* ignore)> RaycastToWorld;
		void SetRay(RaycastToWorld rayHit) { this->rayHit = rayHit; }
		RaycastToWorld rayHit;
		

	protected:
		Camera* playerCam;
		float pYaw;
		float speed;

		const Controller* playerController = nullptr;
	};

	class KittenObject : public GameObject {
	public:
		KittenObject() :GameObject() {
			tag = 2;
			collected = false;
			following = false;
		}
		~KittenObject();

		virtual void OnCollisionBegin(GameObject* otherObject) override {
			if (otherObject->GetTag() == 1) {
				collected = true;
				player = (PlayerObject*)otherObject;
				std::cout << "yes" << "\n";
			}
		}
		
		void SetFollowing(bool f) {
			following = f;
		}

		bool GetFollowing() const {
			return following;
		}

		bool GetCollected() const {
			return collected;
		}

	protected:
		bool collected;
		bool following;
		PlayerObject* player;

	};
}

