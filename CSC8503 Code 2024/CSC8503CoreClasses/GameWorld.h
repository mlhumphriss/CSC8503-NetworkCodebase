#pragma once
#include <random>

#include "Ray.h"
#include "CollisionDetection.h"
#include "QuadTree.h"
#include "GameObject.h"
namespace NCL {
		class Camera;
		using Maths::Ray;
	namespace CSC8503 {
		class GameObject;
		class Constraint;

		typedef std::function<void(GameObject*)> GameObjectFunc;
		typedef std::vector<GameObject*>::const_iterator GameObjectIterator;
		typedef std::vector<KittenObject*>::const_iterator KittenIterator;

		class GameWorld	{
		public:
			GameWorld();
			~GameWorld();

			void Clear();
			void ClearAndErase();

			void AddGameObject(GameObject* o);
			void AddPlayerObject(PlayerObject* p);
			void AddEnemyObject(EnemyObject* e);
			void AddKittenObject(KittenObject* k);
			void RemoveGameObject(GameObject* o, bool andDelete = false);

			void AddConstraint(Constraint* c);
			void RemoveConstraint(Constraint* c, bool andDelete = false);

			PerspectiveCamera& GetMainCamera()  {
				return mainCamera;
			}
			PlayerObject* GetPlayer() {
				return player;
			}
			EnemyObject* GetEnemy() {
				return enemy;
			}

			void ShuffleConstraints(bool state) {
				shuffleConstraints = state;
			}

			void ShuffleObjects(bool state) {
				shuffleObjects = state;
			}
			void ShuffleKittens(bool state) {
				shuffleKittens = state;
			}


			bool Raycast(Ray& r, RayCollision& closestCollision, bool closestObject = false, GameObject* ignore = nullptr) const;

			virtual void UpdateWorld(float dt);

			void OperateOnContents(GameObjectFunc f);

			void GetObjectIterators(
				GameObjectIterator& first,
				GameObjectIterator& last) const;

			void GetConstraintIterators(
				std::vector<Constraint*>::const_iterator& first,
				std::vector<Constraint*>::const_iterator& last) const;

			void GetKittenIterators(
				KittenIterator& first,
				KittenIterator& last) const;

			int GetWorldStateID() const {
				return worldStateCounter;
			}

			bool GetSelectMode() const {
				return selectMode;
			}

			void SetSelectMode(bool s) {
				selectMode = s;
			}

		protected:
			std::vector<GameObject*> gameObjects;
			std::vector<Constraint*> constraints;
			std::vector<KittenObject*> kittens;

			PerspectiveCamera mainCamera;
			PlayerObject* player;
			EnemyObject* enemy;

			bool shuffleConstraints;
			bool shuffleObjects;
			bool shuffleKittens;
			int		worldIDCounter;
			int		worldStateCounter;
			bool selectMode;
		};
	}
}

