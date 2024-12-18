#pragma once
#include "Constraint.h"

namespace NCL {
	namespace CSC8503 {
		class GameObject;

		class FollowConstraint : public Constraint
		{
		public:
			FollowConstraint(GameObject* a, GameObject* b, float d);
			~FollowConstraint();

			void UpdateConstraint(float dt) override;

		protected:
			GameObject* objectA;
			GameObject* objectB;

			float distance;
		};
	}
}