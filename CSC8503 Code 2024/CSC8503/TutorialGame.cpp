#include "TutorialGame.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "NavigationGrid.h"
#include "NavigationMesh.h"

#include "PushdownMachine.h"
#include "PushdownState.h"

using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame() : controller(*Window::GetWindow()->GetKeyboard(), *Window::GetWindow()->GetMouse()) {
	world		= new GameWorld();
#ifdef USEVULKAN
	renderer	= new GameTechVulkanRenderer(*world);
	renderer->Init();
	renderer->InitStructures();
#else 
	renderer = new GameTechRenderer(*world);
#endif

	physics		= new PhysicsSystem(*world);

	forceMagnitude	= 10.0f;
	useGravity		= true;
	inSelectionMode = false;
	gameEnded		= false;
	devMode			= false;
	physics->UseGravity(useGravity);

	world->GetMainCamera().SetController(controller);

	controller.MapAxis(0, "Sidestep");
	controller.MapAxis(1, "UpDown");
	controller.MapAxis(2, "Forward");

	controller.MapAxis(3, "XLook");
	controller.MapAxis(4, "YLook");


	InitialiseAssets();

	world->GetPlayer()->SetController(controller); 

	paused = true;
}

void TutorialGame::PauseMenu() {
	bool check = true;
	while (check = true) {
	Debug::Print("Press N to Play in Normal Mode", Vector2(40, 40), Debug::RED);
	Debug::Print("Press S to Play in Sandbox Mode", Vector2(40, 60), Debug::RED);
	if (Window::GetKeyboard()->KeyDown(KeyCodes::N)) {
		devMode = false;
		check = false;
	}
	if (Window::GetKeyboard()->KeyDown(KeyCodes::S)) {
		devMode = true;
		check = false;
	}
	}
	if (check = false) { paused = false; }
	return;
}

/*
class PauseScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState)override {
		if (Window::GetKeyboard()->KeyPressed(KeyCodes::U)) {
			return PushdownResult::Pop;
		}
		return PushdownResult::NoChange;
	}
	void OnAwake() override {
		Debug::Print("U to unpause", Vector2(5, 65), Debug::RED);
	}
};
class GameScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState)override {
		paused = false;

		if (Window::GetKeyboard()->KeyDown(KeyCodes::P)) {
			*newState = new PauseScreen();
			paused = true;
			return PushdownResult::Push;
		}
		return PushdownResult::NoChange;
	};
	void OnAwake() override {
		Debug::Print("P to pause", Vector2(5, 65), Debug::RED);
	}
protected:
	bool paused;
};
class IntroScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (Window::GetKeyboard()->KeyDown(KeyCodes::N)) {
			*newState = new GameScreen();
			dev = false;
			return PushdownResult::Push;
		}
		if (Window::GetKeyboard()->KeyDown(KeyCodes::S)) {
			*newState = new GameScreen();
			dev = true;
			return PushdownResult::Push;
		}
		return PushdownResult::NoChange;

	};
	void OnAwake() override {
		Debug::Print("Press N to Play in Normal Mode", Vector2(40, 40), Debug::RED);
		Debug::Print("Press S to Play in Sandbox Mode", Vector2(40, 60), Debug::RED);
	}
protected:
	bool dev;
};




void TestPushdownAutomata(float dt, bool gameEnded) {
	PushdownMachine machine(new IntroScreen());
	while (gameEnded == false) {
		if (!machine.Update(dt)) {
			return;
		}
	}
}
*/
/*

Each of the little demo scenarios used in the game uses the same 2 meshes, 
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	cubeMesh	= renderer->LoadMesh("cube.msh");
	sphereMesh	= renderer->LoadMesh("sphere.msh");
	catMesh		= renderer->LoadMesh("ORIGAMI_Chat.msh");
	kittenMesh	= renderer->LoadMesh("Kitten.msh");

	enemyMesh	= renderer->LoadMesh("Keeper.msh");
	gooseMesh = renderer->LoadMesh("Goose.msh");
	bonusMesh	= renderer->LoadMesh("19463_Kitten_Head_v1.msh");
	capsuleMesh = renderer->LoadMesh("capsule.msh");

	basicTex	= renderer->LoadTexture("checkerboard.png");
	basicShader = renderer->LoadShader("scene.vert", "scene.frag");



	InitCamera();
	InitWorld();
}

TutorialGame::~TutorialGame()	{
	delete cubeMesh;
	delete sphereMesh;
	delete catMesh;
	delete kittenMesh;
	delete enemyMesh;
	delete bonusMesh;
	delete gooseMesh;

	delete basicTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
}

void TutorialGame::UpdateGame(float dt) {
	renderer->Update(dt);
	renderer->Render();
	Debug::UpdateRenderables(dt);
	
	//machine.Update(dt);
	if (devMode == false) {
		Debug::Print("Press M to Play in Sandbox Mode", Vector2(5, 5), Debug::RED);
		if (Window::GetKeyboard()->KeyPressed(KeyCodes::M)) {
			devMode = true;
		}
	}
	else {
		Debug::Print("Press N to Play in Normal Mode", Vector2(5, 5), Debug::RED);
		if (Window::GetKeyboard()->KeyPressed(KeyCodes::N)) {
			devMode = false;
		}
	}
	
	if (!inSelectionMode) {
		world->SetSelectMode(false);
	}
	else { world->SetSelectMode(true); }
	if (lockedObject != nullptr) {
		Vector3 objPos = lockedObject->GetTransform().GetPosition();
		Vector3 camPos = objPos + lockedOffset;

		Matrix4 temp = Matrix::View(camPos, objPos, Vector3(0, 1, 0));

		Matrix4 modelMat = Matrix::Inverse(temp);

		Quaternion q(modelMat);
		Vector3 angles = q.ToEuler(); //nearly there now!

		world->GetMainCamera().SetPosition(camPos);
		world->GetMainCamera().SetPitch(angles.x);
		world->GetMainCamera().SetYaw(angles.y);
	}

	if (world->GetPlayer()->GetGameEnd()) {
		if (world->GetPlayer()->GetScore() >= world->GetPlayer()->GetMaxScore() && gameEnded == false) {
			gameWon = true;
			Debug::Print("YOU WON !", Vector2(20, 50), Debug::GREEN);
			gameEnded = true;
		}
		else {
			gameWon = false;
			Debug::Print("YOU LOST !", Vector2(20, 50), Debug::RED);
			gameEnded = true;
		}
	}

	UpdateKeys();
	int playerScore = world->GetPlayer()->GetScore();
	std::string ps = std::to_string(playerScore);
	std::string score = std::string("Score: " + ps);

	Debug::Print(score, Vector2(5, 95), Debug::RED);

	if (devMode) {
		Debug::Print("Dev Mode", Vector2(80, 90), Debug::BLUE);

		if (useGravity) {
			Debug::Print("(G)ravity on", Vector2(5, 15), Debug::RED);
		}
		else {
			Debug::Print("(G)ravity off", Vector2(5, 15), Debug::RED);
		}
	}

	//This year we can draw debug textures as well!
	//Debug::DrawTex(*basicTex, Vector2(10, 10), Vector2(5, 5), Debug::MAGENTA);

	RayCollision closestCollision;
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::K) && selectionObject) {
		Vector3 rayPos;
		Vector3 rayDir;

		rayDir = selectionObject->GetTransform().GetOrientation() * Vector3(0, 0, -1);

		rayPos = selectionObject->GetTransform().GetPosition();

		Ray r = Ray(rayPos, rayDir);

		if (world->Raycast(r, closestCollision, true, selectionObject)) {
			if (objClosest) {
				objClosest->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
			}
			objClosest = (GameObject*)closestCollision.node;

			objClosest->GetRenderObject()->SetColour(Vector4(1, 0, 1, 1));
		}
	}

	if (world->GetEnemy()) {
		world->GetEnemy()->Update(dt);
	}

	if (testStateObject) { testStateObject->Update(dt); }

	//Debug::DrawLine(Vector3(), Vector3(0, 100, 0), Vector4(1, 0, 0, 1));

	if (devMode) {
		SelectObject();
		MoveSelectedObject();
	}

	AddKittenConstraints();

	world->UpdateWorld(dt);
	physics->Update(dt, world);

	/*
	if (paused = true) {
		PauseMenu();
	}*/
	/*
	renderer->Update(dt);
	renderer->Render();
	Debug::UpdateRenderables(dt);*/
}

void TutorialGame::UpdateKeys() {
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F1)) {
		InitWorld(); //We can reset the simulation at any time with F1
		selectionObject = nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::G) && devMode == true) {
		useGravity = !useGravity; //Toggle gravity!
		physics->UseGravity(useGravity);
	}
	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F9)) {
		world->ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F10)) {
		world->ShuffleConstraints(false);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F7)) {
		world->ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::F8)) {
		world->ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement();
	}
	else {
		DebugObjectMovement();
	}
}

void TutorialGame::LockedObjectMovement() {
	Matrix4 view		= world->GetMainCamera().BuildViewMatrix();
	Matrix4 camWorld	= Matrix::Inverse(view);

	Vector3 rightAxis = Vector3(camWorld.GetColumn(0)); //view is inverse of model!

	//forward is more tricky -  camera forward is 'into' the screen...
	//so we can take a guess, and use the cross of straight up, and
	//the right axis, to hopefully get a vector that's good enough!

	Vector3 fwdAxis = Vector::Cross(Vector3(0, 1, 0), rightAxis);
	fwdAxis.y = 0.0f;
	fwdAxis = Vector::Normalise(fwdAxis);

	if (Window::GetKeyboard()->KeyDown(KeyCodes::UP)) {
		selectionObject->GetPhysicsObject()->AddForce(fwdAxis);
	}

	if (Window::GetKeyboard()->KeyDown(KeyCodes::DOWN)) {
		selectionObject->GetPhysicsObject()->AddForce(-fwdAxis);
	}

	if (Window::GetKeyboard()->KeyDown(KeyCodes::NEXT)) {
		selectionObject->GetPhysicsObject()->AddForce(Vector3(0,-10,0));
	}
}

void TutorialGame::DebugObjectMovement() {
//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KeyCodes::LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM5)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
		}
	}
}

void TutorialGame::InitCamera() {
	world->GetMainCamera().SetNearPlane(0.1f);
	world->GetMainCamera().SetFarPlane(700.0f);
	world->GetMainCamera().SetPitch(-15.0f);
	world->GetMainCamera().SetYaw(-180.0f);
	world->GetMainCamera().SetPosition(Vector3(-60, 40, 60));
	lockedObject = nullptr;
}

void TutorialGame::InitWorld() {
	world->ClearAndErase();
	physics->Clear();

	BridgeConstraintTest();
	InitTable(Vector3(-50,-18,50));
	InitGameExamples();
	InitDefaultFloor();
	InitMaze();
	InitSphereGridWorld(Vector3(-190, -16, -190), 9,9,10.0f,10.0f,5.0f);
	InitCubeGridWorld(Vector3(-49, -15, 51), 5, 2, 3, 3.0f, 3.0f, 2.2f, Vector3(1, 1, 1));
	AddDoor(Vector3(110, -10, 20));

	testStateObject = AddStateObjectToWorld(Vector3(0, -10, 0));
}
/**/
void TutorialGame::AddKittenConstraints() {
	std::vector<KittenObject*>::const_iterator first;
	std::vector<KittenObject*>::const_iterator last;
	world->GetKittenIterators(first, last);
	for (auto i = first; i != last; ++i) {
		if ((*i)->GetCollected() == true && (*i)->GetFollowing() == false) {
			FollowConstraint* constraint = new FollowConstraint((*i), world->GetPlayer(), 5.0f);
			world->AddConstraint(constraint);
			(*i)->SetFollowing(true);
		}
	}

}
//*/
/*

A single function to add a large immoveable cube to the bottom of our world

*/
GameObject* TutorialGame::AddFloorToWorld(const Vector3& position, Vector3 floorSize, bool obb) {
	GameObject* floor = new GameObject();
	floor->SetAsFloor(true);

	if (!obb) {
		AABBVolume* volume = new AABBVolume(floorSize);
		floor->SetBoundingVolume((CollisionVolume*)volume);
	}
	else{
		OBBVolume* volume = new OBBVolume(floorSize); 
		floor->SetBoundingVolume((CollisionVolume*)volume);
	}
	
	floor->GetTransform()
		.SetScale(floorSize * 2.0f)
		.SetPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(floor);

	return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple' 
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(sphere);

	return sphere;
}

GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();
	cube->SetDim(dimensions);

	//AABBVolume* volume = new AABBVolume(dimensions);
	OBBVolume* volume = new OBBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2.0f);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	float meshSize		= 1.0f;
	float inverseMass	= 0.5f;
	
	PlayerObject* character = new PlayerObject();
	SphereVolume* volume  = new SphereVolume(1.0f);

	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), catMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();
	character->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));

	world->AddGameObject(character);
	world->AddPlayerObject(character);

	character->SetRay([&](Ray& r, RayCollision& closestCollision, bool closestObject, GameObject* ignore) -> bool { return world->Raycast(r, closestCollision, closestObject, ignore); });

	return character;
}

GameObject* TutorialGame::AddKittenToWorld(const Vector3& position) {
	float meshSize = 1.0f;
	float inverseMass = 0.5f;

	KittenObject* character = new KittenObject();
	SphereVolume* volume = new SphereVolume(1.0f);

	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), kittenMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();
	character->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));

	world->AddGameObject(character);
	world->AddKittenObject(character);

	return character;
}

GameObject* TutorialGame::AddEnemyToWorld(const Vector3& position) {
	float meshSize		= 2.0f;
	float inverseMass	= 0.5f;

	EnemyObject* character = new EnemyObject();
	character->SetRespawn(position);

	SphereVolume* volume = new SphereVolume(meshSize);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), gooseMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();
	character->GetRenderObject()->SetColour(Vector4(1,0,0,1));

	world->AddGameObject(character);
	world->AddEnemyObject(character);

	character->SetRay([&](Ray& r, RayCollision& closestCollision, bool closestObject, GameObject* ignore) -> bool { return world->Raycast(r, closestCollision, closestObject, ignore); });

	return character;
}

GameObject* TutorialGame::AddBonusToWorld(const Vector3& position, int tag) {
	GameObject* apple = new GameObject();

	SphereVolume* volume = new SphereVolume(0.5f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(2, 2, 2))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();
	apple->SetTag(tag);

	if (tag == 4) { apple->GetRenderObject()->SetColour(Vector4(0, 0, 1, 1)); }
	if (tag == 7) { apple->GetRenderObject()->SetColour(Vector4(0, 1, 1, 1)); }
	
	world->AddGameObject(apple);

	return apple;
}

StateGameObject* TutorialGame::AddStateObjectToWorld(const Vector3& position) {
	StateGameObject* apple = new StateGameObject();

	SphereVolume* volume = new SphereVolume(2.0f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(2, 2, 2))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), enemyMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();
	apple->SetTag(3);
	apple->GetRenderObject()->SetColour(Vector4(1, 0, 0, 1));

	world->AddGameObject(apple);

	return apple;
}

void TutorialGame::BridgeConstraintTest() {
	Vector3 cubeSize = Vector3(4, 4, 4);

	float invCubeMass = 5;
	int numLinks = 5;
	float maxDistance = 20;
	float cubeDistance = 15;

	Vector3 startPos = Vector3(20, -20, -30);

	GameObject* start = AddCubeToWorld(startPos + Vector3(0, 0, 0), cubeSize, 0);
	GameObject* end = AddCubeToWorld(startPos + Vector3((numLinks + 2) * cubeDistance, (numLinks + 2) * cubeDistance * 0.75f, 0), cubeSize, 0);

	GameObject* previous = start;

	for (int i = 0; i < numLinks; ++i) {
		GameObject* block = AddCubeToWorld(startPos + Vector3((i + 1) * cubeDistance, (i + 1) * cubeDistance * 0.75f, 0), cubeSize, invCubeMass);
		block->GetPhysicsObject()->SetRestitution(0.2f);
		PositionConstraint* constraint = new PositionConstraint(previous, block, maxDistance);
		world->AddConstraint(constraint);
		previous = block;
	}
	PositionConstraint* constraint = new PositionConstraint(previous, end, maxDistance);
	world->AddConstraint(constraint);
}

void TutorialGame::AddDoor(Vector3 startPos) {
	Vector3 sideSizes = Vector3(1, 3, 1);
	Vector3 doorSizes = Vector3(3, 2, 1);

	GameObject* postLeft = AddCubeToWorld(startPos, sideSizes, 0);
	GameObject* doorLeft = AddCubeToWorld(startPos + Vector3(6, 0, 0), doorSizes);
	HingeConstraint* postConstraint1 = new HingeConstraint(postLeft, doorLeft);
	PositionConstraint* positionConstraint1 = new PositionConstraint(postLeft, doorLeft, 5);

	GameObject* postRight = AddCubeToWorld(startPos + Vector3(16, 0, 0), sideSizes, 0);
	GameObject* doorRight = AddCubeToWorld(startPos + Vector3(10.5f, 0, 0), doorSizes);
	HingeConstraint* postConstraint2 = new HingeConstraint(postRight, doorRight);
	PositionConstraint* positionConstraint2 = new PositionConstraint(postRight, doorRight, 5);

	postLeft->GetRenderObject()->SetColour(Vector4(0, 0, 1, 1));
	doorLeft->GetRenderObject()->SetColour(Vector4(0, 0, 1, 1));
	postRight->GetRenderObject()->SetColour(Vector4(0, 0, 1, 1));
	doorRight->GetRenderObject()->SetColour(Vector4(0, 0, 1, 1));

	world->AddConstraint(postConstraint1);
	world->AddConstraint(postConstraint2);
	world->AddConstraint(positionConstraint1);
	world->AddConstraint(positionConstraint2);
}


void TutorialGame::InitDefaultFloor() {
	AddFloorToWorld(Vector3(0, -20, 0), Vector3(200, 2, 200),true);
	AddFloorToWorld(Vector3(80, -3, 80), Vector3(80, 2, 80), true);
	InitMapWalls(Vector3(200, 2, 200), -20.0f);
	AddCubeToWorld(Vector3(100,-11.3f, -17.3f), Vector3(20,2,20),0)->GetTransform().SetOrientation(Quaternion::EulerAnglesToQuaternion(25, 180, 0));
	AddCubeToWorld(Vector3(100, -11.3f, 177.3f), Vector3(20, 2, 20), 0)->GetTransform().SetOrientation(Quaternion::EulerAnglesToQuaternion(25, 0, 0));

	AddCubeToWorld(Vector3(-150, -16.0f, -100.0f), Vector3(50, 2, 2), 0);
	AddCubeToWorld(Vector3(-98, -16.0f, -148.0f), Vector3(2, 2, 50), 0);
}

void TutorialGame::InitMaze() {
	Vector3 wallCubeSize = Vector3(5.0f, 7.0f, 5.0f);
	float worldToLocalScale = 10.0f;
	for (int i = 0; i < 16; i++) {
		if (i != 10) {
			for (int j = 0; j < 16; j++) {
				if (i == 0 || i == 15) { AddCubeToWorld(Vector3(10 * i + 5, 6, 10 * j + 5), wallCubeSize, 0); }
				else if (j == 0 || j == 15) { AddCubeToWorld(Vector3(10 * i + 5, 6, 10 * j + 5), wallCubeSize, 0); }
			}
		
		}
	}
	AddCubeToWorld(Vector3(35, 6, 15), wallCubeSize, 0);
	AddCubeToWorld(Vector3(45, 6, 15), wallCubeSize, 0);
	for (int i = 0; i < 7; i++) {
		AddCubeToWorld(Vector3(65 + (i*10), 6, 25), wallCubeSize, 0);
	}
	for (int i = 0; i < 3; i++) {
		AddCubeToWorld(Vector3(25 + (i * 10), 6, 35), wallCubeSize, 0);
	}
	AddCubeToWorld(Vector3(85, 6, 35), wallCubeSize, 0);
	AddCubeToWorld(Vector3(25, 6, 45), wallCubeSize, 0);
	for (int i = 0; i < 2; i++) {
		AddCubeToWorld(Vector3(55 + (i * 10), 6, 45), wallCubeSize, 0);
	}
	AddCubeToWorld(Vector3(85, 6, 45), wallCubeSize, 0);
	for (int i = 0; i < 5; i++) {
		AddCubeToWorld(Vector3(105 + (i * 10), 6, 45), wallCubeSize, 0);
	}
	AddCubeToWorld(Vector3(25, 6, 55), wallCubeSize, 0);
	AddCubeToWorld(Vector3(45, 6, 55), wallCubeSize, 0);
	AddCubeToWorld(Vector3(95, 6, 65), wallCubeSize, 0);
	AddCubeToWorld(Vector3(115, 6, 65), wallCubeSize, 0);
	for (int i = 0; i < 2; i++) {
		AddCubeToWorld(Vector3(135 + (i * 10), 6, 65), wallCubeSize, 0);
	}
	for (int i = 0; i < 3; i++) {
		AddCubeToWorld(Vector3(25 + (i * 10), 6, 75), wallCubeSize, 0);
	}
	for (int i = 0; i < 3; i++) {
		AddCubeToWorld(Vector3(65, 6, 75 + (i * 10)), wallCubeSize, 0);
	}
	for (int i = 0; i < 3; i++) {
		AddCubeToWorld(Vector3(85, 6, 75 + (i * 10)), wallCubeSize, 0);
	}
	AddCubeToWorld(Vector3(95, 6, 75), wallCubeSize, 0);
	AddCubeToWorld(Vector3(115, 6, 75), wallCubeSize, 0);
	for (int i = 0; i < 3; i++) {
		AddCubeToWorld(Vector3(45, 6, 85 + (i * 10)), wallCubeSize, 0);
	}
	for (int i = 0; i < 3; i++) {
		AddCubeToWorld(Vector3(115 + (i * 10), 6, 85), wallCubeSize, 0);
	}
	for (int i = 0; i < 2; i++) {
		AddCubeToWorld(Vector3(15 + (i * 10), 6, 95), wallCubeSize, 0);
	}
	for (int i = 0; i < 3; i++) {
		AddCubeToWorld(Vector3(105 + (i * 10), 6, 95), wallCubeSize, 0);
	}
	for (int i = 0; i < 2; i++) {
		AddCubeToWorld(Vector3(25, 6, 105 + (i * 10)), wallCubeSize, 0);
	}
	AddCubeToWorld(Vector3(145, 6, 105), wallCubeSize, 0);
	for (int i = 0; i < 4; i++) {
		AddCubeToWorld(Vector3(115 + (i * 10), 6, 115), wallCubeSize, 0);
	}
	for (int i = 0; i < 5; i++) {
		AddCubeToWorld(Vector3(45 + (i * 10), 6, 125), wallCubeSize, 0);
	}
	for (int i = 0; i < 2; i++) {
		AddCubeToWorld(Vector3(25 + (i * 10), 6, 135), wallCubeSize, 0);
	}
	for (int i = 0; i < 4; i++) {
		AddCubeToWorld(Vector3(95 + (i * 10), 6, 135), wallCubeSize, 0);
	}
	for (int i = 0; i < 3; i++) {
		AddCubeToWorld(Vector3(55 + (i * 10), 6, 145), wallCubeSize, 0);
	}
}


void TutorialGame::InitGameExamples() {
	AddPlayerToWorld(Vector3(10, 5, -10));
	AddKittenToWorld(Vector3(-160, -16, -170));
	AddKittenToWorld(Vector3(-170, -16, -190));
	AddKittenToWorld(Vector3(-47, -16, 55));
	AddKittenToWorld(Vector3(125, 63, -30));
	AddKittenToWorld(Vector3(75, 2, 85));
	AddEnemyToWorld(Vector3(55, 10, 85));
	world->GetEnemy()->SetPlayer(world->GetPlayer());
	AddBonusToWorld(Vector3(15, 8, 105), (int) 4);
	AddBonusToWorld(Vector3(-175, -16, -175), (int) 7);
}

void TutorialGame::InitSphereGridWorld(Vector3 startPos, int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = startPos + Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddSphereToWorld(position, radius, 10.0f);
		}
	}
}

void TutorialGame::InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing) {
	float sphereRadius = 2.0f;
	Vector3 cubeDims = Vector3(2, 2, 2);

	/**/
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);

			if (rand() % 2) {
				AddCubeToWorld(position, cubeDims);
			}
			else {
				AddSphereToWorld(position, sphereRadius);
			}
		}
	}//*/
}

void TutorialGame::InitCubeGridWorld(Vector3 startPos, int numRows, int numCols, int numLayers, float rowSpacing, float colSpacing, float laySpacing, const Vector3& cubeDims) {
	for (int x = 1; x < numCols+1; ++x) {
		for (int y = 1; y < numLayers + 1; ++y) {
			for (int z = 1; z < numRows + 1; ++z) {
				Vector3 position = startPos + Vector3(x * colSpacing, y * laySpacing, z * rowSpacing);
				AddCubeToWorld(position, cubeDims, 1.0f);
			}
		}
	}
}

void TutorialGame::InitTable(const Vector3 tablePos) {
	Vector3 legDims = Vector3(0.5f, 2.0f, 0.5f);
	Vector3 tabTopDims = Vector3(5.0f, 0.25f, 10.0f);
	Vector3 positions[5] = { Vector3(0.5f, 2.0f, 0.5f),Vector3(9.5f, 2.0f,0.5f), Vector3(0.5f,2.0f,19.5f), Vector3(9.5f, 2.0f, 19.5f), Vector3(5.0f, 4.25f,10.0f) };
	for (int i = 0; i < 4; ++i) {
		AddCubeToWorld(tablePos + positions[i], legDims, 0.0f);
	}
	AddCubeToWorld(tablePos + positions[4], tabTopDims, 0.0f);

}

void TutorialGame::InitMapWalls(const Vector3 mapSize, float mapHeight) {
	Vector3 wallDimsZ = Vector3(mapSize.x, 15.0f, 5.0f);
	Vector3 wallDimsX = Vector3(5.0f, 15.0f, mapSize.z);
	AddCubeToWorld(Vector3(0.0f, mapHeight + 15.0f,  mapSize.z), wallDimsZ, 0.0f);
	AddCubeToWorld(Vector3(0.0f, mapHeight + 15.0f, -mapSize.z), wallDimsZ, 0.0f);
	AddCubeToWorld(Vector3( mapSize.x, mapHeight + 15.0f, 0.0f), wallDimsX, 0.0f);
	AddCubeToWorld(Vector3(-mapSize.x, mapHeight + 15.0f, 0.0f), wallDimsX, 0.0f);
}

/*
Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be 
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around. 

*/
bool TutorialGame::SelectObject() {
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::Q)) {
		inSelectionMode = !inSelectionMode;
		if (inSelectionMode) {
			Window::GetWindow()->ShowOSPointer(true);
			Window::GetWindow()->LockMouseToWindow(false);
		}
		else {
			Window::GetWindow()->ShowOSPointer(false);
			Window::GetWindow()->LockMouseToWindow(true);
		}
	}
	if (inSelectionMode) {
		Debug::Print("Press Q to change to camera mode!", Vector2(35, 15));

		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::Left)) {
			if (selectionObject) {	//set colour to deselected;
				selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
				selectionObject = nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(world->GetMainCamera());

			RayCollision closestCollision;
			if (world->Raycast(ray, closestCollision, true)) {
				selectionObject = (GameObject*)closestCollision.node;

				selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
				return true;
			}
			else {
				return false;
			}
		}
		if (Window::GetKeyboard()->KeyPressed(NCL::KeyCodes::L)) {
			if (selectionObject) {
				if (lockedObject == selectionObject) {
					lockedObject = nullptr;
				}
				else {
					lockedObject = selectionObject;
				}
			}
		}
	}
	else {
		Debug::Print("Press Q to change to select mode!", Vector2(35, 15));
	}
	return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/

void TutorialGame::MoveSelectedObject() {

	Debug::Print("Click Force:" + std::to_string(forceMagnitude), Vector2(55, 20));
	forceMagnitude += Window::GetMouse()->GetWheelMovement() * 100.0f;

	if (!selectionObject) {
		return;//we haven't selected anything!
	}
	//Push the selected object!
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::Right)) {
		Ray ray = CollisionDetection::BuildRayFromMouse(world->GetMainCamera());

		RayCollision closestCollision;
		if (world->Raycast(ray, closestCollision, true)) {
			if (closestCollision.node == selectionObject) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
			}
		}
	}
}


