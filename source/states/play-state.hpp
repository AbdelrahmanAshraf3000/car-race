#pragma once

#include <application.hpp>

#include <ecs/world.hpp>
#include <systems/forward-renderer.hpp>
#include <systems/free-camera-controller.hpp>
#include <systems/rigidbodySystem.hpp>
#include <systems/ColliderSystem.hpp>
#include <systems/movement.hpp>
#include <asset-loader.hpp>
#include <btBulletCollisionCommon.h>
#include <systems/soundSystem.hpp>
#include <systems/miniaudio.h>

#include <btBulletDynamicsCommon.h>

class Playstate : public our::State
{
    our::World world;
    our::ForwardRenderer renderer;
    our::FreeCameraControllerSystem cameraController;
    our::MovementSystem movementSystem;
    our::RigidbodySystem rigidbodySystem;
    our::soundSystem soundSystem;
    our::ColliderSystem colliderSystem;
    
    // Physics World Pointers
    btDiscreteDynamicsWorld *dynamicsWorld = nullptr;
    btBroadphaseInterface *broadphase = nullptr;
    btDefaultCollisionConfiguration *collisionConfiguration = nullptr;
    btCollisionDispatcher *dispatcher = nullptr;
    btSequentialImpulseConstraintSolver *solver = nullptr;

    void onInitialize() override
    {
        // 1. Setup Bullet Physics (Initialize members)
        broadphase = new btDbvtBroadphase();
        collisionConfiguration = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfiguration);
        solver = new btSequentialImpulseConstraintSolver();

        dynamicsWorld = new btDiscreteDynamicsWorld(
            dispatcher,
            broadphase,
            solver,
            collisionConfiguration);

        dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

        // 2. Create Ground (Static)
        btCollisionShape *groundShape = new btBoxShape(btVector3(500, 0.5f, 500));
        btDefaultMotionState *groundMotionState = new btDefaultMotionState(
            btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 1.0f, 0)));

        btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(
            0, // mass = 0 (static)
            groundMotionState,
            groundShape,
            btVector3(0, 0, 0)
        );

        btRigidBody *groundRigidBody = new btRigidBody(groundRigidBodyCI);
        dynamicsWorld->addRigidBody(groundRigidBody);

        // 3. Load Scene
        auto &config = getApp()->getConfig()["scene"];
        if (config.contains("assets")) our::deserializeAllAssets(config["assets"]);
        if (config.contains("world")) world.deserialize(config["world"]);

        // 4. Initialize Systems
        our::Application *appPtr = getApp();
        cameraController.enter(appPtr);
        rigidbodySystem.enter(dynamicsWorld, appPtr);
        soundSystem.initialize();

        // 5. Initialize Renderer
        auto size = getApp()->getFrameBufferSize();
        renderer.setWorld(dynamicsWorld);
        renderer.initialize(size, config["renderer"]);
    }

    void onDraw(double deltaTime) override
    {
        // Update Logic
        movementSystem.update(&world, (float)deltaTime);
        cameraController.update(&world, (float)deltaTime);
        rigidbodySystem.update(&world, (float)deltaTime);
        soundSystem.update(&world, (float)deltaTime);
        colliderSystem.update(&world, (float)deltaTime);
        
        // Update Physics
        dynamicsWorld->stepSimulation((float)deltaTime, 10);

        // Draw
        renderer.render(&world);

        // Input
        if (getApp()->getKeyboard().justPressed(GLFW_KEY_ESCAPE))
        {
            getApp()->changeState("menu");
        }
    }

    void onDestroy() override
    {
        soundSystem.destroy();
        renderer.destroy();
        cameraController.exit();
        world.clear();
        our::clearAllAssets();

        // Cleanup Physics
        delete dynamicsWorld;
        delete solver;
        delete dispatcher;
        delete collisionConfiguration;
        delete broadphase;
    }
};