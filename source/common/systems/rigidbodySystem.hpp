#pragma once
#include "../ecs/world.hpp"
#include "../components/rigidbody.hpp"
#include "../application.hpp"
#include <tinyobj/tiny_obj_loader.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/trigonometric.hpp>
#include <glm/gtx/fast_trigonometry.hpp>
#include <memory>
#include <iostream>
#include <unordered_map>
#include <stdexcept>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include "../components/movement.hpp"

namespace our
{
    class RigidbodySystem
    {
    private:
        btDiscreteDynamicsWorld *dynWorld = nullptr;

        btRigidBody *createRigidBody(const std::string &meshPath, const btVector3 &position,
                                     const btVector3 &rotation, float mass, const btVector3 &scale = btVector3(1.0f, 1.0f, 1.0f))
        {
            // Load mesh using tinyobj
            tinyobj::attrib_t attrib;
            std::vector<tinyobj::shape_t> shapes;
            std::vector<tinyobj::material_t> materials;
            std::string warn, err;

            bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, meshPath.c_str());
            if (!ret)
                throw std::runtime_error("Failed to load mesh: " + meshPath + " " + err);

            btTriangleMesh *triangleMesh = new btTriangleMesh();
            for (const auto &shape : shapes)
            {
                for (size_t f = 0; f < shape.mesh.indices.size() / 3; f++)
                {
                    btVector3 vertices[3];
                    for (size_t v = 0; v < 3; v++)
                    {
                        tinyobj::index_t idx = shape.mesh.indices[3 * f + v];
                        vertices[v] = btVector3(
                            attrib.vertices[3 * idx.vertex_index] * scale.getX(),
                            attrib.vertices[3 * idx.vertex_index + 1] * scale.getY(),
                            attrib.vertices[3 * idx.vertex_index + 2] * scale.getZ());
                    }
                    triangleMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
                }
            }

            // Create the collision shape
            btCollisionShape *collisionShape;
            if (triangleMesh->getNumTriangles() > 0)
            {
                if (mass == 0.0f)
                {
                    // Static objects use triangle mesh
                    bool useQuantizedBvhTree = true;
                    collisionShape = new btBvhTriangleMeshShape(triangleMesh, useQuantizedBvhTree);
                }
                else
                {
                    // Dynamic objects use convex hull
                    btConvexHullShape *tmpShape = new btConvexHullShape();
                    for (size_t i = 0; i < attrib.vertices.size(); i += 3)
                    {
                        btVector3 vertex(
                            attrib.vertices[i] * scale.getX(),
                            attrib.vertices[i + 1] * scale.getY(),
                            attrib.vertices[i + 2] * scale.getZ());
                        tmpShape->addPoint(vertex);
                    }
                    collisionShape = tmpShape;
                }
            }
            else
            {
                // Fallback to box shape
                collisionShape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
            }
            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(position);
            transform.setRotation(btQuaternion(rotation.x(), rotation.y(), rotation.z()));

            btVector3 localInertia(0, 0, 0);
            if (mass != 0.0f)
            {
                collisionShape->calculateLocalInertia(mass, localInertia);
            }

            btDefaultMotionState *motionState = new btDefaultMotionState(transform);

            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, collisionShape, localInertia);

            btRigidBody *body = new btRigidBody(rbInfo);
            dynWorld->addRigidBody(body);

            return body;
        }

    public:
        Application *app;
        void enter(btDiscreteDynamicsWorld *world, Application *app)
        {
            this->app = app;
            if (!world)
                throw std::invalid_argument("Dynamic world cannot be null");
            dynWorld = world;
        }
        void quaternionToEuler(const btQuaternion &quat, float &yaw, float &pitch, float &roll)
        {
            // Constants to avoid magic numbers
            constexpr float SINGULARITY_THRESHOLD = 0.499f; // Close to 0.5
            constexpr float HALF_PI = glm::half_pi<float>();

            // Check for normalized quaternion
            float norm = quat.length2();
            // if (std::abs(norm - 1.0f) > 1e-6f)
            // {
            //     // Handle warning or error for non-normalized quaternion
            //     // Could throw an exception or normalize the quaternion here
            // }

            // Calculate common terms once
            float xy = quat.x() * quat.y();
            float zw = quat.z() * quat.w();
            float test = xy + zw; // sin(pitch) test for gimbal lock

            if (test > SINGULARITY_THRESHOLD)
            {
                // North pole singularity case (pitch = +90 degrees)
                yaw = HALF_PI;
                pitch = 2.0f * std::atan2(quat.x(), quat.w());
                roll = 0.0f;
            }
            else if (test < -SINGULARITY_THRESHOLD)
            {
                // South pole singularity case (pitch = -90 degrees)
                yaw = -HALF_PI;
                pitch = -2.0f * std::atan2(quat.x(), quat.w());
                roll = 0.0f;
            }
            else
            {
                // Regular case
                float xx = quat.x() * quat.x();
                float yy = quat.y() * quat.y();
                float zz = quat.z() * quat.z();

                yaw = std::atan2(2.0f * (quat.x() * quat.w() - quat.y() * quat.z()),
                                 1.0f - 2.0f * (xx + zz));
                pitch = std::asin(2.0f * test);
                roll = std::atan2(2.0f * (quat.y() * quat.w() - quat.x() * quat.z()),
                                  1.0f - 2.0f * (yy + zz));
            }
        }
        void update(World *world, float deltaTime)
        {
            if (!world) return;

            for (auto entity : world->getEntities())
            {
                auto *rigidbodyComponent = entity->getComponent<RigidbodyComponent>();
                if (!rigidbodyComponent) continue;

                // --- 1. Rigidbody Creation (Existing Logic) ---
                if (!rigidbodyComponent->addedToWorld)
                {
                    rigidbodyComponent->addedToWorld = true;
                    std::string path = "./assets/models/" + rigidbodyComponent->mesh + ".obj";
                    rigidbodyComponent->rigidbody = createRigidBody(
                        path,
                        btVector3(rigidbodyComponent->position.x, rigidbodyComponent->position.y, rigidbodyComponent->position.z),
                        btVector3(rigidbodyComponent->rotation.x, rigidbodyComponent->rotation.y, rigidbodyComponent->rotation.z),
                        rigidbodyComponent->mass,
                        btVector3(rigidbodyComponent->scale.x, rigidbodyComponent->scale.y, rigidbodyComponent->scale.z));

                    rigidbodyComponent->rigidbody->setFriction(0.7f);
                    rigidbodyComponent->rigidbody->setRollingFriction(0.1f);
                }

                // --- 2. Vehicle Logic ---
                if (rigidbodyComponent->input == 1 && rigidbodyComponent->mass != 0.0f)
                {
                    // Create Vehicle if needed
                    if (!rigidbodyComponent->vehicle) {
                        
                        btRaycastVehicle::btVehicleTuning tuning;
                         tuning = btRaycastVehicle::btVehicleTuning();
                         rigidbodyComponent->rigidbody->setActivationState(DISABLE_DEACTIVATION);
                         rigidbodyComponent->rigidbody->setDamping(0.1f, 0.5f);
                         btVehicleRaycaster *raycaster = new btDefaultVehicleRaycaster(dynWorld);
                         rigidbodyComponent->vehicle = new btRaycastVehicle(tuning, rigidbodyComponent->rigidbody, raycaster);
                         rigidbodyComponent->vehicle->setCoordinateSystem(0, 1, 2);
                         
                         btBoxShape *boxShape = static_cast<btBoxShape *>(rigidbodyComponent->rigidbody->getCollisionShape());
                         btVector3 chassisHalfExtents = boxShape->getHalfExtentsWithoutMargin();
                         
                         btVector3 wheelDir(0, -1, 0);
                         btVector3 wheelAxle(-1, 0, 0);
                         float restLength = 0.1f; 
                         float radius = 0.3f;     
                         float offsetX = 0.3f;    
                         float offsetZ = 0.4f;    
                         
                         btVector3 wheelPositions[] = {
                             btVector3(chassisHalfExtents.x() + radius + offsetX, 0, chassisHalfExtents.z() + radius + offsetZ),
                             btVector3(-chassisHalfExtents.x() - radius - offsetX, 0, chassisHalfExtents.z() + radius + offsetZ),
                             btVector3(chassisHalfExtents.x() + radius + offsetX, 0, -chassisHalfExtents.z() - radius - offsetZ),
                             btVector3(-chassisHalfExtents.x() - radius - offsetX, 0, -chassisHalfExtents.z() - radius - offsetZ)};
                         
                         for (int i = 0; i < 4; i++) {
                             bool isFrontWheel = (i < 2);
                             rigidbodyComponent->vehicle->addWheel(wheelPositions[i], wheelDir, wheelAxle, restLength, radius, tuning, isFrontWheel);
                             btWheelInfo &wheel = rigidbodyComponent->vehicle->getWheelInfo(i);
                             wheel.m_frictionSlip = 10000.0f;
                             wheel.m_rollInfluence = 1.5f;   
                         }
                         dynWorld->addVehicle(rigidbodyComponent->vehicle);
                    }

                    // --- 3. INPUT & BOOST LOGIC (Modified) ---
                    
                    // Check Boost Status FIRST
                    bool boostActive = app->getKeyboard().isPressed(GLFW_KEY_X);

                    // Define Limits based on state
                    float maxSpeed = boostActive ? 25.0f : 7.0f;   // Limit goes up when boosting
                    float acceleration = boostActive ? 4000.0f : 700.0f; // Force goes up when boosting

                    // Clamp Velocity Dynamicallly
                    btVector3 velocity = rigidbodyComponent->rigidbody->getLinearVelocity();
                    float currentSpeed = velocity.length();
                    if (currentSpeed > maxSpeed)
                    {
                        velocity = velocity.normalized() * maxSpeed;
                        rigidbodyComponent->rigidbody->setLinearVelocity(velocity);
                    }

                    // Input Variables
                    static float engineForce = 0.0f;
                    static float steeringValue = 0.0f;
                    float brakeForce = 0.0f;

                    // Handle Engine Force
                    if (boostActive) {
                         engineForce = acceleration; // Immediate boost power
                    }
                    else if (app->getKeyboard().isPressed(GLFW_KEY_UP)) {
                         engineForce += 2000.0f * deltaTime; // Smooth acceleration
                         if(engineForce > acceleration) engineForce = acceleration;
                    }
                    else if (app->getKeyboard().isPressed(GLFW_KEY_DOWN)) {
                         engineForce -= 2000.0f * deltaTime;
                         if(engineForce < -acceleration) engineForce = -acceleration;
                    }
                    else {
                         engineForce = 0.0f;
                         brakeForce = 2.0f; // Gradual deceleration
                    }

                    // Handle Steering (Smooth rotation)
                    float steeringSpeed = 2.0f; 
                    if (app->getKeyboard().isPressed(GLFW_KEY_LEFT)) {
                         steeringValue += steeringSpeed * deltaTime;
                    }
                    else if (app->getKeyboard().isPressed(GLFW_KEY_RIGHT)) {
                         steeringValue -= steeringSpeed * deltaTime;
                    }
                    else {
                         // Return to center
                         if(steeringValue > 0) steeringValue -= steeringSpeed * deltaTime;
                         if(steeringValue < 0) steeringValue += steeringSpeed * deltaTime;
                         if(abs(steeringValue) < 0.05f) steeringValue = 0;
                    }

                    // Clamp steering based on speed
                    float maxSteeringAngle = glm::mix(0.4f, 0.1f, glm::clamp(currentSpeed / 20.0f, 0.0f, 1.0f));
                    steeringValue = glm::clamp(steeringValue, -maxSteeringAngle, maxSteeringAngle);

                    if (app->getKeyboard().isPressed(GLFW_KEY_SPACE)) {
                        brakeForce = 50.0f;
                        engineForce = 0.0f;
                    }

                    // Apply to Vehicle
                    for (int i = 0; i < rigidbodyComponent->vehicle->getNumWheels(); i++)
                    {
                        rigidbodyComponent->vehicle->applyEngineForce(engineForce, i);
                        rigidbodyComponent->vehicle->setBrake(brakeForce, i);
                        if (i < 2) {
                            rigidbodyComponent->vehicle->setSteeringValue(steeringValue, i);
                        }
                    }
                    
                    rigidbodyComponent->vehicle->updateVehicle(deltaTime);
                    
                    // Handle Air Logic
                    bool isInAir = true;
                    for (int i = 0; i < rigidbodyComponent->vehicle->getNumWheels(); i++) {
                        if (rigidbodyComponent->vehicle->getWheelInfo(i).m_raycastInfo.m_isInContact) {
                            isInAir = false;
                            break;
                        }
                    }
                    if (isInAir) {
                        rigidbodyComponent->rigidbody->applyCentralForce(btVector3(0, 15, 0));
                        rigidbodyComponent->rigidbody->setAngularVelocity(btVector3(0, 0, 0));
                        dynWorld->setGravity(btVector3(0, -2, 0));
                    } else {
                        dynWorld->setGravity(btVector3(0, -9.81, 0));
                    }
                }
                
                // --- 4. Sync Visuals to Physics ---
                else if (rigidbodyComponent->rigidbody)
                {
                     btTransform transform;
                     rigidbodyComponent->rigidbody->getMotionState()->getWorldTransform(transform);
                     btVector3 pos = transform.getOrigin();
                     btQuaternion rot = transform.getRotation();
                     float yaw, pitch, roll;
                     quaternionToEuler(rot, yaw, pitch, roll);
                     rigidbodyComponent->position = glm::vec3(pos.x(), pos.y(), pos.z());
                     rigidbodyComponent->rotation = glm::vec3(yaw, roll, pitch);
                     entity->localTransform.position = rigidbodyComponent->position;
                     entity->localTransform.rotation = rigidbodyComponent->rotation;
                }

                // Sync Vehicle Visuals
                if (rigidbodyComponent->vehicle)
                {
                     btTransform transform;
                     rigidbodyComponent->rigidbody->getMotionState()->getWorldTransform(transform);
                     btVector3 pos = transform.getOrigin();
                     btQuaternion rot = transform.getRotation();
                     float yaw, pitch, roll;
                     quaternionToEuler(rot, yaw, roll, pitch); 
                     
                     rigidbodyComponent->position = glm::vec3(pos.x(), pos.y(), pos.z());
                     rigidbodyComponent->rotation = glm::vec3(yaw, pitch, roll);
                     
                     entity->localTransform.position = rigidbodyComponent->position;
                     entity->localTransform.position.y -= 0.07f; 
                     entity->localTransform.rotation = rigidbodyComponent->rotation;
                     
                     // Sync Wheels
                     float currentSteering = rigidbodyComponent->vehicle->getSteeringValue(0);
                     btVector3 velocity = rigidbodyComponent->rigidbody->getLinearVelocity();

                     for (auto child : world->getEntities()) {
                         if (child->parent == entity) {
                             std::string name = child->name;
                              if (name.find("tire") != std::string::npos) {
                                   MovementComponent *movement = child->getComponent<MovementComponent>();
                                   if(movement) {
                                        float groundSpeed = velocity.length();
                                        float forwardSpeed = velocity.dot(rigidbodyComponent->rigidbody->getWorldTransform().getBasis().getColumn(2));
                                        if (rigidbodyComponent->vehicle->getWheelInfo(0).m_raycastInfo.m_isInContact) {
                                            float rotationSpeed = groundSpeed * 2 * (forwardSpeed >= 0 ? 1 : -1);
                                            movement->angularVelocity = glm::vec3(rotationSpeed, 0.0f, 0.0f);
                                        } else {
                                            movement->angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
                                        }
                                   }
                                   if (name == "tireFront") {
                                        child->localTransform.rotation = glm::vec3(child->localTransform.rotation.x, currentSteering, 0.0f);
                                   }
                              }
                         }
                     }
                }
            }
        }
    };
}