#pragma once
#include "../ecs/world.hpp"
#include "../components/rigidbody.hpp"
#include "../application.hpp"
#define TINYOBJLOADER_IMPLEMENTATION
#include <tinyobj/tiny_obj_loader.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

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
            if (!world)
                return;

            for (auto entity : world->getEntities())
            {
                auto *rigidbodyComponent = entity->getComponent<RigidbodyComponent>();
                if (!rigidbodyComponent)
                    continue;

                // Create the rigid body if not already in the world
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

                    // Set friction for the rigid body
                    rigidbodyComponent->rigidbody->setFriction(0.7f);
                    rigidbodyComponent->rigidbody->setRollingFriction(0.1f);
                }

                // If this object needs a vehicle and has non-zero mass
                if (rigidbodyComponent->input == 1 && rigidbodyComponent->mass != 0.0f)
                {
                    // Create RaycastVehicle once
                    if (!rigidbodyComponent->vehicle)
                    {
                        // Vehicle tuning and setup
                        btRaycastVehicle::btVehicleTuning tuning;
                        tuning = btRaycastVehicle::btVehicleTuning();
                        // Configure vehicle rigid body
                        rigidbodyComponent->rigidbody->setActivationState(DISABLE_DEACTIVATION);
                        // Add angular and linear damping
                        rigidbodyComponent->rigidbody->setDamping(0.1f, 0.5f); // Set linear and angular damping

                        btVehicleRaycaster *raycaster = new btDefaultVehicleRaycaster(dynWorld);
                        rigidbodyComponent->vehicle = new btRaycastVehicle(tuning, rigidbodyComponent->rigidbody, raycaster);
                        rigidbodyComponent->vehicle->setCoordinateSystem(0, 1, 2);

                        btBoxShape *boxShape = static_cast<btBoxShape *>(rigidbodyComponent->rigidbody->getCollisionShape());
                        btVector3 chassisHalfExtents = boxShape->getHalfExtentsWithoutMargin();

                        btVector3 wheelDir(0, -1, 0);
                        btVector3 wheelAxle(-1, 0, 0);
                        float restLength = 0.1f; // Increase rest length
                        float radius = 0.3f;     // Smaller wheel radius for stability
                        float offsetX = 0.3f;    // Wider wheel base
                        float offsetZ = 0.4f;    // Longer wheel base
                        btVector3 wheelPositions[] = {
                            btVector3(chassisHalfExtents.x() + radius + offsetX, 0, chassisHalfExtents.z() + radius + offsetZ),
                            btVector3(-chassisHalfExtents.x() - radius - offsetX, 0, chassisHalfExtents.z() + radius + offsetZ),
                            btVector3(chassisHalfExtents.x() + radius + offsetX, 0, -chassisHalfExtents.z() - radius - offsetZ),
                            btVector3(-chassisHalfExtents.x() - radius - offsetX, 0, -chassisHalfExtents.z() - radius - offsetZ)};

                        // Add wheels with friction
                        for (int i = 0; i < 4; i++)
                        {
                            bool isFrontWheel = (i < 2);
                            rigidbodyComponent->vehicle->addWheel(wheelPositions[i], wheelDir, wheelAxle, restLength, radius, tuning, isFrontWheel);

                            // Set wheel friction
                            btWheelInfo &wheel = rigidbodyComponent->vehicle->getWheelInfo(i);
                            wheel.m_frictionSlip = 10000.0f; // Lower friction slip
                            wheel.m_rollInfluence = 1.5f;    // Reduce roll influence for better stability
                        }
                        dynWorld->addVehicle(rigidbodyComponent->vehicle);
                    }
                }
            }
        }
    };
}
