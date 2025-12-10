#pragma once

#include "../ecs/component.hpp"
#include <glm/glm.hpp>
#include <btBulletDynamicsCommon.h>
#include <vector>
#include <string>
namespace our {

// Structure to represent a vertex
struct VertexRigidbody {
    float x, y, z;
};

// Structure to represent a face (triangle)
struct Face {
    int v1, v2, v3;
};

class RigidbodyComponent : public Component {
public:
    glm::vec3 position = {0, 0, 0};    // Initial position
    glm::vec3 rotation = {0, 0, 0};    // Initial rotation in degrees
    glm::vec3 scale = {1, 1, 1};       // Initial scale
    float mass = 1.0f;                 // Mass of the rigid body
    std::string mesh = "";             // Path to the OBJ file
    bool addedToWorld = false;         // Has the rigid body been added to the world?
    int input = 0;                     // Is the rigid body controlled by input?
    btRigidBody *rigidbody; 
    btRaycastVehicle *vehicle;         // EASE ADDED
    float steeringAngle = 0.0f;        // Current steering angle
    static std::string getID() {return "Rigidbody"; }   
    
    void deserialize(const nlohmann::json& data) override;
};

} 
