#include "rigidbody.hpp"
#include "../ecs/entity.hpp"
#include "../deserialize-utils.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

namespace our
{

    void RigidbodyComponent::deserialize(const nlohmann::json& data)
    {
        if (!data.is_object())
            return;

        position = data.value("position", position);
        rotation = glm::radians(data.value("rotation", rotation)); // Convert degrees to radians
        mesh = data.value("mesh", mesh);
        mass = data.value("mass", mass);
        scale = data.value("scale", scale);
        input = data.value("input", input);
        steeringAngle = data.value("steeringAngle", steeringAngle);
    }
}
