#include "collider.hpp"
#include "../ecs/entity.hpp"
#include "../deserialize-utils.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

namespace our
{

    void ColliderComponent::deserialize(const nlohmann::json& data)
    {
        if (!data.is_object())
            return;

        center = data.value("center", center);  
        size = data.value("size", size);
        id = data.value("id", id);
        mesh = data.value("mesh", mesh);
    }
}
