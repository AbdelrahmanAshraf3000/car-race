#pragma once

#include "../ecs/component.hpp"

#include <glm/glm.hpp>

namespace our
{

    class InputComponent : public Component
    {
    public:
        static std::string getID() { return "InputMovement";}

        void deserialize(const nlohmann::json &data) override;
        InputComponent() = default;
        ~InputComponent() override = default;
    };

}