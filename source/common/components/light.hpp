#pragma once

#include "../ecs/component.hpp"
#include "../shader/shader.hpp"
#include <glm/glm.hpp>

namespace our{
    enum class LightType{
        DIRECTIONAL,
        POINT,
        SPOT
    };
    class LightComponent : public Component
    {
    public:

        LightType lightType = LightType::DIRECTIONAL ;
        glm::vec3 color = glm::vec3(1.0f,1.0f,1.0f);
        glm::vec3 attenuation = glm::vec3(1.0f,1.0f,1.0f);
        float inner_cone_angle = 0.0f ; // Theta_p
        float outer_cone_angle = 0.0f; // Theta_u
        
        static std::string getID() { return "Light"; }

        // Reads Light parameters from the given json object
        void deserialize(const nlohmann::json& data) override;
   
    };


}