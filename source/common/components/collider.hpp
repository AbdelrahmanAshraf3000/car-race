#pragma once
#include "../ecs/component.hpp"
#include "../shader/shader.hpp"
#include <glm/glm.hpp>
#include <string>
namespace our
{  
     
    class ColliderComponent : public Component
    {
    public:
        glm::vec3 center;
        glm::vec3 size;
        std::string id;
        std::string mesh;
        std::vector<std::string> collidingWith;
        static  std::string getID() { return "Collider"; }
        void deserialize(const nlohmann::json& data) override;
    };;
}