#pragma once

#include "../components/light.hpp"
#include "../ecs/world.hpp"
#include "../components/camera.hpp"
#include "../components/mesh-renderer.hpp"
#include "../asset-loader.hpp"
#include "../components/rigidbody.hpp"

#include <glad/gl.h>
#include <vector>
#include <algorithm>
#include <btBulletDynamicsCommon.h> // Ensure Bullet is included

namespace our
{
    
    struct RenderCommand {
        glm::mat4 localToWorld;
        glm::vec3 center;
        Mesh* mesh;
        Material* material;
    };

    class ForwardRenderer {
        glm::ivec2 windowSize;
        std::vector<RenderCommand> opaqueCommands;
        std::vector<RenderCommand> transparentCommands;
        std::vector<LightComponent*> lightCommands;
        
        Mesh* skySphere;
        TexturedMaterial* skyMaterial;
        
        // Physics Debugging Members
        btDiscreteDynamicsWorld *dynWorld = nullptr;        
        // Postprocessing Members
        GLuint postprocessFrameBuffer, postProcessVertexArray;
        Texture2D *colorTarget, *depthTarget;
        TexturedMaterial* postprocessMaterial;

        bool debug = false;

    public:
        // --- THIS IS THE MISSING FUNCTION CAUSING YOUR ERROR ---
        void setWorld(btDiscreteDynamicsWorld *world){
            dynWorld = world;
        }
        // -------------------------------------------------------

        void initialize(glm::ivec2 windowSize, const nlohmann::json& config);
        void destroy();
        void render(World* world);
    };

}