#include "forward-renderer.hpp"
#include "../mesh/mesh-utils.hpp"
#include "../texture/texture-utils.hpp"

namespace our {

    void ForwardRenderer::initialize(glm::ivec2 windowSize, const nlohmann::json& config){
        // First, we store the window size for later use
        this->windowSize = windowSize;

        // Then we check if there is a sky texture in the configuration
        if(config.contains("sky")){
            // First, we create a sphere which will be used to draw the sky
            this->skySphere = mesh_utils::sphere(glm::ivec2(16, 16));
            
            // We can draw the sky using the same shader used to draw textured objects
            ShaderProgram* skyShader = new ShaderProgram();
            skyShader->attach("assets/shaders/textured.vert", GL_VERTEX_SHADER);
            skyShader->attach("assets/shaders/textured.frag", GL_FRAGMENT_SHADER);
            skyShader->link();
            
            PipelineState skyPipelineState{};
            skyPipelineState.depthTesting.enabled = true;
            skyPipelineState.depthTesting.function = GL_LEQUAL;
            skyPipelineState.faceCulling.enabled = true;
            skyPipelineState.faceCulling.culledFace = GL_FRONT;
            
            // Load the sky texture
            std::string skyTextureFile = config.value<std::string>("sky", "");
            Texture2D* skyTexture = texture_utils::loadImage(skyTextureFile, false);

            // Setup a sampler for the sky 
            Sampler* skySampler = new Sampler();
            skySampler->set(GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            skySampler->set(GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            skySampler->set(GL_TEXTURE_WRAP_S, GL_REPEAT);
            skySampler->set(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

            // Combine all the aforementioned objects (except the mesh) into a material 
            this->skyMaterial = new TexturedMaterial();
            this->skyMaterial->shader = skyShader;
            this->skyMaterial->texture = skyTexture;
            this->skyMaterial->sampler = skySampler;
            this->skyMaterial->pipelineState = skyPipelineState;
            this->skyMaterial->tint = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
            this->skyMaterial->alphaThreshold = 1.0f;
            this->skyMaterial->transparent = false;
        }

        // Then we check if there is a postprocessing shader in the configuration
        if(config.contains("postprocess")){
            glGenFramebuffers(1, &postprocessFrameBuffer);
            glBindFramebuffer(GL_FRAMEBUFFER, postprocessFrameBuffer);

            colorTarget = texture_utils::empty(GL_RGBA8, windowSize);
            depthTarget = texture_utils::empty(GL_DEPTH_COMPONENT24, windowSize);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTarget->getOpenGLName(), 0);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTarget->getOpenGLName(), 0);
            
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            // Create a vertex array to use for drawing the texture
            glGenVertexArrays(1, &postProcessVertexArray);

            // Create a sampler to use for sampling the scene texture in the post processing shader
            Sampler* postprocessSampler = new Sampler();
            postprocessSampler->set(GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            postprocessSampler->set(GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            postprocessSampler->set(GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            postprocessSampler->set(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

            // Create the post processing shader
            ShaderProgram* postprocessShader = new ShaderProgram();
            postprocessShader->attach("assets/shaders/fullscreen.vert", GL_VERTEX_SHADER);
            postprocessShader->attach(config.value<std::string>("postprocess", ""), GL_FRAGMENT_SHADER);
            postprocessShader->link();

            // Create a post processing material
            postprocessMaterial = new TexturedMaterial();
            postprocessMaterial->shader = postprocessShader;
            postprocessMaterial->texture = colorTarget;
            postprocessMaterial->sampler = postprocessSampler;
            postprocessMaterial->pipelineState.depthMask = false;
        }
    }

    void ForwardRenderer::destroy(){
        // Delete all objects related to the sky
        if(skyMaterial){
            delete skySphere;
            delete skyMaterial->shader;
            delete skyMaterial->texture;
            delete skyMaterial->sampler;
            delete skyMaterial;
        }
        // Delete all objects related to post processing
        if(postprocessMaterial){
            glDeleteFramebuffers(1, &postprocessFrameBuffer);
            glDeleteVertexArrays(1, &postProcessVertexArray);
            delete colorTarget;
            delete depthTarget;
            delete postprocessMaterial->sampler;
            delete postprocessMaterial->shader;
            delete postprocessMaterial;
        }
    }

    void ForwardRenderer::render(World* world){
        // First of all, we search for a camera and for all the mesh renderers
        CameraComponent* camera = nullptr;
        opaqueCommands.clear();
        transparentCommands.clear();
        lightCommands.clear(); // CHANGED: lights -> lightCommands
        
        for(auto entity : world->getEntities()){
            // If we hadn't found a camera yet, we look for a camera in this entity
            if(!camera) camera = entity->getComponent<CameraComponent>();
            
            // CHANGED: lights -> lightCommands
            if(auto light = entity->getComponent<LightComponent>(); light )
                lightCommands.push_back(light);
            
            // If this entity has a mesh renderer component
            if(auto meshRenderer = entity->getComponent<MeshRendererComponent>(); meshRenderer){
                // We construct a command from it
                RenderCommand command;
                command.localToWorld = meshRenderer->getOwner()->getLocalToWorldMatrix();
                command.center = glm::vec3(command.localToWorld * glm::vec4(0, 0, 0, 1));
                command.mesh = meshRenderer->mesh;
                command.material = meshRenderer->material;
                // if it is transparent, we add it to the transparent commands list
                if(command.material->transparent){
                    transparentCommands.push_back(command);
                } else {
                // Otherwise, we add it to the opaque command list
                    opaqueCommands.push_back(command);
                }
            }
        }

        // If there is no camera, we return (we cannot render without a camera)
        if(camera == nullptr) return;

        auto M = camera->getOwner()->getLocalToWorldMatrix();
        glm::vec3 eye = M * glm::vec4(0, 0, 0, 1);
        glm::vec3 center = M * glm::vec4(0, 0, -1, 1);
        glm::vec3 cameraForward = glm::normalize(center - eye);

        std::sort(transparentCommands.begin(), transparentCommands.end(), [cameraForward](const RenderCommand& first, const RenderCommand& second){
            float firstDistance = glm::dot(first.center, cameraForward);
            float secondDistance = glm::dot(second.center, cameraForward);
            return firstDistance > secondDistance;
        });

        glm::mat4 VP = camera->getProjectionMatrix(windowSize) * camera->getViewMatrix();
        
        glViewport(0, 0, windowSize.x, windowSize.y);
        
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClearDepth(1.0f);
        
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glDepthMask(GL_TRUE);

        if(postprocessMaterial){
            glBindFramebuffer(GL_FRAMEBUFFER, postprocessFrameBuffer);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        for(auto& command : opaqueCommands){
            command.material->setup();
            glm::mat4 modelMatrix = command.localToWorld;
            glm::mat4 transform = VP * modelMatrix;
            command.material->shader->set("transform", transform);
            
            if(dynamic_cast<LitMaterial*>(command.material)){
                command.material->shader->set("M", command.localToWorld);
                command.material->shader->set("VP", VP);
                command.material->shader->set("camera_position", eye);
                glm::mat4 M_IT = glm::transpose(glm::inverse(command.localToWorld));
                command.material->shader->set("M_IT", M_IT);
                
                // CHANGED: lights -> lightCommands
                command.material->shader->set("light_count", (int)lightCommands.size());
                command.material->shader->set("ambient_light", glm::vec3(0.1f, 0.1f, 0.1f));

                // CHANGED: lights -> lightCommands
                for(int i = 0; i < lightCommands.size() && i < 8; i++){ 
                    std::string prefix = "lights["+std::to_string(i)+"].";
                    
                    command.material->shader->set(prefix +"type", (int)lightCommands[i]->lightType);
                    command.material->shader->set(prefix +"color", lightCommands[i]->color);
                    command.material->shader->set(prefix +"attenuation", lightCommands[i]->attenuation);
                    command.material->shader->set(prefix +"inner_cone_angle", lightCommands[i]->inner_cone_angle);
                    command.material->shader->set(prefix +"outer_cone_angle", lightCommands[i]->outer_cone_angle);

                    auto modalMat = lightCommands[i]->getOwner()->getLocalToWorldMatrix();
                    glm::vec3 lightPosition = modalMat * glm::vec4(0, 0, 0, 1);
                    glm::vec3 lightDirection = glm::normalize(glm::vec3(modalMat * glm::vec4(0, 0, -1, 0))); 

                    command.material->shader->set(prefix + "position", lightPosition);
                    command.material->shader->set(prefix + "direction", lightDirection);
                }
            }
            command.mesh->draw();
        }

        if(this->skyMaterial){
            this->skyMaterial->setup();
            
            glm::vec3 cameraPosition = M * glm::vec4(0, 0, 0, 1);
            glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), cameraPosition);
            glm::mat4 alwaysBehindTransform = glm::mat4(
                1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 1.0f
            );
            glm::mat4 transform = alwaysBehindTransform * VP * modelMatrix;
            skyMaterial->shader->set("transform", transform);
            
            skySphere->draw();
        }

        for(auto& command : transparentCommands){
            command.material->setup();
            glm::mat4 modelMatrix = command.localToWorld;
            glm::mat4 transform = VP * modelMatrix;
            command.material->shader->set("transform", transform);
            
            if(dynamic_cast<LitMaterial*>(command.material)){
                command.material->shader->set("M", command.localToWorld);
                command.material->shader->set("VP", VP);
                command.material->shader->set("camera_position", eye);
                glm::mat4 M_IT = glm::transpose(glm::inverse(command.localToWorld));
                command.material->shader->set("M_IT", M_IT);
                
                // CHANGED: lights -> lightCommands
                command.material->shader->set("light_count", (int)lightCommands.size());
                command.material->shader->set("ambient_light", glm::vec3(0.1f, 0.1f, 0.1f));

                // CHANGED: lights -> lightCommands
                for(int i = 0; i < lightCommands.size() && i < 8; i++){ 
                    std::string prefix = "lights["+std::to_string(i)+"].";
                    
                    command.material->shader->set(prefix +"type", (int)lightCommands[i]->lightType);
                    command.material->shader->set(prefix +"color", lightCommands[i]->color);
                    command.material->shader->set(prefix +"attenuation", lightCommands[i]->attenuation);
                    command.material->shader->set(prefix +"inner_cone_angle", lightCommands[i]->inner_cone_angle);
                    command.material->shader->set(prefix +"outer_cone_angle", lightCommands[i]->outer_cone_angle);

                    auto modalMat = lightCommands[i]->getOwner()->getLocalToWorldMatrix();
                    glm::vec3 lightPosition = modalMat * glm::vec4(0, 0, 0, 1);
                    glm::vec3 lightDirection = glm::normalize(glm::vec3(modalMat * glm::vec4(0, 0, -1, 0))); 

                    command.material->shader->set(prefix + "position", lightPosition);
                    command.material->shader->set(prefix + "direction", lightDirection);
                }
            }
            command.mesh->draw();
        }

        if(postprocessMaterial){
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glViewport(0, 0, windowSize.x, windowSize.y);

            postprocessMaterial->setup();
            glBindVertexArray(postProcessVertexArray);
            glDrawArrays(GL_TRIANGLES, 0, 3);
            glBindVertexArray(0);
        }
    }
}