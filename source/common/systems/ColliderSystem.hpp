#pragma once
#include "../ecs/world.hpp"
#include "../components/collider.hpp"
#include "../components/input.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/trigonometric.hpp>
#include <glm/gtx/fast_trigonometry.hpp>
#include <iostream>
namespace our
{

    class ColliderSystem
    {
    public:
        bool checkCollision(ColliderComponent *colliderA, ColliderComponent *colliderB)
        {
            std::cout << "Checking collision between " << colliderA->id << " and " << colliderB->id << std::endl;

            glm::vec3 minA = colliderA->center - colliderA->size * 0.5f;
            glm::vec3 maxA = colliderA->center + colliderA->size * 0.5f;
            glm::vec3 minB = colliderB->center - colliderB->size * 0.5f;
            glm::vec3 maxB = colliderB->center + colliderB->size * 0.5f;

            bool collisionX = maxA.x >= minB.x && minA.x <= maxB.x;
            bool collisionY = maxA.y >= minB.y && minA.y <= maxB.y;
            bool collisionZ = maxA.z >= minB.z && minA.z <= maxB.z;

            return collisionX && collisionY && collisionZ;
        }
        void update(World *world, float deltaTime)
        {
       

            for (auto entity : world->getEntities())
            {
                ColliderComponent *collider = entity->getComponent<ColliderComponent>();
                if (collider)
                {
              
                    collider->collidingWith.clear();
                    collider->center = entity->localTransform.position;
                }
            }
            for (auto entityA : world->getEntities())
            {
                ColliderComponent *colliderA = entityA->getComponent<ColliderComponent>();
                if (!colliderA)
                    continue;
                
                    
                

                for (auto entityB : world->getEntities())
                {
                    if (entityA == entityB)
                        continue;

                    ColliderComponent *colliderB = entityB->getComponent<ColliderComponent>();
                    if (!colliderB)
                        continue;

                    if (checkCollision(colliderA, colliderB))
                    {
                        
                        colliderA->collidingWith.push_back(colliderB->id);
                        colliderB->collidingWith.push_back(colliderA->id);
                    }
                }
            }
        }
    };

}
