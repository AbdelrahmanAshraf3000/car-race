#pragma once

#include "../ecs/world.hpp"
#include "../components/sound.hpp"
#include "miniaudio.h"

namespace our
{

    // The sound system is responsible for moving every entity which contains a soundComponent.
    // This system is added as a simple example for how use the ECS framework to implement logic.
    // For more information, see "common/components/sound.hpp"
    class soundSystem
    {
    private:
        ma_engine engine;

    public:
        void initialize()
        {
            ma_result result = ma_engine_init(NULL, &engine);
            if (result != MA_SUCCESS)
            {
                std::cout << "Failed to initialize audio engine." << std::endl;
            }
        }
        // This should be called every frame to update all entities containing a soundComponent.
        void update(World *world, float deltaTime)
        {
            // For each entity in the world
            for (auto entity : world->getEntities())
            {
                // Get the sound component if it exists
                SoundComponent *soundComp = entity->getComponent<SoundComponent>();
                // If the sound component exists
                if (soundComp)
                {
                    if (!soundComp->playing)
                    {
                        const char *soundFile = soundComp->soundPath.c_str();
                        ma_result result = ma_sound_init_from_file(&engine, soundFile, MA_SOUND_FLAG_DECODE, NULL, NULL, &soundComp->sound);
                        if (result != MA_SUCCESS)
                        {
                            std::cout << "Failed to initialize sound from file." << std::endl;
                        }
                        else
                        {
                            ma_sound_set_volume(&soundComp->sound, soundComp->volume / 100.0);
                            if (soundComp->looped)
                            {
                                ma_sound_set_looping(&soundComp->sound, true); // Loop the sound
                            }
                            ma_sound_start(&soundComp->sound);
                            soundComp->playing = true;
                        }
                    }
                }
            }
        }
        void destroy()
        {
            ma_engine_uninit(&engine);
        }
    };

}
