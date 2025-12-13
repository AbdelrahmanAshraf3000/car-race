#pragma once
#include "../ecs/component.hpp"
#include "../application.hpp"
#include "../systems/miniaudio.h"

namespace our
{
    class SoundComponent : public Component
    {
    public:
        std::string soundPath;
        bool looped = false;
        bool playing = false;
        ma_sound sound;
        int volume = 100;

        static std::string getID() { return "Sound"; }

        // Load settings from JSON
        void deserialize(const nlohmann::json& data) override;
    };
} // namespace our
