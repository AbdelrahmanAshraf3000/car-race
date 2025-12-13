#include "sound.hpp"
#include "../deserialize-utils.hpp"
#include "../application.hpp"
#define MINIAUDIO_IMPLEMENTATION
#include "../systems/miniaudio.h"

namespace our
{
    void SoundComponent::deserialize(const nlohmann::json &data)
    {
        soundPath = data.value("soundPath", "");
        looped = data.value("looped", false);
        playing = data.value("playing", false);
        volume = data.value("volume", 100);
    }
}


