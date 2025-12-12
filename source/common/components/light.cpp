#include "light.hpp"
#include "../deserialize-utils.hpp"


namespace our{

        void LightComponent::deserialize(const nlohmann::json& data){
            if(!data.is_object()) return;
            

            std::string lightTypeStr = data.value("lightType","directional");
            std::transform(lightTypeStr.begin(), lightTypeStr.end(), lightTypeStr.begin(),
                   [](unsigned char c){ return std::tolower(c); });

            if(lightTypeStr == "directional"){
                lightType = LightType::DIRECTIONAL;
            }
            else if(lightTypeStr == "point"){
                lightType = LightType::POINT;
            }
            else if (lightTypeStr == "spot"){
                lightType = LightType::SPOT;
            }
            else{
                lightType = LightType::DIRECTIONAL;
            }
            attenuation = data.value("attenuation",attenuation);
            inner_cone_angle = data.value("innerConeAngle",inner_cone_angle);
            outer_cone_angle = data.value("outerConeAngle",outer_cone_angle);

        }
}