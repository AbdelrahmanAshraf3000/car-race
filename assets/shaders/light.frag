#version 330

// Output of Fragment Shader is Frag Color [Pixel Color]
out vec4 frag_color;


// Varyings
in Varyings {
    vec4 color;
    vec2 tex_coord;
    vec3 normal;
    vec3 view;
    vec3 world_position; // Position in the World Space
} fs_in;


// Light Properties
#define DIRECTIONAL 0
#define POINT       1
#define SPOT        2

#define MAX_LIGHTS 8

struct Light {
    int type; // Type of the Light Source
    vec3 direction; // Drection of the Light Source
    vec3 position; // Position of the Light Source
    vec3 color; // Color of the Light
    vec3 attenuation; // Attenuation of the Light

    // Cone Angles
    float inner_cone_angle; // Theta_p
    float outer_cone_angle; // Theta_u
};

uniform Light lights[MAX_LIGHTS];
uniform int light_count;

uniform vec3 ambient_light;


// Material Properties
struct Material {
    sampler2D albedo_map;
    sampler2D specular_map;
    sampler2D roughness_map;
    sampler2D ambient_occlusion_map;
    sampler2D emissive_map;
};
uniform Material material;


void main(){
    // Normalize the Varyings [Normal and View]
    vec3 view = normalize(fs_in.view);
    vec3 normal = normalize(fs_in.normal);

    // 1. Sample Texture Maps
    vec3 material_diffuse = texture(material.albedo_map, fs_in.tex_coord).rgb;
    vec3 material_specular = texture(material.specular_map, fs_in.tex_coord).rgb;
    float material_roughness = texture(material.roughness_map, fs_in.tex_coord).r; // Roughness 1 Channel is required :D
    vec3 material_ambient = material_diffuse * texture(material.ambient_occlusion_map, fs_in.tex_coord).r; // Ambient Occlusion Map we will use only 1 Channel :D
    vec3 material_emissive = texture(material.emissive_map, fs_in.tex_coord).rgb;

    // 2. Compute Material Shininess from Roughness [alpha]
    // shiness = 2 / (roughness^4) - 2 --> we added clamp to avoid division by zero and infinity
    float material_shininess = 2.0 / pow(clamp(material_roughness, 0.001, 0.999), 4.0) - 2.0; // shiness ranges [0.001, 0.999] 

    // Initialize the Color of the Pixel to Black
    vec3 color = vec3(0.0);

    // Compute Ambient (Independent of the Light Source) :D
    // vec3 ambient = ambient_light * material.ambient;
    vec3 ambient = ambient_light * material_ambient;

    // Add Ambient Component
    color += ambient;

    // 3. Add Emissive Component
    color += material_emissive;

    // Compute Diffuse and Specular Components for each Light Source
    for(int light_idx = 0; light_idx < light_count; light_idx++){
        // Get the Light Source
        Light light = lights[light_idx];

        vec3 light_direction; // Vector from the Fragment to the Light Source
        float attenuation = 1.0;

        if(light.type == DIRECTIONAL){
            light_direction = - light.direction; // Direction of the Light Source
        } else {    

            // Point Light or Spot Light
            vec3 frag_light_vector = light.position - fs_in.world_position; // Vector from the Fragment to the Light Source

            float distance = length(frag_light_vector); // Distance
            light_direction = frag_light_vector / distance; // Normalize the Light Direction

            // Compute Attenuation [1.0 / (c * 1 + l * d + q * d^2)] (Distance)
            attenuation = 1.0 / dot(light.attenuation, vec3(1.0, distance, distance * distance));

            if(light.type == SPOT){
                // Comoute Cone Attenuation
                float theta_s = acos(dot(light.direction, -light_direction));
                attenuation *= smoothstep(light.outer_cone_angle, light.inner_cone_angle, theta_s);
            }
        }

        // Diffuse Component [Diffuse = Kd * Id * Max(0,l.n) ]
        float lambert = max(0.0,dot(normal, light_direction)); // Lambert's Cosine Law
        // vec3 diffuse = light.color * material.diffuse * lambert;
        vec3 diffuse = light.color * material_diffuse * lambert;

        // Specular Component [Specular = Ks * Is * Max(0, (r.v))^alpha]
        vec3 r = reflect(-light_direction, normal); // both light_direction and normal are normalized vectors ---> r is also normalized
        // float phong = pow(max(0.0, dot(r,view)), material.alpha); // Note: r and view must be normalized :D  or else the result will be wrong <3
        float phong = pow(max(0.0, dot(r,view)), material_shininess); // Note: r and view must be normalized :D  or else the result will be wrong <3
        vec3 specular = light.color * material_specular * phong;


        // Add Diffuse and Specular Components of the current Light Source
        color += (diffuse + specular) * attenuation;
    }   
    
    // Set the color of the pixel
    frag_color = vec4(color , 1.0); 
}