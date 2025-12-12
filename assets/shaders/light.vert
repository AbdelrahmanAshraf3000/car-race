#version 330 // define glsl version 330 Compatable with OpenGL 3.3


// Varyings
out Varyings {
    vec4 color;
    vec2 tex_coord;
    vec3 normal;
    vec3 view; // Vector from the Fragment to the Camera
    vec3 world_position; // Position in the World Space
} vs_out;


// Attribute
layout(location = 0) in vec3 position;
layout(location = 1) in vec4 color; 
layout(location = 2) in vec2 texcoord;
layout(location = 3) in vec3 normal;

// Uniforms
uniform mat4 M;
uniform mat4 VP;
uniform mat4 M_IT;
uniform vec3 camera_position;


void main(){
    // Set the Position of the Vertex
    vec3 vertix_world_position = (M * vec4(position, 1.0)).xyz;
    gl_Position = VP * vec4(vertix_world_position, 1.0);


    // Set the Color of the Vertex
    vs_out.color = color;

    // Set the Texture Coordinate of the Vertex
    vs_out.tex_coord = texcoord;

    // Set the Normal of the Vertex 
    vs_out.normal = normalize(M_IT * vec4(normal,1.0)).xyz;

    // Set the View Vector
    vs_out.view = camera_position - vertix_world_position;
    
    // Set the Position of the Vertex in the World Space for the Fragment Shader
    vs_out.world_position = vertix_world_position;
}