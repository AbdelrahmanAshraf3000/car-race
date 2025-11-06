#version 330 core

in Varyings {
    vec4 color;
    vec2 tex_coord;
} fs_in;

out vec4 frag_color;

uniform vec4 tint;
uniform sampler2D tex;
uniform float alphaThreshold;

void main(){
    //TODO: (Req 7) Modify the following line to compute the fragment color
    // by multiplying the tint with the vertex color and with the texture color 
    vec4 texColor = texture(tex, fs_in.tex_coord);
    if(texColor.a < alphaThreshold) discard;
    frag_color = tint * fs_in.color * texColor;
}