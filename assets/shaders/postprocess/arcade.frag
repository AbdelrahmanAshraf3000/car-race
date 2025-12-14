#version 330

// The texture holding the scene pixels
uniform sampler2D tex;

// Read "assets/shaders/fullscreen.vert" to know what "tex_coord" holds;
in vec2 tex_coord;

out vec4 frag_color;

// Retro Arcade CRT Shader
// Simulates an old TV/Arcade monitor with curvature, scanlines, and vignette.

// Helper function to curve the UV coordinates like a glass tube
vec2 curve(vec2 uv) {
    uv = (uv - 0.5) * 2.0;
    uv *= 1.1; // Zoom out slightly to fit the curve
    uv.x *= 1.0 + pow((abs(uv.y) / 5.0), 2.0);
    uv.y *= 1.0 + pow((abs(uv.x) / 4.0), 2.0);
    uv = (uv / 2.0) + 0.5;
    uv = uv;
    return uv;
}

void main(){
    // 1. Apply Screen Curvature
    vec2 uv = curve(tex_coord);

    // 2. Check Bounds (Don't draw outside the "glass")
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0){
        frag_color = vec4(0.0, 0.0, 0.0, 1.0);
    } else {
        // 3. Chromatic Aberration (Simulate slight electron beam misalignment)
        float r = texture(tex, uv + vec2(0.001, 0.0)).r;
        float g = texture(tex, uv).g;
        float b = texture(tex, uv - vec2(0.001, 0.0)).b;
        vec3 color = vec3(r, g, b);

        // 4. Scanlines (Dark horizontal bands)
        // Uses a sine wave based on the Y coordinate
        // 800.0 controls the density of lines
        float scanline = sin(uv.y * 800.0) * 0.04;
        color -= scanline;

        // 5. Vignette (Darken the corners of the tube)
        float vig = (0.0 + 1.0 * 16.0 * uv.x * uv.y * (1.0 - uv.x) * (1.0 - uv.y));
        color *= vec3(pow(vig, 0.15));

        // 6. Boost brightness slightly to simulate phosphor glow
        color *= 1.1;

        frag_color = vec4(color, 1.0);
    }
}