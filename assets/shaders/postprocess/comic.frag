#version 330

// The texture holding the scene pixels
uniform sampler2D tex;

// Read "assets/shaders/fullscreen.vert" to know what "tex_coord" holds;
in vec2 tex_coord;

out vec4 frag_color;

// Comic Book / Edge Detection Shader
// Detects edges in the image to draw outlines, giving a "Toon" or "Borderlands" style look.

float grayscale(vec3 c) {
    return dot(c, vec3(0.299, 0.587, 0.114));
}

void main(){
    // 1. Get the size of one pixel in texture coordinates
    // We need this to sample neighboring pixels
    vec2 tex_size = textureSize(tex, 0);
    vec2 pixel_step = 1.0 / tex_size;

    // 2. Define the neighboring pixels (kernel)
    // We sample the pixels around the current one to detect contrast changes
    float top    = grayscale(texture(tex, tex_coord + vec2(0.0, pixel_step.y)).rgb);
    float bottom = grayscale(texture(tex, tex_coord + vec2(0.0, -pixel_step.y)).rgb);
    float left   = grayscale(texture(tex, tex_coord + vec2(-pixel_step.x, 0.0)).rgb);
    float right  = grayscale(texture(tex, tex_coord + vec2(pixel_step.x, 0.0)).rgb);

    // 3. Calculate edge strength (Sobel-like filter approximation)
    // If the difference between top/bottom or left/right is high, it's an edge
    float edge_x = right - left;
    float edge_y = top - bottom;
    float edge = sqrt(edge_x * edge_x + edge_y * edge_y);

    // 4. Threshold the edge
    // If edge strength > 0.1, draw black line. Otherwise keep original color.
    vec4 scene_color = texture(tex, tex_coord);
    
    // Posterization: Reduce colors to give a simpler "painted" look (Optional)
    // floor(scene_color * n) / n
    scene_color.rgb = floor(scene_color.rgb * 8.0) / 8.0;

    if (edge > 0.05) {
        // Draw Outline (Black)
        frag_color = vec4(0.0, 0.0, 0.0, 1.0);
    } else {
        // Draw Scene
        frag_color = scene_color;
    }
}