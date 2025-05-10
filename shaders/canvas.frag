#version 460 core

out vec4 fragColor;

in vec2 fTexCoord;
uniform sampler2D tex;
uniform int imgHeight;
uniform int imgWidth;

void main() {
    vec3 color = texture(tex, fTexCoord).rgb;
    fragColor = vec4(color, 1.0);
}