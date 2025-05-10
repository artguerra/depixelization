#version 460 core

out vec4 fragColor;

in vec2 fTexCoord;
uniform sampler2D tex;

void main() {
  fragColor = vec4(texture(tex, fTexCoord).rgb, 1.0);
}