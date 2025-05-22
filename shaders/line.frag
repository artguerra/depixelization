#version 460 core

out vec4 fragColor;

uniform vec3 strokeColor;

void main() {
  fragColor = vec4(strokeColor, 1.0);
}