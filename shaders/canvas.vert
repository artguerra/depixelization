#version 460 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aTexCoord;

out vec2 fTexCoord;

uniform mat4 projection;

void main() {
  gl_Position = projection * vec4(aPos, 1.0);
  fTexCoord = aTexCoord;
}