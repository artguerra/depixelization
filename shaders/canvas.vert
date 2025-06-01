#version 460 core

layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aTexCoord;

out vec2 fTexCoord;

uniform mat4 projection;

void main() {
  gl_Position = projection * vec4(aPos, 0.0, 1.0);
  fTexCoord = vec2(aTexCoord.x, 1.0 - aTexCoord.y); // Flip Y coordinate for texture
}