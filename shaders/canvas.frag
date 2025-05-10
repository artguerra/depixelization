#version 460 core

out vec4 fragColor;

in vec2 fTexCoord;

uniform sampler2D tex;
uniform int imgHeight;
uniform int imgWidth;
uniform vec2 mousePos;

void main() {
  vec3 color = texture(tex, fTexCoord).rgb;

  // highlight the pixel under the mouse cursor
  vec2 pixelBegin = vec2(floor(fTexCoord.x * imgWidth), floor(fTexCoord.y * imgHeight));
  vec2 pixelEnd = vec2(ceil(fTexCoord.x * imgWidth), ceil(fTexCoord.y * imgHeight));

  if (mousePos.x > pixelBegin.x && mousePos.x < pixelEnd.x &&
    mousePos.y > pixelBegin.y && mousePos.y < pixelEnd.y) {
    color = mix(color, vec3(0.6, 0.6, 0.6), 0.5);
  }

  fragColor = vec4(color, 1.0);
}