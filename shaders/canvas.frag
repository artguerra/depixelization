#version 460 core

out vec4 fragColor;

in vec2 fTexCoord;

uniform sampler2D tex;
uniform int imgHeight;
uniform int imgWidth;
uniform vec2 mousePos;

void main() {
  vec4 color = texture(tex, fTexCoord);

  // highlight the pixel under the mouse cursor
  vec2 pixelBegin = vec2(floor(fTexCoord.x * imgWidth), floor(fTexCoord.y * imgHeight));
  vec2 pixelEnd = vec2(ceil(fTexCoord.x * imgWidth), ceil(fTexCoord.y * imgHeight));

  if (color.a == 0.0) {
    // checkerboard pattern
    // vec2 checker = mod(pixelBegin, 2.0);
    // if (checker.x == 0.0 && checker.y == 0.0) {
    //   color = vec4(1.0, 1.0, 1.0, 1.0);
    // } else if (checker.x == 1.0 && checker.y == 1.0) {
    //   color = vec4(1.0, 1.0, 1.0, 1.0);
    // } else {
    //   color = vec4(0.8, 0.8, 0.8, 1.0);
    // }

    color = vec4(1.0, 1.0, 1.0, 1.0);
  }

  if (mousePos.x > pixelBegin.x && mousePos.x < pixelEnd.x &&
    mousePos.y > pixelBegin.y && mousePos.y < pixelEnd.y) {
    color.rgb = mix(color.rgb, vec3(0.6, 0.6, 0.6), 0.5);
  }

  fragColor = color;
}