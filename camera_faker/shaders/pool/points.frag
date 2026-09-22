#version 330 core
in vec3 vColor;
uniform float exposure, highlight;
uniform vec3 highlightColor;
out vec4 frag;
void main() {
  // Sensor RGB is the post-processed image; undo gamma and exposure so the
  // point reproduces that color once the post pass runs again.
  vec3 linear = pow(vColor, vec3(2.2)) / max(exposure, 1e-3);
  vec3 tinted = pow(highlightColor, vec3(2.2)) / max(exposure, 1e-3);
  frag = vec4(mix(linear, tinted, highlight), 1.);
}
