#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec3 color;
uniform mat4 model, view, projection;
uniform float pointSize;
out vec3 vColor;
void main() {
  gl_Position = projection * view * model * vec4(position, 1.);
  // Pull each sample slightly toward the eye so it wins the depth test
  // against the surface it was measured on.
  gl_Position.z -= 2e-3 * gl_Position.w;
  gl_PointSize = pointSize;
  vColor = color;
}
