#version 330 core
// Vertex attributes
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNorm;
layout (location = 2) in vec4 aColor;

// Transformation matrices
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// Outputs to fragment shader
out vec3 worldPos;
out vec3 relativePos;
out vec3 norm;
out vec4 color;

void main()
{
    worldPos = (model * vec4(aPos, 1.0f)).xyz;
    relativePos = (view * model * vec4(aPos, 1.0f)).xyz;
    gl_Position =  projection * view * model * vec4(aPos, 1.0f);

    norm = aNorm;
    color = aColor;
}