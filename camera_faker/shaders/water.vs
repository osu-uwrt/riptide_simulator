#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aDistCord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 worldPos;
out vec4 clipspace;
out vec2 distCords;

void main()
{
    worldPos = (model * vec4(aPos, 1.0f)).xyz;
    clipspace = projection * view * model * vec4(aPos, 1.0f);
    gl_Position = clipspace;
    distCords = aDistCord;
}