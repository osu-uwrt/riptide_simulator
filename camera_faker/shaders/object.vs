#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;
layout (location = 2) in vec2 aCausticCoord;

out vec2 TexCoord;
out vec2 causticTexCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 worldPos;
out float fragDepth;

void main()
{
    worldPos = (model * vec4(aPos, 1.0f)).xyz;
    vec4 viewPos = view * model * vec4(aPos, 1.0f);
    gl_Position = projection * viewPos;
    fragDepth = viewPos.z / viewPos.w; 
    TexCoord = aTexCoord;
    causticTexCoord = aCausticCoord;
}
