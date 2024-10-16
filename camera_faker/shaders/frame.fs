#version 330 core
layout(location=0) out vec4 fragColor;
layout(location=1) out vec4 fragDepth;
  
in vec2 TexCoords;

uniform sampler2D ourTexture;
uniform sampler2D depthTexture;

void main()
{
    fragColor = texture(ourTexture, TexCoords);
    float depth = texture(depthTexture, TexCoords).r;
    fragDepth = vec4(vec3(depth), fragColor.a);
}