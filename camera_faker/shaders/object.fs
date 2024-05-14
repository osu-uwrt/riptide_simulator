#version 330 core
layout(location=0) out vec4 fragColor;
layout(location=1) out vec4 fragDepth;
  
in vec2 TexCoord;
in vec3 worldPos;
in vec2 causticTexCoord;

// Water settings values
uniform vec4 fogColor;
uniform float fogStrength;
uniform float causticStrength;

// Textures
uniform sampler2D ourTexture;
uniform sampler2D causticTexture;

// Camera stuff
uniform vec3 cameraPos;

const float near = 0.2;  // Hardcoded near plane value
const float far = 100.0; // Hardcoded far plane value

float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0; // Back to NDC
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main()
{
    // Blend caustic images together, get strength of caustic at current pixel
    vec4 lightColor = vec4(0.8,0.95,0.95,1.0f);
    vec4 causticColor = texture(causticTexture, causticTexCoord);
    lightColor += causticStrength * causticColor;

    float dist = length(worldPos - cameraPos);
    float fogFactor = exp(-fogStrength * dist);
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    vec4 texColor = texture(ourTexture, TexCoord);
    if (texColor.a < 0.5)
        discard;
    // Set the pixel color, and distance in the depth map
    fragColor = mix(fogColor, texColor * lightColor, fogFactor);
    float linearDepth = LinearizeDepth(gl_FragCoord.z);
    fragDepth = vec4(vec3(linearDepth), 1.0);
}
