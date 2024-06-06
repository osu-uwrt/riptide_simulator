#version 330 core
// Output to the color and depth texture
layout(location=0) out vec4 fragColor;
layout(location=1) out vec4 fragDepth;

// Pixel information from vertex shader  
in vec2 TexCoord;
in vec3 worldPos;
in vec3 relativePos;
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

void main()
{
    // Get strength of caustics at current pixel, increase light according
    vec4 lightColor = vec4(0.8,0.95,0.95,1.0f);
    vec4 causticColor = texture(causticTexture, causticTexCoord);
    lightColor += causticStrength * causticColor;

    // Make it foggier the further away the point is
    float dist = length(worldPos - cameraPos);
    float fogFactor = exp(-fogStrength * dist);
    fogFactor = clamp(fogFactor, 0.0, 1.0);

    // Set the pixel color, and distance in the depth map
    vec4 texColor = texture(ourTexture, TexCoord);

    // Discard pixel if it's pretty transparent (Uga booga method so objects don't need sorted by distance)
    // Otherwise, it would fill the depth testing buffer up and prevent other pixels from being drawn behind it
    if (texColor.a < 0.5)
        discard;
    // Set the pixel color, and distance in the depth map
    fragColor = mix(fogColor, texColor * lightColor, fogFactor);
    fragDepth = vec4(vec3(-relativePos.z), 1.0);
}
