#version 330 core
// Output to the color and depth texture
layout(location=0) out vec4 fragColor;
layout(location=1) out vec4 fragDepth;

// Pixel information from vertex shader  
in vec3 worldPos;
in vec3 relativePos;
in vec3 norm;
in vec4 color;

// Caustic inputs
uniform float causticScaleFactor;
uniform sampler2D causticTexture;
uniform float causticStrength;

// Water settings values
uniform vec4 fogColor;
uniform float fogStrength;

// Camera stuff
uniform vec3 cameraPos;
uniform int clippingMode;

void main()
{
    // Apply clipping based on the clippingMode
    // This is used for water reflections so object above water isn't used for reflections etc.
    if (clippingMode == 1 && worldPos.z > 0.0) {
        discard;
    } else if (clippingMode == 2 && worldPos.z < 0.0) {
        discard;
    }
    vec4 lightStrength = vec4(vec3((dot(norm,vec3(0,0,1))*.35+.65)),1.0);
    if (worldPos.z > 0)
    {
        fragColor = color*lightStrength;
        return;
    }

    vec4 lightColor = vec4(0.8,0.95,0.95,1.0f);
    // Make it foggier the further away the point is
    float dist = length(worldPos - cameraPos);
    float fogFactor = exp(-fogStrength*dist);
    fogFactor = clamp(fogFactor, 0.0, 1.0);

    vec4 causticColor = texture(causticTexture, vec2(worldPos.x/causticScaleFactor, worldPos.y/causticScaleFactor));
    float causticNormalStrength = clamp(10*dot(norm,vec3(0,0,1)),0,1);

    lightStrength += causticStrength * causticNormalStrength *causticColor;
    


    // Discard pixel if it's pretty transparent (Uga booga method so objects don't need sorted by distance)
    // Otherwise, it would fill the depth testing buffer up and prevent other pixels from being drawn behind it
    if (color.a < 0.5)
        discard;

    fragColor = mix(fogColor,color*lightColor*lightStrength,fogFactor);
    fragDepth = vec4(vec3(-relativePos.z),1.0);
}