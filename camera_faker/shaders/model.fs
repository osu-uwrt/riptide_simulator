#version 330 core
// Output to the color and depth texture
layout(location=0) out vec4 fragColor;
layout(location=1) out vec4 fragDepth;

// Pixel information from vertex shader  
in vec3 worldPos;
in vec3 relativePos;
in vec3 norm;
in vec4 color;

// Water settings values
uniform vec4 fogColor;
uniform float fogStrength;

// Camera stuff
uniform vec3 cameraPos;

void main()
{
    vec4 lightColor = vec4(0.8,0.95,0.95,1.0f);

    // Make it foggier the further away the point is
    float dist = length(worldPos - cameraPos);
    float fogFactor = exp(-fogStrength*dist);
    fogFactor = clamp(fogFactor, 0.0, 1.0);

    vec4 lightStrength = vec4(vec3((dot(norm,vec3(0,0,1))+1.0)/2.0),1.0);
    // Discard pixel if it's pretty transparent (Uga booga method so objects don't need sorted by distance)
    // Otherwise, it would fill the depth testing buffer up and prevent other pixels from being drawn behind it
    if (color.a < 0.5)
        discard;

    fragColor = mix(fogColor,color*lightColor*lightStrength,fogFactor);
    fragDepth = vec4(vec3(-relativePos.z),1.0);
}