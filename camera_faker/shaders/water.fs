#version 330 core
layout(location=0) out vec4 fragColor;
layout(location=1) out vec4 fragDepth;
  
in vec3 worldPos;
in vec2 distCords;
in vec4 clipspace;

// Textures
uniform sampler2D reflectionTex;
uniform sampler2D dudv;
uniform sampler2D normal;

uniform vec4 fogColor;
uniform float moveFactor;
uniform float fogStrength;
uniform float waveDistortion;

// Camera stuff
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 cameraPos;
uniform float near;
uniform float far;

float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0; // Back to NDC
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main()
{
    vec3 displacement = worldPos - cameraPos; 
    float dist = length(displacement);

    float fogFactor = exp(-fogStrength*dist);
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    
    vec2 ndc = (clipspace.xy/clipspace.w)/2.0 + 0.5;
    vec2 reflectionTexCord = vec2(ndc.x,-ndc.y);

    vec2 distortion1 = texture(dudv,vec2(distCords.x - moveFactor, distCords.y)).rg*2.0 - 1.0;
    vec2 distortion2 = texture(dudv,vec2(-distCords.x - moveFactor, distCords.y - moveFactor)).rg*2.0 - 1.0;
    reflectionTexCord += waveDistortion*(distortion1 + distortion2);
    reflectionTexCord.x = clamp(reflectionTexCord.x, 0.001,0.999);
    reflectionTexCord.y = clamp(reflectionTexCord.y, -0.999, -0.001);
    vec4 reflectionColor = texture(reflectionTex,reflectionTexCord);

    vec3 normalMap1 = texture(normal,vec2(distCords.x - moveFactor, distCords.y)).rgb;
    vec3 normalMap2 = texture(normal,vec2(-distCords.x - moveFactor, distCords.y - moveFactor)).rgb;
    vec3 normalMap = (normalMap1 + normalMap2)/2.0;
    vec3 norm = normalize(vec3((normalMap.r*2 - 1),(normalMap.g*2-1), normalMap.b+1));
    float fresnel = dot(norm,normalize(displacement));
    fresnel = clamp(fresnel,0.0,1.0);

    vec4 refractionColor = vec4(reflectionColor.rgb, 0.0f);
    if (cameraPos.z > 0)
    {
        reflectionColor = refractionColor;
        refractionColor = vec4(0.568f, 0.878f, 1.0f, 1.0f);
    }
    fragColor = mix(reflectionColor,refractionColor,fresnel);
    float linearDepth = LinearizeDepth(gl_FragCoord.z);
    fragDepth = vec4(vec3(linearDepth), 1.0);
}
