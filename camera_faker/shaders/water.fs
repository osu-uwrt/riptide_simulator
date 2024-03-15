#version 330 core
out vec4 FragColor;
  
in vec3 worldPos;
in vec2 distCords;
in vec4 clipspace;

// Textures
uniform sampler2D reflectionTex;
uniform sampler2D dudv;
uniform sampler2D normal;

uniform float moveFactor;

// Camera stuff
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 cameraPos;

void main()
{
    vec3 displacement = worldPos - cameraPos; 
    float distance = length(displacement);

    float fogFactor = distance/15.0;
    fogFactor = clamp(fogFactor, 0.1, 1.0);
    
    vec2 ndc = (clipspace.xy/clipspace.w)/2.0 + 0.5;
    vec2 reflectionTexCord = vec2(ndc.x,-ndc.y);

    vec2 distortion1 = texture(dudv,vec2(distCords.x - moveFactor, distCords.y)).rg*2.0 - 1.0;
    vec2 distortion2 = texture(dudv,vec2(-distCords.x - moveFactor, distCords.y - moveFactor)).rg*2.0 - 1.0;

    reflectionTexCord += 0.03*(distortion1 + distortion2);

    vec4 reflectionColor = texture(reflectionTex,reflectionTexCord);

    vec3 normalMap1 = texture(normal,vec2(distCords.x - moveFactor, distCords.y)).rgb;
    vec3 normalMap2 = texture(normal,vec2(-distCords.x - moveFactor, distCords.y - moveFactor)).rgb;
    vec3 normalMap = (normalMap1 + normalMap2)/2.0;
    vec3 norm = vec3((normalMap.r*2 - 1), normalMap.b, (normalMap.g*2-1));
    norm = normalize(norm);
    float fresnel = sqrt(dot(norm,normalize(displacement))*5);
    fresnel = clamp(fresnel,0.0,1.0);
    //fresnel = sqrt(fresnel);
    fresnel = -1.0/(2*fresnel-3);
    //fresnel = sqrt(fresnel);
    fresnel = (fresnel + sqrt(dot(norm,normalize(displacement))))/2.0;
    fresnel = clamp(fresnel,0.3,1.0);


    vec4 refractionColor = vec4(0.443f, 0.807f, 0.976f, 1.0f);
    refractionColor = mix(refractionColor,vec4(0.404f, 0.752f, 0.945f, 1.0f),(1-dot(normalize(displacement),vec3(0,1,0))));
    FragColor = mix(reflectionColor,refractionColor,fresnel);
    FragColor = mix(FragColor,vec4(0.004f, 0.552f, 0.645f, 1.0f),fogFactor);
    FragColor = texture(normal,distCords);
}