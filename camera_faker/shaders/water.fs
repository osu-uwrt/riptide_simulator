#version 330 core
// Output to the color and depth texture
layout(location=0) out vec4 fragColor;
layout(location=1) out vec4 fragDepth;

// Pixel location information  
in vec3 worldPos;
in vec2 distCords;
in vec4 clipspace;

// Textures
uniform sampler2D reflectionTex;
uniform sampler2D dudv;
uniform sampler2D normal;

// Water settings
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
    // Find distance to point from camera
    vec3 displacement = worldPos - cameraPos; 
    float dist = length(displacement);

    // Use that distance to figure out how foggy the pixel should be
    float fogFactor = exp(-fogStrength*dist);
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    
    // Find the normalized device coordinate, use that to find reflection texture coordinate
    vec2 ndc = (clipspace.xy/clipspace.w)/2.0 + 0.5;
    vec2 reflectionTexCord = vec2(ndc.x,-ndc.y);

    // To make the water wavy, will be distorting it with a distortion texture
    // Where the distortion texture is being samples should change over time
    // To not make it look like move in one direction, sample in two wacky directions and average
    vec2 distortion1 = texture(dudv,vec2(distCords.x - moveFactor, distCords.y)).rg*2.0 - 1.0;
    vec2 distortion2 = texture(dudv,vec2(-distCords.x - moveFactor, distCords.y - moveFactor)).rg*2.0 - 1.0;
    reflectionTexCord += waveDistortion*(distortion1 + distortion2);
    // Sample the reflection texture coordinate at the calculated spot
    reflectionTexCord.x = clamp(reflectionTexCord.x, 0.001,0.999);
    reflectionTexCord.y = clamp(reflectionTexCord.y, -0.999, -0.001);
    vec4 reflectionColor = texture(reflectionTex,reflectionTexCord);

    // Sample the normal map at two changing locations like above and average
    vec3 normalMap1 = texture(normal,vec2(distCords.x - moveFactor, distCords.y)).rgb;
    vec3 normalMap2 = texture(normal,vec2(-distCords.x - moveFactor, distCords.y - moveFactor)).rgb;
    vec3 normalMap = (normalMap1 + normalMap2)/2.0;
    // The water should be more reflective the shallower angle it's looked at and more see through the more perpinicular it's looked at
    // This is called the Fresnel Effect and will be used to blend between the reflection and refraction texture
    // Figure out how alligned the camera view is with the normal vector
    vec3 norm = normalize(vec3((normalMap.r*2 - 1),(normalMap.g*2-1), normalMap.b+1));
    float fresnel = dot(norm,normalize(vec3(displacement.x, displacement.y, abs(displacement.z))));
    fresnel = clamp(fresnel,0.0,1.0);
                
    // Refraction color should just be see through (alpha = 0, transparent)
    vec4 refractionColor = vec4(reflectionColor.rgb, 0.0f);
    // If the camera is above water, need to change some things
    if (cameraPos.z > 0)
    {
        reflectionColor = refractionColor;
    }

    // Output the blended reflection and refraction texture and the distance
    fragColor = mix(reflectionColor,refractionColor,fresnel);
    float linearDepth = LinearizeDepth(gl_FragCoord.z);
    fragDepth = vec4(vec3(linearDepth), 1.0);
}
