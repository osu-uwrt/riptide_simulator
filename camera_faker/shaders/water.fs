#version 330 core
// Output to the color and depth texture
layout(location=0) out vec4 fragColor;
layout(location=1) out vec4 fragDepth;

// Pixel location information  
in vec3 worldPos;
in vec3 relativePos;
in vec2 distCords;
in vec4 clipspace;

// Textures
uniform sampler2D reflectionTex;
uniform sampler2D distortionTex;
uniform sampler2D refractionTex;

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

void main()
{
    // Find displacement vector from camera to the pixel
    vec3 displacement = worldPos - cameraPos; 
    
    // Find the normalized device coordinate, use that to find reflection texture coordinate
    vec2 ndc = (clipspace.xy/clipspace.w)/2.0 + 0.5;
    vec2 refractionTexCord = vec2(ndc.x, ndc.y);
    vec2 reflectionTexCord = vec2(ndc.x,-ndc.y);

    // To make the water wavy, will be distorting it with a distortion texture
    // Where the distortion texture is being samples should change over time
    // To not make it look like move in one direction, sample in two directions and average
    vec2 distortion1 = texture(distortionTex,vec2(distCords.x + moveFactor, distCords.y + moveFactor)).rg*2.0 - 1.0; 
    vec2 distortion2 = texture(distortionTex,vec2(distCords.y - moveFactor, distCords.x - moveFactor)).rg*2.0 - 1.0;
    vec2 reflectionTexCordDist = reflectionTexCord + waveDistortion*(distortion1 + distortion2);
    // Make sure the distorted texture won't sample off the screen!
    reflectionTexCordDist.x = clamp(reflectionTexCordDist.x, 0.001,0.999);
    reflectionTexCordDist.y = clamp(reflectionTexCordDist.y, -0.999, -0.001); // Negative because reflection gets flippedS

    // Sample the reflection and refraction textures (refraction isn't getting distorted, couldn't make it look good)
    vec4 reflectionColor = texture(reflectionTex, reflectionTexCordDist);
    vec4 refractionColor = texture(refractionTex, refractionTexCord);
    // This is jank but it worked the best so it's staying. When sampling the reflection teture,
    // there is an issue where it can sample outside of the water plane (but still on the screen) and it makes
    // the edges look terrible. To fix it, in the reflection rendering, the skybox isn't drawn, so the screen is just
    // black. If the reflectionColor is close to black (aka distorted outside of water rectangle) then just don't distort it
    if(all(lessThanEqual(reflectionColor.rgb,vec3(0.1,0.1,0.1))))
        reflectionColor = texture(reflectionTex, reflectionTexCord);

    // Sample the normal map at two changing locations like above and average
    vec3 normalMap1 = texture(distortionTex,vec2(distCords.x + moveFactor, distCords.y+ moveFactor)).rgb;
    vec3 normalMap2 = texture(distortionTex,vec2(distCords.x - moveFactor, distCords.y - moveFactor)).rgb;
    vec3 normalMap = (normalMap1 + normalMap2)/2.0;
    // The water should be more reflective the shallower angle it's looked at and more see through the more perpendicular it's looked at
    // This is called the Fresnel Effect and will be used to blend between the reflection and refraction texture
    // Figure out how alligned the camera view is with the normal vector
    vec3 norm = normalize(vec3((normalMap.r*2 - 1),(normalMap.g*2-1), normalMap.b+1));
    float fresnel = dot(norm,normalize(vec3(displacement.x, displacement.y, abs(displacement.z))));
    fresnel = clamp(fresnel,0.0,1.0);
    // This is just an aethetics choice, if camera is above water tint it blue and make reflections more sparkly
    refractionColor = mix(refractionColor,fogColor,.2);
    if (cameraPos.z>0)
        fresnel = sqrt(sqrt(fresnel));

    // Output the blended reflection and refraction texture and the distance
    fragColor = mix(reflectionColor,refractionColor,fresnel);
    fragDepth = vec4(vec3(-relativePos.z), 1.0);
}
