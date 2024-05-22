#version 330 core
// Output to the color and depth texture
layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec4 fragDepth;
  
in vec2 TexCoords;

// Textures
uniform sampler2D image;
uniform sampler2D depthTexture;
  
uniform bool horizontal;
uniform float weight[5] = float[] (0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216);

void main()
{             
    // gets size of single texel
    vec2 tex_offset = 1.0 / textureSize(image, 0);
    vec3 result = texture(image, TexCoords).rgb * weight[0]; // current fragment's contribution

    // Horizontally blur
    if(horizontal)
    {
        for(int i = 1; i < 5; ++i)
        {
            result += texture(image, TexCoords + vec2(tex_offset.x * i, 0.0)).rgb * weight[i];
            result += texture(image, TexCoords - vec2(tex_offset.x * i, 0.0)).rgb * weight[i];
        }
    }
    // Vertically blur
    else
    {
        for(int i = 1; i < 5; ++i)
        {
            result += texture(image, TexCoords + vec2(0.0, tex_offset.y * i)).rgb * weight[i];
            result += texture(image, TexCoords - vec2(0.0, tex_offset.y * i)).rgb * weight[i];
        }
    }

    // Output the blurred image and transfer the distance over (distance doesn't get blurred)
    fragColor = vec4(result, 1.0);
    fragDepth = vec4(texture(depthTexture, TexCoords).rrr,fragColor.a);

}