#version 330 core
in vec2 texcoord;
uniform sampler2D albedo;
uniform int hasTexture;
uniform int holeCount;
uniform vec3 holes[4];
void main(){for(int i=0;i<holeCount;i++)if(distance(texcoord,holes[i].xy)<holes[i].z)discard;if(hasTexture==1 && texture(albedo,texcoord).a<.4)discard;}
