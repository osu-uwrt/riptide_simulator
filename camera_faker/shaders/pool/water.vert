#version 330 core
layout(location=0)in vec3 position;
uniform mat4 model,view,projection;
out vec3 world;
void main(){world=(model*vec4(position,1)).xyz;gl_Position=projection*view*vec4(world,1);}
