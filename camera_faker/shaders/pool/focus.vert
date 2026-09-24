#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
uniform mat4 view,projection;
uniform vec3 center;
uniform float radius;
out vec3 viewNormal;
void main(){
    viewNormal=mat3(view)*normal;
    gl_Position=projection*view*vec4(center+radius*position,1);
}
