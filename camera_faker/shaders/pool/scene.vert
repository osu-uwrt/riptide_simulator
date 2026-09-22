#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in vec2 uv;
uniform mat4 model, view, projection, lightMatrix, mapToPool;
out vec3 world, norm, poolPosition, poolNormal;
out vec2 texcoord;
out vec4 lightPosition;
void main(){
  vec4 p=model*vec4(position,1);
  world=p.xyz; norm=normalize(transpose(inverse(mat3(model)))*normal);
  poolNormal=mat3(mapToPool)*norm;
  poolPosition=(mapToPool*p).xyz;
  texcoord=uv;lightPosition=lightMatrix*p;
  gl_Position=projection*view*p;
}
