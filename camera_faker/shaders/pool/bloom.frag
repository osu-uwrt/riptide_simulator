#version 330 core
in vec2 uv;
out vec4 frag;
uniform sampler2D source;
uniform vec2 stepSize;
uniform int extractBright;
void main(){
  vec3 c=vec3(0);
  if(extractBright==1){
    // Preserve small bright emitters when reducing the image to quarter size.
    for(int y=0;y<4;++y)for(int x=0;x<4;++x)
      c+=max(texture(source,uv+(vec2(x,y)-vec2(1.5))*stepSize).rgb-vec3(1.2),vec3(0));
    c/=16.;
  }else{
    const float weights[5]=float[5](.227027,.1945946,.1216216,.054054,.016216);
    c=texture(source,uv).rgb*weights[0];
    for(int i=1;i<5;++i)
      c+=(texture(source,uv+stepSize*float(i)).rgb+texture(source,uv-stepSize*float(i)).rgb)*weights[i];
  }
  frag=vec4(c,1);
}
