#version 330 core
in vec3 world;
out vec4 frag;
uniform sampler2D sceneColor,reflectionColor,sceneDepth;
uniform vec3 eye,sunDirection;
uniform vec3 waterTint;
uniform vec2 resolution;
uniform float time,directLight,ambientLight,glare;
uniform int outdoor;
uniform int hasReflection;
uniform mat4 mapToPool;
void main(){
  vec2 p=world.xy;
  vec2 slope=vec2(sin(p.x*2.4+p.y*1.7+time*.85)+.5*sin(p.x*7.1-p.y*3.2-time*1.1),
    cos(p.y*2.1-p.x*.8+time*.61)+.35*cos(p.y*8.2+p.x*4.1-time*1.3))*.034;
  vec3 n=normalize(vec3(slope,1)),v=normalize(eye-world);
  vec2 uv=gl_FragCoord.xy/resolution;
  vec2 offset=slope*.018;
  // Reject distortion across foreground silhouettes to avoid dragging the deck into the pool.
  vec2 refracted=clamp(uv+offset,vec2(.001),vec2(.999));
  if(texture(sceneDepth,refracted).r<gl_FragCoord.z)refracted=uv;
  vec3 below=texture(sceneColor,refracted).rgb;
  float fresnel=.02+.98*pow(1-abs(dot(n,v)),5);
  vec3 reflected=hasReflection==1?texture(reflectionColor,clamp(uv+offset*1.5,vec2(.001),vec2(.999))).rgb:(outdoor==1?vec3(.30,.48,.68):vec3(.13,.16,.18))*ambientLight;
  if(eye.z<0){n=-n;fresnel=clamp(fresnel+.18,0.,.9);reflected=waterTint;}
  vec3 color=mix(below,reflected,fresnel);
  vec3 sun=normalize(sunDirection);
  color+=vec3(1.,.94,.8)*pow(max(dot(reflect(-sun,n),v),0.),220.)*glare*directLight*15.;
  // Narrow bright edge at the pool wall is a visual meniscus, not a collision surface.
  vec2 q=(mapToPool*vec4(world,1)).xy;
  float edge=min(min(q.x,50.-q.x),min(q.y,22.86-q.y));
  color+=vec3(.08,.14,.15)*exp(-max(edge,0.)*32.);
  frag=vec4(color,1);
}
