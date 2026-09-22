#version 330 core
in vec2 uv;
out vec4 frag;
uniform sampler2D sceneColor,sceneDepth,bloomColor;
uniform mat4 inverseViewProjection;
uniform vec3 eye,sunDirection;
uniform float glare;
uniform vec2 texel;
uniform float exposure;
float luma(vec3 c){return dot(c,vec3(.299,.587,.114));}
void main(){
  vec3 c=texture(sceneColor,uv).rgb;
  vec3 n=texture(sceneColor,uv+vec2(0,texel.y)).rgb,s=texture(sceneColor,uv-vec2(0,texel.y)).rgb;
  vec3 e=texture(sceneColor,uv+vec2(texel.x,0)).rgb,w=texture(sceneColor,uv-vec2(texel.x,0)).rgb;
  float range=max(max(luma(n),luma(s)),max(luma(e),luma(w)))-min(min(luma(n),luma(s)),min(luma(e),luma(w)));
  // Conservative edge smoothing; no distortion of the calibrated camera projection.
  c=mix(c,(n+s+e+w)*.25,smoothstep(.12,.5,range)*.23);
  if(glare>0.){
    vec4 farPoint=inverseViewProjection*vec4(uv*2.-1.,1,1);
    vec3 ray=normalize(farPoint.xyz/farPoint.w-eye);
    float alignment=max(dot(ray,sunDirection),0.);
    // Sun disc/halo only over visible sky; foreground geometry occludes it.
    float sky=step(.999999,texture(sceneDepth,uv).r);
    c+=vec3(1.,.87,.60)*glare*sky*(pow(alignment,1800.)*18.+pow(alignment,90.)*.25);
  }
  // Camera bloom applies to full-brightness LEDs indoors too. HDR emission
  // spreads beyond each package without changing geometry, projection or depth.
  c+=texture(bloomColor,uv).rgb*(.60+.48*glare);
  c*=exposure;
  c=clamp((c*(2.51*c+.03))/(c*(2.43*c+.59)+.14),0.,1.);
  frag=vec4(pow(c,vec3(1./2.2)),1);
}
