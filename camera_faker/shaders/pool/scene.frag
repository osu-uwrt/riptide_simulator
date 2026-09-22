#version 330 core
in vec3 world,norm,poolPosition,poolNormal;
in vec2 texcoord;
in vec4 lightPosition;
out vec4 frag;
uniform sampler2D albedo;
uniform sampler2DShadow shadowMap;
uniform vec4 tint;
uniform vec3 eye,sunDirection;
uniform int hasTexture,material,useShadow,clipWater;
uniform float time,caustics,directLight,ambientLight;
uniform float ledRadiance;
uniform vec3 waterTint,waterAbsorption;
uniform float waterLevel;
uniform vec3 poolSize;
uniform float waterScattering,waterDistanceScale,waterDistancePower,waterClearDistance;
uniform int outdoor;
uniform int holeCount;
uniform vec3 holes[4];
float hash(vec2 p){return fract(sin(dot(p,vec2(127.1,311.7)))*43758.5453);}
float grid(vec2 p,float spacing,float thickness){
  vec2 footprint=max(fwidth(p),vec2(.00001));
  vec2 d=abs(fract(p/spacing-.5)-.5)*spacing;
  vec2 edge=smoothstep(vec2(thickness)-footprint*.5,vec2(thickness)+footprint*.5,d);
  // Fade subpixel grout to its area average instead of aliasing against the wall.
  vec2 filtered=mix(edge,vec2(1.-2.*thickness/spacing),smoothstep(spacing*.2,spacing*.9,footprint));
  return filtered.x*filtered.y;
}
float stripe(float p,float start,int count){
  float cell=round((p-start)/2.7432);
  if(cell<0 || cell>float(count-1))return 0.;
  float d=abs(p-(start+cell*2.7432));
  return 1-smoothstep(.127,.127+fwidth(p),d);
}
float visibility(vec3 n,vec3 l){
  if(useShadow==0)return 1.;
  vec3 p=lightPosition.xyz/lightPosition.w*.5+.5;
  if(p.z<0 || p.z>1 || p.x<0 || p.x>1 || p.y<0 || p.y>1)return 1.;
  float shadow=0,bias=max(.00035*(1-dot(n,l)),.00010);
  vec2 texel=1.0/vec2(textureSize(shadowMap,0));
  // Each tap bilinearly filters four comparison results. Tent weights soften
  // the edge without the discrete brightness steps of nearest-depth samples.
  for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++){
    float weight=float((2-abs(x))*(2-abs(y)));
    shadow+=weight*texture(shadowMap,vec3(p.xy+vec2(x,y)*texel,p.z-bias));
  }
  return shadow/16.;
}
float caustic(vec2 p){
  // Interfering ripple fields focus sunlight into moving bands on submerged surfaces.
  p*=3.7;
  p+=vec2(sin(p.y*.71+time*.41),cos(p.x*.62-time*.36))*.58;
  float a=sin(p.x+time*.72)+sin(p.y*1.12-time*.59);
  float b=sin(p.x*.73-p.y*.84+time*.38)+cos(p.y*.67+p.x*.92-time*.47);
  return pow(max(0.,1.-abs(a)*.68),12.)*.65+pow(max(0.,1.-abs(b)*.75),14.)*.45;
}
void main(){
  float worldZ=world.z-waterLevel,eyeZ=eye.z-waterLevel;
  for(int i=0;i<holeCount;i++)if(distance(texcoord,holes[i].xy)<holes[i].z)discard;
  if(clipWater==1 && worldZ<0.015)discard;
  vec4 sampled=hasTexture==1?texture(albedo,texcoord):vec4(1);
  if(sampled.a<.4)discard;
  vec3 base=sampled.rgb*tint.rgb;
  vec3 n=normalize(norm);if(!gl_FrontFacing)n=-n;
  vec3 pn=normalize(poolNormal);
  if(material==1){
    vec2 tile=abs(pn.z)>.5?poolPosition.xy:(abs(pn.x)>.5?poolPosition.yz:poolPosition.xz);
    float g=grid(tile,.1524,.0025);
    base*=mix(.63,1.,g)*(1.+(hash(floor(tile/.1524))-.5)*.035*(1.-smoothstep(.02,.10,max(fwidth(tile.x),fwidth(tile.y)))));
    float lane=0;
    if(abs(pn.z)>.5){
      float s1=stripe(poolPosition.y,(poolSize.y-7.*2.7432)/2.,8);
      float s2=stripe(poolPosition.x,(poolSize.x-16.*2.7432)/2.,17);
      lane=max(s1*step(2.,poolPosition.x)*step(poolPosition.x,(poolSize.x-2.)),s2*step(2.,poolPosition.y)*step(poolPosition.y,(poolSize.y-2.)));
      // T-shaped lane ends, with the same metre-wide heads as the original scene.
      float endX=min(abs(poolPosition.x-2.),abs(poolPosition.x-(poolSize.x-2.)));
      float endY=min(abs(poolPosition.y-2.),abs(poolPosition.y-(poolSize.y-2.)));
      float nearY=abs(mod(poolPosition.y-(poolSize.y-7.*2.7432)/2.+1.3716,2.7432)-1.3716);
      float nearX=abs(mod(poolPosition.x-(poolSize.x-16.*2.7432)/2.+1.3716,2.7432)-1.3716);
      lane=max(lane,(1-smoothstep(.12,.14,endX))*step(nearY,.5));
      lane=max(lane,(1-smoothstep(.12,.14,endY))*step(nearX,.5));
    }else{
      float line=abs(pn.x)>.5?stripe(poolPosition.y,(poolSize.y-7.*2.7432)/2.,8):stripe(poolPosition.x,(poolSize.x-16.*2.7432)/2.,17);
      lane=line*step(poolPosition.z,waterLevel);
      base=mix(base,vec3(.065,.20,.27),step(-.13,poolPosition.z)*step(poolPosition.z,.04));
    }
    base=mix(base,vec3(.035,.07,.09),lane*.91);
  } else if(material==2){
    vec2 tile=abs(pn.z)>.5?poolPosition.xy:poolPosition.xz;
    base*=mix(.8,1.,grid(tile,.6,.004));
  }
  if(material==4){
    float p=abs(n.x)>.5?world.y:world.x;
    float fade=1.-smoothstep(.001,.006,fwidth(p));
    base*=1.-.035*fade*(.5+.5*cos(p*1570.796));
  }
  vec3 l=normalize(sunDirection),v=normalize(eye-world),h=normalize(l+v);
  float nl=max(dot(n,l),0.),nh=max(dot(n,h),0.);
  float vis=visibility(n,l);
  vec3 ambient=mix(vec3(.24,.32,.37),vec3(.48,.55,.56),clamp(n.z*.5+.5,0.,1.));
  float rough=material==5?.06:(material==1?.26:.54);
  float spec=pow(nh,mix(85.,14.,rough))*(material==5?1.4:(material==1?.17:.055));
  vec3 lighting=base*(ambient*ambientLight+(outdoor==1?vec3(1.15,1.08,.95):vec3(.92,1.01,1.08))*nl*vis*directLight)+spec*vis*directLight;
  if(worldZ<0){
    lighting*=vec3(.84,.97,1.04);
    float c=caustic(world.xy+world.z*n.xy*.5);
    lighting+=base*c*caustics*directLight*exp(worldZ*.14)*(.3+.7*max(n.z,0.))*(.35+.65*vis);
    // Approximate incoming surface light with a vertical path through the water.
    // Use physical depth in metres; viewing-distance controls apply below.
    lighting*=exp(-waterAbsorption*(-worldZ));
  }
  // Emissive materials generate their own light and only lose it on the way to the camera.
  if(material==3)lighting=base*1.65;
  if(material==6)lighting=base*ledRadiance;
  float d=length(eye-world),wet=0;
  if(eyeZ<0 && worldZ<0)wet=d;
  else if(eyeZ>=0 && worldZ<0)wet=d*(-worldZ)/max(.001,eye.z-world.z);
  else if(eyeZ<0 && worldZ>=0)wet=d*(-eyeZ)/max(.001,world.z-eye.z);
  float opticalDistance=pow(max(0.,wet-waterClearDistance)*waterDistanceScale,waterDistancePower);
  float transmission=exp(-opticalDistance*waterScattering);
  vec3 attenuation=exp(-opticalDistance*waterAbsorption)*transmission;
  vec3 scatter=waterTint*(.2+.5*ambientLight+.3*directLight);
  lighting=lighting*attenuation+scatter*(1.-transmission);
  float alpha=material==5 ? clamp(tint.a+pow(1.-abs(dot(n,v)),5.)*.55+spec*.15,0.,.85) : 1.;
  frag=vec4(lighting,alpha);
}
