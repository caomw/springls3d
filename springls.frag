#version 330
uniform sampler2D isoTexture;
uniform sampler2D springlsTexture;
uniform sampler2D wireTexture;
uniform sampler2D matcapTexture1;
uniform sampler2D matcapTexture2;

in vec2 texture_coordinates;
uniform float MIN_DEPTH;
uniform float MAX_DEPTH;

const float DISTANCE_TOL=0.3f;
void main(void ){
	vec2 uv=texture_coordinates;
	vec4 spgcolor,wirecolor,isocolor;
	vec4 accumColor=vec4(0.0,0.0,0.0,0.0);
	float isoz,spgz,wirz;
	vec2 shift;
	float wsum=0;
	float w=0;
	isocolor=texture2D(isoTexture,uv);
	
	for(int i=-2;i<=2;i++){
		for(int j=-2;j<=2;j++){
			
			shift.x=i*0.5f/800.0f;
			shift.y=j*0.5f/800.0f;
			w=exp(-dot(shift,shift)/0.5f);
			spgcolor=texture2D(springlsTexture,uv+shift);
			wirecolor=texture2D(wireTexture,uv+shift);
			
			isoz = isocolor.w;
			spgz = spgcolor.w;
			wirz = wirecolor.w;

			if(isocolor.w>0.0f){
				isoz = -(MAX_DEPTH-MIN_DEPTH)*isoz-MIN_DEPTH;
				spgz = -(MAX_DEPTH-MIN_DEPTH)*spgz-MIN_DEPTH;
				wirz = -(MAX_DEPTH-MIN_DEPTH)*wirz-MIN_DEPTH;
				if(spgcolor.w>0.0f&&abs(isoz-spgz)<DISTANCE_TOL){
					if(dot(wirecolor.xyz,wirecolor.xyz)>0.0&&wirecolor.w>0.0f&&abs(isoz-wirz)<DISTANCE_TOL){
						accumColor+=w*texture2D(matcapTexture2,0.5f*wirecolor.xy+0.5f);	
					} else {
						accumColor+=w*texture2D(matcapTexture2,0.5f*isocolor.xy+0.5f);
					}
				} else {
					accumColor+=w*texture2D(matcapTexture1,0.5f*isocolor.xy+0.5f);
				}
			}  else {
				accumColor+=w*vec4(0.0f,0.0f,0.0f,1.0f);
			}
			wsum+=w;
		}
	}
	accumColor=(1.0f/wsum)*accumColor;
	accumColor.w=1.0f;
	gl_FragColor=accumColor;
}
