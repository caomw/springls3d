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
vec4 spgcolor=texture2D(springlsTexture,texture_coordinates);
vec4 wirecolor=texture2D(wireTexture,texture_coordinates);
vec4 isocolor=texture2D(isoTexture,texture_coordinates);
vec4 matcap1=texture2D(matcapTexture1,0.5f*isocolor.xy+0.5f);
vec4 matcap2=texture2D(matcapTexture2,0.5f*isocolor.xy+0.5f);
matcap1.w=1.0f;

float isoz = isocolor.w;
float spgz = spgcolor.w;
float wirz = wirecolor.w;

	if(isocolor.w>0.0f){
		isoz = -(MAX_DEPTH-MIN_DEPTH)*isoz-MIN_DEPTH;
		spgz = -(MAX_DEPTH-MIN_DEPTH)*spgz-MIN_DEPTH;
		wirz = -(MAX_DEPTH-MIN_DEPTH)*wirz-MIN_DEPTH;
		if(spgcolor.w>0.0f&&abs(isoz-spgz)<DISTANCE_TOL){
			if(wirecolor.w>0.0f&&abs(isoz-wirz)<DISTANCE_TOL){
				gl_FragColor=matcap2;
			} else {
				gl_FragColor=matcap1;
			}
		} else {
			gl_FragColor=matcap1;
		}
	}  else {
		float lum=0.2f+0.6f*(texture_coordinates.y);
		gl_FragColor=vec4(lum,lum,lum,1.0f);
	}
}
