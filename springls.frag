#version 330
uniform sampler2D isoTexture;
uniform sampler2D springlsTexture;
uniform sampler2D matcapTexture1;
uniform sampler2D matcapTexture2;

in vec2 texture_coordinates;
uniform float MIN_DEPTH;
uniform float MAX_DEPTH;
		
void main(void ){
vec4 spgcolor=texture2D(springlsTexture,texture_coordinates);
vec4 isocolor=texture2D(isoTexture,texture_coordinates);
float isoz = isocolor.w;
float spgz = spgcolor.w;
	if(isocolor.w>0.0f){
		isoz = -(MAX_DEPTH-MIN_DEPTH)*isoz-MIN_DEPTH;
		spgz = -(MAX_DEPTH-MIN_DEPTH)*spgz-MIN_DEPTH;
		if(spgcolor.w>0.0f&&abs(isoz-spgz)<0.3f){
			isocolor=texture2D(matcapTexture1,0.5f*isocolor.xy+0.5f);
		} else {
			isocolor=texture2D(matcapTexture2,0.5f*isocolor.xy+0.5f);
		}
	}  else {
		float lum=0.2f+0.6f*(texture_coordinates.y);
		isocolor=vec4(lum,lum,lum,1.0f);
	}
	gl_FragColor=isocolor;
}
