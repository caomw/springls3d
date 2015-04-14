/*
 * Copyright(C) 2014, Blake C. Lucas, Ph.D. (img.science@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#version 330
uniform sampler2D isoTexture;
uniform sampler2D springlsTexture;
uniform sampler2D wireTexture;
uniform sampler2D colorTexture;
uniform sampler2D matcapTexture1;
uniform sampler2D matcapTexture2;

in vec2 texture_coordinates;
uniform float MIN_DEPTH;
uniform float MAX_DEPTH;
uniform vec2 SCREEN_DIMS;
uniform vec2 IMG_DIMS;
uniform float SCALE;
float DISTANCE_TOL=SCALE;
void main(void ){
	vec2 uv=texture_coordinates;
	vec4 spgcolor,wirecolor,isocolor,surfcolor;
	vec4 accumColor=vec4(0.0,0.0,0.0,0.0);
	float isoz,spgz,wirz;
	vec2 shift;
	float wsum=0;
	float w=0;
	
	
	for(int i=-2;i<=2;i++){
		for(int j=-2;j<=2;j++){
			shift.x=i*0.5f/IMG_DIMS.x;
			shift.y=j*0.5f/IMG_DIMS.y;
			w=exp(-dot(shift,shift)/0.5f);
			isocolor=texture2D(isoTexture,uv+shift);
			spgcolor=texture2D(springlsTexture,uv+shift);
			wirecolor=texture2D(wireTexture,uv+shift);
			surfcolor=texture2D(colorTexture,uv+shift);
			
			isoz = isocolor.w;
			spgz = spgcolor.w;
			wirz = wirecolor.w;

			if(isocolor.w>0.0f){
				isoz = -(MAX_DEPTH-MIN_DEPTH)*isoz-MIN_DEPTH;
				spgz = -(MAX_DEPTH-MIN_DEPTH)*spgz-MIN_DEPTH;
				wirz = -(MAX_DEPTH-MIN_DEPTH)*wirz-MIN_DEPTH;
				
				if(spgcolor.w>0.0f&&abs(isoz-spgz)<DISTANCE_TOL){
					if(dot(wirecolor.xyz,wirecolor.xyz)>0.0&&wirecolor.w>0.0f&&abs(isoz-wirz)<DISTANCE_TOL){
						w*=0.01;
						accumColor+=w*mix(surfcolor,texture2D(matcapTexture2,0.5f*wirecolor.xy+0.5f),0.5);	
					} else {
						accumColor+=w*mix(surfcolor,texture2D(matcapTexture2,0.5f*isocolor.xy+0.5f),0.5);
					}
					
				} else {
					w*=0.01;
					accumColor+=w*texture2D(matcapTexture1,0.5f*isocolor.xy+0.5f);
				}
			} else {
				accumColor+=(w/255.0)*mix(vec4(230,230,230,255),vec4(30,30,30,255),uv.y+shift.y);
			}
			wsum+=w;
		}
	}
	accumColor=(1.0f/wsum)*accumColor;
	gl_FragColor=accumColor;
}
