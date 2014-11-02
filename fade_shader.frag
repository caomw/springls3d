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
in vec3 pos3d;
uniform sampler2D textureImage;
uniform vec2 IMG_DIMS;
void main() {
	vec4 accum=vec4(0,0,0,0),rgba;
	vec2 shift,uv;
	vec4 bg;
	float w,t;
	float wsum=0;
	float lineThick=0.025;
	for(int i=-2;i<=2;i++){
		for(int j=-2;j<=2;j++){
			shift.x=i*0.5f/IMG_DIMS.x;
			shift.y=j*0.5f/IMG_DIMS.y;
			uv=pos3d.xy+shift;
			rgba=texture2D(textureImage,uv);
			if(rgba.w==0.0f){
				w=1.0f;
				accum+=(w/255.0)*mix(vec4(125,144,164,255),vec4(26,28,30,255),uv.y+shift.y);
			} else {
				w=rgba.w;
				accum+=w*rgba;
			}
			wsum+=w;
		}
	}
	accum=(1.0/wsum)*accum;
	gl_FragColor =accum;
}
