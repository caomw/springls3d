/*
 * Copyright(C) 2014, Blake Lucas (img.science@gmail.com)
 * All rights reserved.
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
			w=exp(-dot(shift,shift)/0.5f);
			uv=pos3d.xy+shift;
			rgba=texture2D(textureImage,uv);
			float r=2*sqrt(dot(uv-vec2(0.5f,0.5f),uv-vec2(0.5f,0.5f)));
			if(rgba.w==0){
				if(r>1.0-lineThick&&r<1.0){
					t=cos(3.141596535*(1-0.5*lineThick-r)/lineThick)*0.5f+0.5f;
					rgba=vec4(1-t,1-t,1-t,0.5);
				} else if(r<1.0-lineThick){
					rgba=vec4(1,1,1,0.5);
				} 
			}
			accum+=w*rgba;
			wsum+=w;
		}
	}
	accum=(1.0/wsum)*accum;
	gl_FragColor =accum;
}
 