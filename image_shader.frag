#version 330
in vec3 pos3d;
uniform sampler2D textureImage;
uniform vec2 IMG_DIMS;
void main() {
	vec4 accum=vec4(0,0,0,0);
	vec2 shift,uv;
	float w;
	float wsum=0;
	for(int i=-2;i<=2;i++){
		for(int j=-2;j<=2;j++){
			shift.x=i*0.5f/IMG_DIMS.x;
			shift.y=j*0.5f/IMG_DIMS.y;
			w=exp(-dot(shift,shift)/0.5f);
			uv=pos3d.xy+shift;
			vec4 rgba=texture2D(textureImage,uv);
			//float x=(1.0-2*max(abs(uv.x-0.5),abs(uv.y-0.5f)));
			rgba.w=max(0.1,rgba.w);
			accum+=w*rgba;
			wsum+=w;
		}
	}
	accum=(1.0/wsum)*accum;
	gl_FragColor =accum;
}
 