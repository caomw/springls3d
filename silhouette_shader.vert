#version 330
in vec3 vp; 
uniform vec2 IMG_POS;
uniform vec2 IMG_DIMS;
uniform vec2 SCREEN_DIMS;
out vec3 pos3d;
void main() {
	pos3d=vp;
	vec2 pos=(vp.xy*IMG_DIMS+IMG_POS);
	pos.x=2*pos.x/SCREEN_DIMS.x-1.0;
	pos.y=1.0-2*pos.y/SCREEN_DIMS.y;
  gl_Position = vec4(pos.x,pos.y,0,1);
}
