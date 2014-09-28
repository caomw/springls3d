#version 330
in vec3 vp; 
in vec2 vt;
uniform vec4 rect;
out vec2 texture_coordinates;
void main () {
  texture_coordinates = vt;
  vec2 pt=2*(vp.xy-vec2(0.5f,0.5f));
  gl_Position = vec4(pt.x,pt.y,0,1);
}

