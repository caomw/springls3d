#version 330
in vec3 vp;
in vec3 vn;

void main(void) {
  gl_Position = vec4(vp,1.0f); 
}
