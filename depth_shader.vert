#version 330
in vec3 vp; // positions from mesh
in vec3 vn; // normals from mesh
uniform mat4 P, V, M; // proj, view, model matrices
out vec3 pos_eye;
out vec3 normal;
uniform float MIN_DEPTH;
uniform float MAX_DEPTH;

void main () {
  vec4 pos = V * M * vec4 (vp, 1.0);
  pos.z=(-pos.z-MIN_DEPTH)/(MAX_DEPTH-MIN_DEPTH);
  pos_eye = pos.xyz;
  normal = vec3 (V * M * vec4 (vn, 0.0));
  gl_Position = P * V * M * vec4 (vp, 1.0);
}