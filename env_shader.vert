#version 330
in vec3 vp; // positions from mesh
in vec3 vn; // normals from mesh
uniform mat4 P, V, M; // proj, view, model matrices
out vec3 normal;

void main () {
  normal = vec3 (V * M * vec4 (vn, 0.0));
  gl_Position = P * V * M * vec4 (vp, 1.0);
}
