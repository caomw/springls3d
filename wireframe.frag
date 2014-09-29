#version 330
in vec3 v0, v1, v2;
in vec3 normal, vert;
const vec4 skyColorEdge = vec4(0.9, 0.9, 0.9, 1.0);
const vec4 skyColorMiddle = vec4(0.2, 0.2, 0.2, 1.0);
const vec4 groundColor = vec4(0.1, 0.1, 0.1, 1.0);

void main(void) {
  vec3 line, vec, proj;
  float dist;

  // compute minimum distance from current interpolated 3d vertex to triangle edges
  // edge v1-v0
  line = normalize(v1 - v0);
  vec = vert - v0;
  proj = dot(vec, line) * line;
  dist = length (vec - proj);

  // edge v2-v0
  line = normalize(v2 - v0);
  proj = dot(vec, line) * line;
  dist = min(dist, length (vec - proj));

  // edge v2-v1
  line = normalize(v2 - v1);
  vec = vert - v1;
  proj = dot(vec, line) * line;
  dist = min(dist, length (vec - proj));

  vec3 center = 0.33333f*(v0 + v1 + v2);
  vec = center - v1;
  proj = dot(vec, line) * line;
  float max_dist = length(vec - proj);
  
  // normalize min distance
  dist /= max_dist;
  float w = 0.5 * (1.0 + dot(normalize(normal), vec3(0.0, 1.0, 0.0)));
  // discard interior of triangle
  if (dist <0.3f){
    vec4 diffuseColor = w * skyColorMiddle + (1.0 - w) * groundColor;
    gl_FragColor = diffuseColor;
  } else {
   vec4 diffuseColor = w * skyColorEdge + (1.0 - w) * groundColor;
    gl_FragColor = diffuseColor;
  }
}
