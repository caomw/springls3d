#version 330
in vec3 v0, v1, v2;
in vec3 normal, vert;
const vec4 skyColorMiddle = vec4(0.5, 0.5, 0.5, 1.0);
const vec4 skyColorEdge = vec4(0.7, 0.7, 0.7, 1.0);
const vec4 groundColor = vec4(0.1, 0.1, 0.1, 1.0);

void main(void) {
  vec3 line, vec, proj;
  float dist;

  // compute minimum distance from current interpolated 3d vertex to triangle edges
  // edge v1-v0
  
  vec = vert - v0;
  line = normalize(v1 - v0);
  proj = dot(vec, line) * line;
  dist = length (vec - proj);
  
  line = normalize(v2 - v0);
  proj = dot(vec, line) * line;
  dist = min(dist, length (vec - proj));

  //dist=min(dist,distance(vert,v0));	
  //dist=min(dist,distance(vert,v1));	  
  //dist=min(dist,distance(vert,v2));
  
  //float w = 0.5 * (1.0 + dot(normalize(normal), vec3(0.0, 1.0, 0.0)));
  // discard interior of triangle
  if (dist <0.1f&&normal.z>0.0f){
    //vec4 diffuseColor = mix(skyColorMiddle,groundColor,w);
    gl_FragColor = skyColorEdge;
  } else {
    discard;
  }
}
