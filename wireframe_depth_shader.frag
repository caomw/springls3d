#version 330
in vec3 v0, v1, v2;
in vec3 normal, vert;
uniform float MIN_DEPTH;
uniform float MAX_DEPTH;
float DISTANCE_TOL=0.1f;
void main(void) {
  vec3 line, vec, proj;
  float dist1,dist2,dist;
  vec3 tan1,tan2;
  // compute minimum distance from current interpolated 3d vertex to triangle edges
  // edge v1-v0
  
  vec = vert - v0;
  line = normalize(v1 - v0);
  proj = dot(vec, line) * line;
  dist1 = length (vec - proj);
  tan1=cross(line,normal);
  
  line = normalize(v0 - v2);
  proj = dot(vec, line) * line;
  dist2 = length (vec - proj);
  tan2=cross(line,normal);

  float w1,w2;
  vec3 outNorm=normalize(normal);
  w1=clamp(1.0f-dist1/DISTANCE_TOL,0.0,1.0);
  w2=clamp(1.0f-dist2/DISTANCE_TOL,0.0,1.0);
    if(dist1<dist2){
      dist=dist1;
      outNorm=normalize(mix(normal,tan1,w1));
    } else {
      dist=dist2;
      outNorm=normalize(mix(normal,tan2,w2));
    }
    if (dist <DISTANCE_TOL&&normal.z>0.0){
      gl_FragColor = vec4(outNorm,(-vert.z-MIN_DEPTH)/(MAX_DEPTH-MIN_DEPTH));
    } else {
      gl_FragColor = vec4(0.0,0.0,0.0,(-vert.z-MIN_DEPTH)/(MAX_DEPTH-MIN_DEPTH));
      //discard;
    }
}
