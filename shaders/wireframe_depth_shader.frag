/*
 * Copyright(C) 2014, Blake C. Lucas, Ph.D. (img.science@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
in vec3 v0, v1, v2;
in vec3 normal, vert;
in float mag;
uniform float MIN_DEPTH;
uniform float MAX_DEPTH;
uniform mat4 P,V,M;
uniform float SCALE;
float DISTANCE_TOL=0.1f*SCALE;
uniform float maxVelocity;
uniform float minVelocity;
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
      if(normal.z>0.0){
        gl_FragColor = vec4(0.0,0.0,0.0,(-vert.z-MIN_DEPTH)/(MAX_DEPTH-MIN_DEPTH));
      } else discard;
    }
}
