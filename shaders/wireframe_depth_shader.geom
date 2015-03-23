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
layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;
out vec3 v0, v1, v2;
out vec3 normal, vert;
uniform mat4 P,V,M;
void main() {
  mat4 PVM=P*V*M;
  mat4 VM=V*M;
  
  vec3 v01 = gl_in[1].gl_Position.xyz - gl_in[0].gl_Position.xyz;
  vec3 v02 = gl_in[2].gl_Position.xyz - gl_in[0].gl_Position.xyz;
  vec3 fn =  normalize(cross( v01, v02 ));
  vec4 p0=gl_in[0].gl_Position;
  vec4 p1=gl_in[1].gl_Position;
  vec4 p2=gl_in[2].gl_Position;

  v0 = (VM*p0).xyz;
  v1 = (VM*p1).xyz;
  v2 = (VM*p2).xyz;
  
  
  gl_Position=PVM*gl_in[0].gl_Position;  
  vert = v0;
  normal = (VM*vec4(fn,0.0)).xyz;
  EmitVertex();
  
  
  gl_Position=PVM*gl_in[1].gl_Position;  
  vert = v1;
  normal = (VM*vec4(fn,0.0)).xyz;
  EmitVertex();
  
   
  gl_Position=PVM*gl_in[2].gl_Position;  
  vert = v2;
  normal = (VM*vec4(fn,0.0)).xyz;
  EmitVertex();
  
  EndPrimitive();
 }
