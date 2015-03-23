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
in vec3 vp; // positions from mesh
in vec3 vn; // normals from mesh
in vec3 vel;
uniform mat4 P, V, M; // proj, view, model matrices
out vec3 pos_eye;
out vec3 normal;
uniform float MIN_DEPTH;
uniform float MAX_DEPTH;

void main () {
  vec4 pos = V * M * vec4 (vp, 1.0);
  normal = vec3 (V * M * vec4 (vn, 0.0));
  gl_Position = P * pos; 
  pos=pos/pos.w; 
  pos.z=(-pos.z-MIN_DEPTH)/(MAX_DEPTH-MIN_DEPTH);
  pos_eye = pos.xyz;

}
