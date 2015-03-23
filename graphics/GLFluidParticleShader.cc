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
#include "GLFluidParticleShader.h"
#include "../ImageSciUtil.h"
#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <string>
#include <list>
namespace imagesci {

GLFluidParticleShader::GLFluidParticleShader():GLShader(),mTextureId(0) {
	// TODO Auto-generated constructor stub

}
bool GLFluidParticleShader::Init(){
	std::vector<std::string> attrib;
	attrib.push_back("vp");
	attrib.push_back("vel");
	return Initialize(ReadTextFile("shaders/fluid_particle_shader.vert"),ReadTextFile("shaders/fluid_particle_shader.frag"),ReadTextFile("shaders/fluid_particle_shader.geom"),attrib);

}
void GLFluidParticleShader::begin(){
	glUseProgram(GetProgramHandle());


}
void GLFluidParticleShader::end(){
	glUseProgram(0);
}
GLFluidParticleShader::~GLFluidParticleShader() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
