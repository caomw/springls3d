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
#include "GLRenderUI.h"
#include <GL/gl.h>
#include <GLFW/glfw3.h>
namespace imagesci {

GLRenderUI::GLRenderUI():mScreenWidth(0),mScreenHeight(0),x(0),y(0) {
	// TODO Auto-generated constructor stub

}
void GLRenderUI::init(){
	for(std::unique_ptr<GLComponent>& comp:GLRenderComponents){
		comp->updateGL();
	}
}
void GLRenderUI::aim(GLFWwindow* win){
	x=0;
	y=0;
    glfwGetWindowSize(win,&mScreenWidth, &mScreenHeight);
}
void GLRenderUI::aim(int _x,int _y,int width,int height){
	mScreenWidth=width;
	mScreenHeight=height;
	x=_x;
	y=_y;
}
GLRenderUI::~GLRenderUI() {
	// TODO Auto-generated destructor stub
}
void GLRenderUI::render(GLFWwindow* win){
	glViewport(x,y,mScreenWidth,mScreenHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, mScreenWidth, mScreenHeight,0, 0.0, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	for(std::unique_ptr<GLComponent>& comp:GLRenderComponents){
		comp->render(win);
	}
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
}
} /* namespace imagesci */
