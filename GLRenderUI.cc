/*
 * GLRenderUI.cc
 *
 *  Created on: Sep 22, 2014
 *      Author: blake
 */

#include "GLRenderUI.h"
#include <GL/gl.h>
#include <GL/glfw.h>
namespace imagesci {

GLRenderUI::GLRenderUI():mScreenWidth(0),mScreenHeight(0),x(0),y(0) {
	// TODO Auto-generated constructor stub

}
void GLRenderUI::init(){
	for(std::unique_ptr<GLComponent>& comp:GLRenderComponents){
		comp->updateGL();
	}
}
void GLRenderUI::aim(){
	x=0;
	y=0;
    glfwGetWindowSize(&mScreenWidth, &mScreenHeight);
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
void GLRenderUI::render(){
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
		comp->render();
	}
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
}
} /* namespace imagesci */
