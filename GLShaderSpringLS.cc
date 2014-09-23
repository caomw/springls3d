/*
 * GLShaderSpringLS.cc
 *
 *  Created on: Sep 22, 2014
 *      Author: blake
 */

#include "GLShaderSpringLS.h"
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <openvdb/openvdb.h>
namespace imagesci {
const char* const kVSDepthGPU = "varying vec3 N;\n"
		"void main( void )\n"
		"{\n"
		"	N = normalize(gl_NormalMatrix * gl_Normal);\n"
		"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
		"}\n";
const char* const kFSDepthGPU = "varying vec3 N;\n"
		"void main(void )\n"
		"{\n"
		"    float lValue = gl_FragCoord.z;\n"
		"    gl_FragColor = vec4(N,lValue);\n"
		"}\n";
const char* const kFSDepthGPUAttributes[] = { 0 };

GLShaderSpringLS::GLShaderSpringLS(int x, int y, int w, int h) :GLComponent(x, y, w, h), mFrameBufferId(0), mDepthBufferId(0), mTextureId(0) {

}
void GLShaderSpringLS::setMesh(Camera* camera, Mesh* mesh) {
	mCamera = camera;
	mMesh = mesh;
}
void GLShaderSpringLS::updateGL() {
	if (mFrameBufferId == 0) {
		if (!mProgram.Initialize(kVSDepthGPU, kFSDepthGPU,
				kFSDepthGPUAttributes)) {
			std::cout << "Shader compilation failed." << std::endl;
		}
		if (mTextureId == 0) {
			glGenTextures(1, &mTextureId);
		}
		mData.resize(w * h);
		glBindTexture( GL_TEXTURE_2D, mTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,GL_FLOAT, &mData[0]);
		glBindTexture( GL_TEXTURE_2D, 0);
		glEnable(GL_TEXTURE_2D);
		glGenFramebuffers(1, &mFrameBufferId);
		glGenRenderbuffers(1, &mDepthBufferId);
		glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId);
		glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER, mDepthBufferId);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D, mTextureId, 0);
		glDisable(GL_TEXTURE_2D);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);
		glUseProgram((GLuint)NULL);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			throw Exception("Could not initialize frame buffer.");
		}
	}
}
void GLShaderSpringLS::render() {
	//Clear image

	/*
	glBindTexture( GL_TEXTURE_2D, mTextureId);
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
			&mData[0]);
	glBindTexture( GL_TEXTURE_2D, 0);
*/


	glViewport(x,y,w,h);
	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId);
	glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor4f(0.0, 0.0f, 0.0f, 1.0f);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glUseProgram(mProgram.GetProgramHandle());
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor4f(1.0f,0.0f,0.0f,0.0f);
	mCamera->aim(x, y, w, h);
	mMesh->draw(false, false, false, false,false);
	glEnable(GL_BLEND);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	glUseProgram((GLuint)NULL);

	glBindTexture(GL_TEXTURE_2D, mTextureId);
	glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_FLOAT,&mData[0]);
	//Need code to convert z buffer to depth

	//std::cout<<"Save Image... ";
	//WriteImageToFile("render.png",mData,w,h);
	//std::cout<<"done."<<std::endl;

	/*
	glViewport(x,y,w,h);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor4f(0.0, 0.0f, 0.0f, 1.0f);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, w,h,0, 0.0, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_TEXTURE_2D);
	glColor4f(1.0f,1.0f,1.0f,1.0f);
	glBindTexture(GL_TEXTURE_2D, mTextureId);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f, 1.0f);
	glVertex2f(x+w,y+h);
	glTexCoord2f(0.0f, 1.0f);
	glVertex2f(x, y+h);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(x,y);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f(x+w, y);
	glEnd();
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	*/
}
GLShaderSpringLS::~GLShaderSpringLS() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
