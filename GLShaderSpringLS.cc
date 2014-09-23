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
#include <GL/glew.h>
#include <openvdb/openvdb.h>
namespace imagesci {
const char* const kVSDepthGPU =
	"varying vec3 N;\n"
	"void main( void )\n"
	"{\n"
	"	N = normalize(gl_NormalMatrix * gl_Normal);\n"
	"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
	"}\n";
const char* const kFSDepthGPU =
	"varying vec3 N;\n"
	"void main(void )\n"
	"{\n"
	"    float lValue = gl_FragCoord.z;\n"
	"    gl_FragColor = vec4(N,lValue);\n"
	"}\n";
const char* const kFSDepthGPUAttributes[] = {
	0
};

GLShaderSpringLS::GLShaderSpringLS(int x,int y,int w,int h,Camera* camera,Mesh* mesh):GLComponent(x,y,w,h),mCamera(camera),mMesh(mesh),FramebufferName(0),depthrenderbuffer(0),mTextureId(0){

}
void GLShaderSpringLS::updateGL(){
	if (FramebufferName == 0){
		if (!mProgram.Initialize(kVSDepthGPU, kFSDepthGPU, kFSDepthGPUAttributes)){
			std::cout << "Shader compilation failed." << std::endl;
		}
		if(mTextureId==0){
			glGenTextures( 1,&mTextureId);
		}
		mData.resize(w*h,RGBA(0,0,0,0));
		glBindTexture( GL_TEXTURE_2D, mTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,GL_UNSIGNED_BYTE, &mData[0]);
		glBindTexture( GL_TEXTURE_2D, 0);
		glEnable(GL_TEXTURE_2D);
		glGenFramebuffers(1, &FramebufferName);
		glGenRenderbuffers(1, &depthrenderbuffer);
		glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
		glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mTextureId, 0);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE){
			throw Exception("Could not initialize frame buffer.");
		}
	}
}
void GLShaderSpringLS::render(){
			//Clear image
			glBindTexture( GL_TEXTURE_2D, mTextureId);
			glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,GL_UNSIGNED_BYTE, &mData[0]);
			glBindTexture( GL_TEXTURE_2D, 0);
			mCamera->aim(x,y,w,h);
			glDrawBuffer(GL_COLOR_ATTACHMENT0);
			glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
			glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
			glUseProgram(mProgram.GetProgramHandle());
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glColor4f(0.0, 0.0f, 0.0f, 1.0f);
			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			glDisable(GL_LIGHTING);
			glDisable(GL_BLEND);
			glEnable(GL_DEPTH_TEST);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			mMesh->draw(false,false,false,false);
			glPopMatrix();
			glEnable(GL_BLEND);
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			glBindRenderbuffer(GL_RENDERBUFFER, 0);
			glUseProgram(NULL);
			glGetTextureImageEXT(mTextureId,GL_TEXTURE_2D,0,GL_RGBA,GL_UNSIGNED_BYTE,&mData[0]);
}
GLShaderSpringLS::~GLShaderSpringLS() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
