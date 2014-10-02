/*
 * GLFrameBuffer.cc
 *
 *  Created on: Oct 1, 2014
 *      Author: blake
 */

#include "GLFrameBuffer.h"
#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>

namespace imagesci {

GLFrameBuffer::GLFrameBuffer():Image(),mFrameBufferId(0),mDepthBufferId(0) {
	// TODO Auto-generated constructor stub

}

GLFrameBuffer::~GLFrameBuffer() {
	if(mFrameBufferId!=0)glDeleteFramebuffers(1,&mFrameBufferId);
	if(mDepthBufferId!=0)glDeleteRenderbuffers(1,&mDepthBufferId);
}
void GLFrameBuffer::begin(){
	glViewport(0,0,mWidth,mHeight);
	glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId);
	glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId);
	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

}
void GLFrameBuffer::end(){

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
}
void GLFrameBuffer::updateGL(){
	glGenTextures( 1,&mTextureId);
	if(mFloatType){
		mDataf.resize(mWidth*mHeight);
		glBindTexture( GL_TEXTURE_2D, mTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA32F, mWidth, mHeight, 0, GL_RGBA,
			GL_FLOAT, &mDataf[0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	} else {
		mData.resize(mWidth*mHeight);
		glBindTexture( GL_TEXTURE_2D, mTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, mWidth, mHeight, 0, GL_RGBA,
			GL_UNSIGNED_BYTE, &mData[0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	}

	glGenRenderbuffers(1, &mDepthBufferId);
	glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, mWidth, mHeight);

	glGenFramebuffers(1, &mFrameBufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER, mDepthBufferId);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,mTextureId, 0);


	glBindTexture(GL_TEXTURE_2D,0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	if(vao==0){
		glGenVertexArrays (1, &vao);
		if(mPositionBuffer==0){
			glGenBuffers(1, &mPositionBuffer);
			glBindBuffer(GL_ARRAY_BUFFER, mPositionBuffer);
			glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) *3* 6,PositionCoords, GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
		if(mUVBuffer==0){
			glGenBuffers(1, &mUVBuffer);
			glBindBuffer(GL_ARRAY_BUFFER, mUVBuffer);
			glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2*6,TextureCoords, GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
	}

}
} /* namespace imagesci */
