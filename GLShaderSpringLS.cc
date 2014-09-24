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
const char* const vertexShaderDepthGPU = "varying vec3 N;\n"
		"void main( void )\n"
		"{\n"
		"	N = normalize(gl_NormalMatrix * gl_Normal);\n"
		"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
		"}\n";
const char* const fragmentShaderDepthGPU =
		"varying vec3 N;\n"
		"void main(void )\n"
		"{\n"
		"    float lValue = gl_FragCoord.z;\n"
		"    gl_FragColor = vec4(N,lValue);\n"
		"}\n";
const char* const vertexMixerDepthGPU =
		"void main( void )\n"
		"{\n"
		 "gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
		 "gl_TexCoord[0].xy = gl_MultiTexCoord0.xy;\n"
		"}\n";
const char* const fragmentMixerDepthGPU =
		"uniform sampler2D isoTexture;\n"
		"uniform sampler2D springlsTexture;\n"
		"uniform sampler2D matcapTexture1;\n"
		"uniform sampler2D matcapTexture2;\n"
		"uniform float MIN_DEPTH;\n"
		"uniform float MAX_DEPTH;\n"
		""
		"void main(void )\n"
		"{\n"
			"vec4 spgcolor=texture2D(springlsTexture,gl_TexCoord[0].xy);\n"
			"vec4 isocolor=texture2D(isoTexture,gl_TexCoord[0].xy);\n"
			"float isoz = isocolor.w * 2.0f - 1.0f;\n"
			"float spgz = spgcolor.w * 2.0f - 1.0f;\n"
			"if(isocolor.w>0.0f){\n"
				"isoz = -2.0f*MAX_DEPTH*MIN_DEPTH / (isoz*(MAX_DEPTH-MIN_DEPTH)-(MAX_DEPTH+MIN_DEPTH));\n"
				"spgz = -2.0f*MAX_DEPTH*MIN_DEPTH / (spgz*(MAX_DEPTH-MIN_DEPTH)-(MAX_DEPTH+MIN_DEPTH));\n"
			"	if(spgcolor.w>0.0f&&abs(isoz-spgz)<0.3f){\n"
			"		isocolor=texture2D(matcapTexture1,0.5f*isocolor.xy+0.5f);\n"
			"	} else {\n"
			"		isocolor=texture2D(matcapTexture2,0.5f*isocolor.xy+0.5f);\n"
			"	}\n"
			"}  else {\n"
			"	isocolor=vec4(0.0f,0.0f,0.0f,0.0f);\n"
			"}\n"
			"gl_FragColor=isocolor;"
			//" gl_FragColor = mix(texture2D(isoTexture,gl_TexCoord[0].xy), texture2D(springlsTexture,gl_TexCoord[0].xy),0.5f);"
		"}\n";
const char* const NormalAndDepthAttributes[] = { 0 };
const char* const MixerAttributes[] = { 0};
GLShaderSpringLS::GLShaderSpringLS(int x, int y, int w, int h) :GLComponent(x, y, w, h), mFrameBufferId1(0), mDepthBufferId1(0), mIsoTextureId(0) {

}
void GLShaderSpringLS::setMesh(Camera* camera, SpringLevelSet* mesh) {
	mCamera = camera;
	mSpringLS = mesh;
}
void GLShaderSpringLS::updateGL() {
	if (mFrameBufferId1 == 0) {
		if (!mNormalsAndDepthProgram.Initialize(vertexShaderDepthGPU, fragmentShaderDepthGPU,
				NormalAndDepthAttributes)) {
			std::cerr << "Normal / Depth compilation failed." << std::endl;
		}
		if (!mMixerProgram.Initialize(vertexMixerDepthGPU, fragmentMixerDepthGPU,
				MixerAttributes)) {
			std::cerr << "Mixer shader compilation failed." << std::endl;
		}


		mData.resize(w * h);
		glEnable(GL_TEXTURE_2D);

		glGenTextures(1, &mIsoTextureId);
		glBindTexture( GL_TEXTURE_2D, mIsoTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA32F, w, h, 0, GL_RGBA,GL_FLOAT, &mData[0]);
		glBindTexture( GL_TEXTURE_2D, 0);

		glGenTextures(1, &mSpringlTextureId);
		glBindTexture( GL_TEXTURE_2D, mSpringlTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA32F, w, h, 0, GL_RGBA,GL_FLOAT, &mData[0]);
		glBindTexture( GL_TEXTURE_2D, 0);

		glGenTextures(1, &mRenderTextureId);
		glBindTexture( GL_TEXTURE_2D, mRenderTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA32F, w, h, 0, GL_RGBA,GL_FLOAT, &mData[0]);
		glBindTexture( GL_TEXTURE_2D, 0);

		std::vector<RGBA> tmp1,tmp2;
		int iW,iH;
		if(ReadImageFromFile("./matcap/JG_Red.png",tmp1,iW,iH)){
			glGenTextures(1, &mMatCapId1);
			glBindTexture( GL_TEXTURE_2D, mMatCapId1);
			glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, iW, iH, 0, GL_RGBA,GL_UNSIGNED_BYTE, &tmp1[0]);
			glBindTexture( GL_TEXTURE_2D, 0);
		} else {
			std::cerr<<"Could not read matcap1.png"<<std::endl;
		}
		tmp1.clear();

		if(ReadImageFromFile("./matcap/Metal-1.png",tmp2,iW,iH)){
			glGenTextures(1, &mMatCapId2);
			glBindTexture( GL_TEXTURE_2D, mMatCapId2);
			glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, iW, iH, 0, GL_RGBA,GL_UNSIGNED_BYTE, &tmp2[0]);
			glBindTexture( GL_TEXTURE_2D, 0);
		} else {
			std::cerr<<"Could not read matcap1.png"<<std::endl;
		}
		tmp2.clear();

		glGenRenderbuffers(1, &mDepthBufferId1);
		glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId1);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);

		glGenFramebuffers(1, &mFrameBufferId1);
		glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId1);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER, mDepthBufferId1);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D, mIsoTextureId, 0);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);

		glGenRenderbuffers(1, &mDepthBufferId2);
		glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId2);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);

		glGenFramebuffers(1, &mFrameBufferId2);
		glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId2);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER, mDepthBufferId2);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D, mSpringlTextureId, 0);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);

		glDisable(GL_TEXTURE_2D);

		glUseProgram((GLuint)NULL);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			throw Exception("Could not initialize frame buffer.");
		}
	}
}
void GLShaderSpringLS::render() {

	glViewport(0,0,w,h);
	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId1);
	glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId1);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor4f(0.0, 0.0f, 0.0f, 0.0f);
	glUseProgram(mNormalsAndDepthProgram.GetProgramHandle());
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor4f(1.0f,0.0f,0.0f,0.0f);
	mCamera->aim(0,0, w, h);
	mSpringLS->isoSurface.draw(false, false, false, false,false);
	glUseProgram((GLuint)NULL);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId2);
	glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId2);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor4f(0.0, 0.0f, 0.0f, 0.0f);

	glUseProgram(mNormalsAndDepthProgram.GetProgramHandle());
	glColor4f(1.0f,0.0f,0.0f,0.0f);
	mCamera->aim(0,0, w, h);
	mSpringLS->constellation.draw(false, false, false, false,false);
	glEnable(GL_BLEND);
	glUseProgram((GLuint)NULL);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);


	glViewport(x,y,w,h);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glColor4f(0.0, 0.0f, 0.0f, 1.0f);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, w,0,h, 0.0, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor4f(1.0f,1.0f,1.0f,1.0f);

	/*
	glBindTexture(GL_TEXTURE_2D, mIsoTextureId);
	glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_FLOAT,&mData[0]);
	for(Vec4f& c:mData){
		if(c[3]>0.0f){
			std::cout<<c<<std::endl;
		}
	}
	*/
	//std::cout<<"NEAR "<<mCamera->nearPlane()<<" FAR "<<mCamera->farPlane()<<std::endl;
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);



	glDisable(GL_DEPTH_TEST);
	glUseProgram(mMixerProgram.GetProgramHandle());



	glActiveTextureARB(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, mIsoTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glActiveTextureARB(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, mSpringlTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glActiveTextureARB(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, mMatCapId1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glActiveTextureARB(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, mMatCapId2);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"isoTexture"),0);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"springlsTexture"),1);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"matcapTexture1"),2);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"matcapTexture2"),3);

	glUniform1f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
	glUniform1f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());

	glBegin(GL_QUADS);
	glTexCoord2f(1.0f, 1.0f);
	glVertex2f(w,h);
	glTexCoord2f(0.0f, 1.0f);
	glVertex2f(0,h);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(0,0);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f(w,0);
	glEnd();

	glUseProgram((GLuint)NULL);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

}
GLShaderSpringLS::~GLShaderSpringLS() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
