/*
 * GLShaderSpringLS.cc
 *
 *  Created on: Sep 22, 2014
 *      Author: blake
 */

#include "GLShaderSpringLS.h"
#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
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
const char* const NormalAndDepthAttributes[] = { 0 };
const char* const MixerAttributes[] = { 0};
GLShaderSpringLS::GLShaderSpringLS(int x, int y, int w, int h) :
		GLComponent(x, y, w, h),
		mSpringLS(NULL),
		mCamera(NULL),
		mMatCapId1(0),
		mMatCapId2(0),
		mFrameBufferId1(0), mDepthBufferId1(0),
		mDepthBufferId2(0),mFrameBufferId2(0),
		mFrameBufferId3(0), mDepthBufferId3(0){

}
void GLShaderSpringLS::setMesh(Camera* camera, SpringLevelSet* mesh) {
	mCamera = camera;
	mSpringLS = mesh;
}
void GLShaderSpringLS::updateGL() {
	if (mFrameBufferId1 == 0) {
		std::list<std::string> attrib;
		attrib.push_back("vp");
		attrib.push_back("vn");

		if (!mWireframeProgram.Initialize(ReadTextFile("wireframe_depth_shader.vert"),ReadTextFile("wireframe_depth_shader.frag"),ReadTextFile("wireframe_depth_shader.geom"),
				attrib)) {
			throw Exception("Wireframe depth shader compilation failed.");
		}
		if (!mNormalsAndDepthProgram.Initialize(ReadTextFile("depth_shader.vert"),ReadTextFile("depth_shader.frag"),"",
				attrib)) {
			throw Exception("depth shader compilation failed.");
		}
		if (!mMixerProgram.Initialize(ReadTextFile("springls.vert"),ReadTextFile("springls.frag"),"",
				attrib)) {
			throw("Mixer shader compilation failed.");
		}
		mData.resize(w * h);

		isoImage=std::unique_ptr<Image>(new Image(0,0,w,h,w,h,true));
		wireImage=std::unique_ptr<Image>(new Image(0,0,w,h,w,h,true));
		springlImage=std::unique_ptr<Image>(new Image(0,0,w,h,w,h,true));
		renderImage=std::unique_ptr<Image>(new Image(0,0,w,h,w,h,false));

		isoImage->updateGL();
		springlImage->updateGL();
		renderImage->updateGL();
		wireImage->updateGL();

		std::vector<RGBA> tmp1,tmp2;
		int iW,iH;
		if(ReadImageFromFile("./matcap/JG_Red.png",tmp1,iW,iH)){
			glGenTextures(1, &mMatCapId1);
			glBindTexture( GL_TEXTURE_2D, mMatCapId1);
			glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, iW, iH, 0, GL_RGBA,GL_UNSIGNED_BYTE, &tmp1[0]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

			glBindTexture( GL_TEXTURE_2D, 0);
		} else {
			std::cerr<<"Could not read JG_Red.png"<<std::endl;
		}
		tmp1.clear();

		if(ReadImageFromFile("./matcap/JG_Gold.png",tmp2,iW,iH)){
			glGenTextures(1, &mMatCapId2);
			glBindTexture( GL_TEXTURE_2D, mMatCapId2);
			glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, iW, iH, 0, GL_RGBA,GL_UNSIGNED_BYTE, &tmp2[0]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

			glBindTexture( GL_TEXTURE_2D, 0);
		} else {
			std::cerr<<"Could not read JG_Gold.png"<<std::endl;
		}
		tmp2.clear();

		glGenRenderbuffers(1, &mDepthBufferId1);
		glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId1);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);

		glGenFramebuffers(1, &mFrameBufferId1);
		glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId1);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER, mDepthBufferId1);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D, isoImage->textureId(), 0);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);

		glGenRenderbuffers(1, &mDepthBufferId3);
		glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId3);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);

		glGenFramebuffers(1, &mFrameBufferId3);
		glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId3);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER, mDepthBufferId3);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D, wireImage->textureId(), 0);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);

		glGenRenderbuffers(1, &mDepthBufferId2);
		glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId2);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w, h);

		glGenFramebuffers(1, &mFrameBufferId2);
		glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId2);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER, mDepthBufferId2);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D, springlImage->textureId(), 0);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);

		glUseProgram((GLuint)NULL);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			throw Exception("Could not initialize frame buffer.");
		}
	}
}
void GLShaderSpringLS::render() {
	 glViewport(0,0,w,h);
	glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId1);
	glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId1);
	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(mNormalsAndDepthProgram.GetProgramHandle());
	glUniform1f(glGetUniformLocation(mNormalsAndDepthProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
	glUniform1f(glGetUniformLocation(mNormalsAndDepthProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());

	mCamera->aim(0,0, w, h,mNormalsAndDepthProgram);
	mSpringLS->isoSurface.draw(false, false, false, false,false);
	glUseProgram((GLuint)NULL);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

	glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId2);
	glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId2);
	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(mNormalsAndDepthProgram.GetProgramHandle());
	glUniform1f(glGetUniformLocation(mNormalsAndDepthProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
	glUniform1f(glGetUniformLocation(mNormalsAndDepthProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());

	mCamera->aim(0,0, w, h,mNormalsAndDepthProgram);
	mSpringLS->constellation.draw(false, false, false, false,false);

	glUseProgram((GLuint)NULL);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

	glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferId3);
	glBindRenderbuffer(GL_RENDERBUFFER, mDepthBufferId3);
	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(mWireframeProgram.GetProgramHandle());
	glUniform1f(glGetUniformLocation(mWireframeProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
	glUniform1f(glGetUniformLocation(mWireframeProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());

	mCamera->aim(0,0, w, h,mWireframeProgram);
	mSpringLS->constellation.draw(false, false, false, false,false);

	glUseProgram((GLuint)NULL);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

/*
	std::cout<<"Write To FIle "<<std::endl;
	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, isoImage->textureId());
	glGetTexImage(GL_TEXTURE_2D_MULTISAMPLE,0,GL_RGBA,GL_FLOAT,&mData[0]);
	WriteImageToFile("iso.png",mData,w,h);

	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, springlImage->textureId());
	glGetTexImage(GL_TEXTURE_2D_MULTISAMPLE,0,GL_RGBA,GL_FLOAT,&mData[0]);
	WriteImageToFile("springls.png",mData,w,h);
	std::cout<<"Done Write! "<<std::endl;

	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, wireImage->textureId());
	glGetTexImage(GL_TEXTURE_2D_MULTISAMPLE,0,GL_RGBA,GL_FLOAT,&mData[0]);
	WriteImageToFile("wire.png",mData,w,h);
	std::cout<<"Done Write! "<<std::endl;
*/
/*
	glDisable(GL_DEPTH_TEST);



*/

	glViewport(h/2,0,w,h);
	glDisable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(mMixerProgram.GetProgramHandle());

	glUniform4f(glGetUniformLocation(mMixerProgram.GetProgramHandle(), "rect"), renderImage->x,renderImage->y,renderImage->w,renderImage->h);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"isoTexture"),0);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"springlsTexture"),1);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"wireTexture"),2);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"matcapTexture1"),3);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"matcapTexture2"),4);

	glUniform1f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
	glUniform1f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,isoImage->textureId());

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, springlImage->textureId());

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, wireImage->textureId());

	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, mMatCapId1);

	glActiveTexture(GL_TEXTURE4);
	glBindTexture(GL_TEXTURE_2D, mMatCapId2);

	//For some reason, I need this. Otherwise the last texture isn't found by the shader.
	glActiveTexture(GL_TEXTURE5);


	renderImage->render();

	glUseProgram((GLuint)NULL);
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
}
GLShaderSpringLS::~GLShaderSpringLS() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
