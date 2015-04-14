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
#include "GLSpringlShader.h"
#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
#include <openvdb/openvdb.h>
namespace imagesci {
GLSpringlShader::GLSpringlShader(int x, int y, int w, int h) :
		GLComponent(x, y, w, h),
		mSpringLS(NULL),
		mCamera(NULL),
		mMatCapId1(0),
		mMatCapId2(0),
		mColormapId(0),
		colorMapValue(0){

}
void GLSpringlShader::setMesh(Camera* camera, SpringLevelSet* mesh,const std::string& springlMatcap,const std::string& isoMatcap,const std::string& colormapFile) {
	mCamera = camera;
	mSpringLS = mesh;
	mIsoMatcap=isoMatcap;
	mSpringlMatcap=springlMatcap;
	mColorMap=colormapFile;
}
void GLSpringlShader::updateGL() {
		std::vector<std::string> attrib;
		attrib.push_back("vp");
		attrib.push_back("vn");
		if (!mWireframeProgram.Initialize(ReadTextFile("shaders/wireframe_depth_shader.vert"),ReadTextFile("shaders/wireframe_depth_shader.frag"),ReadTextFile("shaders/wireframe_depth_shader.geom"),
				attrib)) {
			throw Exception("Wireframe depth shader compilation failed.");
		}
		if (!mNormalsAndDepthProgram.Initialize(ReadTextFile("shaders/depth_shader.vert"),ReadTextFile("shaders/depth_shader.frag"),"",
				attrib)) {
			throw Exception("depth shader compilation failed.");
		}

		if (!mMixerProgram.Initialize(ReadTextFile("shaders/springls.vert"),ReadTextFile("shaders/springls.frag"),"",attrib)) {
			throw Exception("Mixer shader compilation failed.");
		}
		attrib.push_back("vel");
		if (!mColorProgram.Initialize(ReadTextFile("shaders/color_shader.vert"),ReadTextFile("shaders/color_shader.frag"),"",attrib)) {
			throw Exception("color shader compilation failed.");
		}
		mData.resize(w * h);

		isoImage=std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer(0,0,w,h,w,h));
		colorImage=std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer(0,0,w,h,w,h));
		wireImage=std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer(0,0,w,h,w,h));
		springlImage=std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer(0,0,w,h,w,h));
		renderImage=std::unique_ptr<GLImage>(new GLImage(0,0,w,h,w,h,false));

		colorImage->updateGL();
		isoImage->updateGL();
		springlImage->updateGL();
		wireImage->updateGL();
		renderImage->updateGL();
		renderImage->setShadeEnabled(false);
		std::vector<RGBA> tmp1,tmp2;
		int iW,iH;
		if(ReadImageFromFile(mSpringlMatcap,tmp1,iW,iH)){
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

		if(ReadImageFromFile(mIsoMatcap,tmp2,iW,iH)){
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

		if(ReadImageFromFile(mColorMap,tmp2,iW,iH)){
			glGenTextures(1, &mColormapId);
			glBindTexture( GL_TEXTURE_2D, mColormapId);
			glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, iW, iH, 0, GL_RGBA,GL_UNSIGNED_BYTE, &tmp2[0]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glBindTexture( GL_TEXTURE_2D, 0);
			if(glGetError()!=GL_NO_ERROR)throw Exception("GL Error loading color map.");
		} else {
			std::cerr<<"Could not read "<<mColorMap<<std::endl;
		}
		tmp2.clear();
		glUseProgram((GLuint)NULL);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			throw Exception("Could not initialize frame buffer.");
		}
}

bool GLSpringlShader::save(const std::string& file){
	return renderImage->write(file);
}
void GLSpringlShader::compute(GLFWwindow* win){
	glEnable(GL_DEPTH_TEST);
	isoImage->begin();
		glUseProgram(mNormalsAndDepthProgram.GetProgramHandle());
		glUniform1f(glGetUniformLocation(mNormalsAndDepthProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
		glUniform1f(glGetUniformLocation(mNormalsAndDepthProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());
		mCamera->aim(0,0, w, h,mNormalsAndDepthProgram);
		glDisable(GL_BLEND);
		mSpringLS->mIsoSurface.draw();
		glUseProgram((GLuint)NULL);
	isoImage->end();
	springlImage->begin();
		glUseProgram(mNormalsAndDepthProgram.GetProgramHandle());
		glUniform1f(glGetUniformLocation(mNormalsAndDepthProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
		glUniform1f(glGetUniformLocation(mNormalsAndDepthProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());
		mCamera->aim(0,0, w, h,mNormalsAndDepthProgram);
		glDisable(GL_BLEND);
		mSpringLS->mConstellation.draw();
		glUseProgram((GLuint)NULL);
	springlImage->end();
	colorImage->begin();
		glUseProgram(mColorProgram.GetProgramHandle());
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, mColormapId);
		glUniform1i(glGetUniformLocation(mColorProgram.GetProgramHandle(),"colormapTexture"),0);
		//glUniform1f(glGetUniformLocation(mColorProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
		//glUniform1f(glGetUniformLocation(mColorProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());
		glUniform1f(glGetUniformLocation(mColorProgram.GetProgramHandle(),"colorMapValue"),colorMapValue);
		glUniform1f(glGetUniformLocation(mColorProgram.GetProgramHandle(),"minVelocity"),mSpringLS->mParticleVolume.mMinVelocityMagnitude);
		glUniform1f(glGetUniformLocation(mColorProgram.GetProgramHandle(),"maxVelocity"),mSpringLS->mParticleVolume.mMaxVelocityMagnitude);
		mCamera->aim(0,0, w, h,mColorProgram);
		glDisable(GL_BLEND);
		mSpringLS->mConstellation.draw();
		glUseProgram((GLuint)NULL);
	colorImage->end();
	wireImage->begin();
		glUseProgram(mWireframeProgram.GetProgramHandle());
		mCamera->aim(0,0, w, h,mWireframeProgram);
		float scale=mCamera->getScale();
		glUniform1f(glGetUniformLocation(mWireframeProgram.GetProgramHandle(),"SCALE"),scale);
		glUniform1f(glGetUniformLocation(mWireframeProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
		glUniform1f(glGetUniformLocation(mWireframeProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());
		glDisable(GL_BLEND);
		mSpringLS->mConstellation.draw();
		glUseProgram((GLuint)NULL);
	wireImage->end();
}
void GLSpringlShader::render(GLFWwindow* win) {
	int winw,winh;
	glfwGetWindowSize(win,&winw,&winh);
	//isoImage->render(win);
	//springlImage->render(win);
	//wireImage->render(win);
	//colorImage->render(win);
	float scale=mCamera->getScale();
	glDisable(GL_DEPTH_TEST);
	glUseProgram(mMixerProgram.GetProgramHandle());
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"isoTexture"),0);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"springlsTexture"),1);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"wireTexture"),2);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"colorTexture"),3);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"matcapTexture1"),4);
	glUniform1i(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"matcapTexture2"),5);

	glUniform1f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"SCALE"),scale);
	glUniform1f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"MAX_DEPTH"),mCamera->farPlane());
	glUniform1f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"MIN_DEPTH"),mCamera->nearPlane());
	glUniform2f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"IMG_POS"),x,y);
	glUniform2f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"IMG_DIMS"),w,h);
	glUniform2f(glGetUniformLocation(mMixerProgram.GetProgramHandle(),"SCREEN_DIMS"),winw,winh);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,isoImage->textureId());

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, springlImage->textureId());

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, wireImage->textureId());

	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, colorImage->textureId());

	glActiveTexture(GL_TEXTURE4);
	glBindTexture(GL_TEXTURE_2D, mMatCapId1);

	glActiveTexture(GL_TEXTURE5);
	glBindTexture(GL_TEXTURE_2D, mMatCapId2);


	//For some reason, I need this. Otherwise the last texture isn't found by the shader.
	glEnable(GL_BLEND);
	renderImage->render(win);
	glUseProgram((GLuint)NULL);

}
GLSpringlShader::~GLSpringlShader() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
