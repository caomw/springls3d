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

#include "GLImage.h"
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include "ImageSciUtil.h"
namespace imagesci {
GLuint GLImage::vao=0;
GLuint GLImage::mPositionBuffer=0;
GLuint GLImage::mUVBuffer=0;
std::unique_ptr<GLShader> GLImage::defaultShader;
const UV GLImage::TextureCoords[6]={
		UV(1.0f,1.0f),UV(0.0f,1.0f),UV(0.0f,0.0f),
		UV(0.0f,0.0f),UV(1.0f,0.0f),UV(1.0f,1.0f)};
const float3 GLImage::PositionCoords[6]={
		float3(1.0f,1.0f,0.0f),float3(0.0f,1.0f,0.0f),float3(0.0f,0.0f,0.0f),
		float3(0.0f,0.0f,0.0f),float3(1.0f,0.0f,0.0f),float3(1.0f,1.0f,0.0f)};
GLShader* GLImage::getShader(){
	if(imageShader==NULL){
		if(defaultShader.get()==nullptr){
			std::vector<std::string> attrib={"vp","uv"};
			defaultShader=std::unique_ptr<GLShader>(new GLShader(ReadTextFile("image_shader.vert"),ReadTextFile("image_shader.frag"),"",attrib));
		}
		imageShader=defaultShader.get();
	}
	return imageShader;
}
void GLImage::render(GLFWwindow* win) {
	glBindVertexArray (vao);
	GLShader* shader=NULL;
	if(mShadeEnabled){
		shader=getShader();
		int winw,winh;
		glfwGetWindowSize(win,&winw,&winh);

		shader->begin();
		glEnable(GL_BLEND);
		glActiveTexture(GL_TEXTURE0);
		glUniform1i(glGetUniformLocation(shader->GetProgramHandle(),"textureImage"),0);;
		glUniform2f(glGetUniformLocation(shader->GetProgramHandle(),"IMG_POS"),x,y);
		glUniform2f(glGetUniformLocation(shader->GetProgramHandle(),"IMG_DIMS"),w,h);
		glUniform2f(glGetUniformLocation(shader->GetProgramHandle(),"SCREEN_DIMS"),winw,winh);
		glBindTexture(GL_TEXTURE_2D,mTextureId);
	}

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, mPositionBuffer);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	if(mShadeEnabled){
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, mUVBuffer);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
	}
	glDrawArrays(GL_TRIANGLES,0,6);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER,0);

	if(mShadeEnabled){
		glBindTexture(GL_TEXTURE_2D, 0);
		shader->end();
	}

}
GLImage::GLImage(int _x,int _y,int _width,int _height,int imageWidth,int imageHeight,bool floatType):imageShader(NULL),mFloatType(floatType),GLComponent(_x,_y,_width,_height),mWidth(imageWidth),mHeight(imageHeight),mTextureId(0),mData(imageWidth*imageHeight,RGBA(0,0,0,0)),mShadeEnabled(true){

}
GLImage::GLImage(const std::vector<RGBA>& data,int width,int height):imageShader(NULL),mFloatType(false),GLComponent(0,0,width,height),mWidth(width),mHeight(height),mTextureId(0),mShadeEnabled(true){
	mData=data;
}
GLImage::GLImage(const std::vector<RGBAf>& data,int width,int height):imageShader(NULL),mFloatType(true),GLComponent(0,0,width,height),mWidth(width),mHeight(height),mTextureId(0),mShadeEnabled(true){
	mDataf=data;
}
GLImage* GLImage::read(const std::string& file){
	int w,h;
	std::vector<RGBA> data;
	if(ReadImageFromFile(file,data,w,h)){
		return new GLImage(data,w,h);
	} else {
		return NULL;
	}
}
bool GLImage::write(const std::string& file){
	return WriteImageToFile(file,mData,mWidth,mHeight);
}
bool GLImage::writeTexture(const std::string& file){

	glBindTexture(GL_TEXTURE_2D,mTextureId);
	if(mFloatType){

		glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA32F,GL_FLOAT,&mDataf[0]);
		WriteImageToFile(file,mDataf,mWidth,mHeight);
	} else {
		glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_UNSIGNED_BYTE,&mData[0]);
		WriteImageToFile(file,mData,mWidth,mHeight);
	}
	glBindTexture(GL_TEXTURE_2D, 0);
}
GLImage::~GLImage(){
	if(mTextureId!=0){
		glDeleteTextures(1,&mTextureId);
		mTextureId=0;
	}
}
void GLImage::updateGL() {
	if(mTextureId==0){
		glGenTextures( 1,&mTextureId);
	}
	if(mFloatType){
		glBindTexture( GL_TEXTURE_2D, mTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA32F, mWidth, mHeight, 0, GL_RGBA,
			GL_FLOAT, &mDataf[0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	} else {
		glBindTexture( GL_TEXTURE_2D, mTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, mWidth, mHeight, 0, GL_RGBA,
			GL_UNSIGNED_BYTE, &mData[0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	}
	glBindTexture( GL_TEXTURE_2D, 0);
	if(vao==0){
		glGenVertexArrays (1, &vao);
		if(mPositionBuffer==0){
			glGenBuffers(1, &mPositionBuffer);
			glBindBuffer(GL_ARRAY_BUFFER, mPositionBuffer);
			glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) *3* 6,PositionCoords, GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindTexture( GL_TEXTURE_2D, 0);
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
