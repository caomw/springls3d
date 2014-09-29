/*
 * Image.cc
 *
 *  Created on: Sep 21, 2014
 *      Author: blake
 */

#include "Image.h"
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
namespace imagesci {
const UV Image::TextureCoords[6]={
		UV(1.0f,1.0f),UV(0.0f,1.0f),UV(0.0f,0.0f),
		UV(0.0f,0.0f),UV(1.0f,0.0f),UV(1.0f,1.0f)};
const float3 Image::PositionCoords[6]={
		float3(1.0f,1.0f,0.0f),float3(0.0f,1.0f,0.0f),float3(0.0f,0.0f,0.0f),
		float3(0.0f,0.0f,0.0f),float3(1.0f,0.0f,0.0f),float3(1.0f,1.0f,0.0f)};
void Image::render() {
	glBindVertexArray (vao);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, mPositionBuffer);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, mUVBuffer);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindTexture(GL_TEXTURE_2D, mTextureId);

	glDrawArrays(GL_TRIANGLES,0,6);
	glBindVertexArray (0);

	glBindTexture(GL_TEXTURE_2D, 0);
	glBindBuffer(GL_ARRAY_BUFFER,0);
}
Image::Image(int _x,int _y,int _width,int _height,int imageWidth,int imageHeight,bool floatType):mFloatType(floatType),GLComponent(_x,_y,_width,_height),mWidth(imageWidth),mHeight(imageHeight),mTextureId(0),mData(imageWidth*imageHeight,RGBA(0,0,0,0)),mUVBuffer(0),mPositionBuffer(0),vao(0){

}
Image::Image(const std::vector<RGBA>& data,int width,int height):mFloatType(false),GLComponent(0,0,width,height),mWidth(width),mHeight(height),mTextureId(0),mUVBuffer(0),mPositionBuffer(0),vao(0){
	mData=data;
}
Image::Image(const std::vector<RGBAf>& data,int width,int height):mFloatType(true),GLComponent(0,0,width,height),mWidth(width),mHeight(height),mTextureId(0),mUVBuffer(0),mPositionBuffer(0),vao(0){
	mDataf=data;
}
Image* Image::read(const std::string& file){
	int w,h;
	std::vector<RGBA> data;
	if(ReadImageFromFile(file,data,w,h)){
		return new Image(data,w,h);
	} else {
		return NULL;
	}
}
bool Image::write(const std::string& file){
	return WriteImageToFile(file,mData,mWidth,mHeight);
}
Image::~Image(){
	if(mTextureId!=0){
		glDeleteTextures(1,&mTextureId);
		mTextureId=0;
	}
}
void Image::updateGL() {
	if(mTextureId==0){
		glGenTextures( 1,&mTextureId);
	}
	glGenVertexArrays (1, &vao);
	glBindTexture( GL_TEXTURE_2D, mTextureId);
	if(mFloatType){
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA32F, mWidth, mHeight, 0, GL_RGBA,
				GL_FLOAT, &mDataf[0]);
	} else {
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, mWidth, mHeight, 0, GL_RGBA,
			GL_UNSIGNED_BYTE, &mData[0]);

	}
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glBindTexture( GL_TEXTURE_2D, 0);
	glGenBuffers(1, &mPositionBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, mPositionBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) *3* 6,PositionCoords, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture( GL_TEXTURE_2D, 0);

	glGenBuffers(1, &mUVBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, mUVBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2*6,TextureCoords, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
} /* namespace imagesci */
