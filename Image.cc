/*
 * Image.cc
 *
 *  Created on: Sep 21, 2014
 *      Author: blake
 */

#include "Image.h"
#include <GL/gl.h>
namespace imagesci {
void Image::render() {
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
}
Image::Image(int _x,int _y,int _width,int _height,int imageWidth,int imageHeight):GLComponent(_x,_y,_width,_height),mWidth(imageWidth),mHeight(imageHeight),mTextureId(0),mData(imageWidth*imageHeight,RGBA(0,0,0,0)){

}
Image::Image(const std::vector<RGBA>& data,int width,int height):GLComponent(0,0,width,height),mWidth(width),mHeight(height),mTextureId(0){
	mData=data;
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

	glBindTexture( GL_TEXTURE_2D, mTextureId);
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, mWidth, mHeight, 0, GL_RGBA,
			GL_UNSIGNED_BYTE, &mData[0]);
	glBindTexture( GL_TEXTURE_2D, 0);
}
} /* namespace imagesci */
