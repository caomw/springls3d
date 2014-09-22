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
	glBindTexture(GL_TEXTURE_2D, mTextureId);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f, 1.0f);
	glVertex2f(mWidth, mHeight);
	glTexCoord2f(0.0f, 1.0f);
	glVertex2f(0, mHeight);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(0, 0);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f(mWidth, 0);
	glEnd();
	glBindTexture(GL_TEXTURE_2D, 0);
}
Image::Image(int width,int height):GLComponent(),mWidth(width),mHeight(height),mTextureId(0),mData(width*height,RGBA(0,0,0,0)){

}
Image::Image(const std::vector<RGBA>& data,int width,int height):GLComponent(),mWidth(width),mHeight(height),mTextureId(0){
	mData=data;
}
std::unique_ptr<Image> Image::read(const std::string& file){
	int w,h;
	std::vector<RGBA> data;
	if(ReadImageFromFile(file,data,w,h)){
		return std::unique_ptr<Image>(new Image(data,w,h));
	} else {
		return std::unique_ptr<Image>();
	}
}
bool Image::write(const std::string& file){
	return WriteImageToFile(file,mData,mWidth,mHeight);
}
Image::~Image(){
	if(mTextureId>0)glDeleteTextures(1,&mTextureId);
}
void Image::updateGL() {
	if(mTextureId==0){
		glGenTextures( 1,&mTextureId);
	}
	glBindTexture( GL_TEXTURE_2D, mTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, mWidth, mHeight, 0, GL_RGBA,
			GL_UNSIGNED_BYTE, &mData[0]);
	glBindTexture( GL_TEXTURE_2D, 0);
}
} /* namespace imagesci */
