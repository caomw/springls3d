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

void Image::updateGL() {
	if(mTextureId>0){
		glDeleteTextures(1,&mTextureId);
	}
	glGenTextures( 1,&mTextureId);
	glBindTexture( GL_TEXTURE_2D, mTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, mWidth, mHeight, 0, GL_RGBA,
			GL_UNSIGNED_BYTE, &data[0]);
	glBindTexture( GL_TEXTURE_2D, 0);
}
Image::~Image() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
