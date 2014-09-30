/*
 * Image.h
 *
 *  Created on: Sep 21, 2014
 *      Author: blake
 */

#ifndef IMAGE_H_
#define IMAGE_H_
#include "ImageSciUtil.h"
#include "GLComponent.h"
#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
#include <openvdb/openvdb.h>
namespace imagesci {
class Image:public GLComponent {
protected:
	std::vector<RGBA> mData;
	std::vector<RGBAf> mDataf;
	int mWidth;
	int mHeight;
	GLuint vao;
	GLuint mPositionBuffer;
	GLuint mUVBuffer;
	unsigned int mTextureId;
	bool mFloatType;
public:
	static const UV TextureCoords[6];
	static const openvdb::Vec3s PositionCoords[6];
	std::vector<RGBA>& data(){
		return mData;
	}
	inline int width(){return mWidth;}
	inline int height(){return mHeight;}
	unsigned int textureId(){return mTextureId;}
	RGBA& At(int i,int j){return mData[clamp(j,0,mHeight-1)*mWidth+clamp(i,0,mWidth-1)];}
	Image():GLComponent(),mWidth(0),mHeight(0),mData(0),mTextureId(0),mUVBuffer(0),mPositionBuffer(0),vao(0),mFloatType(false){

	}
	void setBounds(int _x,int _y,int _w,int _h){
		x=_x;
		y=_y;
		w=_w;
		h=_h;
	}
	Image(int _x,int _y,int _width,int _height,int imageWidth,int imageHeight,bool floatvType=false);
	Image(const std::vector<RGBA>& data,int width,int height);
	Image(const std::vector<RGBAf>& data,int width,int height);
	static Image* read(const std::string& file);
	bool write(const std::string& file);
	bool writeTexture(const std::string& file);
	virtual void updateGL();
	virtual void render();
	virtual ~Image();
};

} /* namespace imagesci */

#endif /* IMAGE_H_ */
