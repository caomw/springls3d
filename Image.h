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
#include "GLShader.h"
namespace imagesci {
class Image:public GLComponent {
protected:
	std::vector<RGBA> mData;
	std::vector<RGBAf> mDataf;
	int mWidth;
	int mHeight;
	bool mShadeEnabled;
	static GLuint vao;
	static GLuint mPositionBuffer;
	static GLuint mUVBuffer;
	unsigned int mTextureId;
	bool mFloatType;
	static GLShader* GetShader(){
		if(imageShader.get()==nullptr){
			std::vector<std::string> attrib={"vp","uv"};
			imageShader=std::unique_ptr<GLShader>(new GLShader(ReadTextFile("image_shader.vert"),ReadTextFile("image_shader.frag"),"",attrib));
		}
		return imageShader.get();
	}
public:
	static std::unique_ptr<GLShader> imageShader;
	static const UV TextureCoords[6];
	static const openvdb::Vec3s PositionCoords[6];
	std::vector<RGBA>& data(){
		return mData;
	}
	inline void setShadeEnabled(bool shade){
		mShadeEnabled=shade;
	}
	inline int width(){return mWidth;}
	inline int height(){return mHeight;}
	unsigned int textureId(){return mTextureId;}
	RGBA& At(int i,int j){return mData[clamp(j,0,mHeight-1)*mWidth+clamp(i,0,mWidth-1)];}
	Image():GLComponent(),mWidth(0),mHeight(0),mData(0),mTextureId(0),mFloatType(false),mShadeEnabled(true){

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
	virtual void render(GLFWwindow* win);
	virtual ~Image();
};

} /* namespace imagesci */

#endif /* IMAGE_H_ */
