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
class GLImage:public GLComponent {
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
	GLShader* getShader();
public:
	static std::unique_ptr<GLShader> defaultShader;
	GLShader* imageShader;
	static const UV TextureCoords[6];
	static const openvdb::Vec3s PositionCoords[6];
	std::vector<RGBA>& data(){
		return mData;
	}
	inline void setShadeEnabled(bool shade){
		mShadeEnabled=shade;
	}
	void setShader(GLShader* shader){
		this->imageShader=shader;
	}
	inline int width(){return mWidth;}
	inline int height(){return mHeight;}
	unsigned int textureId(){return mTextureId;}
	RGBA& At(int i,int j){return mData[clamp(j,0,mHeight-1)*mWidth+clamp(i,0,mWidth-1)];}
	GLImage():GLComponent(),imageShader(NULL),mWidth(0),mHeight(0),mData(0),mTextureId(0),mFloatType(false),mShadeEnabled(true){

	}
	void setBounds(int _x,int _y,int _w,int _h){
		x=_x;
		y=_y;
		w=_w;
		h=_h;
	}
	GLImage(int _x,int _y,int _width,int _height,int imageWidth,int imageHeight,bool floatvType=false);
	GLImage(const std::vector<RGBA>& data,int width,int height);
	GLImage(const std::vector<RGBAf>& data,int width,int height);
	static GLImage* read(const std::string& file);
	bool write(const std::string& file);
	bool writeTexture(const std::string& file);
	virtual void updateGL();
	virtual void render(GLFWwindow* win);
	virtual ~GLImage();
};

} /* namespace imagesci */

#endif /* IMAGE_H_ */
