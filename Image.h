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
#include <openvdb/openvdb.h>
namespace imagesci {
class Image:public GLComponent {
protected:
	std::vector<RGBA> mData;
	int mWidth;
	int mHeight;
	unsigned int mTextureId;
public:
	std::vector<RGBA>& data(){
		return mData;
	}
	inline int width(){return mWidth;}
	inline int height(){return mHeight;}
	unsigned int textureId(){return mTextureId;}
	RGBA& At(int i,int j){return mData[clamp(j,0,mHeight-1)*mWidth+clamp(i,0,mWidth-1)];}
	Image():GLComponent(),mWidth(0),mHeight(0),mData(0),mTextureId(0){

	}
	Image(int width,int height);
	Image(const std::vector<RGBA>& data,int width,int height);
	static std::unique_ptr<Image> read(const std::string& file);
	bool write(const std::string& file);
	void updateGL();
	void render();
	virtual ~Image();
};

} /* namespace imagesci */

#endif /* IMAGE_H_ */
