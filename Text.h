/*
 * Text.h
 *
 *  Created on: Sep 21, 2014
 *      Author: blake
 */

#ifndef TEXT_H_
#define TEXT_H_
//#include FT_FREETYPE_H

#include <freetype2/ft2build.h>
#include <freetype2/fttypes.h>
#include <freetype2/ftglyph.h>
#include <freetype2/freetype.h>
#include <freetype2/ftoutln.h>
#include <freetype2/fttrigon.h>
#include "Image.h"
namespace imagesci {
class Text:public Image {
protected:
	void CreateBitmapString(FT_Face face, const char* message,int fontHeight,int textureId,bool center);
	FT_Face m_face;
	FT_Library m_library;
	std::string fontName;
	std::string mText;
public:

	Text(int x,int y,int width,int height,const char* fname="verdana.ttf");
	void updateGL();
	void setText(const std::string& message,int fontHeight=14,bool center=false);
	virtual ~Text();
};

} /* namespace imagesci */

#endif /* TEXT_H_ */
