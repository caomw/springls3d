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
#include "GLText.h"

#include <GL/gl.h>

namespace imagesci {
GLText::~GLText() {
	if (mTextureId != 0) {
		glDeleteTextures(1, &mTextureId);
		mTextureId=0;
	}
}
inline int next_p2(int a) {
	int rval = 1;
	while (rval < a)
		rval <<= 1;
	return rval;
}
void GLText::updateGL() {
	if (mTextureId == 0) {
		glGenTextures(1, &mTextureId);
	}
}
GLText::GLText(int x, int y, int width, int height, const char* fname) :
		GLImage(x, y, width, height, width, height,false), fontName(fname), m_face(
				NULL), m_library(NULL) {
	if (FT_Init_FreeType(&m_library))
		std::cerr << ("FT_Init_FreeType failed") << std::endl;
	//This is where we load in the font information from the file.
	//Of all the places where the code might die, this is the most likely,
	//as FT_New_Face will die if the font file does not exist or is somehow broken.
	if (m_library != NULL
			&& FT_New_Face(m_library, fontName.c_str(), 0, &m_face)) {
		std::cerr
				<< ("FT_New_Face failed (there is probably a problem with your font file)");
		return;
	}
}
void GLText::CreateBitmapString(FT_Face face, const char* message, int fontHeight,
		int textureId, bool center) {
	if (face == NULL)
		return;
	FT_Set_Char_Size(m_face, fontHeight << 6, fontHeight << 6, 96, 96);
	unsigned char ch;
	int padding = std::max(fontHeight / 8, 1);
	//Allocate memory for the texture data.
	int sz = mWidth * mHeight;
	std::vector<GLubyte> txtImage(2 * sz, 0);
	std::vector<GLubyte> txtImageSwap(2 * sz, 0);
	int yOffset = 0;
	int xOffset = padding;
	int spaceBonus = 0;
	for (int i = 0; (ch = message[i]) != '\0'; i++) {
		if (ch == '\n') {
			yOffset += 2 * padding + fontHeight;
			xOffset = padding;
			continue;
		}
		spaceBonus = (ch == ' ') ? std::max(fontHeight / 4, 1) : 0;
		FT_UInt v = FT_Get_Char_Index(face, ch);
		if (FT_Load_Glyph(face, v, FT_LOAD_DEFAULT)) {
			std::cerr << ("FT_Load_Glyph failed") << std::endl;
		}
		FT_Glyph glyph;
		if (FT_Get_Glyph(face->glyph, &glyph)) {
			std::cerr << ("FT_Get_Glyph failed") << std::endl;
		}
		//Convert the glyph to a bitmap.
		FT_Glyph_To_Bitmap(&glyph, ft_render_mode_normal, 0, 1);
		FT_BitmapGlyph bitmap_glyph = (FT_BitmapGlyph) glyph;
		//This reference will make accessing the bitmap easier
		FT_Bitmap& bitmap = bitmap_glyph->bitmap;
		FT_GlyphSlot slot = face->glyph;
		int down = 2 * padding + fontHeight - (slot->metrics.horiBearingY >> 6);
		int left= (slot->metrics.horiBearingX>>6);
		for (int j = 0; j < bitmap.rows; j++) {
			for (int i = 0; i < bitmap.width; i++) {
				int ii = std::max(std::min(xOffset + i+left, mWidth - 1), 0);
				int jj = std::max(std::min(yOffset + down + j, mHeight - 1), 0);
				int offset = (ii + jj * mWidth);
				if (offset > sz) {
					std::cout << "Text string exceeds texture size"
							<< std::endl;
					return;
				}
				unsigned char val = bitmap.buffer[i + bitmap.width * j];
				txtImage[2 * offset] = txtImage[2 * offset + 1] = val;
			}
		}
		xOffset += spaceBonus + (slot->metrics.horiAdvance >> 6);
		if (xOffset + bitmap.width >= mWidth) {
			yOffset += 2 * padding + fontHeight;
			xOffset = padding;
			if (yOffset >= mHeight)
				break;
		}
	}
	yOffset += 4 * padding + fontHeight;
	int shiftX = (center) ? (mWidth - xOffset) / 2 : 0;
	int shiftY = (mHeight - yOffset) / 2;

	for (int j = 0; j < mHeight; j++) {
		for (int i = 0; i < mWidth; i++) {
			int offset = i + j * mWidth;
			int offset2 = i - shiftX + (j - shiftY) * mWidth;
			if (i >= shiftX && j >= shiftY) {
				txtImageSwap[2 * offset] = txtImage[2 * (offset2)];
				txtImageSwap[2 * offset + 1] = txtImage[2 * (offset2) + 1];
			}
		}
	}
	glBindTexture( GL_TEXTURE_2D, textureId);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, mWidth, mHeight, 0,
			GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, &txtImageSwap[0]);
	glBindTexture( GL_TEXTURE_2D, 0);

}
void GLText::setText(const std::string& message, int fontHeight, bool center) {
	if (message != mText) {
		CreateBitmapString(m_face, message.c_str(), fontHeight, mTextureId,
				center);
		mText = message;
	}
}
} /* namespace imagesci */
