/*
 * Copyright(C) 2014, Blake Lucas (img.science@gmail.com)
 * All rights reserved.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
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
#include "GLImage.h"
namespace imagesci {
class GLText:public GLImage {
protected:
	void CreateBitmapString(FT_Face face, const char* message,int fontHeight,int textureId,bool center);
	FT_Face m_face;
	FT_Library m_library;
	std::string fontName;
	std::string mText;
public:

	GLText(int x,int y,int width,int height,const char* fname="verdana.ttf");
	void updateGL();
	void setText(const std::string& message,int fontHeight=14,bool center=false);
	virtual ~GLText();
};

} /* namespace imagesci */

#endif /* TEXT_H_ */
