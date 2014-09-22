/*
 * Text.cc
 *
 *  Created on: Sep 21, 2014
 *      Author: blake
 */

#include "Text.h"

#include <GL/gl.h>


namespace imagesci {
Text::~Text() {
}
inline int next_p2 ( int a )
{
	int rval=1;
	while(rval<a) rval<<=1;
	return rval;
}

void Text::CreateBitmapString(FT_Face face, const char* message,int fontHeight,int textureId,bool center){
	if (face == NULL)return;
	unsigned char ch;
	int padding=std::max(fontHeight/8,1);
	//Allocate memory for the texture data.
	int sz=mWidth*mHeight;
	GLubyte* expanded_data = new GLubyte[2* sz];
	GLubyte* expanded_data_swap = new GLubyte[2* sz];
	memset(expanded_data,0,sizeof(GLubyte)*2*sz);
	memset(expanded_data_swap,0,sizeof(GLubyte)*2*sz);
	int yOffset=0;
	int xOffset=padding;
	int spaceBonus=0;
	for(int i=0;(ch=message[i])!='\0';i++){
		if(ch=='\n'){
			yOffset+=2*padding+fontHeight;
			xOffset=padding;
			continue;
		}
		spaceBonus=(ch==' ')?std::max(fontHeight/4,1):0;
		FT_UInt v=FT_Get_Char_Index( face, ch );
		if(FT_Load_Glyph( face, v, FT_LOAD_DEFAULT ))
		{
			delete [] expanded_data;
			delete [] expanded_data_swap;
			throw std::runtime_error("FT_Load_Glyph failed");
		}
		FT_Glyph glyph;
		if(FT_Get_Glyph( face->glyph, &glyph ))
		{
			delete [] expanded_data;
			delete [] expanded_data_swap;
			throw std::runtime_error("FT_Get_Glyph failed");
		}
		//Convert the glyph to a bitmap.
		FT_Glyph_To_Bitmap( &glyph, ft_render_mode_normal, 0, 1 );
		FT_BitmapGlyph bitmap_glyph = (FT_BitmapGlyph)glyph;
		//This reference will make accessing the bitmap easier
		FT_Bitmap& bitmap=bitmap_glyph->bitmap;
		FT_GlyphSlot slot = face->glyph;

		int down =2*padding+fontHeight- (slot->metrics.horiBearingY  >> 6);
		for(int j=0; j <bitmap.rows;j++)
		{
			for(int i=0; i < bitmap.width; i++)
			{
				int ii=std::max(std::min(xOffset+i,mWidth-1),0);
				int jj=std::max(std::min(yOffset+down+j,mHeight-1),0);
				int offset=(ii+jj*mWidth);
				if(offset>sz)
				{
					std::cout<<"Text string exceeds texture size"<<std::endl;
					delete [] expanded_data;
					delete [] expanded_data_swap;
					return;
				}
				unsigned char val= bitmap.buffer[i + bitmap.width*j];
				expanded_data[2*offset]=expanded_data[2*offset+1]=val;
			}
		}
		xOffset+=spaceBonus+padding+(slot->metrics.width>>6);
		if(xOffset+bitmap.width>=mWidth)
		{
			yOffset+=2*padding+fontHeight;
			xOffset=padding;
			if(yOffset>=mHeight)break;
		}
	}
		yOffset+=4*padding+fontHeight;
		int shiftX=(center)?(mWidth-xOffset)/2:0;
		int shiftY=(mHeight-yOffset)/2;

		for(int j=0; j <mHeight;j++)
		{
			for(int i=0; i < mWidth; i++)
			{
				int offset=i+j*mWidth;
				int offset2=i-shiftX+(j-shiftY)*mWidth;
				if(i>=shiftX&&j>=shiftY)
				{
					expanded_data_swap[2*offset]=expanded_data[2*(offset2)];
					expanded_data_swap[2*offset+1]=expanded_data[2*(offset2)+1];
				}
			}
		}

	//Now we just setup some texture paramaters.
	glBindTexture( GL_TEXTURE_2D, textureId);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	//Here we actually create the texture itself, notice
	//that we are using GL_LUMINANCE_ALPHA to indicate that
	//we are using 2 channel data.
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA,mWidth, mHeight,0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, expanded_data_swap);
	glBindTexture( GL_TEXTURE_2D,0);

	//With the texture created, we don't need to expanded data anymore
	delete [] expanded_data;
	delete [] expanded_data_swap;
}
void Text::setText(const std::string& message,bool center){
	if(mTextureId==0){
		glGenTextures(1,&mTextureId);
	}
	CreateBitmapString(m_face,message.c_str(),m_fontHeight,mTextureId,center);
}
} /* namespace imagesci */
