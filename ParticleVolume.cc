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

#include "ParticleVolume.h"
#include "ImageSciUtil.h"
using namespace openvdb;
using namespace openvdb::tools;
namespace imagesci {
ParticleVolume::ParticleVolume() :
		mBoundingBox(Vec3d(0, 0, 0), Vec3d(1, 1, 1)), mGL(), mParticleCount(0), mPose(
				openvdb::math::Mat4f::identity()) {
	// TODO Auto-generated constructor stub
}

ParticleVolume::~ParticleVolume() {
	// TODO Auto-generated destructor stub
}
void ParticleVolume::reset() {
	mParticles.clear();
	//mColors.clear();
}
//Implement me
bool ParticleVolume::save(const std::string& f) {
	return false;
}
//Implement me
bool ParticleVolume::open(const std::string& file) {
	return false;
}
void ParticleVolume::mapIntoBoundingBox(float voxelSize) {
	Vec3s minPt = mBoundingBox.min();
	for (Vec4s& pt : mParticles) {
		pt = (pt - Vec4s(minPt[0],minPt[1],minPt[2],0.0)) / voxelSize;
	}
}
void ParticleVolume::mapOutOfBoundingBox(float voxelSize) {
	Vec3s minPt = mBoundingBox.min();
	for (Vec4s& pt : mParticles) {
		pt = pt * voxelSize + Vec4s(minPt[0],minPt[1],minPt[2],0.0);
	}
}
void ParticleVolume::draw() {
	glBindVertexArray(mGL.mVao);
	if (mGL.mParticleBuffer > 0) {
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mParticleBuffer);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	}
	glDrawArrays(GL_POINTS, 0, mParticleCount);
	glDisableVertexAttribArray(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ParticleVolume::updateGL() {
	mParticleCount = 0;
	if (mGL.mVao == 0)
		glGenVertexArrays(1, &mGL.mVao);
	if (mParticles.size() > 0) {
		if (glIsBuffer(mGL.mParticleBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mParticleBuffer);
		glGenBuffers(1, &mGL.mParticleBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mParticleBuffer);
		if (glIsBuffer(mGL.mParticleBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create particle buffer");
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * mParticles.size(),
				&mParticles[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		mParticleCount = mParticles.size();
	}
	/*
	if (mColors.size() > 0) {
		if (glIsBuffer(mGL.mColorBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mColorBuffer);

		glGenBuffers(1, &mGL.mColorBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mColorBuffer);
		if (glIsBuffer(mGL.mColorBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create color buffer");

		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * mColors.size(),
				&mColors[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	*/
}
} /* namespace imagesci */
