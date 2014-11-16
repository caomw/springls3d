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

#ifndef PARTICLEVOLUME_H_
#define PARTICLEVOLUME_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/LevelSetUtil.h>
#undef OPENVDB_REQUIRE_VERSION_NAME

#include <mutex>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>
namespace imagesci {
struct GLParticleVolume{
public:
	GLuint mVao;
	GLuint mParticleBuffer;
	//GLuint mColorBuffer;
	GLParticleVolume():mVao(0),mParticleBuffer(0){
	}
};
class ParticleVolume {
private:
GLParticleVolume mGL;
openvdb::math::Mat4f mPose;
openvdb::math::BBox<openvdb::Vec3d> mBoundingBox;
public:
	GLuint mParticleCount;
	std::vector<openvdb::Vec4s> mParticles;
	//The "w" component of the color encodes the radius.
	//std::vector<openvdb::Vec4s> mColors;
	ParticleVolume();
	inline openvdb::BBoxd getBoundingBox(){return mBoundingBox;}
	void setBoundingBox(const openvdb::BBoxd& bboxd){
		mBoundingBox=bboxd;
	}

	inline void setPose(openvdb::Mat4s& pose){
		mPose=pose;
	}
	inline openvdb::Mat4s& getPose(){
		return mPose;
	}
	void draw();
	bool save(const std::string& f);
	bool open(const std::string& f);
	void updateGL();
	void reset();
	void mapIntoBoundingBox(float voxelSize);
	void mapOutOfBoundingBox(float voxelSize);
	~ParticleVolume();
};

} /* namespace imagesci */

#endif /* PARTICLEVOLUME_H_ */
