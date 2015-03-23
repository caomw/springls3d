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
#include "ply_io.h"
using namespace openvdb;
using namespace openvdb::tools;
namespace imagesci {
PlyProperty ParticleVertProps[] = { // property information for a vertex
		{ "x", Float32, Float32, static_cast<int>(offsetof(plyParticle, x)), 0, 0,
				0, 0 },
		{ "y", Float32, Float32,
				static_cast<int>(offsetof(plyParticle, x) + sizeof(float)), 0, 0,
				0, 0 },
				{ "z", Float32, Float32,
				static_cast<int>(offsetof(plyParticle, x) + sizeof(float)
						+ sizeof(float)), 0, 0, 0, 0 },
		{ "intensity", Float32, Float32,
				static_cast<int>(offsetof(plyParticle, radius)), 0, 0, 0, 0 }, };


ParticleVolume::ParticleVolume() :
		mBoundingBox(Vec3d(0, 0, 0), Vec3d(1, 1, 1)), mMinVelocityMagnitude(0.0), mMaxVelocityMagnitude(0.0f),mGL(),mVelocityCount(0), mParticleCount(0), mPose(
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
	if(mParticles.size()==0)return false;
	std::cout<<"Saving "<<f<<" ... ";
	int i, j, idx;
	char* elemNames[]={"vertex"};
	const char* fileName = f.c_str();
	PlyFile *ply;
	// Get input and check data
	ply = open_for_writing_ply(fileName, 1, elemNames, PLY_BINARY_LE);
	if (ply == NULL){
		std::cout<<"Failed. "<<std::endl;
		return false;
	}
	// compute colors, if any
	int numPts = mParticles.size();
	// describe what properties go into the vertex and face elements
	element_count_ply(ply, "vertex", numPts);
	ply_describe_property(ply, "vertex", &ParticleVertProps[0]);
	ply_describe_property(ply, "vertex", &ParticleVertProps[1]);
	ply_describe_property(ply, "vertex", &ParticleVertProps[2]);
	ply_describe_property(ply, "vertex", &ParticleVertProps[3]);
	// write a comment and an object information field
	append_comment_ply(ply, "PLY File");
	append_obj_info_ply(ply, "ImageSci");
	// complete the header
	header_complete_ply(ply);
	// set up and write the vertex elements
	plyParticle vert;
	put_element_setup_ply(ply, "vertex");
	for (i = 0; i < numPts; i++) {
		Vec4s& pt = mParticles[i];
		vert.x[0] = pt[0];
		vert.x[1] = pt[1];
		vert.x[2] = pt[2];
		vert.radius=pt[3];
		put_element_ply(ply, (void *) &vert);
	}
	// close the PLY file
	close_ply(ply);
	free_ply(ply);
	std::cout<<"Done."<<std::endl;
	return true;
}
//Implement me
bool ParticleVolume::open(const std::string& file) {
	int i, j, k;
	int numPts = 0, numPolys = 0;
	// open a PLY file for reading
	PlyFile *ply;
	int nelems = 3, fileType = PLY_BINARY_LE, numElems, nprops;
	char** elist = NULL;
	char* elemName;
	float version;
	if (!(ply = ply_open_for_reading(file.c_str(), &nelems, &elist, &fileType,
			&version))) {
		std::cerr << "Could not open ply file. ["<<file<<"]" << std::endl;
		free_ply(ply);
		return false;
	}
	// Check to make sure that we can read geometry
	PlyElement *elem;
	int index;
	if ((elem = find_element(ply, "vertex")) == NULL
			|| find_property(elem, "x", &index) == NULL
			|| find_property(elem, "y", &index) == NULL
			|| find_property(elem, "z", &index) == NULL
			|| find_property(elem, "intensity", &index) == NULL) {
		std::cerr << "Cannot read geometry" << std::endl;
		close_ply(ply);
		return false;
	}
	this->mParticles.clear();
	plyParticle vertex;
	// Okay, now we can grab the data
	for (i = 0; i < nelems; i++) {
		//get the description of the first element */
		elemName = elist[i];
		get_element_description_ply(ply, elemName, &numElems, &nprops);
		// if we're on vertex elements, read them in
		if (elemName && !strcmp("vertex", elemName)) {
			// Create a list of points
			numPts = numElems;
			this->mParticles.resize(numPts,Vec4s(0.0f));
			// Setup to read the PLY elements
			ply_get_property(ply, elemName, &ParticleVertProps[0]);
			ply_get_property(ply, elemName, &ParticleVertProps[1]);
			ply_get_property(ply, elemName, &ParticleVertProps[2]);
			ply_get_property(ply, elemName, &ParticleVertProps[3]);

			for (j = 0; j < numPts; j++) {
				get_element_ply(ply, &vertex);
				this->mParticles[j] = Vec4s(vertex.x[0], vertex.x[1], vertex.x[2],vertex.radius);
			}
		}
	} //for all elements of the PLY file
	close_ply(ply);
	free_ply(ply);
	if (this->mParticles.size() > 0) {
		return true;
	} else {
		return false;
	}
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
	if (mGL.mVelocityBuffer > 0) {
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mVelocityBuffer);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	}
	glDrawArrays(GL_POINTS, 0, mParticleCount);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
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
	if (mVelocities.size() > 0) {
		if (glIsBuffer(mGL.mVelocityBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mVelocityBuffer);
		glGenBuffers(1, &mGL.mVelocityBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mVelocityBuffer);
		if (glIsBuffer(mGL.mVelocityBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create velocity buffer");

		mMaxVelocityMagnitude=0.0f;
		mMinVelocityMagnitude=std::numeric_limits<float>::min();
		int count=0;
		for(openvdb::Vec3s& vel:mVelocities){
			//vel=Vec3s((float)(count++)/(float)mVelocities.size(),0.0f,0.0f);
			float l=vel.lengthSqr();
			mMaxVelocityMagnitude=std::max(l,mMaxVelocityMagnitude);
			mMinVelocityMagnitude=std::min(l,mMinVelocityMagnitude);
		}
		mMaxVelocityMagnitude=std::sqrt(mMaxVelocityMagnitude);
		mMinVelocityMagnitude=std::sqrt(mMinVelocityMagnitude);

		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * mVelocities.size(),
				&mVelocities[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		mVelocityCount = mVelocities.size();
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
