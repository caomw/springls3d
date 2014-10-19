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
#ifndef MESH_H_
#define MESH_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/LevelSetUtil.h>
#undef OPENVDB_REQUIRE_VERSION_NAME

#include <mutex>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>
namespace imagesci {
struct GLMesh{
public:
	GLuint mVao;
	GLuint mVertexBuffer;
	GLuint mParticleBuffer;
	GLuint mNormalBuffer;
	GLuint mParticleNormalBuffer;
	GLuint mColorBuffer;
	GLuint mTriIndexBuffer;
	GLuint mQuadIndexBuffer;
	GLuint mLineBuffer;
	GLMesh():
		mVao(0),
		mVertexBuffer(0),
		mParticleBuffer(0),
		mNormalBuffer(0),
		mParticleNormalBuffer(0),
		mColorBuffer(0),
		mTriIndexBuffer(0),
		mQuadIndexBuffer(0),
		mLineBuffer(0){

	}
};
class Mesh{
	private:
	GLMesh mGL;
	openvdb::math::Mat4f mPose;
	openvdb::math::BBox<openvdb::Vec3d> mBoundingBox;
	public:
		enum PrimitiveType {QUADS=4,TRIANGLES=3};
		GLuint mTriangleCount;
		GLuint mParticleCount;
		GLuint mQuadCount;
		GLuint mTriangleIndexCount;
		GLuint mQuadIndexCount;


		std::vector<openvdb::Vec3s> mLines;
		std::vector<openvdb::Vec3s> mParticles;
		std::vector<openvdb::Vec3s> mParticleNormals;
		std::vector<openvdb::Vec3s> mVertexes;
		std::vector<openvdb::Vec3s> mColors;
		std::vector<openvdb::Vec3s> mVertexNormals;
		std::vector<openvdb::Index32> mQuadIndexes;
		std::vector<openvdb::Index32> mTriIndexes;
		std::vector<openvdb::Vec2s> uvMap;
		std::vector<openvdb::Vec3s> mVertexAuxBuffer;
		std::vector<openvdb::Vec3s> mParticleAuxBuffer;
		std::vector<openvdb::Vec4I> mFaces;
		Mesh();
		void create(openvdb::tools::VolumeToMesh& mesher,openvdb::FloatGrid::Ptr grid);
		inline openvdb::BBoxd getBoundingBox(){return mBoundingBox;}
		openvdb::math::BBox<openvdb::Vec3d>& updateBoundingBox();
		void draw();
		void scale(float sc);
		inline void setPose(openvdb::Mat4s& pose){
			mPose=pose;
		}
		inline openvdb::Mat4s& getPose(){
			return mPose;
		}
		void updateGL();
		void updateVertexNormals(int SMOOTH_ITERATIONS=0,float DOT_TOLERANCE=0.75f);
		void mapIntoBoundingBox(float voxelSize);
		void mapOutOfBoundingBox(float voxelSize);
		bool openMesh(const std::string& file);
		bool openGrid(const std::string& file);
		bool save(const std::string& file);
		void create(openvdb::FloatGrid::Ptr grid);
		float estimateVoxelSize(int stride=4);
		~Mesh();
};
class MeshVertexRange {
public:
	class Iterator {
	public:
		Iterator(const MeshVertexRange& range, size_t pos) :
				mRange(range), mPos(pos) {
			assert(this->isValid());
		}

		/// Advance to the next leaf node.
		Iterator& operator++() {
			++mPos;
			return *this;
		}
		/// Return a reference to the leaf node to which this iterator is pointing.
		size_t operator*() const {
			return mPos;
		}

		/// Return a pointer to the leaf node to which this iterator is pointing.
		size_t operator->() const {
			return *this;
		}

		/// Return the index into the leaf array of the current leaf node.
		size_t pos() const {
			return mPos;
		}
		bool isValid() const {
			return mPos >= mRange.mBegin && mPos <= mRange.mEnd;
		}
		/// Return @c true if this iterator is not yet exhausted.
		bool test() const {
			return mPos < mRange.mEnd;
		}
		/// Return @c true if this iterator is not yet exhausted.
		operator bool() const {
			return this->test();
		}
		/// Return @c true if this iterator is exhausted.
		bool empty() const {
			return !this->test();
		}
		bool operator!=(const Iterator& other) const {
			return (mPos != other.mPos) || (&mRange != &other.mRange);
		}
		bool operator==(const Iterator& other) const {
			return !(*this != other);
		}
		const MeshVertexRange& leafRange() const {
			return mRange;
		}

	protected:
		const MeshVertexRange& mRange;
		size_t mPos;
	}; // end Iterator

	MeshVertexRange(size_t begin, size_t end, Mesh& mesh,
			size_t grainSize = 1) :
			mEnd(end), mBegin(begin), mGrainSize(grainSize), mMesh(
					mesh) {
	}

	MeshVertexRange(Mesh& mesh, size_t grainSize = 1) :
			mEnd(mesh.mVertexes.size()), mBegin(0), mGrainSize(
					grainSize), mMesh(mesh) {
	}

	Iterator begin() const {
		return Iterator(*this, mBegin);
	}

	Iterator end() const {
		return Iterator(*this, mEnd);
	}

	size_t size() const {
		return mEnd - mBegin;
	}

	size_t grainsize() const {
		return mGrainSize;
	}

	const Mesh& getMesh() const {
		return mMesh;
	}

	bool empty() const {
		return !(mBegin < mEnd);
	}

	bool is_divisible() const {
		return mGrainSize < this->size();
	}

	MeshVertexRange(MeshVertexRange& r, tbb::split) :
			mEnd(r.mEnd), mBegin(doSplit(r)), mGrainSize(r.mGrainSize), mMesh(
					r.mMesh) {
	}
	Mesh& mMesh;
private:
	size_t mEnd, mBegin, mGrainSize;

	static size_t doSplit(MeshVertexRange& r) {
		assert(r.is_divisible());
		size_t middle = r.mBegin + (r.mEnd - r.mBegin) / 2u;
		r.mEnd = middle;
		return middle;
	}
};
}
#endif /* MESH_H_ */
