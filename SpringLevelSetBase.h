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
 *
 * This is a re-implementation of the following work:
 *
 * Lucas, Blake C., Michael Kazhdan, and Russell H. Taylor. "Spring level sets: a deformable model
 * representation to provide interoperability between meshes and level sets."
 * Visualization and Computer Graphics, IEEE Transactions on 19.5 (2013): 852-865.
 */
#ifndef SPRINGLEVELSETBASE_H_
#define SPRINGLEVELSETBASE_H_
#include <openvdb/util/NullInterrupter.h>
#include <tbb/parallel_for.h>
#include <vector>
#include <list>
#include <iostream>
#include "Mesh.h"
#include "ParticleVolume.h"
#include "ImageSciUtil.h"
#include "AdvectionForce.h"
#include "json/JsonSerializable.h"
#include "fluid/fluid_common.h"
#undef OPENVDB_REQUIRE_VERSION_NAME

namespace imagesci {
enum TemporalIntegrationScheme {
	UNKNOWN_TIS = openvdb::math::TemporalIntegrationScheme::UNKNOWN_TIS,
	RK1 = openvdb::math::TemporalIntegrationScheme::TVD_RK1,
	RK2 = openvdb::math::TemporalIntegrationScheme::TVD_RK2,
	RK3 = openvdb::math::TemporalIntegrationScheme::TVD_RK3,
	RK4a,
	RK4b
};
enum MotionScheme {
	UNDEFINED,
	IMPLICIT,
	SEMI_IMPLICIT,
	EXPLICIT
};
MotionScheme DecodeMotionScheme(const std::string& name);
std::string EncodeMotionScheme(MotionScheme name);

struct Springl {
private:
	Mesh* mesh;
public:
	openvdb::Index32 id;
	openvdb::Index32 offset;
	openvdb::Vec3s& normal() const {
		return mesh->mParticleNormals[id];
	}
	openvdb::Vec3s& particle() const {
		return mesh->mParticles[id];
	}
	openvdb::Vec3s& velocity() const {
		return mesh->mParticleVelocity[id];
	}
	uint8_t& label() const {
		return mesh->mParticleLabel[id];
	}
	openvdb::Vec3s& operator[](size_t idx) {
		return mesh->mVertexes[offset + idx];
	}
	const openvdb::Vec3s& operator[](size_t idx) const {
		return mesh->mVertexes[offset + idx];
	}
	Springl(Mesh* _mesh = NULL) :
			id(0), offset(0), mesh(_mesh) {
	}
	int size() const;
	float area() const;
	openvdb::math::BBox<Vec3s> getBoundingBox();
	float distanceToFace(const openvdb::Vec3s& pt);
	float signedDistanceToFace(const openvdb::Vec3s& pt);
	float distanceToFaceSqr(const openvdb::Vec3s& pt);
	float signedDistanceToFaceSqr(const openvdb::Vec3s& pt);
	float distanceToParticle(const openvdb::Vec3s& pt);
	float distanceToParticleSqr(const openvdb::Vec3s& pt);
	float distanceToEdgeSqr(const openvdb::Vec3s& pt, int e);
	float distanceToEdge(const openvdb::Vec3s& pt, int e);
	openvdb::Vec3s computeCentroid() const;
	openvdb::Vec3s computeNormal(const float eps = 1E-6f) const;
	~Springl() {
	}
};

struct SpringlNeighbor {
public:
	openvdb::Index32 springlId;
	int edgeId;
	float distance;
	SpringlNeighbor(openvdb::Index32 id = 0, int nbr = -1, float _distance = 0) :
			springlId(id), edgeId(nbr), distance(_distance) {
	}

	friend bool operator<(const SpringlNeighbor& first,
			const SpringlNeighbor& second) {
		return (first.distance < second.distance);
	}
};

std::ostream& operator<<(std::ostream& ostr, const SpringlNeighbor& classname);
class Constellation: public Mesh {
public:

	std::vector<Springl> springls;
	void create(Mesh* mesh);
	virtual ~Constellation() {
	}
	inline size_t getNumSpringls() const {
		return springls.size();
	}
	inline size_t getNumVertexes() const {
		return mVertexes.size();
	}

	Springl& operator[](size_t idx) {
		return springls[idx];
	}
	const Springl& operator[](size_t idx) const {
		return springls[idx];
	}
	openvdb::Vec3s closestPointOnEdge(const openvdb::Vec3s& start,
			const SpringlNeighbor& ci);
};

typedef std::vector<std::list<SpringlNeighbor>> NearestNeighborMap;
typedef openvdb::FloatGrid::Ptr SLevelSetPtr;
typedef openvdb::VectorGrid::Ptr SGradientPtr;
typedef openvdb::Int32Grid::Ptr SIndexPtr;
class SpringLevelSetDescription: public JsonSerializable{
	public:
		std::string mConstellationFile;
		std::string mIsoSurfaceFile;
		std::string mParticleVolumeFile;
		std::string mSignedLevelSetFile;
		static std::vector<std::string> mMetricNames;
		std::map<std::string,double> mMetricValues;
		SpringLevelSetDescription();
		void serialize(Json::Value& root_in);
		void deserialize(Json::Value& root_in);
};
class SpringLevelSet {
protected:
	openvdb::tools::VolumeToMesh mVolToMesh;
	openvdb::math::Transform::Ptr mTransform;
	std::list<int> fillList;
	int mFillCount;
	int mCleanCount;
public:
	static const float NEAREST_NEIGHBOR_RANGE; //voxel units
	static const int MAX_NEAREST_NEIGHBORS;
	static const float PARTICLE_RADIUS;
	static const float MAX_VEXT;
	static const float FILL_DISTANCE;
	static const float CLEAN_DISTANCE;
	static const float SHARPNESS;
	static const float SPRING_CONSTANT;
	static const float RELAX_TIMESTEP;
	static const float MIN_ASPECT_RATIO;
	static const float MAX_AREA;
	static const float MIN_AREA;

	Mesh mIsoSurface;
	ParticleVolume mParticleVolume;
	Constellation mConstellation;
	NearestNeighborMap mNearestNeighbors;
	SLevelSetPtr mSignedLevelSet;
	SLevelSetPtr mUnsignedLevelSet;
	SGradientPtr mGradient;
	SIndexPtr mSpringlIndexGrid;

	inline openvdb::math::Transform& transform() {
		return *mTransform;
	}
	inline openvdb::math::Transform::Ptr transformPtr() {
		return mTransform;
	}
	Springl& getSpringl(const openvdb::Index32 id);
	openvdb::Vec3s& getParticle(const openvdb::Index32 id);
	openvdb::Vec3s& getParticleNormal(const openvdb::Index32 id);
	openvdb::Vec3s& getSpringlVertex(const openvdb::Index32 id, const int i);
	openvdb::Vec3s& getSpringlVertex(const openvdb::Index32 gid);
	std::list<SpringlNeighbor>& getNearestNeighbors(openvdb::Index32 id,
			int8_t e);
	inline int getLastFillCount() const {
		return mFillCount;
	}
	inline int getLastCleanCount() const {
		return mCleanCount;
	}
	void resetMetrics(){
		mCleanCount=0;
		mFillCount=0;
	}
	void draw();
	int clean();
	int fill();
	void fillWithNearestNeighbors();
	void fillWithVelocityField(MACGrid<float>& grid,float radius);
	void evolve();
	void updateLines();
	void updateGradient();
	void updateIsoSurface();
	void updateUnSignedLevelSet(
			double distance = openvdb::LEVEL_SET_HALF_WIDTH);
	void updateSignedLevelSet();
	void computeStatistics(Mesh& mesh, FloatGrid& levelSet);
	void computeStatistics(Mesh& mesh);
	void relax(int iters = 10);
	double distanceToConstellation(const Vec3s& pt);
	void updateNearestNeighbors(bool threaded = true);
	void create(Mesh* mesh, openvdb::math::Transform::Ptr transform =
			openvdb::math::Transform::createLinearTransform());
	void create(FloatGrid& grid);
	void create(RegularGrid<float>& grid);
	SpringLevelSet() :
			mCleanCount(0),mFillCount(0),mVolToMesh(0.0), mTransform(
					openvdb::math::Transform::createLinearTransform(1.0)) {
	}

	~SpringLevelSet() {
	}
};

class SpringlRange {
public:
	class Iterator {
	public:
		Iterator(const SpringlRange& range, size_t pos) :
				mRange(range), mPos(pos) {
			assert(this->isValid());
		}
		/// Advance to the next leaf node.
		Iterator& operator++() {
			++mPos;
			return *this;
		}
		/// Return a reference to the leaf node to which this iterator is pointing.
		Springl& operator*() const {
			return mRange.mConstellation[mPos];
		}
		/// Return a pointer to the leaf node to which this iterator is pointing.
		Springl* operator->() const {
			return &(this->operator*());
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
		const SpringlRange& leafRange() const {
			return mRange;
		}

	protected:
		const SpringlRange& mRange;
		size_t mPos;
	}; // end Iterator

	SpringlRange(size_t begin, size_t end, Constellation& constellation,
			size_t grainSize = 1) :
			mEnd(end), mBegin(begin), mGrainSize(grainSize), mConstellation(
					constellation) {
	}

	SpringlRange(Constellation& constellation, size_t grainSize = 1) :
			mEnd(constellation.getNumSpringls()), mBegin(0), mGrainSize(
					grainSize), mConstellation(constellation) {
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

	const Constellation& getConstellation() const {
		return mConstellation;
	}

	bool empty() const {
		return !(mBegin < mEnd);
	}

	bool is_divisible() const {
		return mGrainSize < this->size();
	}

	SpringlRange(SpringlRange& r, tbb::split) :
			mEnd(r.mEnd), mBegin(doSplit(r)), mGrainSize(r.mGrainSize), mConstellation(
					r.mConstellation) {
	}
private:
	Constellation& mConstellation;
	size_t mEnd, mBegin, mGrainSize;
	static size_t doSplit(SpringlRange& r) {
		assert(r.is_divisible());
		size_t middle = r.mBegin + (r.mEnd - r.mBegin) / 2u;
		r.mEnd = middle;
		return middle;
	}
};


}

#endif
