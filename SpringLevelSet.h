/*
 * Constellation.h
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#ifndef SPRINGLEVELSET_H_
#define SPRINGLEVELSET_H_
#include <openvdb/util/NullInterrupter.h>

#include <tbb/parallel_for.h>
#include <vector>
#include <list>
#include <iostream>
#include "Mesh.h"
#include "ImageSciUtil.h"
#include "AdvectionForce.h"
#undef OPENVDB_REQUIRE_VERSION_NAME
namespace imagesci {
enum SpringlTemporalIntegrationScheme {
	UNKNOWN_TIS = openvdb::math::TemporalIntegrationScheme::UNKNOWN_TIS,
	TVD_RK1 = openvdb::math::TemporalIntegrationScheme::TVD_RK1,
	TVD_RK2 = openvdb::math::TemporalIntegrationScheme::TVD_RK2,
	TVD_RK3 = openvdb::math::TemporalIntegrationScheme::TVD_RK3,
	TVD_RK4,
	TVD_RK5,
	TVD_RK6,
};
struct Springl;
struct SpringlNeighbor {
public:
	openvdb::Index32 springlId;
	int edgeId;
	float distance;
	SpringlNeighbor(openvdb::Index32 id = 0, int nbr = -1, float _distance = 0) :
			springlId(id), edgeId(nbr), distance(_distance) {
	}

	friend bool operator<(const SpringlNeighbor& first,const SpringlNeighbor& second){
		return (first.distance<second.distance);
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
		return vertexes.size();
	}

	Springl& operator[](size_t idx) {
		return springls[idx];
	}
	const Springl& operator[](size_t idx) const {
		return springls[idx];
	}
	openvdb::Vec3s closestPointOnEdge(const openvdb::Vec3s& start,const SpringlNeighbor& ci);
};
typedef std::vector<std::list<SpringlNeighbor>> NearestNeighborMap;
class SpringLevelSet {
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
	static const float MAX_ANGLE_TOLERANCE;
	static const float MIN_ANGLE_TOLERANCE;
	static const float MIN_AREA;
	openvdb::math::Transform::Ptr transform;
	openvdb::FloatGrid::Ptr signedLevelSet;
	openvdb::FloatGrid::Ptr unsignedLevelSet;
	openvdb::VectorGrid::Ptr gradient;
	openvdb::Int32Grid::Ptr springlIndexGrid;
	Mesh isoSurface;
	Constellation constellation;
	NearestNeighborMap nearestNeighbors;

	Springl& GetSpringl(const openvdb::Index32 id);
	openvdb::Vec3s& GetParticle(const openvdb::Index32 id);
	openvdb::Vec3s& GetParticleNormal(const openvdb::Index32 id);
	openvdb::Vec3s& GetSpringlVertex(const openvdb::Index32 id, const int i);
	openvdb::Vec3s& GetSpringlVertex(const openvdb::Index32 gid);
	std::list<SpringlNeighbor>& GetNearestNeighbors(openvdb::Index32 id,
			int8_t e);
	void draw(bool colorEnabled = false, bool wireframe = true, bool particles =
			false, bool particleNormals = false);
	int clean();
	int fill();
	void evolve();
	inline openvdb::math::Transform::Ptr transformPtr() {
		return transform;
	}
	void updateLines();
	void updateGradient();
	void updateIsoSurface();
	void updateUnsignedLevelSet();
	void updateSignedLevelSet();

	void relax(int iters = 10);
	void updateNearestNeighbors(bool threaded = true);
	void create(Mesh* mesh, openvdb::math::Transform::Ptr transform =
			openvdb::math::Transform::createLinearTransform());
	void create(FloatGrid& grid);
	SpringLevelSet() {
	}
	~SpringLevelSet() {
	}
};

struct Springl {
private:
	Mesh* mesh;
public:
	openvdb::Index32 id;
	openvdb::Index32 offset;

	openvdb::Vec3s& normal() const {
		return mesh->particleNormals[id];
	}
	openvdb::Vec3s& particle() const {
		return mesh->particles[id];
	}
	openvdb::Vec3s& operator[](size_t idx) {
		return mesh->vertexes[offset + idx];
	}
	const openvdb::Vec3s& operator[](size_t idx) const {
		return mesh->vertexes[offset + idx];
	}
	Springl(Mesh* _mesh = NULL) :
			id(0), offset(0), mesh(_mesh) {
	}
	int size() const;
	float area() const;
	float distanceToFace(const openvdb::Vec3s& pt);
	float distanceToFaceSqr(const openvdb::Vec3s& pt);
	float distanceToParticle(const openvdb::Vec3s& pt);
	float distanceToParticleSqr(const openvdb::Vec3s& pt);
	float distanceEdgeSqr(const openvdb::Vec3s& pt, int e);
	float distanceEdge(const openvdb::Vec3s& pt, int e);
	openvdb::Vec3s computeCentroid() const;
	openvdb::Vec3s computeNormal(const float eps = 1E-6f) const;
	~Springl() {
	}
};
class MeshRange {
public:
	class Iterator {
	public:
		Iterator(const MeshRange& range, size_t pos) :
				mRange(range), mPos(pos) {
			assert(this->isValid());
		}
		/*
		 Iterator& operator=(const Iterator& other)
		 {
		 mRange = other.mRange;
		 mPos = other.mPos;
		 return *this;
		 }
		 */
		/// Advance to the next leaf node.
		Iterator& operator++() {
			++mPos;
			return *this;
		}
		/// Return a reference to the leaf node to which this iterator is pointing.
		size_t operator*() const {
			return mPos;
		}

		/*
		/// Return a pointer to the leaf node to which this iterator is pointing.
		size_t* operator->() const {
			return this->operator*());
		}
*/

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
		const MeshRange& leafRange() const {
			return mRange;
		}

	protected:
		const MeshRange& mRange;
		size_t mPos;
	}; // end Iterator

	MeshRange(size_t begin, size_t end, Mesh& mesh,
			size_t grainSize = 1) :
			mEnd(end), mBegin(begin), mGrainSize(grainSize), mMesh(
					mesh) {
	}

	MeshRange(Mesh& mesh, size_t grainSize = 1) :
			mEnd(mesh.vertexes.size()), mBegin(0), mGrainSize(
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

	MeshRange(MeshRange& r, tbb::split) :
			mEnd(r.mEnd), mBegin(doSplit(r)), mGrainSize(r.mGrainSize), mMesh(
					r.mMesh) {
	}
	Mesh& mMesh;
private:
	size_t mEnd, mBegin, mGrainSize;

	static size_t doSplit(MeshRange& r) {
		assert(r.is_divisible());
		size_t middle = r.mBegin + (r.mEnd - r.mBegin) / 2u;
		r.mEnd = middle;
		return middle;
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
		/*
		 Iterator& operator=(const Iterator& other)
		 {
		 mRange = other.mRange;
		 mPos = other.mPos;
		 return *this;
		 }
		 */
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
	Constellation& mConstellation;
private:
	size_t mEnd, mBegin, mGrainSize;

	static size_t doSplit(SpringlRange& r) {
		assert(r.is_divisible());
		size_t middle = r.mBegin + (r.mEnd - r.mBegin) / 2u;
		r.mEnd = middle;
		return middle;
	}
};
// end of SpringlRange

template<typename OperatorT,
		typename InterruptT = openvdb::util::NullInterrupter>
class ComputeOperator {
public:
	SpringLevelSet& mGrid;
	ComputeOperator(SpringLevelSet& grid, InterruptT* _interrupt, double t) :
			mGrid(grid), mInterrupt(_interrupt), mTime(t) {

	}
	virtual ~ComputeOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		SpringlRange range(mGrid.constellation);
		if (threaded) {
			tbb::parallel_for(range, *this);
		} else {
			(*this)(range);
		}

		if (mInterrupt)
			mInterrupt->end();
	}

	/// @note Never call this public method directly - it is called by
	/// TBB threads only!
	void operator()(const SpringlRange& range) const {
		if (openvdb::util::wasInterrupted(mInterrupt))
			tbb::task::self().cancel_group_execution();
		for (typename SpringlRange::Iterator springl = range.begin(); springl;
				++springl) {
			OperatorT::compute(*springl, mGrid, mTime);
		}
	}

protected:
	double mTime;
	InterruptT* mInterrupt;

};
template<typename OperatorT, typename FieldT,
		typename InterruptT = openvdb::util::NullInterrupter>
class AdvectMeshOperator {
public:
	SpringLevelSet& mGrid;
	AdvectMeshOperator(SpringLevelSet& grid, const FieldT& field,
			InterruptT* _interrupt, double t) :
				mGrid(grid), mField(field), mInterrupt(_interrupt), mTime(t) {

	}
	virtual ~AdvectMeshOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		MeshRange range(mGrid.isoSurface);
		if (threaded) {
			tbb::parallel_for(range, *this);
		} else {
			(*this)(range);
		}

		if (mInterrupt)
			mInterrupt->end();
	}

	/// @note Never call this public method directly - it is called by
	/// TBB threads only!
	void operator()(const MeshRange& range) const {
		if (openvdb::util::wasInterrupted(mInterrupt))
			tbb::task::self().cancel_group_execution();
		for (typename MeshRange::Iterator vert = range.begin(); vert;
				++vert) {
			OperatorT::compute(*vert, mGrid, mField, mTime);
		}
	}

protected:
	double mTime;
	const FieldT& mField;
	InterruptT* mInterrupt;

};
template<typename OperatorT, typename FieldT,
		typename InterruptT = openvdb::util::NullInterrupter>
class AdvectSpringlOperator {
public:
	SpringLevelSet& mGrid;
	AdvectSpringlOperator(SpringLevelSet& grid, const FieldT& field,
			InterruptT* _interrupt, double t) :
			mGrid(grid), mField(field), mInterrupt(_interrupt), mTime(t) {

	}
	virtual ~AdvectSpringlOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		SpringlRange range(mGrid.constellation);
		if (threaded) {
			tbb::parallel_for(range, *this);
		} else {
			(*this)(range);
		}

		if (mInterrupt)
			mInterrupt->end();
	}

	/// @note Never call this public method directly - it is called by
	/// TBB threads only!
	void operator()(const SpringlRange& range) const {
		if (openvdb::util::wasInterrupted(mInterrupt))
			tbb::task::self().cancel_group_execution();
		for (typename SpringlRange::Iterator springl = range.begin(); springl;
				++springl) {
			OperatorT::compute(*springl, mGrid, mField, mTime);
		}
	}

protected:
	double mTime;
	const FieldT& mField;
	InterruptT* mInterrupt;

};
template<typename OperatorT,
		typename InterruptT = openvdb::util::NullInterrupter>
class MaxOperator {
public:
	double mMaxAbsV;
	SpringLevelSet& mGrid;
	bool mIsMaster;
	MaxOperator(SpringLevelSet& grid, InterruptT* _interrupt) :
			mIsMaster(true), mGrid(grid), mInterrupt(_interrupt), mMaxAbsV(
					std::numeric_limits<double>::min()) {

	}
	MaxOperator(MaxOperator& other, tbb::split) :
			mGrid(other.mGrid), mMaxAbsV(other.mMaxAbsV), mIsMaster(false), mInterrupt(
			NULL) {
	}
	virtual ~MaxOperator() {
	}
	double process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		SpringlRange range(mGrid.constellation);
		if (threaded) {
			tbb::parallel_reduce(range, *this);
		} else {
			(*this)(range);
		}
		if (mInterrupt)
			mInterrupt->end();
		return mMaxAbsV;
	}
	void join(const MaxOperator& other) {
		mMaxAbsV = std::max(mMaxAbsV, other.mMaxAbsV);
	}

	/// @note Never call this public method directly - it is called by
	/// TBB threads only!
	void operator()(const SpringlRange& range) {
		if (openvdb::util::wasInterrupted(mInterrupt))
			tbb::task::self().cancel_group_execution();
		for (typename SpringlRange::Iterator springl = range.begin(); springl;
				++springl) {
			mMaxAbsV = std::max(mMaxAbsV,
					OperatorT::findTimeStep(*springl, mGrid));
		}
	}

protected:
	InterruptT* mInterrupt;

};
template<typename OperatorT,
		typename InterruptT = openvdb::util::NullInterrupter>
class ApplySpringlOperator {
public:
	SpringLevelSet& mGrid;
	double mDt;
	ApplySpringlOperator(SpringLevelSet& grid, InterruptT* _interrupt, double dt) :
			mGrid(grid), mInterrupt(_interrupt), mDt(dt) {

	}
	virtual ~ApplySpringlOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		SpringlRange range(mGrid.constellation);
		if (threaded) {
			tbb::parallel_for(range, *this);
		} else {
			(*this)(range);
		}

		if (mInterrupt)
			mInterrupt->end();
	}

	/// @note Never call this public method directly - it is called by
	/// TBB threads only!
	void operator()(const SpringlRange& range) const {
		if (openvdb::util::wasInterrupted(mInterrupt))
			tbb::task::self().cancel_group_execution();
		for (typename SpringlRange::Iterator springl = range.begin(); springl;
				++springl) {
			OperatorT::apply(*springl, mGrid, mDt);
		}
	}

protected:
	InterruptT* mInterrupt;

};
template<typename OperatorT,
		typename InterruptT = openvdb::util::NullInterrupter>
class ApplyMeshOperator {
public:
	SpringLevelSet& mGrid;
	double mDt;
	ApplyMeshOperator(SpringLevelSet& grid, InterruptT* _interrupt, double dt) :
			mGrid(grid), mInterrupt(_interrupt), mDt(dt) {

	}
	virtual ~ApplyMeshOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		MeshRange range(mGrid.isoSurface);
		if (threaded) {
			tbb::parallel_for(range, *this);
		} else {
			(*this)(range);
		}

		if (mInterrupt)
			mInterrupt->end();
	}

	/// @note Never call this public method directly - it is called by
	/// TBB threads only!
	void operator()(const MeshRange& range) const {
		if (openvdb::util::wasInterrupted(mInterrupt))
			tbb::task::self().cancel_group_execution();
		for (typename MeshRange::Iterator vert = range.begin(); vert;
				++vert) {
			OperatorT::apply(*vert, mGrid, mDt);
		}
	}

protected:
	InterruptT* mInterrupt;

};
class NearestNeighborOperation {
public:
	static void init(SpringLevelSet& mGrid);
	static void compute(Springl& springl, SpringLevelSet& mGrid, double t);
	static double findTimeStep(SpringLevelSet& mGrid) {
		return std::numeric_limits<double>::max();
	}
	static void apply(Springl& springl, SpringLevelSet& mGrid, double dt) {
	}
};
class RelaxOperation {
private:

public:
	static void init(SpringLevelSet& mGrid);
	static void compute(Springl& springl, SpringLevelSet& mGrid, double t);
	static void apply(Springl& springl, SpringLevelSet& mGrid, double dt);
	static double findTimeStep(SpringLevelSet& mGrid) {
		return 1.0f;
	}
};
template<typename FieldT> class AdvectVertexOperation {
private:
	SpringlTemporalIntegrationScheme mIntegrationScheme;
public:
	AdvectVertexOperation(SpringlTemporalIntegrationScheme integrationScheme) :
			mIntegrationScheme(integrationScheme) {

	}
	static void init(SpringLevelSet& mGrid) {
		mGrid.constellation.vertexDisplacement.resize(mGrid.constellation.vertexes.size());
	}
	static void compute(Springl& springl, SpringLevelSet& mGrid,
			const FieldT& field, double t) {
		int K = springl.size();
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		for (int k = 0; k < K; k++) {
			Vec3d v = Vec3d(springl[k]);
			Vec3d pt = trans->indexToWorld(v);
			Vec3d vel = field(pt, t);
			//std::cout<<"Evaluate "<<pt<<" "<<v<<" "<<vel<<t<<std::endl;
			mGrid.constellation.vertexDisplacement[springl.offset + k] = vel;	//Apply integration scheme here, need buffer for previous time points?
		}
	}
	static double findTimeStep(Springl& springl, SpringLevelSet& mGrid) {
		Vec3d vec=mGrid.constellation.vertexDisplacement[springl.id];
		return std::max(std::max(fabs(vec[0]), fabs(vec[1])), fabs(vec[2]));
		//(SpringLevelSet::MAX_VEXT/maxV);//What should this be? depends on temporal integration scheme
	}
	static void apply(Springl& springl, SpringLevelSet& mGrid, double dt) {
		int K = springl.size();
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3s newCenter(0.0f);
		for (int k = 0; k < K; k++) {
			 springl[k] += dt * mGrid.constellation.vertexDisplacement[springl.offset + k];//Apply integration scheme here, need buffer for previous time points?
			 newCenter+=springl[k];
		}
		newCenter/=(float)K;
		springl.particle()=newCenter;
	}
};
template<typename FieldT> class AdvectParticleOperation {
private:
	SpringlTemporalIntegrationScheme mIntegrationScheme;
public:
	AdvectParticleOperation(SpringlTemporalIntegrationScheme integrationScheme) :
			mIntegrationScheme(integrationScheme) {

	}
	static void init(SpringLevelSet& mGrid) {
		mGrid.constellation.particleDisplacement.resize(mGrid.constellation.particles.size());
	}
	static void compute(Springl& springl, SpringLevelSet& mGrid,
			const FieldT& field, double t) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3d v = Vec3d(springl.particle());
		Vec3d pt = trans->indexToWorld(v);
		Vec3d vel = field(pt, t);
		mGrid.constellation.particleDisplacement[springl.id] = vel;	//Apply integration scheme here, need buffer for previous time points?
	}
	static double findTimeStep(Springl& springl, SpringLevelSet& mGrid) {
		Vec3d vec=mGrid.constellation.particleDisplacement[springl.id];
		return std::max(std::max(fabs(vec[0]), fabs(vec[1])), fabs(vec[2]));
	}
	static void apply(Springl& springl, SpringLevelSet& mGrid, double dt) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3s vel=dt*mGrid.constellation.particleDisplacement[springl.id];
		springl.particle()+=vel;//Apply integration scheme here, need buffer for previous time points?
		int K=springl.size();
		for(int k=0;k<K;k++){
			springl[k]+=vel;
		}
	}
};
template<typename FieldT> class AdvectMeshOperation {
private:
	SpringlTemporalIntegrationScheme mIntegrationScheme;
public:
	AdvectMeshOperation(SpringlTemporalIntegrationScheme integrationScheme) :
			mIntegrationScheme(integrationScheme) {

	}
	static void init(SpringLevelSet& mGrid) {
		mGrid.isoSurface.vertexDisplacement.resize(mGrid.isoSurface.vertexes.size());
	}
	static void compute(size_t vid, SpringLevelSet& mGrid,
			const FieldT& field, double t) {
		Vec3s vert=mGrid.isoSurface.vertexes[vid];
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3d v = Vec3d(vert);
		Vec3d pt = trans->indexToWorld(v);
		Vec3d vel = field(pt, t);
		mGrid.isoSurface.vertexDisplacement[vid] = vel;	//Apply integration scheme here, need buffer for previous time points?
	}
	static double findTimeStep(size_t vid, Mesh& mMesh) {
		Vec3f vec=mMesh.vertexDisplacement[vid];
		return std::max(std::max(fabs(vec[0]), fabs(vec[1])), fabs(vec[2]));
	}
	static void apply(size_t vid, SpringLevelSet& mGrid, double dt) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3s vel=dt*mGrid.isoSurface.vertexDisplacement[vid];
		mGrid.isoSurface.vertexes[vid]+=vel;
	}
};
// end of ConstellationOperator class
/// @brief Compute the gradient of a scalar grid.
template<typename InterruptT = openvdb::util::NullInterrupter>
class NearestNeighbors {
public:
	NearestNeighbors(SpringLevelSet& grid, InterruptT* interrupt = NULL) :
			mGrid(grid), mInterrupt(interrupt) {
	}
	void process(bool threaded = true) {
		typedef NearestNeighborOperation OpT;
		ComputeOperator<OpT, InterruptT> op(mGrid, mInterrupt, 0.0);
		op.process(threaded);
	}
	SpringLevelSet& mGrid;
	InterruptT* mInterrupt;
};
// end of Gradient class

template<typename InterruptT = openvdb::util::NullInterrupter>
class Relax {
public:
	Relax(SpringLevelSet& grid, InterruptT* interrupt = NULL) :
			mGrid(grid), mInterrupt(interrupt) {
	}
	void process(bool threaded = true) {
		typedef RelaxOperation OpT;
		ComputeOperator<OpT, InterruptT> op1(mGrid, mInterrupt, 0.0);
		op1.process(threaded);
		ApplySpringlOperator<OpT, InterruptT> op2(mGrid, mInterrupt, 1.0f);
		op2.process(threaded);
	}
	SpringLevelSet& mGrid;
	InterruptT* mInterrupt;
};
}

#endif /* CONSTELLATION_H_ */
