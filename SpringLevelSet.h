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
	RK1 = openvdb::math::TemporalIntegrationScheme::TVD_RK1,
	RK2 = openvdb::math::TemporalIntegrationScheme::TVD_RK2,
	RK3 = openvdb::math::TemporalIntegrationScheme::TVD_RK3,
	RK4a,
	RK4b
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
protected:
	openvdb::tools::VolumeToMesh mesher;
	openvdb::math::Transform::Ptr mTransform;
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

	openvdb::FloatGrid::Ptr signedLevelSet;
	openvdb::FloatGrid::Ptr unsignedLevelSet;
	openvdb::VectorGrid::Ptr gradient;
	openvdb::Int32Grid::Ptr springlIndexGrid;
	Mesh isoSurface;
	Constellation constellation;
	NearestNeighborMap nearestNeighbors;

	openvdb::math::Transform& transform(){
		return *mTransform;
	}
	openvdb::math::Transform::Ptr transformPtr(){
		return mTransform;
	}
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
	void updateLines();
	void updateGradient();
	void updateIsoSurface();
	void updateUnsignedLevelSet(double distance=openvdb::LEVEL_SET_HALF_WIDTH);
	void updateSignedLevelSet();
	void computeStatistics(Mesh& mesh,FloatGrid& levelSet);
	void computeStatistics(Mesh& mesh);
	void relax(int iters = 10);
	double distanceToConstellation(const Vec3s& pt);
	void updateNearestNeighbors(bool threaded = true);
	void create(Mesh* mesh, openvdb::math::Transform::Ptr transform =
			openvdb::math::Transform::createLinearTransform());
	void create(FloatGrid& grid);
	SpringLevelSet():mesher(0.0) {
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
template<typename FieldT> Vec3d ComputeVelocity(const FieldT& field, SpringlTemporalIntegrationScheme scheme,Vec3d pt,double t,double h){
	Vec3d velocity(0.0);
	Vec3d k1,k2,k3,k4;
	switch(scheme){
		case SpringlTemporalIntegrationScheme::RK1:
			velocity=h*field(pt,t);
			break;
		case SpringlTemporalIntegrationScheme::RK3:
			k1=h*field(pt,t);
			k2=h*field(pt+0.5*k1,t+0.5f*h);
			k3=h*field(pt-1.0*k1+2.0*k2,t+h);
			velocity=(1.0f/6.0f)*(k1+4*k2+k3);
			break;
		case SpringlTemporalIntegrationScheme::RK4a:
			k1=h*field(pt,t);
			k2=h*field(pt+0.5f*k1,t+0.5f*h);
			k3=h*field(pt+0.5f*k2,t+0.5f*h);
			k4=h*field(pt+k3,t+h);
			velocity=(1.0f/6.0f)*(k1+2*k2+2*k3+k4);
			break;
		case SpringlTemporalIntegrationScheme::RK4b:
		default:
			k1=h*field(pt,t);
			k2=h*field(pt+(1/3.0)*k1,t+(1/3.0)*h);
			k3=h*field(pt-(1/3.0)*k1+k2,t+(2/3.0)*h);
			k4=h*field(pt+k1-k2+k3,t+h);
			velocity=(1.0f/8.0f)*(k1+3*k2+3*k3+k4);
			break;

	}
	return velocity;
}

// end of SpringlRange

template<typename OperatorT,
		typename InterruptT = openvdb::util::NullInterrupter>
class ComputeOperator {
public:
	SpringLevelSet& mGrid;
	ComputeOperator(SpringLevelSet& grid, InterruptT* _interrupt, double t=0.0,double dt=0.0,SpringlTemporalIntegrationScheme scheme=SpringlTemporalIntegrationScheme::RK1) :
			mGrid(grid), mInterrupt(_interrupt),mIntegrationScheme(scheme), mTime(t),mTimeStep(dt) {

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
		//if (openvdb::util::wasInterrupted(mInterupt))tbb::task::self().cancel_group_execution();
		//OperatorT OpT(mIntegrationScheme);
		for (typename SpringlRange::Iterator springl = range.begin(); springl;
				++springl) {
			OperatorT::compute(*springl, mGrid, mTime);
		}
	}

protected:
	double mTime;
	double mTimeStep;
	SpringlTemporalIntegrationScheme mIntegrationScheme;
	InterruptT* mInterrupt;

};
template<typename OperatorT, typename FieldT,
		typename InterruptT = openvdb::util::NullInterrupter>
class AdvectMeshOperator {
public:
	SpringLevelSet& mGrid;
	AdvectMeshOperator(SpringLevelSet& grid, const FieldT& field,	SpringlTemporalIntegrationScheme scheme, double t,double dt,InterruptT* _interrupt) :
				mGrid(grid), mField(field), mInterrupt(_interrupt),mIntegrationScheme(scheme), mTime(t),mTimeStep(dt) {

	}
	virtual ~AdvectMeshOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
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
		OperatorT OpT(mIntegrationScheme);
		for (typename MeshRange::Iterator vert = range.begin(); vert;
				++vert) {
			OpT.compute(*vert, mGrid, mField, mTime,mTimeStep);
		}
	}

protected:
	double mTime;
	double mTimeStep;
	SpringlTemporalIntegrationScheme mIntegrationScheme;

	const FieldT& mField;
	InterruptT* mInterrupt;

};
template<typename OperatorT, typename FieldT,
		typename InterruptT = openvdb::util::NullInterrupter>
class AdvectSpringlOperator {
public:
	SpringLevelSet& mGrid;
	AdvectSpringlOperator(SpringLevelSet& grid, const FieldT& field,SpringlTemporalIntegrationScheme scheme,double t,double dt,InterruptT* _interrupt) :
			mGrid(grid), mField(field),mIntegrationScheme(scheme), mInterrupt(_interrupt), mTime(t),mTimeStep(dt) {

	}
	virtual ~AdvectSpringlOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
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
		OperatorT OpT(mIntegrationScheme);
		for (typename SpringlRange::Iterator springl = range.begin(); springl;
				++springl) {
			OpT.compute(*springl, mGrid, mField, mTime,mTimeStep);
		}
	}

protected:
	double mTime;
	double mTimeStep;
	SpringlTemporalIntegrationScheme mIntegrationScheme;
	const FieldT& mField;
	InterruptT* mInterrupt;

};
template<typename OperatorT,typename FieldT,
		typename InterruptT = openvdb::util::NullInterrupter>
class MaxOperator {
public:
	double mMaxAbsV;
	SpringLevelSet& mGrid;
	bool mIsMaster;
	MaxOperator(SpringLevelSet& grid,const FieldT& field,double t, InterruptT* _interrupt) :
			mIsMaster(true), mGrid(grid),mField(field), mTime(t),mInterrupt(_interrupt), mMaxAbsV(
					std::numeric_limits<double>::min()) {

	}
	MaxOperator(MaxOperator& other, tbb::split) :
			mGrid(other.mGrid), mMaxAbsV(other.mMaxAbsV),mField(other.mField),mTime(other.mTime), mIsMaster(false), mInterrupt(
			NULL) {
	}
	virtual ~MaxOperator() {
	}
	double process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
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
		OperatorT OpT;
		for (typename SpringlRange::Iterator springl = range.begin(); springl;
				++springl) {
			mMaxAbsV = std::max(mMaxAbsV,
					OpT.findTimeStep(*springl, mGrid,mField,mTime));
		}
	}

protected:
	double mTime;
	InterruptT* mInterrupt;
	const FieldT& mField;
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
/*
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
	void compute(Springl& springl, SpringLevelSet& mGrid,
			const FieldT& field, double t,double h) {
		int K = springl.size();
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		for (int k = 0; k < K; k++) {
			Vec3d v = Vec3d(springl[k]);
			Vec3d pt = trans->indexToWorld(v);
			Vec3d vel = ComputeVelocity(field,mIntegrationScheme,pt,t,h);
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
*/
template<typename FieldT> class AdvectParticleOperation {
private:
	SpringlTemporalIntegrationScheme mIntegrationScheme;
public:
	AdvectParticleOperation(SpringlTemporalIntegrationScheme integrationScheme=SpringlTemporalIntegrationScheme::UNKNOWN_TIS) :
			mIntegrationScheme(integrationScheme) {
	}
	void compute(Springl& springl, SpringLevelSet& mGrid,
			const FieldT& field, double t,double h) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3d v = Vec3d(springl.particle());
		Vec3d pt = trans->indexToWorld(v);
		Vec3d vel = ComputeVelocity(field,mIntegrationScheme,pt,t,h);
		springl.particle()=trans->worldToIndex(pt+vel);//Apply integration scheme here, need buffer for previous time points?
		int K=springl.size();
		for(int k=0;k<K;k++){
			pt = trans->indexToWorld(springl[k]);
			vel = ComputeVelocity(field,mIntegrationScheme,pt,t,h);
			springl[k]=trans->worldToIndex(pt+vel);
		}
	}
	double findTimeStep(Springl& springl, SpringLevelSet& mGrid,const FieldT& field, double t) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3d v = Vec3d(springl.particle());
		Vec3d pt = trans->indexToWorld(v);
		Vec3d vec=field(pt,t);
		return std::max(std::max(fabs(vec[0]), fabs(vec[1])), fabs(vec[2]));
	}

};
template<typename FieldT> class AdvectMeshOperation {
private:
	SpringlTemporalIntegrationScheme mIntegrationScheme;
public:
	AdvectMeshOperation(SpringlTemporalIntegrationScheme integrationScheme=SpringlTemporalIntegrationScheme::UNKNOWN_TIS) :
			mIntegrationScheme(integrationScheme) {
	}
	double findTimeStep(size_t vid, SpringLevelSet& mGrid, Mesh& mMesh,const FieldT& field, double t) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3s vert=mGrid.isoSurface.vertexes[vid];
		Vec3d v = Vec3d(vert);
		Vec3d pt = trans->indexToWorld(v);
		Vec3f vec=field(pt,t);
		return std::max(std::max(fabs(vec[0]), fabs(vec[1])), fabs(vec[2]));
	}
	void compute(size_t vid, SpringLevelSet& mGrid,const FieldT& field,double t, double dt) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3s vert=mGrid.isoSurface.vertexes[vid];
		Vec3d v = Vec3d(vert);
		Vec3d pt = trans->indexToWorld(v);
		Vec3d vel = ComputeVelocity(field,mIntegrationScheme,pt,t,dt);
		mGrid.isoSurface.vertexes[vid]=trans->worldToIndex(pt+vel);
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
		ComputeOperator<OpT, InterruptT> op(mGrid, mInterrupt);
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
		ComputeOperator<OpT, InterruptT> op1(mGrid, mInterrupt);
		op1.process(threaded);
		ApplySpringlOperator<OpT, InterruptT> op2(mGrid, mInterrupt, 1.0f);
		op2.process(threaded);
	}
	SpringLevelSet& mGrid;
	InterruptT* mInterrupt;
};
}

#endif /* CONSTELLATION_H_ */
