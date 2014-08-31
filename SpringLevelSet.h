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
class Constellation;
struct Springl;
struct SpringlNeighbor {
public:
	openvdb::Index32 springlId;
	int edgeId;
	float distance;
	SpringlNeighbor(openvdb::Index32 id = 0, int nbr = -1, float _distance = 0) :
			springlId(id), edgeId(nbr), distance(_distance) {
	}
};

std::ostream& operator<<(std::ostream& ostr, const SpringlNeighbor& classname);
typedef std::vector<std::list<SpringlNeighbor>> NearestNeighborMap;
class SpringLevelSet {
public:
	static const float NEAREST_NEIGHBOR_RANGE; //voxel units
	static const int MAX_NEAREST_NEIGHBORS;
	static const float PARTICLE_RADIUS;
	static const float MAX_VEXT;
	static const float FILL_DISTANCE;
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
	boost::shared_ptr<Mesh> isoSurface;
	boost::shared_ptr<Constellation> constellation;
	NearestNeighborMap nearestNeighbors;
	std::vector<openvdb::Vec3s> vertexDisplacement;
	std::vector<openvdb::Vec3s> particleDisplacement;
	Springl& GetSpringl(const openvdb::Index32 id);
	openvdb::Vec3s& GetParticle(const openvdb::Index32 id);
	openvdb::Vec3s& GetParticleNormal(const openvdb::Index32 id);
	openvdb::Vec3s& GetSpringlVertex(const openvdb::Index32 id, const int i);
	openvdb::Vec3s& GetSpringlVertex(const openvdb::Index32 gid);
	std::list<SpringlNeighbor>& GetNearestNeighbors(openvdb::Index32 id,
			int8_t e);
	void draw(bool colorEnabled = false, bool wireframe = true, bool particles =
			false, bool particleNormals = false);
	void clean();
	void fill(bool updateIsoSurface = false);
	void evolve();
	inline openvdb::math::Transform::Ptr transformPtr() {
		return transform;
	}
	void updateGradient();
	void updateIsoSurface();
	void updateUnsignedLevelSet();
	void relax(int iters = 10);
	void updateNearestNeighbors(bool threaded = true);
	void create(Mesh* mesh, openvdb::math::Transform::Ptr transform =
			openvdb::math::Transform::createLinearTransform());
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
class Constellation: public Mesh {
public:

	std::vector<Springl> springls;
	Constellation(Mesh* mesh);
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
		SpringlRange range(*mGrid.constellation);
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
class AdvectOperator {
public:
	SpringLevelSet& mGrid;
	AdvectOperator(SpringLevelSet& grid, const FieldT& field,
			InterruptT* _interrupt, double t) :
			mGrid(grid), mField(field), mInterrupt(_interrupt), mTime(t) {

	}
	virtual ~AdvectOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		SpringlRange range(*mGrid.constellation);
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
		SpringlRange range(*mGrid.constellation);
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
class ApplyOperator {
public:
	SpringLevelSet& mGrid;
	double mDt;
	ApplyOperator(SpringLevelSet& grid, InterruptT* _interrupt, double dt) :
			mGrid(grid), mInterrupt(_interrupt), mDt(dt) {

	}
	virtual ~ApplyOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		SpringlRange range(*mGrid.constellation);
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
		mGrid.vertexDisplacement.resize(mGrid.constellation->vertexes.size());
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
			mGrid.vertexDisplacement[springl.offset + k] = vel;	//Apply integration scheme here, need buffer for previous time points?
		}
	}
	static double findTimeStep(Springl& springl, SpringLevelSet& mGrid) {
		return mGrid.vertexDisplacement[springl.id].lengthSqr();
		//(SpringLevelSet::MAX_VEXT/maxV);//What should this be? depends on temporal integration scheme
	}
	static void apply(Springl& springl, SpringLevelSet& mGrid, double dt) {
		int K = springl.size();
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		for (int k = 0; k < K; k++) {
			Vec3d pt = springl[k];
			springl[k] = pt + dt * mGrid.vertexDisplacement[springl.offset + k];//Apply integration scheme here, need buffer for previous time points?
		}
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
		mGrid.vertexDisplacement.resize(mGrid.constellation->vertexes.size());
	}
	static void compute(Springl& springl, SpringLevelSet& mGrid,
			const FieldT& field, double t) {
		int K = springl.size();
		for (int k = 0; k < K; k++) {
			Vec3d pt(springl[k]);
			mGrid.vertexDisplacement[springl.offset + k] = field(pt, t);//Apply integration scheme here, need buffer for previous time points?
		}
	}
	static void apply(Springl& springl, SpringLevelSet& mGrid, double dt) {
		int K = springl.size();
		for (int k = 0; k < K; k++) {
			springl[k] += dt * mGrid.particleDisplacement[k];//Apply integration scheme here, need buffer for previous time points?
		}
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
		ApplyOperator<OpT, InterruptT> op2(mGrid, mInterrupt, 1.0f);
		op2.process(threaded);
	}
	SpringLevelSet& mGrid;
	InterruptT* mInterrupt;
};
template<typename FieldT, typename InterruptT = openvdb::util::NullInterrupter>
class AdvectVertex {
private:
	double mStartTime, mEndTime;
public:
	AdvectVertex(SpringLevelSet& grid, const FieldT& field, double t0,
			double t1, InterruptT* interrupt = NULL) :
			mGrid(grid), mField(field), mStartTime(t0), mEndTime(t1), mInterrupt(
					interrupt) {
	}
	const double EPS = 1E-30f;
	void process(bool threaded = true) {
		typedef AdvectVertexOperation<FieldT> OpT;
		double dt = 0.0;
		//Assume isotropic voxels!
		Vec3d vsz = mGrid.transformPtr()->voxelSize();
		double scale = std::max(std::max(vsz[0], vsz[1]), vsz[2]);
		double elapsedVoxelDistance = 0.0f;
		for (double time = mStartTime; time < mEndTime; time += dt * scale) {
			AdvectOperator<OpT, FieldT, InterruptT> op1(mGrid, mField,
					mInterrupt, time);
			op1.process(threaded);
			MaxOperator<OpT, InterruptT> op2(mGrid, mInterrupt);
			double maxV = op2.process(threaded);
			dt = std::max(0.0,
					std::min(SpringLevelSet::MAX_VEXT / std::sqrt(maxV),
							(mEndTime - time) / scale));
			elapsedVoxelDistance += SpringLevelSet::MAX_VEXT;
			if (dt < EPS)
				break;
			ApplyOperator<OpT, InterruptT> op3(mGrid, mInterrupt, dt);
			op3.process(threaded);
			//if (elapsedVoxelDistance >= LEVEL_SET_HALF_WIDTH) {
				//std::cout << "Update " << time <<" "<<elapsedVoxelDistance<< std::endl;
			mGrid.updateUnsignedLevelSet();
			mGrid.evolve();
				//elapsedVoxelDistance = 0.0f;
			//}
		}
	}
	SpringLevelSet& mGrid;
	const FieldT& mField;
	InterruptT* mInterrupt;
};
template<typename FieldT, typename InterruptT = openvdb::util::NullInterrupter>
class AdvectParticle {
public:
	AdvectParticle(SpringLevelSet& grid, const FieldT& field,
			InterruptT* interrupt = NULL) :
			mGrid(grid), mField(field), mInterrupt(interrupt) {
	}
	void process(bool threaded = true) {
		typedef AdvectParticleOperation<FieldT> OpT;
		AdvectOperator<OpT, FieldT, InterruptT> op1(mGrid, mField, mInterrupt);
		op1.process(threaded);
		MaxOperator<OpT, InterruptT> op2(mGrid, mInterrupt);
		double maxV = op2.process(threaded);
		double dt = SpringLevelSet::MAX_VEXT / std::sqrt(maxV);
		ApplyOperator<OpT, InterruptT> op3(mGrid, mInterrupt, dt);
		op3.process(threaded);
	}
	SpringLevelSet& mGrid;
	const FieldT& mField;
	InterruptT* mInterrupt;
};
}

#endif /* CONSTELLATION_H_ */
