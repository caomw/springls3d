/*
 * SpringLevelSetOperations.h
 *
 *  Created on: Oct 12, 2014
 *      Author: blake
 */

#ifndef SPRINGLEVELSETOPERATIONS_H_
#define SPRINGLEVELSETOPERATIONS_H_

#include "SpringLevelSetBase.h"
namespace imagesci {
template<typename FieldT> Vec3d ComputeVelocity(const FieldT& field,
		imagesci::TemporalIntegrationScheme scheme, Vec3d pt, double t,
		double h);
template<typename OperatorT,
		typename InterruptT = openvdb::util::NullInterrupter>
class ComputePertubationOperator {
public:
	SpringLevelSet& mGrid;
	ComputePertubationOperator(SpringLevelSet& grid, InterruptT* _interrupt,
			double t = 0.0, double dt = 0.0,
			imagesci::TemporalIntegrationScheme scheme =
					imagesci::TemporalIntegrationScheme::RK1) :
			mGrid(grid), mInterrupt(_interrupt), mIntegrationScheme(scheme), mTime(
					t), mTimeStep(dt) {
	}
	virtual ~ComputePertubationOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		SpringlRange range(mGrid.mConstellation);
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
	double mTimeStep;
	imagesci::TemporalIntegrationScheme mIntegrationScheme;
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

template<typename FieldT> class AdvectSpringlParticleOperation {
private:
	imagesci::TemporalIntegrationScheme mIntegrationScheme;
public:
	AdvectSpringlParticleOperation(
			imagesci::TemporalIntegrationScheme integrationScheme =
					imagesci::TemporalIntegrationScheme::UNKNOWN_TIS) :
			mIntegrationScheme(integrationScheme) {
	}
	void compute(Springl& springl, SpringLevelSet& mGrid, const FieldT& field,
			double t, double h) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3d v = Vec3d(springl.particle());
		Vec3d pt = trans->indexToWorld(v);
		Vec3d vel = ComputeVelocity(field, mIntegrationScheme, pt, t, h);
		springl.particle() = trans->worldToIndex(pt + vel);	//Apply integration scheme here, need buffer for previous time points?
		int K = springl.size();
		for (int k = 0; k < K; k++) {
			pt = trans->indexToWorld(springl[k]);
			vel = ComputeVelocity(field, mIntegrationScheme, pt, t, h);
			springl[k] = trans->worldToIndex(pt + vel);
		}
	}
	double findTimeStep(Springl& springl, SpringLevelSet& mGrid,
			const FieldT& field, double t) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3d v = Vec3d(springl.particle());
		Vec3d pt = trans->indexToWorld(v);
		Vec3d vec = field(pt, t);
		return std::max(std::max(fabs(vec[0]), fabs(vec[1])), fabs(vec[2]));
	}

};
template<typename FieldT> class AdvectMeshVertexOperation {
private:
	imagesci::TemporalIntegrationScheme mIntegrationScheme;
public:
	AdvectMeshVertexOperation(
			imagesci::TemporalIntegrationScheme integrationScheme =
					imagesci::TemporalIntegrationScheme::UNKNOWN_TIS) :
			mIntegrationScheme(integrationScheme) {
	}
	double findTimeStep(size_t vid, SpringLevelSet& mGrid, Mesh& mMesh,
			const FieldT& field, double t) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3s vert = mGrid.mIsoSurface.mVertexes[vid];
		Vec3d v = Vec3d(vert);
		Vec3d pt = trans->indexToWorld(v);
		Vec3f vec = field(pt, t);
		return std::max(std::max(fabs(vec[0]), fabs(vec[1])), fabs(vec[2]));
	}
	void compute(size_t vid, SpringLevelSet& mGrid, const FieldT& field,
			double t, double dt) {
		openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
		Vec3s vert = mGrid.mIsoSurface.mVertexes[vid];
		Vec3d v = Vec3d(vert);
		Vec3d pt = trans->indexToWorld(v);
		Vec3d vel = ComputeVelocity(field, mIntegrationScheme, pt, t, dt);
		mGrid.mIsoSurface.mVertexes[vid] = trans->worldToIndex(pt + vel);
	}
};
template<typename OperatorT,
		typename InterruptT = openvdb::util::NullInterrupter>
class PerturbSpringlOperator {
public:
	SpringLevelSet& mGrid;
	double mDt;
	PerturbSpringlOperator(SpringLevelSet& grid, InterruptT* _interrupt,
			double dt) :
			mGrid(grid), mInterrupt(_interrupt), mDt(dt) {

	}
	virtual ~PerturbSpringlOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		SpringlRange range(mGrid.mConstellation);
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
class PerturbMeshVertexOperator {
public:
	SpringLevelSet& mGrid;
	double mDt;
	PerturbMeshVertexOperator(SpringLevelSet& grid, InterruptT* _interrupt,
			double dt) :
			mGrid(grid), mInterrupt(_interrupt), mDt(dt) {

	}
	virtual ~PerturbMeshVertexOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		OperatorT::init(mGrid);
		MeshVertexRange range(mGrid.mIsoSurface);
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
	void operator()(const MeshVertexRange& range) const {
		if (openvdb::util::wasInterrupted(mInterrupt))
			tbb::task::self().cancel_group_execution();
		for (typename MeshVertexRange::Iterator vert = range.begin(); vert;
				++vert) {
			OperatorT::apply(*vert, mGrid, mDt);
		}
	}

protected:
	InterruptT* mInterrupt;

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
		ComputePertubationOperator<OpT, InterruptT> op(mGrid, mInterrupt);
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
		ComputePertubationOperator<OpT, InterruptT> op1(mGrid, mInterrupt);
		op1.process(threaded);
		PerturbSpringlOperator<OpT, InterruptT> op2(mGrid, mInterrupt, 1.0f);
		op2.process(threaded);
	}
	SpringLevelSet& mGrid;
	InterruptT* mInterrupt;
};
template<typename OperatorT, typename FieldT,
		typename InterruptT = openvdb::util::NullInterrupter>
class MaxVelocityOperator {
public:
	double mMaxAbsV;
	SpringLevelSet& mGrid;
	bool mIsMaster;
	MaxVelocityOperator(SpringLevelSet& grid, const FieldT& field, double t,
			InterruptT* _interrupt) :
			mIsMaster(true), mGrid(grid), mField(field), mTime(t), mInterrupt(
					_interrupt), mMaxAbsV(std::numeric_limits<double>::min()) {

	}
	MaxVelocityOperator(MaxVelocityOperator& other, tbb::split) :
			mGrid(other.mGrid), mMaxAbsV(other.mMaxAbsV), mField(other.mField), mTime(
					other.mTime), mIsMaster(false), mInterrupt(
			NULL) {
	}
	virtual ~MaxVelocityOperator() {
	}
	double process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		SpringlRange range(mGrid.mConstellation);
		if (threaded) {
			tbb::parallel_reduce(range, *this);
		} else {
			(*this)(range);
		}
		if (mInterrupt)
			mInterrupt->end();
		return mMaxAbsV;
	}
	void join(const MaxVelocityOperator& other) {
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
					OpT.findTimeStep(*springl, mGrid, mField, mTime));
		}
	}

protected:
	double mTime;
	InterruptT* mInterrupt;
	const FieldT& mField;
};
template<typename OperatorT, typename FieldT,
		typename InterruptT = openvdb::util::NullInterrupter>
class AdvectSpringlOperator {
public:
	SpringLevelSet& mGrid;
	AdvectSpringlOperator(SpringLevelSet& grid, const FieldT& field,
			imagesci::TemporalIntegrationScheme scheme, double t, double dt,
			InterruptT* _interrupt) :
			mGrid(grid), mField(field), mIntegrationScheme(scheme), mInterrupt(
					_interrupt), mTime(t), mTimeStep(dt) {

	}
	virtual ~AdvectSpringlOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		SpringlRange range(mGrid.mConstellation);
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
			OpT.compute(*springl, mGrid, mField, mTime, mTimeStep);
		}
	}

protected:
	double mTime;
	double mTimeStep;
	imagesci::TemporalIntegrationScheme mIntegrationScheme;
	const FieldT& mField;
	InterruptT* mInterrupt;

};
template<typename OperatorT, typename FieldT,
		typename InterruptT = openvdb::util::NullInterrupter>
class AdvectMeshVertexOperator {
public:
	SpringLevelSet& mGrid;
	AdvectMeshVertexOperator(SpringLevelSet& grid, const FieldT& field,
			imagesci::TemporalIntegrationScheme scheme, double t, double dt,
			InterruptT* _interrupt) :
			mGrid(grid), mField(field), mInterrupt(_interrupt), mIntegrationScheme(
					scheme), mTime(t), mTimeStep(dt) {

	}
	virtual ~AdvectMeshVertexOperator() {
	}
	void process(bool threaded = true) {
		if (mInterrupt)
			mInterrupt->start("Processing springls");
		MeshVertexRange range(mGrid.mIsoSurface);
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
	void operator()(const MeshVertexRange& range) const {
		if (openvdb::util::wasInterrupted(mInterrupt))
			tbb::task::self().cancel_group_execution();
		OperatorT OpT(mIntegrationScheme);
		for (typename MeshVertexRange::Iterator vert = range.begin(); vert;
				++vert) {
			OpT.compute(*vert, mGrid, mField, mTime, mTimeStep);
		}
	}

protected:
	double mTime;
	double mTimeStep;
	imagesci::TemporalIntegrationScheme mIntegrationScheme;

	const FieldT& mField;
	InterruptT* mInterrupt;

};
}

#endif /* SPRINGLEVELSETOPERATIONS_H_ */
