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
#ifndef SPRINGLEVELSETPARTICLEADVECTION_H_
#define SPRINGLEVELSETPARTICLEADVECTION_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetAdvect.h>
#include "SpringLevelSetOperations.h"
namespace imagesci {
template<typename InterruptT = openvdb::util::NullInterrupter>
class SpringLevelSetParticleDeformation {
private:
	SpringLevelSet& mGrid;
	bool mResample;
	int mSignChanges;
	float mConvergenceThresold=0.01;
	std::mutex mSignChangeLock;
public:
	typedef FloatGrid GridType;
	typedef LevelSetTracker<FloatGrid, InterruptT> TrackerT;
	typedef typename TrackerT::RangeType RangeType;
	typedef typename TrackerT::LeafType LeafType;
	typedef typename TrackerT::BufferType BufferType;
	typedef typename TrackerT::ValueType ScalarType;
	void setConvergenceThreshold(float convg){
		mConvergenceThresold=convg;
	}
	imagesci::TemporalIntegrationScheme mTemporalScheme;
	imagesci::MotionScheme mMotionScheme;
	InterruptT* mInterrupt;
	// disallow copy by assignment
	void operator=(const SpringLevelSetParticleDeformation& other) {
	}
	SpringLevelSetParticleDeformation(SpringLevelSet& grid,
			imagesci::MotionScheme scheme =
					imagesci::MotionScheme::SEMI_IMPLICIT,
			InterruptT* interrupt = NULL) :
			mSignChanges(0), mMotionScheme(scheme), mGrid(grid), mInterrupt(interrupt), mTemporalScheme(
					imagesci::TemporalIntegrationScheme::RK4b), mResample(true) {
		mGrid.mConstellation.mParticleVelocity.resize(mGrid.mConstellation.mParticles.size(),Vec3s(0.0));
	}
	/// @return the temporal integration scheme
	imagesci::TemporalIntegrationScheme getTemporalScheme() const {
		return mTemporalScheme;
	}
	/// @brief Set the spatial finite difference scheme
	void setTemporalScheme(imagesci::TemporalIntegrationScheme scheme) {
		mTemporalScheme = scheme;
	}
	/// @brief Set enable resampling
	void setResampleEnabled(bool resample) {
		mResample = resample;
	}
	void advect(double startTime, double endTime) {
			const math::Transform& trans = mGrid.mSignedLevelSet->transform();
			if (trans.mapType() == math::UniformScaleMap::mapType()) {
				advect1<math::UniformScaleMap>(startTime, endTime);
			} else if (trans.mapType()== math::UniformScaleTranslateMap::mapType()) {
				advect1<math::UniformScaleTranslateMap>(startTime,endTime);
			} else if (trans.mapType() == math::UnitaryMap::mapType()) {
				advect1<math::UnitaryMap>(startTime, endTime);
			} else if (trans.mapType() == math::TranslationMap::mapType()) {
				advect1<math::TranslationMap>(startTime, endTime);
			}
	}
	template<typename MapT> void track(double time) {
		const int RELAX_OUTER_ITERS = 1;
		const int RELAX_INNER_ITERS = 5;
		mGrid.updateUnSignedLevelSet();
		for (int iter = 0; iter < RELAX_OUTER_ITERS; iter++) {
			mGrid.updateNearestNeighbors();
			mGrid.relax(RELAX_INNER_ITERS);
		}
		if (mMotionScheme == MotionScheme::SEMI_IMPLICIT) {
			mGrid.updateUnSignedLevelSet(2.5 * openvdb::LEVEL_SET_HALF_WIDTH);
			mGrid.updateGradient();
			TrackerT mTracker(*mGrid.mSignedLevelSet, mInterrupt);
			SpringLevelSetEvolve<MapT> evolve(*this, mTracker, time, 0.75, 32,mConvergenceThresold);
			evolve.process();
		} else if (mMotionScheme == MotionScheme::EXPLICIT) {
			mGrid.mIsoSurface.updateVertexNormals(0);
			mGrid.mIsoSurface.dilate(0.5f);
			mGrid.updateSignedLevelSet();
			mGrid.updateUnSignedLevelSet(2.5 * openvdb::LEVEL_SET_HALF_WIDTH);
			mGrid.updateGradient();
			TrackerT mTracker(*mGrid.mSignedLevelSet, mInterrupt);
			SpringLevelSetEvolve<MapT> evolve(*this, mTracker, time, 0.75, 128,mConvergenceThresold);
			evolve.process();
		}
		if (mResample) {
			int cleaned = mGrid.clean();
			mGrid.updateUnSignedLevelSet();
			mGrid.updateIsoSurface();
			mGrid.fill();
		} else {
			mGrid.updateIsoSurface();
		}

		//std::cout<<"Filled "<<added<<" "<<100*added/(double)mGrid.mConstellation.getNumSpringls()<<"%"<<std::endl;
	}
	template<typename MapT> void advect1(double mStartTime, double mEndTime) {
		double dt = 0.0;
		Vec3d vsz = mGrid.transformPtr()->voxelSize();
		double scale = std::max(std::max(vsz[0], vsz[1]), vsz[2]);
		const double EPS = 1E-30f;
		double voxelDistance = 0;
		double time;
		const double MAX_TIME_STEP = SpringLevelSet::MAX_VEXT;
		mGrid.resetMetrics();
		for (time = mStartTime; time < mEndTime; time += dt) {
			MaxParticleVelocityOperator<InterruptT> op2(mGrid.mConstellation,mInterrupt);
			double err=std::sqrt(op2.process());
			double maxV = std::max(EPS, err);
			dt = clamp(MAX_TIME_STEP * scale / std::max(1E-30, maxV), 0.0,mEndTime - time);
			if (dt < EPS) {
				break;
			}
			int N=mGrid.mConstellation.springls.size();
			openvdb::math::Transform::Ptr trans = mGrid.transformPtr();
#pragma omp for
			for(int n=0;n<N;n++){
				Springl& springl=mGrid.mConstellation.springls[n];
				Vec3d v = Vec3d(springl.particle());
				Vec3d pt = trans->indexToWorld(v);
				Vec3s vel=dt*springl.velocity();
				springl.particle() = trans->worldToIndex(pt + vel);	//Apply integration scheme here, need buffer for previous time points?
				int K=springl.size();
				for (int k = 0; k < K; k++) {
					pt = trans->indexToWorld(springl[k]);
					springl[k] = trans->worldToIndex(pt + vel);
				}
			}
			//if (mMotionScheme == MotionScheme::SEMI_IMPLICIT)track<MapT>(time);
		}
		if (mMotionScheme == MotionScheme::EXPLICIT)track<MapT>(time);
		mGrid.mConstellation.updateVertexNormals(0,0);
	}

	template<typename MapT> class SpringLevelSetEvolve {
	public:
		SpringLevelSetParticleDeformation& mParent;
		typename TrackerT::LeafManagerType& mLeafs;
		TrackerT& mTracker;
		DiscreteField<openvdb::VectorGrid> mDiscreteField;
		const MapT* mMap;
		ScalarType mDt;
		double mTime;
		double mTolerance;
		int mIterations;
		SpringLevelSetEvolve(SpringLevelSetParticleDeformation& parent, TrackerT& tracker,
				double time, double dt, int iterations, double tolerance) :
				mMap(NULL), mParent(parent), mTracker(tracker), mIterations(
						iterations), mDiscreteField(*parent.mGrid.mGradient), mTime(
						time), mDt(dt), mTolerance(tolerance), mLeafs(
						tracker.leafs()) {
			mParent.mSignChanges = 0;
		}
		void process(bool threaded = true) {
			mMap = (mTracker.grid().transform().template constMap<MapT>().get());
			if (mParent.mInterrupt)
			mParent.mInterrupt->start("Processing voxels");
			mParent.mSignChanges=0;
			const int MIN_NUM_SIGN_CHANGES=32;
			int maxSignChanges=MIN_NUM_SIGN_CHANGES;
			int iter;
			for(iter=0;iter<mIterations;iter++) {
				mLeafs.rebuildAuxBuffers(1);
				mParent.mSignChanges=0;
				if (threaded) {
					tbb::parallel_for(mLeafs.getRange(mTracker.getGrainSize()), *this);
				} else {
					(*this)(mLeafs.getRange(mTracker.getGrainSize()));
				}
				mLeafs.swapLeafBuffer(1, mTracker.getGrainSize()==0);
				mLeafs.removeAuxBuffers();
				mTracker.track();
				maxSignChanges=std::max(mParent.mSignChanges,maxSignChanges);
				float ratio=(mParent.mSignChanges/(float)maxSignChanges);
				if(ratio<mTolerance){
					break;
				}
			}
			std::cout<<"SEMI-IMPLICIT "<<iter<<std::endl;
			if (mParent.mInterrupt){
				mParent.mInterrupt->end();
			}
		}
		void operator()(const typename TrackerT::LeafManagerType::RangeType& range) const {
			using namespace openvdb;
			typedef math::BIAS_SCHEME<math::BiasedGradientScheme::FIRST_BIAS> Scheme;
			typedef typename Scheme::template ISStencil<FloatGrid>::StencilType Stencil;
			const math::Transform& trans = mTracker.grid().transform();
			typedef typename LeafType::ValueOnCIter VoxelIterT;
			const MapT& map = *mMap;
			Stencil stencil(mTracker.grid());
			int count=0;
			int signChanges=0;
			for (size_t n=range.begin(), e=range.end(); n != e; ++n) {
				BufferType& result = mLeafs.getBuffer(n, 1);
				for (VoxelIterT iter = mLeafs.leaf(n).cbeginValueOn(); iter;++iter) {
					stencil.moveTo(iter);
					const Vec3s V = mDiscreteField(map.applyMap(iter.getCoord().asVec3d()), mTime);
					const Vec3s G = math::GradientBiased<MapT,BiasedGradientScheme::FIRST_BIAS>::result(map, stencil, V);
					ScalarType delta=mDt * V.dot(G);
					ScalarType old=*iter;
					//Number of sign changes is a good indicator of the interface is moving.
					if(old*(old-delta)<0){
						signChanges++;
					}
					result.setValue(iter.pos(), old -  delta);
				}
			}
			mParent.mSignChangeLock.lock();
				mParent.mSignChanges+=signChanges;
			mParent.mSignChangeLock.unlock();
		}
	};

};
}
#endif
