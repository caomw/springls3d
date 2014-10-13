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
#ifndef SPRINGLEVELSETADVECTION_H_
#define SPRINGLEVELSETADVECTION_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetAdvect.h>
#include "SpringLevelSetOperations.h"
namespace imagesci {
template<typename FieldT,typename InterruptT = openvdb::util::NullInterrupter>
class SpringLevelSetAdvection {
private:
	SpringLevelSet& mGrid;
	const FieldT& mField;
public:
	typedef FloatGrid GridType;
	typedef LevelSetTracker<FloatGrid, InterruptT> TrackerT;
	typedef typename TrackerT::RangeType RangeType;
	typedef typename TrackerT::LeafType LeafType;
	typedef typename TrackerT::BufferType BufferType;
	typedef typename TrackerT::ValueType ScalarType;
	typedef typename FieldT::VectorType VectorType;

	imagesci::TemporalIntegrationScheme mTemporalScheme;
	InterruptT* mInterrupt;
	// disallow copy by assignment
	void operator=(const SpringLevelSetAdvection& other){}
	/// Main constructor
	SpringLevelSetAdvection(SpringLevelSet& grid, const FieldT& field,InterruptT* interrupt = NULL) :mGrid(grid), mField(field), mInterrupt(interrupt), mTemporalScheme(imagesci::TemporalIntegrationScheme::RK4b) {
	}
	/// @return the temporal integration scheme
	imagesci::TemporalIntegrationScheme getTemporalScheme() const {
		return mTemporalScheme;
	}
	/// @brief Set the spatial finite difference scheme
	void setTemporalScheme(imagesci::TemporalIntegrationScheme scheme) {
		mTemporalScheme = scheme;
	}
	size_t advect(double  startTime, double endTime) {
	    const math::Transform& trans = mGrid.mSignedLevelSet->transform();
		if (trans.mapType() == math::UniformScaleMap::mapType()) {
	        return advect1<math::UniformScaleMap>(startTime,endTime);
	    } else if (trans.mapType() == math::UniformScaleTranslateMap::mapType()) {
	        return advect1<math::UniformScaleTranslateMap>(startTime,endTime);
	    } else if (trans.mapType() == math::UnitaryMap::mapType()) {
	        return advect1<math::UnitaryMap>(startTime,endTime);
	    } else if (trans.mapType() == math::TranslationMap::mapType()) {
	        return advect1<math::TranslationMap>(startTime,endTime);
	    } else {
	        OPENVDB_THROW(ValueError, "MapType not supported!");
	        return 0;
	    }
	}
	template<typename MapT> void track(double time,bool clean){
		const int RELAX_OUTER_ITERS=1;
		const int RELAX_INNER_ITERS=5;
		mGrid.updateUnSignedLevelSet();
		for(int iter=0;iter<RELAX_OUTER_ITERS;iter++){
			mGrid.updateNearestNeighbors();
			mGrid.relax(RELAX_INNER_ITERS);
		}
		mGrid.updateUnSignedLevelSet(2.5*openvdb::LEVEL_SET_HALF_WIDTH);
		mGrid.updateGradient();
		TrackerT mTracker(*mGrid.mSignedLevelSet,mInterrupt);
		SpringLevelSetEvolve<MapT> evolve(*this,mTracker,time,0.75,32);
		evolve.process();
		if(clean){
			int cleaned=mGrid.clean();
			mGrid.updateUnSignedLevelSet();
			std::cout<<"Cleaned "<<cleaned<<" "<<100*cleaned/(double)mGrid.mConstellation.getNumSpringls()<<"%"<<std::endl;
		}
		mGrid.updateIsoSurface();
		int added=mGrid.fill();
		std::cout<<time<<"Filled "<<added<<" "<<100*added/(double)mGrid.mConstellation.getNumSpringls()<<"%"<<std::endl;
	}
	template<typename MapT> size_t advect1(double  mStartTime, double mEndTime) {
	    typedef AdvectSpringlParticleOperation<FieldT> OpT;
		double dt = 0.0;
		Vec3d vsz = mGrid.transformPtr()->voxelSize();
		double scale = std::max(std::max(vsz[0], vsz[1]), vsz[2]);
		const double EPS=1E-30f;
		double voxelDistance=0;
		const double MAX_TIME_STEP=SpringLevelSet::MAX_VEXT;
		for (double time = mStartTime; time < mEndTime; time += dt) {
			MaxVelocityOperator<OpT, FieldT, InterruptT> op2(mGrid,mField,time, mInterrupt);
			double maxV = std::max(EPS, std::sqrt(op2.process()));
			dt=clamp(MAX_TIME_STEP*scale/std::max(1E-30,maxV),0.0,mEndTime-time);
			if (dt < EPS)break;
			AdvectSpringlOperator<OpT, FieldT, InterruptT> op1(mGrid, mField,mTemporalScheme, time,dt,mInterrupt);
			op1.process();
		}
		track<MapT>(mEndTime,true);

		mGrid.mConstellation.updateVertexNormals();
		return 0;
	}

	template<typename MapT> class SpringLevelSetEvolve {
	public:
		SpringLevelSetAdvection& mParent;
		typename TrackerT::LeafManagerType& leafs;
		const MapT* mMap;
		double mDt;
		double mTime;
		int mIterations;
		TrackerT& mTracker;
		DiscreteField<openvdb::VectorGrid> mField;
		SpringLevelSetEvolve(SpringLevelSetAdvection& parent,TrackerT& tracker,double time, double dt,int iterations) :
				mMap(NULL),
				mParent(parent),
				mTracker(tracker),
				mIterations(iterations),
				mField(DiscreteField<openvdb::VectorGrid>(*(mParent.mGrid.mGradient))),
				mTime(time), mDt(dt),leafs(tracker.leafs()) {

		}
		void process(bool threaded=true) {
			mMap= (mTracker.grid().transform().template constMap<MapT>().get());
			if (mParent.mInterrupt)
				mParent.mInterrupt->start("Processing voxels");
			for(int iter=0;iter<mIterations;iter++){
				leafs.rebuildAuxBuffers(1);
				if (threaded) {
					tbb::parallel_for(leafs.getRange(mTracker.getGrainSize()), *this);
				} else {
					(*this)(leafs.getRange(mTracker.getGrainSize()));
				}
				leafs.swapLeafBuffer(1, mTracker.getGrainSize()==0);
				leafs.removeAuxBuffers();
				mTracker.track();
			}

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
			for (size_t n=range.begin(), e=range.end(); n != e; ++n) {
				BufferType& result = leafs.getBuffer(n, 1);
				for (VoxelIterT iter = leafs.leaf(n).cbeginValueOn(); iter;++iter) {
					stencil.moveTo(iter);
					const VectorType V = mField(map.applyMap(iter.getCoord().asVec3d()), mTime);
					const VectorType G = math::GradientBiased<MapT,BiasedGradientScheme::FIRST_BIAS>::result(map, stencil, V);
					result.setValue(iter.pos(), *iter - mDt * V.dot(G));
				}
			}
		}
	};

};
}
#endif
