#ifndef SPRINGLEVELSETADVECTION_H_
#define SPRINGLEVELSETADVECTION_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetAdvect.h>
namespace imagesci {
template<typename FieldT = openvdb::tools::EnrightField<float>,
		typename InterruptT = openvdb::util::NullInterrupter>
class SpringLevelSetAdvection {
public:
	typedef FloatGrid GridType;
	typedef LevelSetTracker<FloatGrid, InterruptT> TrackerT;
	typedef typename TrackerT::RangeType RangeType;
	typedef typename TrackerT::LeafType LeafType;
	typedef typename TrackerT::BufferType BufferType;
	typedef typename TrackerT::ValueType ScalarType;
	typedef typename FieldT::VectorType VectorType;
	SpringLevelSet& mGrid;
	const FieldT& mField;
	SpringlTemporalIntegrationScheme mTemporalScheme;
	InterruptT* mInterrupt;
	// disallow copy by assignment
	void operator=(const SpringLevelSetAdvection& other){}
	/// Main constructor
	SpringLevelSetAdvection(SpringLevelSet& grid, const FieldT& field,
			InterruptT* interrupt = NULL) :
			mGrid(grid), mField(field), mInterrupt(interrupt), mTemporalScheme(
					SpringlTemporalIntegrationScheme::RK4b) {
	}
	/// @return the temporal integration scheme
	SpringlTemporalIntegrationScheme getTemporalScheme() const {
		return mTemporalScheme;
	}
	/// @brief Set the spatial finite difference scheme
	void setTemporalScheme(SpringlTemporalIntegrationScheme scheme) {
		mTemporalScheme = scheme;
	}
	size_t advect(double  startTime, double endTime) {
	    const math::Transform& trans = mGrid.signedLevelSet->transform();
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
	template<typename MapT> size_t advect1(double  mStartTime, double mEndTime) {
//	    typedef AdvectVertexOperation<FieldT> OpT;
	    typedef AdvectParticleOperation<FieldT> OpT;
	    typedef AdvectMeshOperation<FieldT> OpS;

		double dt = 0.0;
		Vec3d vsz = mGrid.transformPtr()->voxelSize();
		double scale = std::max(std::max(vsz[0], vsz[1]), vsz[2]);
		const double EPS=1E-30f;
		double voxelDistance=0;
		/*
		for (double time = mStartTime; time < mEndTime; time += dt * scale) {

			AdvectSpringlOperator<OpT, FieldT, InterruptT> op1(mGrid, mField,
					mInterrupt, time);
			op1.process();
			AdvectMeshOperator<OpS, FieldT, InterruptT> opm1(mGrid, mField,
					mInterrupt, time);
			opm1.process();
			MaxOperator<OpT, InterruptT> op2(mGrid, mInterrupt);
			double maxV = std::max(EPS, std::sqrt(op2.process()));
			dt = std::max(0.0,
					std::min(SpringLevelSet::MAX_VEXT / maxV,
							(mEndTime - time) / scale));
			voxelDistance=voxelDistance+dt*maxV;
			std::cout << "Time " << time <<" voxel distance "<<voxelDistance<< std::endl;
			if (dt < EPS)break;
			ApplySpringlOperator<OpT, InterruptT> op3(mGrid, mInterrupt, dt);
			op3.process();

			ApplyMeshOperator<OpS, InterruptT> opm3(mGrid, mInterrupt, dt);
			opm3.process();

		}*/
		dt=mEndTime-mStartTime;
		std::cout<<"DT "<<dt<<std::endl;
		AdvectSpringlOperator<OpT, FieldT, InterruptT> op1(mGrid, mField,mInterrupt, mTemporalScheme,mStartTime,dt);
		op1.process();

		AdvectMeshOperator<OpS, FieldT, InterruptT> opm1(mGrid, mField,mInterrupt,  mTemporalScheme,mStartTime,dt);
		opm1.process();

		/*
		MaxOperator<OpT, InterruptT> op2(mGrid, mInterrupt);
		double maxV = std::max(EPS, std::sqrt(op2.process()));
		std::cout<<"Max Velocity "<<maxV<<std::endl;

		dt = std::max(0.0,
				std::min(SpringLevelSet::MAX_VEXT / maxV,
						(mEndTime - time) / scale));
		voxelDistance=voxelDistance+dt*maxV;
		*/
		//std::cout << "Time " << time <<" voxel distance "<<voxelDistance<< std::endl;
		//if (dt < EPS)break;

		ApplySpringlOperator<OpT, InterruptT> op3(mGrid, mInterrupt, 1.0);
		op3.process();

		ApplyMeshOperator<OpS, InterruptT> opm3(mGrid, mInterrupt, 1.0);
		opm3.process();


		const int RELAX_OUTER_ITERS=2;
		const int RELAX_INNER_ITERS=5;

		mGrid.updateUnsignedLevelSet();
		for(int iter=0;iter<RELAX_OUTER_ITERS;iter++){
			mGrid.updateNearestNeighbors();
			mGrid.relax(RELAX_INNER_ITERS);
		}
		mGrid.updateSignedLevelSet();
		mGrid.clean();
		mGrid.updateUnsignedLevelSet();
		mGrid.updateGradient();
		TrackerT mTracker(*mGrid.signedLevelSet,mInterrupt);
		SpringLevelSetEvolve<MapT> evolve(*this,mTracker,mEndTime,1.0,4);
		evolve.process();
		mGrid.updateIsoSurface();
		int added=mGrid.fill();
		mGrid.constellation.updateVertexNormals();
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
				mField(DiscreteField<openvdb::VectorGrid>(*(mParent.mGrid.gradient))),
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
//end of LevelSetAdvection
}
#endif
