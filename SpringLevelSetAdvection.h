#ifndef SPRINGLEVELSETADVECTION_H_
#define SPRINGLEVELSETADVECTION_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetAdvect.h>
namespace imagesci {
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

		for (double time = mStartTime; time < mEndTime; time += dt * scale) {
			std::cout << "Time " << time << std::endl;
			AdvectOperator<OpT, FieldT, InterruptT> op1(mGrid, mField,
					mInterrupt, time);
			op1.process(threaded);
			MaxOperator<OpT, InterruptT> op2(mGrid, mInterrupt);
			double maxV = std::max(EPS, std::sqrt(op2.process(threaded)));
			dt = std::max(0.0,
					std::min(SpringLevelSet::MAX_VEXT / maxV,
							(mEndTime - time) / scale));
			if (dt < EPS)
				break;
			ApplyOperator<OpT, InterruptT> op3(mGrid, mInterrupt, dt);
			//op3.process(threaded);
			mGrid.updateUnsignedLevelSet();
			mGrid.evolve();
			break;
		}
		WriteToRawFile(mGrid.signedLevelSet, "/home/blake/tmp/signedLevelSet");
		WriteToRawFile(mGrid.unsignedLevelSet,"/home/blake/tmp/unsignedLevelSet");
		WriteToRawFile(mGrid.gradient, "/home/blake/tmp/gradient");

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

	/// Main constructor
	SpringLevelSetAdvection(SpringLevelSet& grid, const FieldT& field,
			InterruptT* interrupt = NULL) :
			mGrid(grid), mField(field), mInterrupt(interrupt), mTemporalScheme(
					SpringlTemporalIntegrationScheme::TVD_RK4) {
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

	    typedef AdvectVertexOperation<FieldT> OpT;
		double dt = 0.0;
		Vec3d vsz = mGrid.transformPtr()->voxelSize();
		double scale = std::max(std::max(vsz[0], vsz[1]), vsz[2]);
		const double EPS=1E-30f;
		try {
		for (double time = mStartTime; time < mEndTime; time += dt * scale) {
			std::cout << "Time " << time << std::endl;
			AdvectOperator<OpT, FieldT, InterruptT> op1(mGrid, mField,
					mInterrupt, time);
			op1.process();
			MaxOperator<OpT, InterruptT> op2(mGrid, mInterrupt);
			double maxV = std::max(EPS, std::sqrt(op2.process()));
			dt = std::max(0.0,
					std::min(SpringLevelSet::MAX_VEXT / maxV,
							(mEndTime - time) / scale));
			if (dt < EPS)
				break;
			ApplyOperator<OpT, InterruptT> op3(mGrid, mInterrupt, dt);
			//op3.process(threaded);
			mGrid.updateUnsignedLevelSet();
			mGrid.updateGradient();
			std::cout<<"Start Evolve "<<std::endl;
			WriteToRawFile(mGrid.signedLevelSet, "/home/blake/tmp/signed");
			if ( !mGrid.signedLevelSet->hasUniformVoxels() ) {
					        std::cout<<"The transform must have uniform scale for the LevelSetTracker to function"<<std::endl;
					 }
			if (mGrid.signedLevelSet->getGridClass() != GRID_LEVEL_SET) {
					    	std::cout<<"LevelSetTracker only supports level sets!\n" <<"However, only level sets are guaranteed to work!\n"<<"Hint: Grid::setGridClass(openvdb::GRID_LEVEL_SET)"<<std::endl;
			}
			TrackerT mTracker(*mGrid.signedLevelSet,mInterrupt);
			std::cout<<"Start evolve"<<std::endl;
			SpringLevelSetAdvect<MapT> evolve(*this,mTracker,time,1.0);
			evolve.process(false);
			std::cout<<"End evolve "<<std::endl;
			break;
		}
		} catch(std::exception* e){
			std::cout<<"Error "<<e->what()<<std::endl;
		}
	    return 0;
	}
	template<typename MapT> class SpringLevelSetAdvect {
	public:
		SpringLevelSetAdvection& mParent;
		typename TrackerT::LeafManagerType& leafs;
		const MapT* mMap;
		double mDt;
		double mTime;
		TrackerT& mTracker;
		DiscreteField<openvdb::VectorGrid> mField;
		SpringLevelSetAdvect(SpringLevelSetAdvection& parent,TrackerT& tracker,double time, double dt) :
				mMap(NULL),
				mParent(parent),
				mTracker(tracker),
				mField(DiscreteField<openvdb::VectorGrid>(*(mParent.mGrid.gradient))),
				mTime(time), mDt(dt),leafs(tracker.leafs()) {

		}
		void process(bool threaded = true) {
			mMap= (mTracker.grid().transform().template constMap<MapT>().get());
			if (mParent.mInterrupt)
				mParent.mInterrupt->start("Processing voxels");

			std::cout<<"Init tracker "<<leafs.leafCount()<<std::endl;
			leafs.rebuildAuxBuffers(1);

			std::cout<<"Evolve "<<leafs.leafCount()<<std::endl;
			if (threaded) {
				tbb::parallel_for(leafs.getRange(mTracker.getGrainSize()), *this);
			} else {
				(*this)(leafs.getRange(mTracker.getGrainSize()));
			}

	        std::cout<<"Update "<<std::endl;
	        leafs.removeAuxBuffers();
	        mTracker.track();
			if (mParent.mInterrupt)
				mParent.mInterrupt->end();
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
	SpringLevelSet& mGrid;
	const FieldT& mField;
	SpringlTemporalIntegrationScheme mTemporalScheme;
	InterruptT* mInterrupt;
	// disallow copy by assignment
	void operator=(const SpringLevelSetAdvection& other){}
};
//end of LevelSetAdvection
}
#endif
