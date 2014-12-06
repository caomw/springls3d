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
 *  This implementation of a PIC/FLIP fluid simulator is derived from:
 *
 *  Ando, R., Thurey, N., & Tsuruno, R. (2012). Preserving fluid sheets with adaptively sampled anisotropic particles.
 *  Visualization and Computer Graphics, IEEE Transactions on, 18(8), 1202-1214.
 */
#ifndef FLIPFLUIDSOLVER_H_
#define FLIPFLUIDSOLVER_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/Dense.h>
#include <openvdb/tools/DenseSparseTools.h>

#include "fluid_common.h"
#include "fluid_sorter.h"
#include "../ParticleVolume.h"
#include "../Simulation.h"
#include "../SpringLevelSetParticleDeformation.h"
#include "../DistanceField.h"
#include "FluidVelocityField.h"
#include "FluidTrackingField.h"
#undef OPENVDB_REQUIRE_VERSION_NAME


namespace imagesci {
namespace fluid{
/*
 *
 */
class FluidSimulation :public Simulation{
	protected:
		std::unique_ptr<imagesci::SpringLevelSetParticleDeformation<openvdb::util::NullInterrupter> > mAdvect;
		//Constant, even though gravity really isn't constant on earth.
		const static float GRAVITY ;
		float mMaxDensity;
		float mPicFlipBlendWeight ;
		float mFluidParticleDiameter;
		DistanceField mDistanceField;
		MACGrid<float> mVelocity;
		MACGrid<float> mVelocityLast;
		RegularGrid<char> mLabel;
		RegularGrid<float> mLaplacian;
		RegularGrid<float> mDivergence;
		RegularGrid<float> mPressure;
		RegularGrid<openvdb::Vec3s> mWallNormal;
		RegularGrid<openvdb::Vec3s> mDenseMap;
		RegularGrid<float> mWallWeight;
		RegularGrid<float> mLevelSet;
		openvdb::FloatGrid::Ptr mSparseLevelSet;
		openvdb::Coord mGridSize;
		int mStuckParticleCount;
		float mVoxelSize;
		float mWallThickness;
		std::unique_ptr<ParticleLocator> mParticleLocator;
		std::vector<CollisionObject> mCollisionObjects;
		std::vector<ParticlePtr> mParticles;
		void copyGridToBuffer();
		void subtractGrid();
		void placeObjects();
		void placeWalls();
		void damBreakTest();
		void computeParticleDensity(float maxDensity);
		void computeWallNormals();
		void advectParticles();
		void solvePicFlip();
		void addExternalForce();
		void pourWater(int limit,float maxDensity);
		void extrapolateVelocity();
		void repositionParticles(std::vector<int>& indices) ;
		void addParticle( openvdb::Vec3s pt, openvdb::Vec3s center,char type );
		void project();
		void createLevelSet();
		void updateParticleVolume();
		void enforceBoundaryCondition();
		float SmoothKernel( float r2, float h );
		float SharpKernel( float r2, float h );
		float Distance(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1);
		float DistanceSquared(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1);
		float LengthSquared( float a, float b, float c );
		void ShuffleCoordinates( std::vector<openvdb::Coord> &waters );
		float linear( RegularGrid<float>& q, float x, float y, float z ) ;
		void resampleParticles( ParticleLocator *sort, openvdb::Vec3f& p, openvdb::Vec3f& u, float re );
		void correctParticles( ParticleLocator *sort, std::vector<ParticlePtr>& particle, float dt, float re);
		double implicit_func( ParticleLocator *sort, openvdb::Vec3f& p, float density );
		double implicit_func( std::vector<FluidParticle*> &neighbors,openvdb::Vec3f& p, float density,float voxelSize);
		void MapParticlesToGrid();
		void MapGridToParticles();
		inline float isWallIndicator( char a ) {
			return ((a == WALL) ? 1.0f : -1.0f);
		}
	public:
		FluidSimulation(const openvdb::Coord& dims,float voxelSize,MotionScheme scheme) ;
		virtual bool init();
		virtual bool step();
		virtual void cleanup();
		virtual ~FluidSimulation();
	};
}
} /* namespace imagesci */

#endif /* FLIPFLUIDSOLVER_H_ */
