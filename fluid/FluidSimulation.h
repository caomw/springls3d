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
#undef OPENVDB_REQUIRE_VERSION_NAME


namespace imagesci {
namespace fluid{
/*
 *
 */
class FluidSimulation :public Simulation{
	protected:
		//Constant, even though gravity really isn't constant on earth.
		const static float GRAVITY ;
		float mMaxDensity;
		float mPicFlipBlendWeight ;
		float mFluidParticleDiameter;
		MACGrid<float> mVelocity;
		MACGrid<float> mVelocityLast;
		RegularGrid<char> mLabel;
		RegularGrid<float> mLaplacian;
		RegularGrid<float> mDivergence;
		RegularGrid<float> mPressure;
		RegularGrid<openvdb::Vec3s> mWallNormal;
		RegularGrid<float> mWallWeight;
		RegularGrid<float> mLevelSet;
		openvdb::Coord mGridSize;
		int mStuckParticleCount;
		float mVoxelSize;
		float mWallThickness;
		std::unique_ptr<ParticleLocator> mParticleLocator;
		std::vector<CollisionObject> mCollisionObjects;
		std::vector<ParticlePtr> mParticles;
		void getParticles(ParticleVolume& pv);
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
		void addParticle( double x, double y, double z, char type );
		void project();
		void createLevelSet();
		void enforceBoundaryCondition();
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
