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

#ifndef FLIPFLUIDSOLVER_H_
#define FLIPFLUIDSOLVER_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/Dense.h>
#include <openvdb/tools/DenseSparseTools.h>
#include "fluid_common.h"
#include "fluid_sorter.h"
#include "../Simulation.h"
#undef OPENVDB_REQUIRE_VERSION_NAME


namespace imagesci {
namespace fluid{
/*
 *
 */
class FluidSimulation :public Simulation{

	protected:
		int pourTime;
		openvdb::Vec2f mPourPosition;
		float mPourRadius;
		double mMaxDensity;
		//int MAX_STEP;
		const static float mPicFlipBlendWeight ;
		const static float mTimeStep   ;
		const static float mDensityIsoLevel;
		const static float GRAVITY ;
		MACGrid<float> mVelocity;
		MACGrid<float> mVelocityLast;
		RegularGrid<char> mLabel;
		RegularGrid<float> mLaplacian;
		RegularGrid<float> mDivergence;
		RegularGrid<float> mPressure;
		RegularGrid<openvdb::Vec3f> mWallNormal;
		RegularGrid<float> mWallWeight;
		RegularGrid<float> mLevelSet;
		int mGridSize;
		int mStuckParticleCount;
		float mWallThickness;
		std::unique_ptr<sorter> mSorter;
		std::vector<Object> mSimulationObjects;
		void save_grid();
		void subtract_grid();
		void placeObjects();
		void placeWalls();
		void damBreakTest();
		void computeDensity();
		void compute_wall_normal();
		bool step();
		void advect_particle();
		void solve_picflip();
		void add_ExtForce();
		void pourWater(int limit);
		void extrapolate_velocity();
		void reposition(std::vector<int>& indices, std::vector<ParticlePtr> particles ) ;
		void pushParticle( double x, double y, double z, char type );
		void project();
		void createLevelSet();
		void enforce_boundary();
		void cleanup();
		inline float isWallIndicator( char a ) {
			return ((a == WALL) ? 1.0f : -1.0f);
		}

	public:
		std::vector<ParticlePtr> mParticles;
		FluidSimulation(int gridSize,MotionScheme scheme) ;
		bool init();
		virtual ~FluidSimulation();
	};
}
} /* namespace imagesci */

#endif /* FLIPFLUIDSOLVER_H_ */
