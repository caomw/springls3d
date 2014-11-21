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
#include "fluid_sorter.h"
#include "fluid_utility.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
using namespace std;
namespace imagesci {
namespace fluid {
ParticleLocator::ParticleLocator(openvdb::Coord dims, float voxelSize) :
		mVoxelSize(voxelSize), cells(dims, voxelSize,
				std::vector<FluidParticle*>()), mGridSize(dims) {
}
ParticleLocator::~ParticleLocator() {
}

void ParticleLocator::update(std::vector<ParticlePtr>& particles) {
	// Clear All Cells
	OPENMP_FOR FOR_EVERY_GRID_CELL(cells)
		{
			cells(i, j, k).clear();
		}END_FOR
	// Store Into The Cells
	for (ParticlePtr& p : particles) {
		int i = clamp((int) (mGridSize[0] * p->mLocation[0]), 0,
				mGridSize[0] - 1);
		int j = clamp((int) (mGridSize[1] * p->mLocation[1]), 0,
				mGridSize[1] - 1);
		int k = clamp((int) (mGridSize[2] * p->mLocation[2]), 0,
				mGridSize[2] - 1);
		cells(i, j, k).push_back(p.get());
	}
}

std::vector<FluidParticle*> ParticleLocator::getNeigboringWallParticles(int i,
		int j, int k, int w, int h, int d) {
	std::vector<FluidParticle*> res;
	for (int si = max(i - w, 0); si <= min(i + w - 1, mGridSize[0] - 1); si++) {
		for (int sj = max(j - h, 0); sj <= min(j + h - 1, mGridSize[1] - 1);
				sj++) {
			for (int sk = max(k - d, 0); sk <= min(k + d - 1, mGridSize[2] - 1);
					sk++) {
				for (FluidParticle* p : cells(si, sj, sk)) {
					res.push_back(p);
				}
			}
		}
	}
	return res;
}

std::vector<FluidParticle*> ParticleLocator::getNeigboringCellParticles(int i,
		int j, int k, int w, int h, int d) {
	std::vector<FluidParticle*> res;
	for (int si = max(i - w, 0); si <= min(i + w, mGridSize[0] - 1); si++) {
		for (int sj = max(j - h, 0); sj <= min(j + h, mGridSize[1] - 1); sj++) {
			for (int sk = max(k - d, 0); sk <= min(k + d, mGridSize[2] - 1);
					sk++) {
				for (FluidParticle* p : cells(si, sj, sk)) {
					res.push_back(p);
				}
			}
		}
	}
	return res;
}

int ParticleLocator::getParticleCount(int i, int j, int k) {
	return cells(i, j, k).size();
}

float ParticleLocator::getLevelSetValue(int i, int j, int k,
		RegularGrid<float>& halfwall, float density) {
	float accm = 0.0;
	for (FluidParticle* p : cells(i, j, k)) {
		if (p->mObjectType == FLUID) {
			accm += p->mDensity;
		} else {
			return 1.0;
		}
	}
	float MAX_VOLUME = 1.0 / (density * density * density);
	const float alpha = 0.2f;
	return alpha * MAX_VOLUME - accm;
}

void ParticleLocator::markAsWater(RegularGrid<char>& A,
		RegularGrid<float>& halfwall, float density) {
	OPENMP_FOR FOR_EVERY_GRID_CELL(cells)
		{
			A(i, j, k) = AIR;
			for (FluidParticle* p : cells(i, j, k)) {
				if (p->mObjectType == WALL) {
					A(i, j, k) = WALL;
					break;
				}
			}
			if (A(i, j, k) != WALL)
				A(i, j, k) =
						getLevelSetValue(i, j, k, halfwall, density) < 0.0 ?
								FLUID : AIR;
		}END_FOR
}
void ParticleLocator::deleteAllParticles() {
	OPENMP_FOR FOR_EVERY_GRID_CELL(cells)
		{
			cells(i, j, k).clear();
		}END_FOR
}
}
}
