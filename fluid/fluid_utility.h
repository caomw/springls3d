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
#ifndef _FLUIDUTILITY_H
#define _FLUIDUTILITY_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "../ImageSciUtil.h"
#include "fluid_common.h"
#include "fluid_sorter.h"

#define FOR_EVERY_X_FLOW(N)	for( int i=0; i<N+1; i++ ) for( int j=0; j<N; j++ ) for( int k=0; k<N; k++ ) {
#define FOR_EVERY_Y_FLOW(N)	for( int i=0; i<N; i++ ) for( int j=0; j<N+1; j++ ) for( int k=0; k<N; k++ ) {
#define FOR_EVERY_Z_FLOW(N)	for( int i=0; i<N; i++ ) for( int j=0; j<N; j++ ) for( int k=0; k<N+1; k++ ) {
#define FOR_EVERY_CELL(n)		for( int i=0; i<n; i++ ) for( int j=0; j<n; j++ ) for( int k=0; k<n; k++ ) {
#define END_FOR }

namespace imagesci{
namespace fluid{
template<typename ValueT> RegularGrid<ValueT> alloc3D( int w, int h, int d ) {
	return RegularGrid<ValueT>(w,h,d);
}

unsigned long getMicroseconds();
double dumptime();
float smooth_kernel( float r2, float h );
float sharp_kernel( float r2, float h );
float length(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1);
float length2(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1);
float hypot2( float a, float b, float c );
void mapParticlesToGrid( ParticleLocator *sort, std::vector<ParticlePtr>& particles,MACGrid<float>&  grid, int gn );
void mapGridToParticles( std::vector<ParticlePtr>& particles, MACGrid<float>& grid, int gn );
float linear( RegularGrid<float>& q, float x, float y, float z, int w, int h, int d ) ;
void fetchVelocity(openvdb::Vec3f& p,openvdb::Vec3f& u,MACGrid<float>& grid, int gn );
void resampleParticles( ParticleLocator *sort, openvdb::Vec3f& p, openvdb::Vec3f& u, float re );
void correctParticles( ParticleLocator *sort, std::vector<ParticlePtr>& particle, float dt, float re);
double implicit_func( ParticleLocator *sort, openvdb::Vec3f& p, float density );
static void dump(const char *format, ...) {
	va_list args;
	va_start(args, format);
	printf(format, args);
	va_end(args);
}

void my_rand_shuffle( std::vector<openvdb::Coord> &waters );

}
}
#endif
