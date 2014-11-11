/*
 *  utility.h
 *  flip3D
 *
 */

#include "fluid_common.h"
#include "fluid_sorter.h"
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#ifndef _FLUIDUTILITY_H
#define _FLUIDUTILITY_H

#define FOR_EVERY_X_FLOW(N)	for( int i=0; i<N+1; i++ ) for( int j=0; j<N; j++ ) for( int k=0; k<N; k++ ) {
#define FOR_EVERY_Y_FLOW(N)	for( int i=0; i<N; i++ ) for( int j=0; j<N+1; j++ ) for( int k=0; k<N; k++ ) {
#define FOR_EVERY_Z_FLOW(N)	for( int i=0; i<N; i++ ) for( int j=0; j<N; j++ ) for( int k=0; k<N+1; k++ ) {
#define FOR_EVERY_CELL(n)		for( int i=0; i<n; i++ ) for( int j=0; j<n; j++ ) for( int k=0; k<n; k++ ) {
#define END_FOR }

#ifdef _OPENMP
#include <omp.h>
#define OPENMP_FOR		_Pragma("omp parallel for" )
#define OPENMP_SECTION  _Pragma("omp section" )
#define OPENMP_BEGIN	_Pragma("omp parallel" ) {
#define OPENMP_END		}
#define OPENMP_FOR_P	_Pragma("omp for" )
#else
#define OPENMP_FOR
#define OPENMP_SECTION
#define OPENMP_BEGIN
#define OPENMP_END
#define OPENMP_FOR_P
#endif
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
