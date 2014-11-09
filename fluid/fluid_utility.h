/*
 *  utility.h
 *  flip3D
 *
 */

#include "fluid_common.h"
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
FLOAT smooth_kernel( FLOAT r2, FLOAT h );
FLOAT sharp_kernel( FLOAT r2, FLOAT h );
FLOAT length(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1);
FLOAT length2(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1);
FLOAT hypot2( FLOAT a, FLOAT b, FLOAT c );
void mapP2G( sorter *sort, std::vector<particlePtr>& particles,MACGrid<float>&  grid, int gn );
void mapG2P( std::vector<particlePtr>& particles, MACGrid<float>& grid, int gn );
FLOAT linear( RegularGrid<float>& q, FLOAT x, FLOAT y, FLOAT z, int w, int h, int d ) ;
void fetchVelocity(openvdb::Vec3f& p,openvdb::Vec3f& u,MACGrid<float>& grid, int gn );
void resample( sorter *sort, openvdb::Vec3f& p, openvdb::Vec3f& u, FLOAT re );
void correct( sorter *sort, std::vector<particlePtr>& particle, FLOAT dt, FLOAT re);
static void dump(const char *format, ...) {
	va_list args;
    
	FILE *console = fopen( "render/log/console.out", "a" );
	va_start(args, format);
	vfprintf(console, format, args);
	va_end(args);
	fclose(console);
    
	va_start(args, format);
	vprintf(format, args);
	va_end(args);
}

void my_rand_shuffle( std::vector<ipos> &waters );



}
}
#endif
