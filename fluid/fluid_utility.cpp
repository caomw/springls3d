/*
 *  utility.cpp
 *  flip3D
 *
 */

#include "fluid_utility.h"
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
namespace imagesci{
namespace fluid{

FLOAT hypot2( FLOAT a, FLOAT b, FLOAT c ) {
    return a*a + b*b + c*c;
}

FLOAT length2(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
	return (p0-p1).lengthSqr();
}

FLOAT length(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
	return (p0-p1).length();
}

void my_rand_shuffle( std::vector<ipos> &waters ) {
	random_shuffle( waters.begin(), waters.end() );
}

unsigned long getMicroseconds() {
	struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000000 + tv.tv_usec;
}
FLOAT smooth_kernel( FLOAT r2, FLOAT h ) {
    return fmax( 1.0-r2/(h*h), 0.0 );
}
FLOAT sharp_kernel( FLOAT r2, FLOAT h ) {
    return fmax( h*h/fmax(r2,1.0e-5) - 1.0, 0.0 );
}
double dumptime() {
	static unsigned prevMicroSec = getMicroseconds();
	unsigned curMicroSec = getMicroseconds();
	double res = (curMicroSec - prevMicroSec)/1000000.0;
	prevMicroSec = curMicroSec;
	return res;
}

}
}
