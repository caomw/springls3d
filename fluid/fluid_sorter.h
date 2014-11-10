/*
 *  sorter.h
 *  flip3D
 */

#include "fluid_common.h"
#include <vector>
#ifndef _SORTER_H
#define _SORTER_H
namespace imagesci{
namespace fluid{
class ParticleLocator {
public:
	ParticleLocator( int gn );
	~ParticleLocator();
	
	void update( std::vector<ParticlePtr>& particles );
	std::vector<FluidParticle*> getNeigboringWallParticles( int i, int j, int k, int w=1, int h=1, int d=1 );
	std::vector<FluidParticle*> getNeigboringCellParticles( int i, int j, int k, int w=1, int h=1, int d=1 );
	float getLevelSetValue( int i, int j, int k, RegularGrid<float>& halfwall, float density );
	int	 getCellSize(){ return mGridSize; }
	int	 getParticleCount( int i, int j, int k );
	void markAsWater(RegularGrid<char>& A, RegularGrid<float>& halfwall, float density );
	void deleteAllParticles();
	
protected:
	RegularGrid<std::vector<ParticlePtr> > cells;
	int mGridSize;
};
}}
#endif
