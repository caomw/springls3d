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
class sorter {
public:
	sorter( int gn );
	~sorter();
	
	void sort( std::vector<ParticlePtr>& particles );
	std::vector<particle*> getNeigboringParticles_wall( int i, int j, int k, int w, int h, int d );
	std::vector<particle*> getNeigboringParticles_cell( int i, int j, int k, int w, int h, int d );
	float levelset( int i, int j, int k, RegularGrid<float>& halfwall, float density );
	
	int	 getCellSize(){ return gn; }
	int	 getNumParticleAt( int i, int j, int k );
	void markWater(RegularGrid<char>& A, RegularGrid<float>& halfwall, float density );
	void deleteAllParticles();
	
protected:
	RegularGrid<std::vector<ParticlePtr> > cells;
	int gn;
};
}}
#endif
