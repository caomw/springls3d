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
	
	void sort( std::vector<particlePtr>& particles );
	std::vector<particle*> getNeigboringParticles_wall( int i, int j, int k, int w, int h, int d );
	std::vector<particle*> getNeigboringParticles_cell( int i, int j, int k, int w, int h, int d );
	FLOAT levelset( int i, int j, int k, RegularGrid<float>& halfwall, FLOAT density );
	
	int	 getCellSize(){ return gn; }
	int	 getNumParticleAt( int i, int j, int k );
	void markWater(RegularGrid<char>& A, RegularGrid<float>& halfwall, FLOAT density );
	void deleteAllParticles();
	
protected:
	RegularGrid<std::vector<particlePtr> > cells;
	int gn;
};
}}
#endif
