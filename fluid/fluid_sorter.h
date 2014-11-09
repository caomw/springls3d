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
	std::vector<particlePtr> getNeigboringParticles_wall( int i, int j, int k, int w, int h, int d );
	std::vector<particlePtr> getNeigboringParticles_cell( int i, int j, int k, int w, int h, int d );
	FLOAT levelset( int i, int j, int k, FLOAT ***halfwall, FLOAT density );
	
	int	 getCellSize(){ return gn; }
	int	 getNumParticleAt( int i, int j, int k );
	void markWater( char ***A, FLOAT ***halfwall, FLOAT density );
	void deleteAllParticles();
	
protected:
	RegularGrid<std::vector<particlePtr> > cells;
	int gn;
};
}}
#endif
