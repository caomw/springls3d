/*
 * Constellation.cc
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#include "Constellation.h"
using namespace openvdb;
namespace imagesci{
	template<typename Description>
	Constellation<Description>::Constellation(Mesh& mesh) {
		size_t faceCount=mesh.faces.size();
		springls.clear();
		springls.reserve(faceCount);
		SpringlBase<Description> springl;
		for(openvdb::Vec4I face:mesh.faces){
			if(face[3]!=util::INVALID_IDX){
				springl=Springl<Description,4>();
				springl[0]=mesh.points[face[0]];
				springl[1]=mesh.points[face[1]];
				springl[2]=mesh.points[face[2]];
				springl[3]=mesh.points[face[3]];
			} else {
				springl=Springl<Description,3>();
				springl[0]=mesh.points[face[0]];
				springl[1]=mesh.points[face[1]];
				springl[2]=mesh.points[face[2]];
			}
			springl.id=springls.size();
			springl.particle=springl.computeCentroid();
			springl.normal=springl.computeNormal();
			springls.push_back(springl);
		}
	}
	template <typename Description> Constellation<Description>::~Constellation() {

	}
}

