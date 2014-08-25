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
		springls.resize(faceCount);

		SpringlBase<Description> springl;
		size_t counter=0;
		size_t pcounter=0;
		storage.faces.reserve(faceCount);
		storage.vertexes.resize(mesh.indexes.size());
		storage.particles.resize(mesh.vertexes.size());
		for(openvdb::Vec4I face:mesh.faces){
			if(face[3]!=util::INVALID_IDX){
				springl=QuadSpringl<Description>();
				storage.faces.push_back(openvdb::Vec4I(counter,counter+1,counter+2,counter+3));
				springl.vertexes[0]=&storage.vertexes[counter++]=mesh.vertexes[face[0]];
				springl.vertexes[1]=&storage.vertexes[counter++]=mesh.vertexes[face[1]];
				springl.vertexes[2]=&storage.vertexes[counter++]=mesh.vertexes[face[2]];
				springl.vertexes[3]=&storage.vertexes[counter++]=mesh.vertexes[face[3]];

			} else {
				springl=TriSpringl<Description>();
				storage.faces.push_back(openvdb::Vec4I(counter,counter+1,counter+2,util::INVALID_IDX));
				springl.vertexes[0]=&storage.vertexes[counter++]=mesh.vertexes[face[0]];
				springl.vertexes[1]=&storage.vertexes[counter++]=mesh.vertexes[face[1]];
				springl.vertexes[2]=&storage.vertexes[counter++]=mesh.vertexes[face[2]];
			}
			springl.id=springls.size();
			springl.particle=&storage.particles[pcounter]=springl.computeCentroid();
			springl.normal=&storage.particleNormals[pcounter++]=springl.computeNormal();
			springls.push_back(springl);
		}
	}
	template <typename Description> Constellation<Description>::~Constellation() {

	}
}
