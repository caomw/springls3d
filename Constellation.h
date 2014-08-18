/*
 * Constellation.h
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#ifndef CONSTELLATION_H_
#define CONSTELLATION_H_
#include "Mesh.h"
#include <openvdb/openvdb.h>
#include <vector>
#include <list>
namespace imagesci{

struct SpringlDesc{
	openvdb::Vec3s trackedPoint;
	openvdb::Index64 label;
};

class Constellation : public Mesh {
	public:
		std::vector<SpringlDesc> descriptions;
		std::vector<std::list<openvdb::Int64>> neighborGraph;
		Constellation();
		virtual ~Constellation();
	};
}

#endif /* CONSTELLATION_H_ */
