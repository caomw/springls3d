/*
 * Mesh.cc
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#include "Mesh.h"
#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tree/LeafManager.h>
#include <openvdb/math/Operators.h>

namespace imagesci {
using namespace openvdb;
Mesh::Mesh() :
		mVertexBuffer(0), mNormalBuffer(0), mColorBuffer(0), mIndexBuffer(0) {
}
template<typename GridType> void Mesh::Create(typename GridType::ConstPtr grid,PrimitiveType primType) {
	using openvdb::Index64;
	openvdb::tools::VolumeToMesh mesher(
			grid->getGridClass() == openvdb::GRID_LEVEL_SET ? 0.0 : 0.01);
	mesher(*grid);
	// Copy points and generate point normals.
	points.resize(mesher.pointListSize() * 3);
	normals.resize(mesher.pointListSize() * 3);
	openvdb::tree::ValueAccessor<const typename GridType::TreeType> acc(
			grid->tree());

	openvdb::math::GenericMap map(grid->transform());
	openvdb::Coord ijk;
	this->points = mesher.pointList();
	// Copy primitives
	openvdb::tools::PolygonPoolList& polygonPoolList = mesher.polygonPoolList();
	Index64 numQuads = 0;
	for (Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
		numQuads += polygonPoolList[n].numQuads();
	}
	indexes.reserve(numQuads * 4);
	openvdb::Vec3d normal, e1, e2;
	for (Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
		const openvdb::tools::PolygonPool& polygons = polygonPoolList[n];
		std::cout << "Polygon " << polygons.numTriangles() << " "
				<< polygons.numQuads() << std::endl;
		for (Index64 i = 0, I = polygons.numQuads(); i < I; ++i) {
			const openvdb::Vec4I& quad = polygons.quad(i);
			indexes.push_back(quad[0]);
			indexes.push_back(quad[1]);
			indexes.push_back(quad[2]);
			indexes.push_back(quad[3]);
			e1 = mesher.pointList()[quad[1]];
			e1 -= mesher.pointList()[quad[0]];
			e2 = mesher.pointList()[quad[2]];
			e2 -= mesher.pointList()[quad[1]];
			normal = e1.cross(e2);
			const double length = normal.length();
			if (length > 1.0e-7)
				normal *= (1.0 / length);
			for (Index64 v = 0; v < 4; ++v) {
				normals[quad[v]] = -normal;
			}
		}
	}

	if (points.size() > 0) {
		if (glIsBuffer(mVertexBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mVertexBuffer);
		glGenBuffers(1, &mVertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		if (glIsBuffer(mVertexBuffer) == GL_FALSE)
			throw "Error: Unable to create vertex buffer";
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * points.size(),
				&points[0], GL_STATIC_DRAW);
		if (GL_NO_ERROR != glGetError())
			throw "Error: Unable to upload vertex buffer data";
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (colors.size() > 0) {
		if (glIsBuffer(mColorBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mColorBuffer);

		glGenBuffers(1, &mColorBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mColorBuffer);
		if (glIsBuffer(mColorBuffer) == GL_FALSE)
			throw "Error: Unable to create color buffer";

		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * colors.size(),
				&colors[0], GL_STATIC_DRAW);
		if (GL_NO_ERROR != glGetError())
			throw "Error: Unable to upload color buffer data";

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (indexes.size() > 0) {
		// clear old buffer
		if (glIsBuffer(mIndexBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mIndexBuffer);

		// gen new buffer
		glGenBuffers(1, &mIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);
		if (glIsBuffer(mIndexBuffer) == GL_FALSE)
			throw "Error: Unable to create index buffer";

		// upload data
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * indexes.size(),
				&indexes[0], GL_STATIC_DRAW); // upload data
		if (GL_NO_ERROR != glGetError())
			throw "Error: Unable to upload index buffer data";

		// release buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	if (normals.size() > 0) {
		if (glIsBuffer(mNormalBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mNormalBuffer);

		glGenBuffers(1, &mNormalBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mNormalBuffer);
		if (glIsBuffer(mNormalBuffer) == GL_FALSE)
			throw "Error: Unable to create normal buffer";

		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * normals.size(),
				&normals[0], GL_STATIC_DRAW);
		if (GL_NO_ERROR != glGetError())
			throw "Error: Unable to upload normal buffer data";

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
}

Mesh::~Mesh() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
