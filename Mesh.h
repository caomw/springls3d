/*
 * Mesh.h
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#ifndef MESH_H_
#define MESH_H_
#include <openvdb/openvdb.h>
#undef OPENVDB_REQUIRE_VERSION_NAME

#include <mutex>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glfw.h>
namespace imagesci {
class Mesh{
	public:
		enum PrimitiveType {QUADS=GL_QUADS,TRIANGLES=GL_TRIANGLES};

		std::mutex drawLock;

		PrimitiveType meshType;
		GLuint mVertexBuffer;
		GLuint mNormalBuffer;
		GLuint mColorBuffer;
		GLuint mIndexBuffer;
		openvdb::math::BBox<openvdb::Vec3d> bbox;
		std::vector<openvdb::Vec3s> points;
		std::vector<openvdb::Vec3s> colors;
		std::vector<openvdb::Vec3s> normals;
		std::vector<openvdb::Index32> indexes;
		std::vector<openvdb::Vec4I> faces;
		Mesh();
		openvdb::math::BBox<openvdb::Vec3d>& updateBBox();
		void draw(bool colorEnabled=false);
		void scale(float sc);
		void updateGL();
		void mapIntoBoundingBox(float voxelSize);
		void mapOutOfBoundingBox(float voxelSize);
		static Mesh* openMesh(const std::string& file);
		static Mesh* openGrid(const std::string& file);
		bool save(const std::string& file);
		static Mesh* create(openvdb::FloatGrid::Ptr grid);
		float EstimateVoxelSize(int stride=4);
		virtual ~Mesh();

};
}
#endif /* MESH_H_ */
