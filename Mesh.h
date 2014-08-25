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
	private:
		openvdb::math::BBox<openvdb::Vec3d> bbox;
	public:
		enum PrimitiveType {QUADS=GL_QUADS,TRIANGLES=GL_TRIANGLES};

		std::mutex drawLock;

		PrimitiveType meshType;
		GLuint mVertexBuffer;
		GLuint mParticleBuffer;
		GLuint mNormalBuffer;
		GLuint mParticleNormalBuffer;
		GLuint mColorBuffer;
		GLuint mIndexBuffer;
		GLuint elementCount;
		std::vector<openvdb::Vec3s> particles;
		std::vector<openvdb::Vec3s> particleNormals;
		std::vector<openvdb::Vec3s> vertexes;
		std::vector<openvdb::Vec3s> colors;
		std::vector<openvdb::Vec3s> normals;
		std::vector<openvdb::Index32> indexes;
		std::vector<openvdb::Vec4I> faces;
		Mesh();
		inline openvdb::BBoxd GetBBox(){return bbox;}
		openvdb::math::BBox<openvdb::Vec3d>& updateBBox();
		void draw(bool colorEnabled=false);
		void scale(float sc);
		void updateGL();
		void mapIntoBoundingBox(float voxelSize);
		void mapOutOfBoundingBox(float voxelSize);
		static Mesh* openMesh(const std::string& file);
		static Mesh* openGrid(const std::string& file);
		bool save(const std::string& file);
		void create(openvdb::FloatGrid::Ptr grid);
		float EstimateVoxelSize(int stride=4);
		virtual ~Mesh();

};
}
#endif /* MESH_H_ */
