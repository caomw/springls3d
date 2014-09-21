/*
 * Mesh.h
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#ifndef MESH_H_
#define MESH_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/LevelSetUtil.h>
#undef OPENVDB_REQUIRE_VERSION_NAME

#include <mutex>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glfw.h>
namespace imagesci {
class Exception:public std::exception{
protected:
	std::string message;
public:

	Exception(const char* str){
		message=str;
	}
	const char* what(){
		return message.c_str();
	}
};
class Mesh{
	private:
		openvdb::math::BBox<openvdb::Vec3d> bbox;
	public:
		enum PrimitiveType {QUADS=4,TRIANGLES=3};
		GLuint mVertexBuffer;
		GLuint mParticleBuffer;
		GLuint mNormalBuffer;
		GLuint mParticleNormalBuffer;
		GLuint mColorBuffer;
		GLuint mTriIndexBuffer;
		GLuint mQuadIndexBuffer;
		GLuint mLineBuffer;
		GLuint triangleCount;
		GLuint particleCount;
		GLuint quadCount;
		GLuint triangleIndexCount;
		GLuint quadIndexCount;
		std::vector<openvdb::Vec3s> lines;
		std::vector<openvdb::Vec3s> particles;
		std::vector<openvdb::Vec3s> particleNormals;
		std::vector<openvdb::Vec3s> vertexes;
		std::vector<openvdb::Vec3s> colors;
		std::vector<openvdb::Vec3s> vertexNormals;
		std::vector<openvdb::Index32> quadIndexes;
		std::vector<openvdb::Index32> triIndexes;
		std::vector<openvdb::Vec2s> uvMap;
		std::vector<openvdb::Vec3s> vertexDisplacement;
		std::vector<openvdb::Vec3s> particleDisplacement;
		std::vector<openvdb::Vec4I> faces;
		Mesh();
		void create(openvdb::tools::VolumeToMesh& mesher,openvdb::FloatGrid::Ptr grid);
		inline openvdb::BBoxd GetBBox(){return bbox;}
		openvdb::math::BBox<openvdb::Vec3d>& updateBBox();
		void draw(bool colorEnabled=false,bool wireframe=true,bool particles=false,bool particleNormals=false);
		void scale(float sc);
		void updateGL();
		void updateVertexNormals();
		void mapIntoBoundingBox(float voxelSize);
		void mapOutOfBoundingBox(float voxelSize);
		bool openMesh(const std::string& file);
		bool openGrid(const std::string& file);
		bool save(const std::string& file);
		void create(openvdb::FloatGrid::Ptr grid);
		float EstimateVoxelSize(int stride=4);
		~Mesh();
};
}
#endif /* MESH_H_ */
