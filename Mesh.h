/*
 * Mesh.h
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#ifndef MESH_H_
#define MESH_H_
#include <openvdb/openvdb.h>
#include <mutex>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glfw.h>
namespace imagesci {
class Mesh {
	public:
		enum PrimitiveType{QUADS,TRIANGLES};
		std::mutex drawLock;

		GLuint mVertexBuffer;
		GLuint mNormalBuffer;
		GLuint mColorBuffer;
		GLuint mIndexBuffer;

		std::vector<openvdb::Vec3s> points;
		std::vector<openvdb::Vec3s> colors;
		std::vector<openvdb::Vec3s> normals;
		std::vector<openvdb::Index32> indexes;

		Mesh();
		template<typename GridType> void Create(typename GridType::ConstPtr grid,enum PrimitiveType primType);
		virtual ~Mesh();
	};
}
#endif /* MESH_H_ */
