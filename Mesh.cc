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
#include <boost/smart_ptr.hpp>
#include <openvdb/Types.h>
#include <openvdb/tree/Tree.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetAdvect.h>
#include <openvdb/tools/LevelSetMeasure.h>
#include <openvdb/tools/LevelSetMorph.h>
#include <openvdb/tools/ValueTransformer.h>
#include <openvdb/tools/VectorTransformer.h>
#include <openvdb/util/Util.h>
#include "ply_io.h"
namespace imagesci {
template<typename T> T clamp(T a, T min, T max) {
	return std::max(std::min(a, max), min);
}
using namespace openvdb;
typedef struct _plyVertex {
	float x[3];             // the usual 3-space position of a vertex
	unsigned char red;
	unsigned char green;
	unsigned char blue;
	unsigned char alpha;
	_plyVertex() {
		x[0] = 0;
		x[1] = 0;
		x[2] = 0;
		red = 0;
		green = 0;
		blue = 0;
		alpha = 0;
	}
} plyVertex;

typedef struct _plyFace {
	unsigned char nverts;    // number of vertex indices in list
	int *verts;              // vertex index list
	unsigned char red;
	unsigned char green;
	unsigned char blue;
	_plyFace() {
		nverts = 0;
		verts = NULL;
		red = 0;
		green = 0;
		blue = 0;
	}
} plyFace;

char* elemNames[] = { "vertex", "face", "normal" };

PlyProperty vertProps[] = { // property information for a vertex
		{ "x", Float32, Float32, static_cast<int>(offsetof(plyVertex, x)), 0, 0,
				0, 0 }, { "y", Float32, Float32,
				static_cast<int>(offsetof(plyVertex, x) + sizeof(float)), 0, 0,
				0, 0 }, { "z", Float32, Float32,
				static_cast<int>(offsetof(plyVertex, x) + sizeof(float)
						+ sizeof(float)), 0, 0, 0, 0 }, { "red", Uint8, Uint8,
				static_cast<int>(offsetof(plyVertex, red)), 0, 0, 0, 0 }, {
				"green", Uint8, Uint8, static_cast<int>(offsetof(plyVertex,
						green)), 0, 0, 0, 0 }, { "blue", Uint8, Uint8,
				static_cast<int>(offsetof(plyVertex, blue)), 0, 0, 0, 0 }, {
				"alpha", Uint8, Uint8, static_cast<int>(offsetof(plyVertex,
						alpha)), 0, 0, 0, 0 }, };

PlyProperty faceProps[] = { // property information for a face
		{ "vertex_indices", Int32, Int32, static_cast<int>(offsetof(plyFace,
				verts)), 1, Uint8, Uint8, static_cast<int>(offsetof(plyFace,
				nverts)) }, { "red", Uint8, Uint8, static_cast<int>(offsetof(
				plyFace, red)), 0, 0, 0, 0 }, { "green", Uint8, Uint8,
				static_cast<int>(offsetof(plyFace, green)), 0, 0, 0, 0 }, {
				"blue", Uint8, Uint8, static_cast<int>(offsetof(plyFace, blue)),
				0, 0, 0, 0 }, };

Mesh::Mesh() :
		mVertexBuffer(0), mNormalBuffer(0), mColorBuffer(0), mIndexBuffer(0), mLineBuffer(0),mParticleBuffer(0),mParticleNormalBuffer(0),elementCount(
				0), meshType(PrimitiveType::TRIANGLES) {
}
bool Mesh::save(const std::string& f) {
	int i, j, idx;
	const char* fileName = f.c_str();
	unsigned char *pointColors = NULL;
	PlyFile *ply;

	// Get input and check data

	ply = open_for_writing_ply(fileName, 2, elemNames, PLY_BINARY_LE);

	if (ply == NULL)
		return false;

	// compute colors, if any
	int numPts = vertexes.size();

	int npts = (meshType == PrimitiveType::TRIANGLES) ? 3 : 4;
	int numPolys = indexes.size() / npts;
	unsigned char* c;
	if (colors.size() > 0) {
		pointColors = c = new unsigned char[3 * numPts];
		for (i = 0; i < numPts; i++) {
			Vec3s d = colors[i];
			unsigned char r = (unsigned char) clamp(d[0] * 255.0f, 0.0f,
					255.0f);
			unsigned char g = (unsigned char) clamp(d[1] * 255.0f, 0.0f,
					255.0f);
			unsigned char b = (unsigned char) clamp(d[2] * 255.0f, 0.0f,
					255.0f);
			*c++ = r;
			*c++ = g;
			*c++ = b;
		}
	}
	// describe what properties go into the vertex and face elements
	element_count_ply(ply, "vertex", numPts);
	ply_describe_property(ply, "vertex", &vertProps[0]);
	ply_describe_property(ply, "vertex", &vertProps[1]);
	ply_describe_property(ply, "vertex", &vertProps[2]);
	if (colors.size() > 0) {
		ply_describe_property(ply, "vertex", &vertProps[3]);
		ply_describe_property(ply, "vertex", &vertProps[4]);
		ply_describe_property(ply, "vertex", &vertProps[5]);
	}
	element_count_ply(ply, "face", numPolys);
	ply_describe_property(ply, "face", &faceProps[0]);

	// write a comment and an object information field
	append_comment_ply(ply, "PLY File");
	append_obj_info_ply(ply, "ImageSci");

	// complete the header
	header_complete_ply(ply);

	// set up and write the vertex elements
	plyVertex vert;
	put_element_setup_ply(ply, "vertex");

	for (i = 0; i < numPts; i++) {
		Vec3s pt = vertexes[i];
		vert.x[0] = pt[0];
		vert.x[1] = pt[1];
		vert.x[2] = pt[2];
		if (pointColors) {
			idx = 3 * i;
			vert.red = *(pointColors + idx);
			vert.green = *(pointColors + idx + 1);
			vert.blue = *(pointColors + idx + 2);
		}
		put_element_ply(ply, (void *) &vert);
	}
	// set up and write the face elements
	plyFace face;
	int verts[256];
	face.verts = verts;
	put_element_setup_ply(ply, "face");

	if (indexes.size() < numPolys) {
		for (int i = 0; i < numPolys; i++) {
			for (j = 0; j < npts; j++) {
				face.nverts = npts;
				verts[j] = 3 * i + j;
			}
			put_element_ply(ply, (void *) &face);
		}
	} else {
		for (int i = 0; i < numPolys; i++) {
			for (j = 0; j < npts; j++) {
				face.nverts = npts;
				verts[j] = indexes[3 * i + j];
			}
			put_element_ply(ply, (void *) &face);
		}
	}
	if (pointColors != NULL)
		delete[] pointColors;
	// close the PLY file
	close_ply(ply);
	return true;
}
Mesh* Mesh::openGrid(const std::string& fileName) {
	openvdb::io::File file(fileName);
	file.open();
	openvdb::GridPtrVecPtr grids = file.getGrids();

	openvdb::GridPtrVec allGrids;
	allGrids.insert(allGrids.end(), grids->begin(), grids->end());
	GridBase::Ptr ptr = allGrids[0];
	Mesh* mesh = new Mesh();
	mesh->create(boost::static_pointer_cast<FloatGrid>(ptr));
	return mesh;
}
Mesh* Mesh::openMesh(const std::string& file) {
	int i, j, k;
	int numPts = 0, numPolys = 0;

	// open a PLY file for reading
	PlyFile *ply;
	int nelems = 3, fileType = PLY_BINARY_LE, numElems, nprops;
	char** elist = NULL;
	char* elemName;
	float version;
	if (!(ply = ply_open_for_reading(file.c_str(), &nelems, &elist, &fileType,
			&version))) {
		std::cerr << "Could not open ply file." << std::endl;
		return NULL;
	}

	// Check to make sure that we can read geometry
	PlyElement *elem;
	int index;
	if ((elem = find_element(ply, "vertex")) == NULL
			|| find_property(elem, "x", &index) == NULL
			|| find_property(elem, "y", &index) == NULL
			|| find_property(elem, "z", &index) == NULL
			|| (elem = find_element(ply, "face")) == NULL
			|| find_property(elem, "vertex_indices", &index) == NULL) {
		std::cerr << "Cannot read geometry" << std::endl;
		close_ply(ply);
		return NULL;
	}
	Mesh* mesh = new Mesh();

	// Check for optional attribute data. We can handle intensity; and the
	// triplet red, green, blue.
	unsigned char intensityAvailable = false, RGBCellsAvailable = false,
			RGBPointsAvailable = false;
	std::vector<Vec3s>& colors = mesh->colors;
	std::vector<Vec3s>& points = mesh->vertexes;
	std::vector<Vec3s>& normals = mesh->normals;
	std::vector<Index32>& indexes = mesh->indexes;

	if ((elem = find_element(ply, "face")) != NULL
			&& find_property(elem, "intensity", &index) != NULL) {
		intensityAvailable = true;
	}

	if ((elem = find_element(ply, "face")) != NULL
			&& find_property(elem, "red", &index) != NULL
			&& find_property(elem, "green", &index) != NULL
			&& find_property(elem, "blue", &index) != NULL) {
		RGBCellsAvailable = true;
	}

	if ((elem = find_element(ply, "vertex")) != NULL
			&& find_property(elem, "red", &index) != NULL
			&& find_property(elem, "green", &index) != NULL
			&& find_property(elem, "blue", &index) != NULL) {
		RGBPointsAvailable = true;
	}

	int verts[256];
	plyFace face;
	plyVertex vertex;
	memset(verts, 0, sizeof(verts));
	// Okay, now we can grab the data
	for (i = 0; i < nelems; i++) {
		//get the description of the first element */
		elemName = elist[i];
		get_element_description_ply(ply, elemName, &numElems, &nprops);
		// if we're on vertex elements, read them in
		if (elemName && !strcmp("vertex", elemName)) {
			// Create a list of points
			numPts = numElems;
			points.resize(numPts);
			// Setup to read the PLY elements
			ply_get_property(ply, elemName, &vertProps[0]);
			ply_get_property(ply, elemName, &vertProps[1]);
			ply_get_property(ply, elemName, &vertProps[2]);

			if (RGBPointsAvailable) {
				colors.resize(numPts);
				ply_get_property(ply, elemName, &vertProps[3]);
				ply_get_property(ply, elemName, &vertProps[4]);
				ply_get_property(ply, elemName, &vertProps[5]);
			}
			for (j = 0; j < numPts; j++) {
				get_element_ply(ply, &vertex);
				points[j] = Vec3s(vertex.x[0], vertex.x[1], vertex.x[2]);
				if (RGBPointsAvailable) {
					colors[j] = Vec3s(vertex.red / 255.0f,
							vertex.green / 255.0f, vertex.blue / 255.0f);
				}
			}
		}			//if vertex
		else if (elemName && !strcmp("face", elemName)) {
			// Create a polygonal array
			numPolys = numElems;

			// Get the face properties
			ply_get_property(ply, elemName, &faceProps[0]);
			if (intensityAvailable) {
				ply_get_property(ply, elemName, &faceProps[1]);
			}
			if (RGBCellsAvailable) {
				ply_get_property(ply, elemName, &faceProps[2]);
				ply_get_property(ply, elemName, &faceProps[3]);
				ply_get_property(ply, elemName, &faceProps[4]);
			}
			indexes.clear();
			indexes.reserve(numPolys * 4);
			for (j = 0; j < numPolys; j++) {

				//grab and element from the file
				face.verts = &verts[0];
				get_element_ply(ply, &face);
				for (k = 0; k < face.nverts; k++) {
					indexes.push_back(face.verts[k]);
				}
				if (face.nverts == 3) {
					mesh->meshType = TRIANGLES;
				} else {
					mesh->meshType = QUADS;
				}
				if (intensityAvailable) {
					//Not used!
					//face.intensity;
				}
				if (RGBCellsAvailable) {
					//Not used!
					//face.red;
					//face.green;
					//face.blue;
				}
			}
		}							//if face

		free(elist[i]); //allocated by ply_open_for_reading

	} //for all elements of the PLY file
	free(elist); //allocated by ply_open_for_reading
	close_ply(ply);
	mesh->indexes.shrink_to_fit();
	mesh->faces.resize(numPolys);
	if (mesh->meshType == TRIANGLES) {
		int counter = 0;
		Vec3s norm;
		normals.resize(points.size());
		for (int i = 0; i < numPolys; i++) {
			int vid1 = mesh->indexes[counter++];
			int vid2 = mesh->indexes[counter++];
			int vid3 = mesh->indexes[counter++];
			Vec3s norm = (points[vid2] - points[vid1]).cross(
					points[vid3] - points[vid1]);
			normals[vid1] += norm;
			normals[vid2] += norm;
			normals[vid3] += norm;
			mesh->faces[i] = Vec4I(vid1, vid2, vid3, util::INVALID_IDX);
		}
		for (Vec3s& norm : normals) {
			norm.normalize(1E-6f);
		}
	} else {
		mesh->faces.resize(numPolys);
		memcpy(&mesh->faces[0], &mesh->indexes[0], sizeof(Vec4I) * numPolys);
	}
	if (points.size() > 0) {
		mesh->updateBBox();
		return mesh;
	} else {
		return NULL;
	}
}
void Mesh::create(FloatGrid::Ptr grid) {

	openvdb::tools::VolumeToMesh mesher(
			grid->getGridClass() == openvdb::GRID_LEVEL_SET ? 0.0 : 0.01);
	mesher(*grid);
	// Copy points and generate point normals.

	openvdb::tree::ValueAccessor<FloatGrid::TreeType> acc(grid->tree());

	openvdb::math::GenericMap map(grid->transform());
	openvdb::Coord ijk;
	vertexes.clear();
	normals.clear();
	faces.clear();
	indexes.clear();

	vertexes.resize(mesher.pointListSize());
	normals.resize(mesher.pointListSize());
	Index64 N = mesher.pointListSize();
	for (Index64 n = 0; n < N; ++n) {
		vertexes[n] = map.applyInverseMap(mesher.pointList()[n]);
		normals[n] = Vec3s(0.0f);
	}
	// Copy primitives
	openvdb::tools::PolygonPoolList& polygonPoolList = mesher.polygonPoolList();

	Index64 numQuads = 0;
	for (Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
		numQuads += polygonPoolList[n].numQuads();
	}
	indexes.reserve(numQuads * 4);
	openvdb::Vec3d normal, e1, e2;

	faces.reserve(mesher.polygonPoolListSize());
	for (Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
		const openvdb::tools::PolygonPool& polygons = polygonPoolList[n];
		//std::cout << "Polygon " << polygons.numTriangles() << " "<< polygons.numQuads() << std::endl;
		for (Index64 i = 0, I = polygons.numQuads(); i < I; ++i) {
			const openvdb::Vec4I& quad = polygons.quad(i);
			faces.push_back(quad);
			indexes.push_back(quad[0]);
			indexes.push_back(quad[1]);
			indexes.push_back(quad[2]);
			indexes.push_back(quad[3]);
			normal = (vertexes[quad[2]] - vertexes[quad[0]]).cross(
					vertexes[quad[1]] - vertexes[quad[0]]);
			normals[quad[0]] += normal;
			normals[quad[1]] += normal;
			normals[quad[2]] += normal;
			normals[quad[3]] += normal;
		}
	}
	for (Vec3s& norm : normals) {
		norm.normalize(1E-6f);
	}
	meshType = PrimitiveType::QUADS;
	updateBBox();
}

float Mesh::EstimateVoxelSize(int stride) {
	float avg = 0.0f;
	int count = 0;
	float maxLength = 0.0f;
	int sz = indexes.size();
	if (meshType == TRIANGLES) {
		for (int i = 0; i < sz; i += 3 * stride) {
			Vec3s v1 = vertexes[indexes[i]];
			Vec3s v2 = vertexes[indexes[i + 1]];
			Vec3s v3 = vertexes[indexes[i + 2]];
			float e1 = (v1 - v2).length();
			float e2 = (v1 - v3).length();
			float e3 = (v2 - v3).length();
			maxLength = std::max(std::max(e1, e2), std::max(maxLength, e3));
			avg += e1 + e2 + e3;
			count += 3;
		}
	} else {
		for (int i = 0; i < sz; i += 4 * stride) {
			Vec3s v1 = vertexes[indexes[i]];
			Vec3s v2 = vertexes[indexes[i + 1]];
			Vec3s v3 = vertexes[indexes[i + 2]];
			Vec3s v4 = vertexes[indexes[i + 3]];
			float e1 = (v1 - v2).length();
			float e2 = (v2 - v3).length();
			float e3 = (v3 - v4).length();
			float e4 = (v4 - v1).length();

			maxLength = std::max(maxLength,
					std::max(std::max(e1, e2), std::max(e3, e4)));
			avg += e1 + e2 + e3 + e4;
			count += 4;
		}
	}
	avg /= count;

	std::cout << "Average Edge Length=" << avg << " Max Edge Length="
			<< maxLength << std::endl;
	return avg;
}
openvdb::math::BBox<openvdb::Vec3d>& Mesh::updateBBox() {
	Vec3s minPt(std::numeric_limits<float>::max(),
			std::numeric_limits<float>::max(),
			std::numeric_limits<float>::max());
	Vec3s maxPt(std::numeric_limits<float>::min(),
			std::numeric_limits<float>::min(),
			std::numeric_limits<float>::min());
	for (Vec3s pt : vertexes) {
		minPt[0] = std::min(minPt[0], pt[0]);
		minPt[1] = std::min(minPt[1], pt[1]);
		minPt[2] = std::min(minPt[2], pt[2]);

		maxPt[0] = std::max(maxPt[0], pt[0]);
		maxPt[1] = std::max(maxPt[1], pt[1]);
		maxPt[2] = std::max(maxPt[2], pt[2]);
	}
	bbox = openvdb::math::BBox<openvdb::Vec3d>(minPt, maxPt);
	return bbox;
}
void Mesh::scale(float sc) {
	for (Vec3s& pt : vertexes) {
		pt *= sc;
	}
	bbox.min() *= static_cast<double>(sc);
	bbox.max() *= static_cast<double>(sc);

}
void Mesh::mapIntoBoundingBox(float voxelSize) {
	Vec3s minPt = bbox.min();
	Vec3s maxPt = bbox.max();
	for (Vec3s& pt : vertexes) {
		pt = (pt - minPt) / voxelSize;
	}
}
void Mesh::mapOutOfBoundingBox(float voxelSize) {
	Vec3s minPt = bbox.min();
	Vec3s maxPt = bbox.max();
	for (Vec3s& pt : vertexes) {
		pt = pt * voxelSize + minPt;
	}
}
void Mesh::draw(bool colorEnabled,bool wireframe,bool showParticles,bool showParticleNormals) {
	if (mVertexBuffer > 0) {
		glEnable(GL_LIGHTING);
		if (indexes.size() > 0) {
			glShadeModel(GL_SMOOTH);
		} else {
			glShadeModel(GL_FLAT);
		}
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_INDEX_ARRAY);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		if (colorEnabled)
			glEnableClientState(GL_COLOR_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		glVertexPointer(3, GL_FLOAT, 0, 0);

		if (colorEnabled) {
			glBindBuffer(GL_ARRAY_BUFFER, mColorBuffer);
			glColorPointer(3, GL_FLOAT, 0, 0);
		}

		glBindBuffer(GL_ARRAY_BUFFER, mNormalBuffer);
		glNormalPointer(GL_FLOAT, 0, 0);

		if (indexes.size() > 0) {
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer);
			glDrawElements(meshType, elementCount, GL_UNSIGNED_INT, NULL);
		} else {
			glDrawArrays(meshType, 0, elementCount);
		}
		glLineWidth(1.0f);

		glDisableClientState(GL_NORMAL_ARRAY);
		glDisable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glColor3f(0.3f, 0.3f, 0.3f);
		if(wireframe){
			if (indexes.size() > 0) {
				glDrawElements(meshType, elementCount, GL_UNSIGNED_INT, 0);
			} else {
				glDrawArrays(meshType, 0, elementCount);
			}
		}
		if (showParticles&&particles.size() > 0) {
			glColor3f(0.3f, 1.0f, 0.3f);
			glPointParameteri(GL_POINT_SMOOTH,GL_TRUE);
			glPointSize(4.0f);
			glBindBuffer(GL_ARRAY_BUFFER, mParticleBuffer);
			glVertexPointer(3, GL_FLOAT, 0, 0);
			glDrawArrays(GL_POINTS, 0, particles.size());
		}

		if (lines.size() > 0) {
			glColor3f(0.5f, 0.5f, 0.5f);
			glEnable(GL_LINE_SMOOTH);
			glLineWidth(1.0f);
			glBindBuffer(GL_ARRAY_BUFFER, mParticleBuffer);
			glVertexPointer(3, GL_FLOAT, 0, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mLineBuffer);
			glDrawElements(GL_LINES, lines.size(), GL_UNSIGNED_INT, NULL);
		}

		glEnable(GL_LIGHTING);
		if (colorEnabled)
			glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_INDEX_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	}
}
void Mesh::updateGL() {
	elementCount = 0;
	if (vertexes.size() > 0) {
		if (glIsBuffer(mVertexBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mVertexBuffer);
		glGenBuffers(1, &mVertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffer);
		if (glIsBuffer(mVertexBuffer) == GL_FALSE)
			throw "Error: Unable to create vertex buffer";
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * vertexes.size(),
				&vertexes[0], GL_STATIC_DRAW);
		if (GL_NO_ERROR != glGetError())
			throw "Error: Unable to upload vertex buffer data";
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	if (particles.size() > 0) {
		if (glIsBuffer(mParticleBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mParticleBuffer);
		glGenBuffers(1, &mParticleBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mParticleBuffer);
		if (glIsBuffer(mVertexBuffer) == GL_FALSE)
			throw "Error: Unable to create particle buffer";
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * particles.size(),
				&particles[0], GL_STATIC_DRAW);
		if (GL_NO_ERROR != glGetError())
			throw "Error: Unable to upload particle buffer data";
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
	if(lines.size()>0){
		std::cout<<"Update lines "<<lines.size()<<std::endl;
		// clear old buffer
		if (glIsBuffer(mLineBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mLineBuffer);
		// gen new buffer
		glGenBuffers(1, &mLineBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mLineBuffer);
		if (glIsBuffer(mLineBuffer) == GL_FALSE)
			throw "Error: Unable to create index buffer";
		// upload data
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * lines.size(),
				&lines[0], GL_STATIC_DRAW); // upload data
		if (GL_NO_ERROR != glGetError())
			throw "Error: Unable to upload index buffer data";
		// release buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		std::cout<<"Disable Lines "<<std::endl;
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

		elementCount = indexes.size();
		// release buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	} else {
		elementCount = vertexes.size();
	}
	if (particleNormals.size() > 0) {
		if (glIsBuffer(mParticleNormalBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mParticleNormalBuffer);

		glGenBuffers(1, &mParticleNormalBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mParticleNormalBuffer);
		if (glIsBuffer(mParticleNormalBuffer) == GL_FALSE)
			throw "Error: Unable to create particle normal buffer";

		glBufferData(GL_ARRAY_BUFFER,
				sizeof(GLfloat) * 3 * particleNormals.size(),
				&particleNormals[0], GL_STATIC_DRAW);
		if (GL_NO_ERROR != glGetError())
			throw "Error: Unable to upload particle normal buffer data";

		glBindBuffer(GL_ARRAY_BUFFER, 0);
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
