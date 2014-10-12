/*
 * Mesh.cc
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#include "Mesh.h"
#include "ImageSciUtil.h"
#include <openvdb/openvdb.h>

#include <openvdb/util/Util.h>
#include <vector>
#include <list>
#include "ply_io.h"
namespace imagesci {

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

char* elemNames[]={"vertex","face","normal"};

typedef struct _plyFace {
	unsigned char nverts;    // number of vertex indices in list
	int *verts;              // vertex index list
	_plyFace() {
		nverts = 0;
		verts = NULL;
	}
} plyFace;
typedef struct _plyFaceTexutre {
	unsigned char nverts;    // number of vertex indices in list
	int *verts;              // vertex index list
	unsigned char uvcount;
	float* uvs;
	_plyFaceTexutre(){
		nverts = 0;
		uvs=NULL;
		verts = NULL;
		uvcount = 6;
	}
} plyFaceTexutre;

PlyProperty vertProps[] = { // property information for a vertex
		{ "x", Float32, Float32, static_cast<int>(offsetof(plyVertex, x)), 0, 0,
				0, 0 },
		{ "y", Float32, Float32,
				static_cast<int>(offsetof(plyVertex, x) + sizeof(float)), 0, 0,
				0, 0 },
				{ "z", Float32, Float32,
				static_cast<int>(offsetof(plyVertex, x) + sizeof(float)
						+ sizeof(float)), 0, 0, 0, 0 }, { "red", Uint8, Uint8,
				static_cast<int>(offsetof(plyVertex, red)), 0, 0, 0, 0 }, {
				"green", Uint8, Uint8, static_cast<int>(offsetof(plyVertex,
						green)), 0, 0, 0, 0 }, { "blue", Uint8, Uint8,
				static_cast<int>(offsetof(plyVertex, blue)), 0, 0, 0, 0 }, {
				"alpha", Uint8, Uint8, static_cast<int>(offsetof(plyVertex,
						alpha)), 0, 0, 0, 0 }, };

PlyProperty faceProps[] = { // property information for a face
	{ "vertex_indices", Int32, Int32, static_cast<int>(offsetof(plyFace, verts)), 1, Uint8, Uint8, static_cast<int>(offsetof(plyFace, nverts)) },
	{ "vertex_indices", Int32, Int32, static_cast<int>(offsetof(plyFaceTexutre, verts)), 1, Uint8, Uint8, static_cast<int>(offsetof(plyFaceTexutre, nverts)) },
	{ "texcoord", Float32, Float32, static_cast<int>(offsetof(plyFaceTexutre, uvs)), 1, Uint8, Uint8, static_cast<int>(offsetof(plyFaceTexutre, uvcount)) },
};

Mesh::Mesh() :
		mGL(),
		mQuadIndexCount(0),
		mParticleCount(0),
		mTriangleIndexCount(0),
		mPose(openvdb::math::Mat4f::identity()),
		mQuadCount(0),mTriangleCount(0) {
}
bool Mesh::save(const std::string& f) {
	std::cout<<"Saving "<<f<<" ... ";
	int i, j, idx;
	const char* fileName = f.c_str();
	bool usingTexture=(uvMap.size()>0);

	PlyFile *ply;

	// Get input and check data
	ply = open_for_writing_ply(fileName, 2, elemNames, PLY_ASCII);

	if (ply == NULL){
		std::cout<<"Failed. "<<std::endl;
		return false;
	}

	// compute colors, if any
	int numPts = mVertexes.size();

	int numPolys =mQuadIndexes.size()/4+mTriIndexes.size()/3;

	std::vector<unsigned char> pointColors;

	if (mColors.size() > 0) {
		size_t inc=0;
		pointColors.resize(3 * mColors.size());
		for (i = 0; i < numPts; i++) {
			Vec3s d = mColors[i];
			pointColors[inc++] = (unsigned char) clamp(d[0] * 255.0f, 0.0f,
					255.0f);
			pointColors[inc++] = (unsigned char) clamp(d[1] * 255.0f, 0.0f,
					255.0f);
			pointColors[inc++] = (unsigned char) clamp(d[2] * 255.0f, 0.0f,
					255.0f);
		}
	}
	// describe what properties go into the vertex and face elements
	element_count_ply(ply, "vertex", numPts);
	ply_describe_property(ply, "vertex", &vertProps[0]);
	ply_describe_property(ply, "vertex", &vertProps[1]);
	ply_describe_property(ply, "vertex", &vertProps[2]);
	if (mColors.size() > 0) {
		ply_describe_property(ply, "vertex", &vertProps[3]);
		ply_describe_property(ply, "vertex", &vertProps[4]);
		ply_describe_property(ply, "vertex", &vertProps[5]);
	}
	element_count_ply(ply, "face", numPolys);

	if (usingTexture){
		ply_describe_property(ply, "face", &faceProps[1]);
		ply_describe_property(ply, "face", &faceProps[2]);
	} else {
		ply_describe_property(ply, "face", &faceProps[0]);
	}

	// write a comment and an object information field
	append_comment_ply(ply, "PLY File");
	if (usingTexture){
		std::string comment = "TextureFile texture.png";
		append_comment_ply(ply, (char*)comment.c_str());
	}
	append_obj_info_ply(ply, "ImageSci");

	// complete the header
	header_complete_ply(ply);

	// set up and write the vertex elements
	plyVertex vert;
	put_element_setup_ply(ply, "vertex");

	for (i = 0; i < numPts; i++) {
		Vec3s pt = mVertexes[i];
		vert.x[0] = pt[0];
		vert.x[1] = pt[1];
		vert.x[2] = pt[2];
		if (pointColors.size()>0) {
			idx = 3 * i;
			vert.red = pointColors[idx];
			vert.green = pointColors[idx+1];
			vert.blue = pointColors[idx+2];
		}
		put_element_ply(ply, (void *) &vert);
	}
	// set up and write the face elements
	plyFace face;
	plyFaceTexutre faceT;
	int verts[256];
	Vec2s uvs[3];
	face.verts = verts;
	faceT.verts = verts;
	faceT.uvs = (float*)uvs;
	put_element_setup_ply(ply, "face");
	if (usingTexture){
		int sz=mQuadIndexes.size()/4;
		for (int i = 0; i < sz; i++) {
			faceT.nverts =4;
			faceT.uvcount=8;
			for (j = 0; j < 4; j++) {
				faceT.verts[j] = mQuadIndexes[4 * i + j];
				uvs[j] = uvMap[4*i+j];
			}
			put_element_ply(ply, (void *) &faceT);
		}
		sz=mTriIndexes.size()/3;
		for (int i = 0; i < sz; i++) {
			faceT.nverts =3;
			faceT.uvcount=6;
			for (j = 0; j < 3; j++) {
				faceT.verts[j] = mTriIndexes[3 * i + j];
				uvs[j] = uvMap[3 * i + j];
			}
			put_element_ply(ply, (void *) &faceT);
		}
	} else {
		int sz=mQuadIndexes.size()/4;
		for (int i = 0; i < sz; i++) {
			for (j = 0; j < 4; j++) {
				face.nverts =4;
				face.verts[j] = mQuadIndexes[4 * i + j];
			}
			put_element_ply(ply, (void *) &face);
		}
		sz=mTriIndexes.size()/3;
		for (int i = 0; i < sz; i++) {
			for (j = 0; j < 3; j++) {
				face.nverts =3;
				face.verts[j] = mTriIndexes[3 * i + j];
			}
			put_element_ply(ply, (void *) &face);
		}
	}
	// close the PLY file
	close_ply(ply);
	std::cout<<"Done."<<std::endl;
	return true;
}
bool Mesh::openGrid(const std::string& fileName) {
	openvdb::io::File file(fileName);
	file.open();
	openvdb::GridPtrVecPtr grids = file.getGrids();

	openvdb::GridPtrVec allGrids;
	allGrids.insert(allGrids.end(), grids->begin(), grids->end());
	GridBase::Ptr ptr = allGrids[0];
	if(ptr.get()!=nullptr){
		create(boost::static_pointer_cast<FloatGrid>(ptr));
		return true;
	} else return false;
}
bool Mesh::openMesh(const std::string& file) {
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
		return false;
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
		return false;
	}
	boost::shared_ptr<Mesh> mesh = boost::shared_ptr<Mesh>(new Mesh());

	// Check for optional attribute data. We can handle intensity; and the
	// triplet red, green, blue.
	bool RGBPointsAvailable = false;
	this->mTriIndexes.clear();
	this->mQuadIndexes.clear();
	this->mFaces.clear();
	this->mVertexes.clear();
	this->mParticles.clear();
	this->mParticleNormals.clear();
	this->mVertexNormals.clear();
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
			this->mVertexes.resize(numPts,Vec3s(0.0f));
			// Setup to read the PLY elements
			ply_get_property(ply, elemName, &vertProps[0]);
			ply_get_property(ply, elemName, &vertProps[1]);
			ply_get_property(ply, elemName, &vertProps[2]);

			if (RGBPointsAvailable) {
				this->mColors.resize(numPts);
				ply_get_property(ply, elemName, &vertProps[3]);
				ply_get_property(ply, elemName, &vertProps[4]);
				ply_get_property(ply, elemName, &vertProps[5]);
			}
			for (j = 0; j < numPts; j++) {
				get_element_ply(ply, &vertex);
				this->mVertexes[j] = Vec3s(vertex.x[0], vertex.x[1], vertex.x[2]);
				if (RGBPointsAvailable) {
					this->mColors[j] = Vec3s(vertex.red / 255.0f,
							vertex.green / 255.0f, vertex.blue / 255.0f);
				}
			}
		}			//if vertex
		else if (elemName && !strcmp("face", elemName)) {
			// Create a polygonal array
			numPolys = numElems;
			// Get the face properties
			ply_get_property(ply, elemName, &faceProps[0]);
			for (j = 0; j < numPolys; j++) {

				//grab and element from the file
				face.verts = &verts[0];
				get_element_ply(ply, &face);

				if(face.nverts==4){
					for (k = 0; k < face.nverts; k++) {
						this->mQuadIndexes.push_back(face.verts[k]);
					}
					this->mFaces.push_back(openvdb::Vec4I(face.verts[0],face.verts[1],face.verts[2],face.verts[3]));
				} else if(face.nverts==3){
					for (k = 0; k < face.nverts; k++) {
						this->mTriIndexes.push_back(face.verts[k]);
					}
					this->mFaces.push_back(openvdb::Vec4I(face.verts[0],face.verts[1],face.verts[2],openvdb::util::INVALID_IDX));
				}
			}
		}							//if face

		free(elist[i]); //allocated by ply_open_for_reading

	} //for all elements of the PLY file
	free(elist); //allocated by ply_open_for_reading
	close_ply(ply);
	if (this->mVertexes.size() > 0) {
		this->updateBBox();
		return true;
	} else {
		return false;
	}
}
void Mesh::create(openvdb::tools::VolumeToMesh& mesher,openvdb::FloatGrid::Ptr grid){
	// Copy points and generate point normals.
	openvdb::math::GenericMap map(grid->transform());
	mVertexes.clear();
	mVertexNormals.clear();
	mFaces.clear();
	mTriIndexes.clear();
	mQuadIndexes.clear();
	mVertexes.resize(mesher.pointListSize());
	Index64 N = mesher.pointListSize();
	for (Index64 n = 0; n < N; ++n) {
		mVertexes[n] = mesher.pointList()[n];//map.applyInverseMap(
	}
	// Copy primitives
	openvdb::tools::PolygonPoolList& polygonPoolList = mesher.polygonPoolList();
	for (Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
		const openvdb::tools::PolygonPool& polygons = polygonPoolList[n];
		//std::cout << "Polygon " << polygons.numTriangles() << " "<< polygons.numQuads() << std::endl;
		for (Index64 i = 0, I = polygons.numQuads(); i < I; ++i) {
			const openvdb::Vec4I& quad = polygons.quad(i);
			mFaces.push_back(openvdb::Vec4I(quad[3],quad[2],quad[1],quad[0]));
			mQuadIndexes.push_back(quad[3]);
			mQuadIndexes.push_back(quad[2]);
			mQuadIndexes.push_back(quad[1]);
			mQuadIndexes.push_back(quad[0]);
		}
		for (Index64 i = 0, I = polygons.numTriangles(); i < I; ++i) {
			const openvdb::Vec3I& quad = polygons.triangle(i);
			mFaces.push_back(openvdb::Vec4I(quad[2],quad[1],quad[0],openvdb::util::INVALID_IDX));
			mTriIndexes.push_back(quad[2]);
			mTriIndexes.push_back(quad[1]);
			mTriIndexes.push_back(quad[0]);
		}
	}
	updateVertexNormals(16);
	updateBBox();
}
void Mesh::create(FloatGrid::Ptr grid) {
	openvdb::tools::VolumeToMesh mesher(0.0);
	mesher(*grid);
	// Copy points and generate point normals.
	openvdb::math::GenericMap map(grid->transform());
	mVertexes.clear();
	mVertexNormals.clear();
	mFaces.clear();
	mTriIndexes.clear();
	mQuadIndexes.clear();
	mVertexes.resize(mesher.pointListSize());
	Index64 N = mesher.pointListSize();
	for (Index64 n = 0; n < N; ++n) {
		mVertexes[n] = map.applyInverseMap(mesher.pointList()[n]);
	}
	// Copy primitives
	openvdb::tools::PolygonPoolList& polygonPoolList = mesher.polygonPoolList();
	for (Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
		const openvdb::tools::PolygonPool& polygons = polygonPoolList[n];
		//std::cout << "Polygon " << polygons.numTriangles() << " "<< polygons.numQuads() << std::endl;
		for (Index64 i = 0, I = polygons.numQuads(); i < I; ++i) {
			const openvdb::Vec4I& quad = polygons.quad(i);
			mFaces.push_back(openvdb::Vec4I(quad[3],quad[2],quad[1],quad[0]));
			mQuadIndexes.push_back(quad[3]);
			mQuadIndexes.push_back(quad[2]);
			mQuadIndexes.push_back(quad[1]);
			mQuadIndexes.push_back(quad[0]);
		}
		for (Index64 i = 0, I = polygons.numTriangles(); i < I; ++i) {
			const openvdb::Vec3I& quad = polygons.triangle(i);
			mFaces.push_back(openvdb::Vec4I(quad[2],quad[1],quad[0],openvdb::util::INVALID_IDX));
			mTriIndexes.push_back(quad[2]);
			mTriIndexes.push_back(quad[1]);
			mTriIndexes.push_back(quad[0]);
		}
	}
	updateVertexNormals(16);
	updateBBox();
}
void Mesh::updateVertexNormals(int SMOOTH_ITERATIONS,float DOT_TOLERANCE){

	Index32 sz=mTriIndexes.size();
	Vec3s pt,norm;
	mVertexNormals.resize(mVertexes.size(),Vec3f(0.0f));
	for (Index32 i = 0; i < sz; i += 3) {
		Vec3s v1 = mVertexes[mTriIndexes[i]];
		Vec3s v2 = mVertexes[mTriIndexes[i + 1]];
		Vec3s v3 = mVertexes[mTriIndexes[i + 2]];
		norm=(v2-v1).cross(v3-v1);
		mVertexNormals[mTriIndexes[i]]+=norm;
		mVertexNormals[mTriIndexes[i+1]]+=norm;
		mVertexNormals[mTriIndexes[i+2]]+=norm;
	}
	sz=mQuadIndexes.size();
	for (int i = 0; i < sz; i += 4) {
		Vec3s v1 = mVertexes[mQuadIndexes[i]];
		Vec3s v2 = mVertexes[mQuadIndexes[i + 1]];
		Vec3s v3 = mVertexes[mQuadIndexes[i + 2]];
		Vec3s v4 = mVertexes[mQuadIndexes[i + 3]];
		norm =(v1-pt).cross(v2-pt);
		norm+=(v2-pt).cross(v3-pt);
		norm+=(v3-pt).cross(v4-pt);
		norm+=(v4-pt).cross(v1-pt);
		mVertexNormals[mQuadIndexes[i]]+=norm;
		mVertexNormals[mQuadIndexes[i+1]]+=norm;
		mVertexNormals[mQuadIndexes[i+2]]+=norm;
		mVertexNormals[mQuadIndexes[i+3]]+=norm;
	}
	for (Vec3s& norm : mVertexNormals) {
		norm.normalize(1E-6f);
	}

	if(SMOOTH_ITERATIONS>0){
		int vertCount=mVertexes.size();

		std::vector<Vec3f> tmp(vertCount);

		std::vector<std::list<int>> vertNbrs(vertCount);
		int indexCount = mQuadIndexes.size();
		int v1, v2, v3,v4;
		Vec3s nnorm;
		for (int i = 0; i < indexCount; i += 4){
			int v1 = mQuadIndexes[i];
			int v2 = mQuadIndexes[i + 1];
			int v3 = mQuadIndexes[i + 2];
			int v4 = mQuadIndexes[i + 3];

			vertNbrs[v1].push_back(v2);
			vertNbrs[v2].push_back(v3);
			vertNbrs[v3].push_back(v1);

			vertNbrs[v3].push_back(v4);
			vertNbrs[v4].push_back(v1);
			vertNbrs[v1].push_back(v3);
		}
		Vec3f avg;
		for(int iter=0;iter<SMOOTH_ITERATIONS;iter++){
			for(int i=0;i<vertCount;i++){
				norm=mVertexNormals[i];
				avg=Vec3f(0.0f);
				for(int nbr:vertNbrs[i]){
					nnorm=mVertexNormals[nbr];;
					if(norm.dot(nnorm)>DOT_TOLERANCE){
						avg+=nnorm;
					} else {
						avg+=norm;
					}
				}
				avg.normalize();
				tmp[i]=avg;
			}
			mVertexNormals=tmp;
		}
	}
}
float Mesh::estimateVoxelSize(int stride) {
	float avg = 0.0f;
	int count = 0;
	float maxLength = 0.0f;
	int sz = mTriIndexes.size();
		for (int i = 0; i < sz; i += 3 * stride) {
			Vec3s v1 = mVertexes[mTriIndexes[i]];
			Vec3s v2 = mVertexes[mTriIndexes[i + 1]];
			Vec3s v3 = mVertexes[mTriIndexes[i + 2]];
			float e1 = (v1 - v2).length();
			float e2 = (v1 - v3).length();
			float e3 = (v2 - v3).length();
			maxLength = std::max(std::max(e1, e2), std::max(maxLength, e3));
			avg += e1 + e2 + e3;
			count += 3;
		}
		sz=mQuadIndexes.size();
		for (int i = 0; i < sz; i += 4 * stride) {
			Vec3s v1 = mVertexes[mQuadIndexes[i]];
			Vec3s v2 = mVertexes[mQuadIndexes[i + 1]];
			Vec3s v3 = mVertexes[mQuadIndexes[i + 2]];
			Vec3s v4 = mVertexes[mQuadIndexes[i + 3]];
			float e1 = (v1 - v2).length();
			float e2 = (v2 - v3).length();
			float e3 = (v3 - v4).length();
			float e4 = (v4 - v1).length();

			maxLength = std::max(maxLength,
					std::max(std::max(e1, e2), std::max(e3, e4)));
			avg += e1 + e2 + e3 + e4;
			count += 4;
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
	const int SAMPLE_STRIDE=8;
	Index32 SZ=mVertexes.size();
	for (Index32 idx=0;idx<SZ;idx+=SAMPLE_STRIDE) {
		Vec3s& pt=mVertexes[idx];
		minPt[0] = std::min(minPt[0], pt[0]);
		minPt[1] = std::min(minPt[1], pt[1]);
		minPt[2] = std::min(minPt[2], pt[2]);

		maxPt[0] = std::max(maxPt[0], pt[0]);
		maxPt[1] = std::max(maxPt[1], pt[1]);
		maxPt[2] = std::max(maxPt[2], pt[2]);
	}
	mBoundingBox = openvdb::math::BBox<openvdb::Vec3d>(minPt, maxPt);
	return mBoundingBox;
}
void Mesh::scale(float sc) {
	for (Vec3s& pt : mVertexes) {
		pt *= sc;
	}
	mBoundingBox.min() *= static_cast<double>(sc);
	mBoundingBox.max() *= static_cast<double>(sc);

}
void Mesh::mapIntoBoundingBox(float voxelSize) {
	Vec3s minPt = mBoundingBox.min();
	for (Vec3s& pt : mVertexes) {
		pt = (pt - minPt) / voxelSize;
	}
}
void Mesh::mapOutOfBoundingBox(float voxelSize) {
	Vec3s minPt = mBoundingBox.min();
	for (Vec3s& pt : mVertexes) {
		pt = pt * voxelSize + minPt;
	}
}
void Mesh::draw() {
	glBindVertexArray (mGL.mVao);
	if (mGL.mVertexBuffer > 0) {
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mVertexBuffer);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	}
	if (mGL.mNormalBuffer > 0) {
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mNormalBuffer);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	}
	if (mGL.mColorBuffer > 0) {
		glEnableVertexAttribArray(2);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mColorBuffer);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);
	}
	if (mQuadIndexCount > 0) {
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mGL.mQuadIndexBuffer);
		glDrawElements(GL_TRIANGLES, mQuadIndexCount, GL_UNSIGNED_INT, NULL);
	}  else if(mQuadCount>0){
		glDrawArrays(GL_TRIANGLES, 0, mQuadCount);
	}
	if (mTriangleIndexCount > 0) {
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mGL.mTriIndexBuffer);
		glDrawElements(GL_TRIANGLES, mTriangleIndexCount, GL_UNSIGNED_INT, NULL);
	} else if(mTriangleCount>0){
		glDrawArrays(GL_TRIANGLES, 0, mTriangleCount);
	}

	glBindVertexArray (0);
	glBindBuffer(GL_ARRAY_BUFFER,0);
}

void Mesh::updateGL() {
	mQuadCount=0;
	mTriangleCount=0;
	mTriangleIndexCount=0;
	mQuadIndexCount=0;
	mParticleCount=0;

	if(mGL.mVao==0)glGenVertexArrays (1, &mGL.mVao);

	if (mVertexes.size() > 0) {
		if (glIsBuffer(mGL.mVertexBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mVertexBuffer);
		glGenBuffers(1, &mGL.mVertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mVertexBuffer);
		if (glIsBuffer(mGL.mVertexBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create vertex buffer");
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * mVertexes.size(),
				&mVertexes[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	if (mParticles.size() > 0) {
		if (glIsBuffer(mGL.mParticleBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mParticleBuffer);
		glGenBuffers(1, &mGL.mParticleBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mParticleBuffer);
		if (glIsBuffer(mGL.mParticleBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create particle buffer");
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * mParticles.size(),
				&mParticles[0], GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		mParticleCount=mParticles.size();
	}
	if (mColors.size() > 0) {
		if (glIsBuffer(mGL.mColorBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mColorBuffer);

		glGenBuffers(1, &mGL.mColorBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mColorBuffer);
		if (glIsBuffer(mGL.mColorBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create color buffer");

		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * mColors.size(),
				&mColors[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	if(mLines.size()>0){
		std::cout<<"Update lines "<<mLines.size()<<std::endl;
		// clear old buffer
		if (glIsBuffer(mGL.mLineBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mLineBuffer);
		// gen new buffer
		glGenBuffers(1, &mGL.mLineBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mLineBuffer);
		if (glIsBuffer(mGL.mLineBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create index buffer");
		// upload data
		glBufferData(GL_ARRAY_BUFFER,  sizeof(GLfloat) * 3  * mLines.size(),
				&mLines[0], GL_STATIC_DRAW); // upload data
		// release buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		std::cout<<"Disable Lines "<<std::endl;
	}
	if (mTriIndexes.size() > 0) {
		// clear old buffer
		if (glIsBuffer(mGL.mTriIndexBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mTriIndexBuffer);

		// gen new buffer
		glGenBuffers(1, &mGL.mTriIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mGL.mTriIndexBuffer);
		if (glIsBuffer(mGL.mTriIndexBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create index buffer");

		// upload data
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * mTriIndexes.size(),
				&mTriIndexes[0], GL_STATIC_DRAW); // upload data


		mTriangleIndexCount = mTriIndexes.size();
		// release buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}
	if (mQuadIndexes.size() > 0) {
		// clear old buffer
		if (glIsBuffer(mGL.mQuadIndexBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mQuadIndexBuffer);

		// gen new buffer
		glGenBuffers(1, &mGL.mQuadIndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mGL.mQuadIndexBuffer);
		if (glIsBuffer(mGL.mQuadIndexBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create index buffer");

		// upload data
		int sz=mQuadIndexes.size();
		std::vector<GLuint> tmp(12*(mQuadIndexes.size()/4));
		int offset=0;
		for(unsigned int i=0;i<sz;i+=4){
			tmp[offset++]=mQuadIndexes[i+1];
			tmp[offset++]=mQuadIndexes[i+2];
			tmp[offset++]=mQuadIndexes[i+0];


			tmp[offset++]=mQuadIndexes[i+2];
			tmp[offset++]=mQuadIndexes[i+3];
			tmp[offset++]=mQuadIndexes[i+1];

			tmp[offset++]=mQuadIndexes[i+0];
			tmp[offset++]=mQuadIndexes[i+1];
			tmp[offset++]=mQuadIndexes[i+3];


			tmp[offset++]=mQuadIndexes[i+3];
			tmp[offset++]=mQuadIndexes[i];
			tmp[offset++]=mQuadIndexes[i+2];

		}
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * tmp.size(),
				&tmp[0], GL_STATIC_DRAW); // upload data


		mQuadIndexCount = tmp.size();
		// release buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}
	if (mParticleNormals.size() > 0) {
		if (glIsBuffer(mGL.mParticleNormalBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mParticleNormalBuffer);

		glGenBuffers(1, &mGL.mParticleNormalBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mParticleNormalBuffer);
		if (glIsBuffer(mGL.mParticleNormalBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create particle normal buffer");

		glBufferData(GL_ARRAY_BUFFER,
				sizeof(GLfloat) * 3 * mParticleNormals.size(),
				&mParticleNormals[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	if (mVertexNormals.size() > 0) {
		if (glIsBuffer(mGL.mNormalBuffer) == GL_TRUE)
			glDeleteBuffers(1, &mGL.mNormalBuffer);

		glGenBuffers(1, &mGL.mNormalBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, mGL.mNormalBuffer);
		if (glIsBuffer(mGL.mNormalBuffer) == GL_FALSE)
			throw Exception("Error: Unable to create normal buffer");

		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * mVertexNormals.size(),
				&mVertexNormals[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

}
Mesh::~Mesh() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
