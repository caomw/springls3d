/*
 * SpringLevelSet.cc
 *
 *  Created on: Aug 26, 2014
 *      Author: blake
 */
#include "SpringLevelSet.h"
#include <openvdb/Grid.h>
#include <openvdb/util/Util.h>
#include <openvdb/math/Stencils.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/LevelSetAdvect.h>

#include <openvdb/openvdb.h>
namespace imagesci {
using namespace openvdb;
using namespace openvdb::math;
using namespace openvdb::tools;

typedef DiscreteField<openvdb::VectorGrid> VelocityField;
typedef LevelSetAdvection<openvdb::FloatGrid, VelocityField> AdvectionTool;
const float SpringLevelSet::NEAREST_NEIGHBOR_RANGE = 1.5f;
const float SpringLevelSet::PARTICLE_RADIUS = 0.05f;
const float SpringLevelSet::MAX_VEXT = 0.5f;
const int SpringLevelSet::MAX_NEAREST_NEIGHBORS = 2;
const float SpringLevelSet::FILL_DISTANCE = 0.3f;
const float SpringLevelSet::CLEAN_DISTANCE = 0.625f;
const float SpringLevelSet::SHARPNESS = 5.0f;
const float SpringLevelSet::SPRING_CONSTANT = 0.3f;
const float SpringLevelSet::RELAX_TIMESTEP = 0.1f;
const float SpringLevelSet::MIN_AREA = 0.05f;
const float SpringLevelSet::MAX_AREA = 2.0 * 2.0f;
const float SpringLevelSet::MIN_ASPECT_RATIO = 0.1f;
std::ostream& operator<<(std::ostream& ostr, const SpringlNeighbor& classname) {
	ostr << "{" << classname.springlId << "|"
			<< static_cast<int>(classname.edgeId) << ":" << std::setprecision(4)
			<< classname.distance << "}";
	return ostr;
}

int Springl::size() const {
	return ((mesh->faces[id][3] == openvdb::util::INVALID_IDX) ? 3 : 4);
}

float Springl::distanceToParticle(const openvdb::Vec3s& pt) {
	return ((particle()) - pt).length();
}

float Springl::distanceToParticleSqr(const openvdb::Vec3s& pt) {
	return ((particle()) - pt).lengthSqr();
}
float Springl::distanceToFace(const openvdb::Vec3s& pt) {
	return std::sqrt(distanceToFaceSqr(pt));
}

float Springl::distanceToFaceSqr(const openvdb::Vec3s& pt) {
	Vec3s closest;
	if (size() == 3) {
		return DistanceToTriangleSqr(pt, (*this)[0], (*this)[1], (*this)[2],
				&closest);
	} else {
		return DistanceToQuadSqr(pt, (*this)[0], (*this)[1], (*this)[2],
				(*this)[3], normal(), &closest);
	}
}
float Springl::distanceEdgeSqr(const openvdb::Vec3s& pt, int e) {
	return DistanceToEdgeSqr(pt, (*this)[e], (*this)[(e + 1) % size()]);
}
float Springl::distanceEdge(const openvdb::Vec3s& pt, int e) {
	return std::sqrt(
			DistanceToEdgeSqr(pt, (*this)[e], (*this)[(e + 1) % size()]));
}

openvdb::Vec3s Springl::computeCentroid() const {
	openvdb::Vec3s centroid = openvdb::Vec3s(0.0f, 0.0f, 0.0f);
	int K = size();
	for (int k = 0; k < K; k++) {
		centroid += (*this)[k];
	}
	centroid = (1.0 / K) * centroid;
	return centroid;
}
openvdb::Vec3s Springl::computeNormal(const float eps) const {
	openvdb::Vec3s norm(0.0f);
	int K = size();
	openvdb::Vec3s pt = particle();
	for (int k = 0; k < K; k++) {
		norm += ((*this)[k] - pt).cross((*this)[(k + 1) % K] - pt);
	}

	norm.normalize(eps);
	return norm;
}
float Springl::area() const {
	openvdb::Vec3s norm(0.0f);
	int K = size();
	for (int k = 0; k < K; k++) {
		norm += ((*this)[k] - (particle())).cross(
				(*this)[(k + 1) % K] - (particle()));
	}
	return 0.5f * norm.length();
}
Vec3s Constellation::closestPointOnEdge(const Vec3s& start,
		const SpringlNeighbor& ci) {
	Vec3s pt;
	Springl& nbr = springls[ci.springlId];
	DistanceToEdgeSqr(start, nbr[ci.edgeId], nbr[(ci.edgeId + 1) % nbr.size()],
			&pt);
	return pt;
}
void SpringLevelSet::draw(bool colorEnabled, bool wireframe, bool particles,
		bool particleNormals) {
	/*
	 if (isoSurface.get() != nullptr) {
	 glColor3f(0.8f,0.3f, 0.3f);
	 isoSurface->draw(colorEnabled, wireframe, particles,
	 particleNormals);
	 }
	 */
	glColor3f(0.3f, 0.3f, 0.8f);
	constellation.draw(colorEnabled, wireframe, particles, particleNormals);

}

openvdb::Vec3s& SpringLevelSet::GetParticle(const openvdb::Index32 id) {
	return (constellation.springls[id].particle());
}
openvdb::Vec3s& SpringLevelSet::GetParticleNormal(const openvdb::Index32 id) {
	return (constellation.springls[id].normal());
}
openvdb::Vec3s& SpringLevelSet::GetSpringlVertex(const openvdb::Index32 id,
		const int i) {
	return constellation.springls[id][i];
}
openvdb::Vec3s& SpringLevelSet::GetSpringlVertex(const openvdb::Index32 id) {
	return constellation.vertexes[id];
}
Springl& SpringLevelSet::GetSpringl(const openvdb::Index32 id) {
	return constellation.springls[id];
}

void RelaxOperation::init(SpringLevelSet& mGrid) {
	mGrid.constellation.vertexDisplacement.resize(
			mGrid.constellation.getNumVertexes());
}
void RelaxOperation::apply(Springl& springl, SpringLevelSet& mGrid, double dt) {
	int K = springl.size();
	for (int k = 0; k < K; k++) {
		springl[k] = mGrid.constellation.vertexDisplacement[springl.offset + k];
	}
}
void RelaxOperation::compute(Springl& springl, SpringLevelSet& mGrid,
		double t) {
	float w, len;
	Vec3s tanget;
	Vec3s dir;
	int K = springl.size();
	std::vector<Vec3s> vertexVelocity(K);
	std::vector<Vec3s> tangets(K);
	std::vector<float> springForce(K);
	std::vector<float> tangetLengths(K);
	Vec3s particlePt = springl.particle();
	Vec3s startVelocity = Vec3s(0);
	Vec3s resultantMoment = Vec3s(0);
	const float MAX_FORCE = 0.999f;
	Vec3s start;
	float dotProd;
	Vec3s pt2;
	for (int k = 0; k < K; k++) {
		std::list<SpringlNeighbor>& map = mGrid.GetNearestNeighbors(springl.id,
				k);
		start = springl[k];
		// edge from pivot to magnet
		tanget = (start - particlePt);
		tangetLengths[k] = tanget.length();
		if (tangetLengths[k] > 1E-6f) {
			tanget *= (1.0f / tangetLengths[k]);
		}
		tangets[k] = tanget;
		startVelocity = Vec3s(0);
		// Sum forces
		//unroll loop
		for (SpringlNeighbor ci : map) {
			//Closest point should be recomputed each time and does not need to be stored

			Springl& nbr = mGrid.GetSpringl(ci.springlId);
			DistanceToEdgeSqr(start, nbr[ci.edgeId],
					nbr[(ci.edgeId + 1) % nbr.size()], &pt2);
			dir = (pt2 - start);
			len = dir.length();
			w = ((len - 2 * SpringLevelSet::PARTICLE_RADIUS)
					/ (SpringLevelSet::MAX_VEXT
							+ 2 * SpringLevelSet::PARTICLE_RADIUS));

			w = atanh(MAX_FORCE * clamp(w, -1.0f, 1.0f));
			startVelocity += (w * dir);
		}
		if (map.size() > 0)
			startVelocity /= map.size();
		len = std::max(1E-6f, startVelocity.length());
		float t = SpringLevelSet::SHARPNESS * len;
		vertexVelocity[k] = SpringLevelSet::RELAX_TIMESTEP * startVelocity
				* (t / len);
		springForce[k] = SpringLevelSet::RELAX_TIMESTEP
				* SpringLevelSet::SPRING_CONSTANT
				* (2 * SpringLevelSet::PARTICLE_RADIUS - tangetLengths[k]);

		resultantMoment += vertexVelocity[k].cross(tangets[k]);
	}
	//std::cout<<"moment "<<resultantMoment<<" Normal "<<*springl.normal<<std::endl;

	openvdb::math::Mat3<float> rot = CreateAxisAngle(resultantMoment,
			-resultantMoment.length());

	//std::cout<<"Rotation\n"<<rot<<std::endl;

	//std::cout<<springl.id<<springl.offset<<" ROTATION "<<resultantMoment<<" "<<springForce[0]<<" "<<vertexVelocity[0]<<std::endl;
	for (int k = 0; k < K; k++) {
		start = springl[k] - particlePt;
		dotProd = std::max(
				start.length() + vertexVelocity[k].dot(tangets[k])
						+ springForce[k], 0.001f);
		start = dotProd * tangets[k];

		//disable rotation
		start = rot * start;
		mGrid.constellation.vertexDisplacement[springl.offset + k] = start
				+ particlePt;
	}
}
void NearestNeighborOperation::init(SpringLevelSet& mGrid) {
	NearestNeighborMap& map = mGrid.nearestNeighbors;
	map.clear();
	map.resize(mGrid.constellation.getNumVertexes(),
			std::list<SpringlNeighbor>());
}
void NearestNeighborOperation::compute(Springl& springl, SpringLevelSet& mGrid,
		double t) {
	const float D2 = SpringLevelSet::NEAREST_NEIGHBOR_RANGE
			* SpringLevelSet::NEAREST_NEIGHBOR_RANGE;

	openvdb::math::DenseStencil<openvdb::Int32Grid> stencil =
			openvdb::math::DenseStencil<openvdb::Int32Grid>(
					*mGrid.springlIndexGrid,
					ceil(SpringLevelSet::NEAREST_NEIGHBOR_RANGE));
	openvdb::Vec3s refPoint = springl.particle();

	stencil.moveTo(
			Coord(std::floor(refPoint[0] + 0.5f),
					std::floor(refPoint[1] + 0.5f),
					std::floor(refPoint[2] + 0.5f)));

	int sz = stencil.size();
	if (sz == 0)
		return;
	Index32 N = mGrid.constellation.getNumSpringls();
	std::vector<openvdb::Index32> stencilCopy;
	for (int i = 0; i < sz; i++) {
		openvdb::Index32 id = stencil.getValue(i);
		if (id >= N)
			continue;
		openvdb::Vec3s nbr = mGrid.GetParticle(id);
		float d = (refPoint - nbr).lengthSqr();
		if (id != springl.id) {
			stencilCopy.push_back(id);
		}
	}
	if (stencilCopy.size() == 0)
		return;
	std::sort(stencilCopy.begin(), stencilCopy.end());
	sz = stencilCopy.size();

	openvdb::Index32 last = -1;
	SpringlNeighbor bestNbr;
	std::vector<SpringlNeighbor> tmpRange;
	for (int k = 0; k < springl.size(); k++) {
		std::list<SpringlNeighbor>& mapList = mGrid.GetNearestNeighbors(
				springl.id, k);
		refPoint = springl[k];
		//
		last = -1;

		tmpRange.clear();
		for (int i = 0; i < sz; i++) {
			openvdb::Index32 nbrId = stencilCopy[i];
			if (nbrId == last)
				continue;
			Springl& snbr = mGrid.GetSpringl(nbrId);
			bestNbr = SpringlNeighbor(nbrId, -1, D2);
			for (int8_t n = 0; n < snbr.size(); n++) {
				float d = snbr.distanceEdgeSqr(refPoint, n);
				if (d <= bestNbr.distance) {
					bestNbr.edgeId = n;
					bestNbr.distance = d;
				}
			}
			if (bestNbr.edgeId >= 0)
				tmpRange.push_back(bestNbr);
			last = nbrId;
		}

		sort(tmpRange.begin(), tmpRange.end());
		for (int nn = 0, nmax = std::min(SpringLevelSet::MAX_NEAREST_NEIGHBORS,
				(int) tmpRange.size()); nn < nmax; nn++) {
			mapList.push_back(tmpRange[nn]);
		}
		//if (bestNbr.springlId >= 0 && bestNbr.springlId < N)mapList.push_back(bestNbr);
	}
}

void SpringLevelSet::updateNearestNeighbors(bool threaded) {
	using namespace openvdb;
	NearestNeighbors<openvdb::util::NullInterrupter> nn(*this);
	nn.process();

}
void SpringLevelSet::updateLines() {
	std::vector<Vec3s>& lines = constellation.lines;
	lines.clear();
	float d;
	Vec3s pt, qt;
	for (Index32 i = 0; i < constellation.getNumSpringls(); i++) {

		Springl& springl = constellation.springls[i];

		for (int k = 0; k < springl.size(); k++) {
			//std::cout <<i << "={"<<k<<":: ";
			for (SpringlNeighbor nbr : GetNearestNeighbors(i, k)) {
				pt = springl[k];
				lines.push_back(pt);
				qt = constellation.closestPointOnEdge(pt, nbr);
				lines.push_back(qt);
				//std::cout << nbr << " ";
			}

			//std::cout << "}" << std::endl;
		}
	}
}
void SpringLevelSet::relax(int iters) {
	Relax<openvdb::util::NullInterrupter> relax(*this);
	for (int iter = 0; iter < iters; iter++) {
		relax.process();
	}
}
void SpringLevelSet::evolve() {

	updateGradient();

	VelocityField grad(*gradient);
	AdvectionTool advect(*signedLevelSet, grad);
	advect.setSpatialScheme(openvdb::math::FIRST_BIAS);
	advect.setTemporalScheme(openvdb::math::TVD_RK2);
	advect.setTrackerSpatialScheme(openvdb::math::FIRST_BIAS);
	advect.setTrackerTemporalScheme(openvdb::math::TVD_RK2);
	int steps = advect.advect(0.0, 4.0);
	std::cout << "Evolution steps " << steps << std::endl;

}
void SpringLevelSet::updateUnsignedLevelSet() {
	openvdb::math::Transform::Ptr trans =
			openvdb::math::Transform::createLinearTransform(1.0f);
	using namespace openvdb::tools;
	using namespace openvdb;
	MeshToVolume<FloatGrid> mtol(trans, GENERATE_PRIM_INDEX_GRID);
	mtol.convertToUnsignedDistanceField(constellation.vertexes,
			constellation.faces, 2.5*float(LEVEL_SET_HALF_WIDTH));
	unsignedLevelSet = mtol.distGridPtr();
	unsignedLevelSet->setBackground(2.5*float(LEVEL_SET_HALF_WIDTH));
	springlIndexGrid = mtol.indexGridPtr();
}
double SpringLevelSet::distanceToConstellation(const Vec3s& pt) {
	openvdb::math::DenseStencil<openvdb::Int32Grid> stencil =
			openvdb::math::DenseStencil<openvdb::Int32Grid>(*springlIndexGrid,
					ceil(FILL_DISTANCE));
	stencil.moveTo(
			Coord((int) floor(pt[0] + 0.5f), (int) floor(pt[1] + 0.5f),
					(int) floor(pt[2] + 0.5f)));
	int sz = stencil.size();
	double levelSetValue = std::numeric_limits<float>::max();
	std::vector<Index32> stencilCopy;
	stencilCopy.clear();
	Index32 last = -1;
	Index32 springlsCount = constellation.getNumSpringls();
	for (unsigned int nn = 0; nn < sz; nn++) {
		openvdb::Index32 id = stencil.getValue(nn);
		if (id >= springlsCount)
			continue;
		stencilCopy.push_back(id);
	}
	sz = stencilCopy.size();
	sort(stencilCopy.begin(), stencilCopy.end());
	for (Index32 id : stencilCopy) {
		if (last != id) {
			float d = constellation.springls[id].distanceToFaceSqr(pt);
			if (d < levelSetValue) {
				levelSetValue = d;
			}
		}
		last = id;
	}
	return std::sqrt(levelSetValue);
}
void SpringLevelSet::updateSignedLevelSet() {
	openvdb::math::Transform::Ptr trans =
			openvdb::math::Transform::createLinearTransform(1.0);
	using namespace openvdb::tools;
	using namespace openvdb;
	MeshToVolume<FloatGrid> mtol(trans);
	mtol.convertToLevelSet(isoSurface.vertexes, isoSurface.faces,
			float(LEVEL_SET_HALF_WIDTH));
	signedLevelSet = mtol.distGridPtr();
}

void SpringLevelSet::updateGradient() {
	//gradient = openvdb::tools::gradient(*unsignedLevelSet);
	gradient = advectionForce(*unsignedLevelSet);

}
std::list<SpringlNeighbor>& SpringLevelSet::GetNearestNeighbors(
		openvdb::Index32 id, int8_t e) {
	return nearestNeighbors[constellation.springls[id].offset + e];
}
void SpringLevelSet::create(Mesh* mesh,
		openvdb::math::Transform::Ptr _transform) {
	this->mTransform = _transform;
	openvdb::math::Transform::Ptr trans =
			openvdb::math::Transform::createLinearTransform(1.0);
	openvdb::tools::MeshToVolume<openvdb::FloatGrid> mtol(trans);
	mtol.convertToLevelSet(mesh->vertexes, mesh->faces);
	signedLevelSet = mtol.distGridPtr();
	isoSurface.create(signedLevelSet);
	constellation.create(&isoSurface);
	updateIsoSurface();
	updateUnsignedLevelSet();
	updateNearestNeighbors();
	updateGradient();
	relax(10);
}
void SpringLevelSet::create(FloatGrid& grid) {
	this->mTransform = grid.transformPtr();
	signedLevelSet = boost::static_pointer_cast<FloatGrid>(
			grid.copyGrid(CopyPolicy::CP_COPY));
	signedLevelSet->setTransform(openvdb::math::Transform::createLinearTransform(1.0));
	isoSurface.create(signedLevelSet);
	constellation.create(&isoSurface);
	updateSignedLevelSet();
	updateIsoSurface();
	updateUnsignedLevelSet();
	updateNearestNeighbors();
	updateGradient();

	relax(10);
}
void SpringLevelSet::updateIsoSurface() {

	mesher(*signedLevelSet);
	isoSurface.create(mesher, signedLevelSet);

}
int SpringLevelSet::fill() {

	openvdb::tree::ValueAccessor<FloatGrid::TreeType> acc(
			signedLevelSet->tree());
	openvdb::math::GenericMap map(signedLevelSet->transform());

	openvdb::math::DenseStencil<openvdb::Int32Grid> stencil =
			openvdb::math::DenseStencil<openvdb::Int32Grid>(*springlIndexGrid,
					ceil(FILL_DISTANCE));

	Index64 N = mesher.pointListSize();
	openvdb::tools::PolygonPoolList& polygonPoolList = mesher.polygonPoolList();
	Vec3s p[4];
	Vec3s refPoint;

	Index32 springlsCount = constellation.getNumSpringls();
	Index32 pcounter = constellation.getNumSpringls();
	Index32 counter = constellation.getNumVertexes();

	int added = 0;
	float levelSetValue;
	std::vector<openvdb::Index32> stencilCopy;
	Index32 last;
	const float D2 = FILL_DISTANCE * FILL_DISTANCE;
	for (Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
		const openvdb::tools::PolygonPool& polygons = polygonPoolList[n];
		for (Index64 i = 0, I = polygons.numQuads(); i < I; ++i) {
			const openvdb::Vec4I& quad = polygons.quad(i);
			p[0] = mesher.pointList()[quad[3]];
			p[1] = mesher.pointList()[quad[2]];
			p[2] = mesher.pointList()[quad[1]];
			p[3] = mesher.pointList()[quad[0]];
			refPoint = 0.25f * (p[0] + p[1] + p[2] + p[3]);
			stencil.moveTo(
					Coord(std::floor(refPoint[0] + 0.5f),
							std::floor(refPoint[1] + 0.5f),
							std::floor(refPoint[2] + 0.5f)));
			int sz = stencil.size();
			levelSetValue = std::numeric_limits<float>::max();
			stencilCopy.clear();
			last = -1;
			for (unsigned int nn = 0; nn < sz; nn++) {
				openvdb::Index32 id = stencil.getValue(nn);
				if (id >= springlsCount)
					continue;
				stencilCopy.push_back(id);
			}
			sz = stencilCopy.size();
			sort(stencilCopy.begin(), stencilCopy.end());
			for (unsigned int nn = 0; nn < sz; nn++) {
				openvdb::Index32 id = stencilCopy[nn];
				if (last != id) {
					float d = constellation.springls[id].distanceToFaceSqr(
							refPoint);
					if (d < levelSetValue) {
						levelSetValue = d;
					}
				}
				last = id;
			}
			if (levelSetValue > D2) {
				added++;
				constellation.quadIndexes.push_back(counter);
				constellation.vertexes.push_back(p[0]);
				constellation.quadIndexes.push_back(counter + 1);
				constellation.vertexes.push_back(p[1]);
				constellation.quadIndexes.push_back(counter + 2);
				constellation.vertexes.push_back(p[2]);
				constellation.quadIndexes.push_back(counter + 3);
				constellation.vertexes.push_back(p[3]);

				Springl springl(&constellation);
				springl.offset = counter;
				springl.id = constellation.springls.size();
				constellation.faces.push_back(
						Vec4I(counter, counter + 1, counter + 2, counter + 3));
				constellation.particles.push_back(springl.computeCentroid());
				openvdb::Vec3s norm = springl.computeNormal();
				constellation.particleNormals.push_back(norm);
				constellation.vertexNormals.push_back(norm);
				constellation.vertexNormals.push_back(norm);
				constellation.vertexNormals.push_back(norm);
				constellation.vertexNormals.push_back(norm);
				constellation.springls.push_back(springl);
				pcounter++;
				counter += 4;
			}
		}
		for (Index64 i = 0, I = polygons.numTriangles(); i < I; ++i) {
			const openvdb::Vec3I& tri = polygons.triangle(i);
			p[0] = mesher.pointList()[tri[2]];
			p[1] = mesher.pointList()[tri[1]];
			p[2] = mesher.pointList()[tri[0]];
			refPoint = 0.25f * (p[0] + p[1] + p[2]);
			stencil.moveTo(
					Coord(std::floor(refPoint[0] + 0.5f),
							std::floor(refPoint[1] + 0.5f),
							std::floor(refPoint[2] + 0.5f)));
			int sz = stencil.size();
			levelSetValue = std::numeric_limits<float>::max();
			stencilCopy.clear();
			last = -1;
			for (unsigned int nn = 0; nn < sz; nn++) {
				openvdb::Index32 id = stencil.getValue(nn);
				if (id >= springlsCount)
					continue;
				stencilCopy.push_back(id);
			}
			sz = stencilCopy.size();
			sort(stencilCopy.begin(), stencilCopy.end());
			for (unsigned int nn = 0; nn < sz; nn++) {
				openvdb::Index32 id = stencilCopy[nn];
				if (last != id) {
					float d = constellation.springls[id].distanceToFaceSqr(
							refPoint);
					if (d < levelSetValue) {
						levelSetValue = d;
					}
				}
				last = id;
			}
			if (levelSetValue > D2) {
				added++;
				constellation.triIndexes.push_back(counter);
				constellation.vertexes.push_back(p[0]);
				constellation.triIndexes.push_back(counter + 1);
				constellation.vertexes.push_back(p[1]);
				constellation.triIndexes.push_back(counter + 2);
				constellation.vertexes.push_back(p[2]);
				Springl springl(&constellation);
				springl.offset = counter;
				springl.id = constellation.springls.size();

				constellation.faces.push_back(
						Vec4I(counter, counter + 1, counter + 2,
								openvdb::util::INVALID_IDX));
				constellation.particles.push_back(springl.computeCentroid());
				openvdb::Vec3s norm = springl.computeNormal();

				constellation.particleNormals.push_back(norm);
				constellation.vertexNormals.push_back(norm);
				constellation.vertexNormals.push_back(norm);
				constellation.vertexNormals.push_back(norm);
				constellation.springls.push_back(springl);
				pcounter++;
				counter += 3;
			}
		}
	}
	return added;
}
void SpringLevelSet::computeStatistics(Mesh& mesh) {
	float area;
	float minEdgeLength, maxEdgeLength;
	int K;
	std::vector<Index32> keepList;
	Index32 newVertexCount = 0;
	Index32 newSpringlCount = 0;
	int N = constellation.getNumSpringls();
	keepList.reserve(N);
	Index32 index = 0;
	double minls = 1E30, bias = 0, maxls = -1E30, meanls = 0, v, sqrs = 0,
			stdev;
	int count = 0;
	for (Vec3s pt : mesh.vertexes) {
		float levelSetValue = distanceToConstellation(pt);
		count++;
		v = fabs(levelSetValue);
		sqrs += v * v;
		meanls += v;
		bias += levelSetValue;
		minls = std::min(minls, v);
		maxls = std::max(maxls, v);
		count++;
	}
	meanls /= count;
	bias /= count;
	stdev = std::sqrt(sqrs / count - meanls * meanls);
	std::cout << ">>Constellation Vertex mean=" << meanls << " std dev.=" << stdev
			<< " bias=" << bias << " [" << minls << "," << maxls << "]"
			<< std::endl;
	meanls = 0;
	bias = 0;
	sqrs = 0;
	minls = 1E30;
	maxls = -1E30;
	count = 0;

	for (Vec3s pt : mesh.particles) {
		float levelSetValue = distanceToConstellation(pt);
		count++;
		v = fabs(levelSetValue);
		sqrs += v * v;
		meanls += v;
		bias += levelSetValue;
		minls = std::min(minls, v);
		maxls = std::max(maxls, v);
		count++;
	}
	meanls /= count;
	bias /= count;

	stdev = std::sqrt(sqrs / count - meanls * meanls);
	if (count > 0)
		std::cout << ">>Constellation Particle mean=" << meanls << " std dev.=" << stdev
				<< " bias=" << bias << " [" << minls << "," << maxls << "]"
				<< std::endl;
}
void SpringLevelSet::computeStatistics(Mesh& mesh, FloatGrid& levelSet) {
	openvdb::math::BoxStencil<openvdb::FloatGrid> stencil(levelSet);
	float area;
	float minEdgeLength, maxEdgeLength;
	int K;
	std::vector<Index32> keepList;
	Index32 newVertexCount = 0;
	Index32 newSpringlCount = 0;
	int N = constellation.getNumSpringls();
	keepList.reserve(N);
	Index32 index = 0;
	double minls = 1E30, bias = 0, maxls = -1E30, meanls = 0, v, sqrs = 0,
			stdev;
	int count = 0;
	for (Vec3s pt : mesh.vertexes) {
		stencil.moveTo(
				Coord(std::floor(pt[0]), std::floor(pt[1]), std::floor(pt[2])));
		float levelSetValue = stencil.interpolation(pt);
		count++;
		v = fabs(levelSetValue);
		sqrs += v * v;
		meanls += v;
		bias += levelSetValue;
		minls = std::min(minls, v);
		maxls = std::max(maxls, v);
		count++;
	}
	meanls /= count;
	bias /= count;

	stdev = std::sqrt(sqrs / count - meanls * meanls);
	std::cout << ">>Vertex mean=" << meanls << " std dev.=" << stdev << " bias="
			<< bias << " [" << minls << "," << maxls << "]" << std::endl;

	meanls = 0;
	bias = 0;
	sqrs = 0;
	minls = 1E30;
	maxls = -1E30;
	count = 0;

	for (Vec3s pt : mesh.particles) {
		stencil.moveTo(
				Coord(std::floor(pt[0]), std::floor(pt[1]), std::floor(pt[2])));
		float levelSetValue = stencil.interpolation(pt);
		count++;
		v = fabs(levelSetValue);
		sqrs += v * v;
		meanls += v;
		bias += levelSetValue;
		minls = std::min(minls, v);
		maxls = std::max(maxls, v);
		count++;
	}
	meanls /= count;
	bias /= count;

	stdev = std::sqrt(sqrs / count - meanls * meanls);
	if (count > 0)
		std::cout << ">>Particle mean=" << meanls << " std dev.=" << stdev
				<< " bias=" << bias << " [" << minls << "," << maxls << "]"
				<< std::endl;

}
void Constellation::create(Mesh* mesh) {
	size_t faceCount = mesh->faces.size();
	size_t counter = 0;
	size_t pcounter = 0;
	springls.clear();
	faces.clear();
	quadIndexes.clear();
	triIndexes.clear();
	vertexes.clear();
	vertexes.resize(mesh->quadIndexes.size() + mesh->triIndexes.size());
	particles.resize(faceCount);
	particleNormals.resize(faceCount);
	vertexNormals.resize(vertexes.size());
	for (openvdb::Vec4I face : mesh->faces) {
		Springl springl(this);
		springl.offset = counter;
		springl.id = springls.size();
		if (face[3] != openvdb::util::INVALID_IDX) {
			faces.push_back(
					openvdb::Vec4I(counter, counter + 1, counter + 2,
							counter + 3));
			quadIndexes.push_back(counter);
			quadIndexes.push_back(counter + 1);
			quadIndexes.push_back(counter + 2);
			quadIndexes.push_back(counter + 3);
			vertexes[counter++] = mesh->vertexes[face[0]];
			vertexes[counter++] = mesh->vertexes[face[1]];
			vertexes[counter++] = mesh->vertexes[face[2]];
			vertexes[counter++] = mesh->vertexes[face[3]];
			particles[pcounter] = springl.computeCentroid();
			openvdb::Vec3s norm = springl.computeNormal();
			particleNormals[pcounter] = norm;
			vertexNormals[counter - 1] = norm;
			vertexNormals[counter - 2] = norm;
			vertexNormals[counter - 3] = norm;
			vertexNormals[counter - 4] = norm;
			springls.push_back(springl);
		} else {
			faces.push_back(
					openvdb::Vec4I(counter, counter + 1, counter + 2,
							openvdb::util::INVALID_IDX));
			triIndexes.push_back(counter);
			triIndexes.push_back(counter + 1);
			triIndexes.push_back(counter + 2);
			vertexes[counter++] = mesh->vertexes[face[0]];
			vertexes[counter++] = mesh->vertexes[face[1]];
			vertexes[counter++] = mesh->vertexes[face[2]];
			particles[pcounter] = springl.computeCentroid();
			openvdb::Vec3s norm = springl.computeNormal();
			particleNormals[pcounter] = norm;
			vertexNormals[counter - 1] = norm;
			vertexNormals[counter - 2] = norm;
			vertexNormals[counter - 3] = norm;

			springls.push_back(springl);
		}
		pcounter++;
	}
	updateBBox();
}

int SpringLevelSet::clean() {
	openvdb::math::BoxStencil<openvdb::FloatGrid> stencil(*signedLevelSet);
	Vec3s pt, pt1, pt2, pt3;
	float area;
	float minEdgeLength, maxEdgeLength;
	int K;
	std::vector<Index32> keepList;
	Index32 newVertexCount = 0;
	Index32 newSpringlCount = 0;
	int N = constellation.getNumSpringls();
	keepList.reserve(N);
	Index32 index = 0;
	double minls = 1E30, bias = 0, maxls = -1E30, meanls = 0, v;
	int count = 0;
	int removeFarCount = 0;
	int removeSmallCount = 0;
	int removeAspectCount = 0;
	for (Springl& springl : constellation.springls) {
		pt = springl.particle();
		stencil.moveTo(
				Coord(std::floor(pt[0]), std::floor(pt[1]), std::floor(pt[2])));
		float levelSetValue = stencil.interpolation(pt);
		K = springl.size();
		count++;
		v = fabs(levelSetValue);
		meanls += v;
		bias += levelSetValue;
		minls = std::min(minls, v);
		maxls = std::max(maxls, v);
		if (fabs(levelSetValue) <= CLEAN_DISTANCE) {
			minEdgeLength = 1E30;
			maxEdgeLength = -1E30;
			area = 0.0f;
			for (int i = 0; i < K; i++) {
				pt1 = springl[i];
				pt2 = springl[(i + 1) % K];
				float len = (pt1 - pt2).length();
				minEdgeLength = std::min(minEdgeLength, len);
				maxEdgeLength = std::max(maxEdgeLength, len);
			}
			float aspect = minEdgeLength / maxEdgeLength;
			area = springl.area();
			if (area >= MIN_AREA && area < MAX_AREA
					&& aspect >= MIN_ASPECT_RATIO) {
				keepList.push_back(springl.id);
				newSpringlCount++;
				newVertexCount += K;
			} else {
				if (area < MIN_AREA || area >= MAX_AREA) {
					removeSmallCount++;
				}
				if (aspect < MIN_ASPECT_RATIO) {
					removeAspectCount++;
				}
			}
		} else {
			removeFarCount++;
		}
		index++;
	}
	meanls /= count;
	bias /= count;
	std::cout << "Clean mean=" << meanls << " bias=" << bias << " [" << minls
			<< "," << maxls << "] [" << removeFarCount << ","
			<< removeSmallCount << "," << removeAspectCount << "]" << std::endl;

	if (newSpringlCount == N)
		return 0;
	Index32 springlOffset = 0;
	Index32 vertexOffset = 0;
	Index32 quadIndex = 0;
	Index32 triIndex = 0;
	for (int n : keepList) {
		Springl& rspringl = constellation.springls[n];
		Springl& springl = constellation.springls[springlOffset];
		K = rspringl.size();
		if (springlOffset != n) {
			constellation.particles[springlOffset] = constellation.particles[n];
			constellation.particleNormals[springlOffset] =
					constellation.particleNormals[n];
			springl.offset = vertexOffset;
			springl.id = springlOffset;
			Vec4I quad;
			quad[3] = openvdb::util::INVALID_IDX;
			for (int k = 0; k < K; k++) {
				constellation.vertexes[vertexOffset + k] =
						constellation.vertexes[rspringl.offset + k];
				constellation.vertexNormals[vertexOffset + k] =
						constellation.vertexNormals[rspringl.offset + k];
				quad[k] = vertexOffset + k;
			}
			if (K == 4) {
				for (int k = 0; k < K; k++) {
					constellation.quadIndexes[quadIndex++] = vertexOffset + k;
				}
			} else if (K == 3) {
				for (int k = 0; k < K; k++) {
					constellation.triIndexes[triIndex++] = vertexOffset + k;
				}
			}
			constellation.faces[springlOffset] = quad;
		} else {
			if (K == 4) {
				quadIndex += K;
			} else if (K == 3) {
				triIndex += K;
			}
		}
		vertexOffset += K;
		springlOffset++;
	}
	constellation.triIndexes.erase(constellation.triIndexes.begin() + triIndex,
			constellation.triIndexes.end());

	constellation.quadIndexes.erase(
			constellation.quadIndexes.begin() + quadIndex,
			constellation.quadIndexes.end());

	constellation.springls.erase(constellation.springls.begin() + springlOffset,
			constellation.springls.end());
	constellation.particles.erase(
			constellation.particles.begin() + springlOffset,
			constellation.particles.end());
	constellation.particleNormals.erase(
			constellation.particleNormals.begin() + springlOffset,
			constellation.particleNormals.end());
	constellation.faces.erase(constellation.faces.begin() + springlOffset,
			constellation.faces.end());

	constellation.vertexNormals.erase(
			constellation.vertexNormals.begin() + vertexOffset,
			constellation.vertexNormals.end());
	constellation.vertexes.erase(constellation.vertexes.begin() + vertexOffset,
			constellation.vertexes.end());
	return (N - newSpringlCount);
}

}

