/*
 * Copyright(C) 2014, Blake C. Lucas, Ph.D. (img.science@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <iostream>
#include <string>
#include <vector>
#include <exception>
#include <stdlib.h>

#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/Filter.h>
#include <openvdb/Types.h>
#include <openvdb/openvdb.h>
#undef OPENVDB_REQUIRE_VERSION_NAME

#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetAdvect.h>
#include <openvdb/tools/LevelSetMeasure.h>
#include <openvdb/tools/LevelSetMorph.h>
#include <openvdb/tools/Morphology.h>
#include <openvdb/tools/PointAdvect.h>
#include <openvdb/tools/PointScatter.h>
#include <openvdb/tools/ValueTransformer.h>
#include <openvdb/tools/VectorTransformer.h>
#include <openvdb/util/Util.h>
#include <openvdb/math/Stats.h>
#include <boost/filesystem.hpp>
#include <tbb/mutex.h>

#ifdef DWA_OPENVDB
#include <logging_base/logging.h>
#include <usagetrack.h>

#endif
#include "Util.h"
#include "SimulationVisualizer.h"
#include "EnrightSimulation.h"
using namespace openvdb;
using namespace imagesci;
tbb::mutex sLock;
int main(int argc, char *argv[]) {
	int status = EXIT_SUCCESS;

	try {
		EnrightSimulation sim;
		SimulationVisualizer::run(static_cast<Simulation*>(&sim),1024,768);
		/*
		if (argc > 2) {
			if (std::string(argv[1]) == "-playback") {
				std::string fileName(argv[2]);
				openvdb::initialize();
				tbb::mutex::scoped_lock(sLock);
				OPENVDB_START_THREADSAFE_STATIC_WRITE
				imagesci::EnrightSpringls* viewer =
						imagesci::EnrightSpringls::GetInstance();
				OPENVDB_FINISH_THREADSAFE_STATIC_WRITE
				if(viewer->openRecording(fileName)){
					if(argc>3){
						viewer->setFrameIndex(atoi(argv[3]));
					}
					viewer->init(1600, 900);
				}

			} else if (std::string(argv[1]) == "-simulate") {
				std::string fileName(argv[2]);
				openvdb::initialize();
				tbb::mutex::scoped_lock(sLock);
				OPENVDB_START_THREADSAFE_STATIC_WRITE
				imagesci::EnrightSpringls* viewer =
						imagesci::EnrightSpringls::GetInstance();
				OPENVDB_FINISH_THREADSAFE_STATIC_WRITE
				std::string ext = boost::filesystem::extension(
						boost::filesystem::path(fileName));
				if (ext == std::string(".ply")) {
					viewer->openMesh(fileName);
				} else if (ext == std::string(".vdb")) {
					viewer->openGrid(fileName);
				}
				viewer->start();
				viewer->init(1600, 900);
			}
		} else {
			openvdb::initialize();
			tbb::mutex::scoped_lock(sLock);
			OPENVDB_START_THREADSAFE_STATIC_WRITE
			imagesci::EnrightSpringls* viewer =
					imagesci::EnrightSpringls::GetInstance();
			const float radius = 0.15f;
			const openvdb::Vec3f center(0.35f, 0.35f, 0.35f);
			int dim = 256;
			if(argc>1){
				dim=atoi(argv[1]);
			}
			float voxelSize = 1 / (float) (dim - 1);
			FloatGrid::Ptr mSignedLevelSet =
					openvdb::tools::createLevelSetSphere<FloatGrid>(radius,
							center, voxelSize);
			viewer->openGrid(*mSignedLevelSet);
			viewer->start();
			viewer->init(1600, 900);
		}
		 */
	} catch (imagesci::Exception& e) {
		std::cout << e.what() << std::endl;
		status = EXIT_FAILURE;
	}
	return status;
}
