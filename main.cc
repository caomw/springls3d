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
#include "SimulationPlayback.h"
#include "ArmadilloTwist.h"
#include <iostream>
using namespace openvdb;
using namespace imagesci;
using namespace std;
int main(int argc, char *argv[]) {
	int status = EXIT_FAILURE;
	try {
		if (argc > 3){
			std::string task=std::string(argv[1]);
			if( task== "-playback") {
				std::string dirName=std::string(argv[2]);
				SimulationPlayback sim(dirName);
				SimulationVisualizer::run(static_cast<Simulation*>(&sim),1024,768,dirName);
				status=EXIT_SUCCESS;
			} else if("-enright"){
				std::string dirName=std::string(argv[2]);
				int dim = 256;
				if(argc>3){
					dim=atoi(argv[3]);
				}
				EnrightSimulation sim(dim);
				SimulationVisualizer::run(static_cast<Simulation*>(&sim),1024,768,dirName);
				status=EXIT_SUCCESS;
			} else if("-twist"){
				std::string dirName=std::string(argv[2]);
				std::string sourceFileName="armadillo.ply";
				if(argc>3){
					sourceFileName=std::string(argv[3]);
				}
				ArmadilloTwist sim(sourceFileName);
				SimulationVisualizer::run(static_cast<Simulation*>(&sim),1024,768,dirName);
				status=EXIT_SUCCESS;
			}
		}
	} catch (imagesci::Exception& e) {
		std::cout << e.what() << std::endl;
	}
	if(status==EXIT_FAILURE){
		std::cout<<"Usage: "<<argv[0]<<" -playback <INPUT_DIRECTORY>"<<endl;
		std::cout<<"Usage: "<<argv[0]<<" -enright <OUTPUT_DIRECTORY> <INTEGER_GRID_SIZE>"<<endl;
		std::cout<<"Usage: "<<argv[0]<<" -twist <OUTPUT_DIRECTORY> <MESH_FILE>"<<endl;
	}
	return status;
}
