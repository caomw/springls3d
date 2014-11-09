/*
 *  solver.h
 *  flip3D
 *
 */

#include "fluid_common.h"
namespace imagesci {
namespace fluid {
	void laplace_solve(RegularGrid<char>& A, RegularGrid<float>& L, RegularGrid<float>& x, RegularGrid<float>& b, int n );
}
}
