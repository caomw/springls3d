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
 *
 *  This implementation of a PIC/FLIP static_cast<char>(ObjectType::FLUID) simulator is derived from:
 *
 *  Ando, R., Thurey, N., & Tsuruno, R. (2012). Preserving fluid sheets with adaptively sampled anisotropic particles.
 *  Visualization and Computer Graphics, IEEE Transactions on, 18(8), 1202-1214.
 */
#include "laplace_solver.h"
#include "fluid_common.h"
#include "fluid_utility.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
using namespace openvdb;
using namespace openvdb::tools;
using namespace std;
namespace imagesci {
namespace fluid {
#define FOR_EVERY_COMP(N) for( int gn=0; gn<(N)*(N)*(N); gn++ ) { int i=(gn%((N)*(N)))%(N); int j=(gn%((N)*(N)))/(N); int k = gn/((N)*(N)); 
// Clamped Fetch
static float x_ref(RegularGrid<char>& A, RegularGrid<float>& L,
		RegularGrid<float>& x, int fi, int fj, int fk, int i, int j, int k) {
	i = clamp(i,0,(int)x.rows()-1);
	j = clamp(j,0,(int)x.cols()-1);
	k = clamp(k,0,(int)x.slices()-1);
	if (A(i,j,k) == static_cast<char>(ObjectType::FLUID))
		return x(i,j,k);
	else if (A(i,j,k) == static_cast<char>(ObjectType::WALL))
		return x(fi,fj,fk);
	return L(i,j,k) / fmin(1.0e-6, L(fi,fj,fk)) * x(fi,fj,fk);
}

// Ans = Ax
static void compute_Ax(RegularGrid<char>& A, RegularGrid<float>& L,
		RegularGrid<float>& x, RegularGrid<float>& ans) {
	float h2 = x.voxelSize()*x.voxelSize();
	OPENMP_FOR FOR_EVERY_GRID_CELL(x)
		{
			if (A(i,j,k) == static_cast<char>(ObjectType::FLUID)) {
				ans(i,j,k) = (6.0 * x(i,j,k)
						- x_ref(A, L, x, i, j, k, i + 1, j, k)
						- x_ref(A, L, x, i, j, k, i - 1, j, k)
						- x_ref(A, L, x, i, j, k, i, j + 1, k)
						- x_ref(A, L, x, i, j, k, i, j - 1, k)
						- x_ref(A, L, x, i, j, k, i, j, k + 1)
						- x_ref(A, L, x, i, j, k, i, j, k - 1)) / h2;
			} else {
				ans(i,j,k) = 0.0;
			}
		}END_FOR;
}

// ans = x^T * x
static double product(RegularGrid<char>& A, RegularGrid<float>& x,
		RegularGrid<float>& y) {
	static double ans;
	ans = 0.0;
#ifdef MP
#pragma omp parallel for reduction(+:ans)
#endif
	FOR_EVERY_GRID_CELL(A)
		{
			if (A(i,j,k) == static_cast<char>(ObjectType::FLUID))
				ans += x(i,j,k) * y(i,j,k);
		}END_FOR;
	return ans;
}

static void flipDivergence(RegularGrid<float>& x) {
	OPENMP_FOR FOR_EVERY_GRID_CELL(x){
		x(i,j,k) = -x(i,j,k);
	} END_FOR;
}

// x <= y
static void copy(RegularGrid<float>& x, RegularGrid<float>& y) {
	y.copyTo(x);
}

// Ans = x + a*y
static void op(RegularGrid<char>& A, RegularGrid<float>& x,
		RegularGrid<float>& y, RegularGrid<float>& ans, float a) {
	RegularGrid<float> tmp(x.rows(),x.cols(),x.slices(),x.voxelSize());
	OPENMP_FOR FOR_EVERY_GRID_CELL(x)
		{
			if (A(i,j,k) == static_cast<char>(ObjectType::FLUID))
				tmp(i,j,k) = x(i,j,k) + a * y(i,j,k);
			else
				tmp(i,j,k) = 0.0;
		}END_FOR;
	copy(ans, tmp);
}

// r = b - Ax
static void residual(RegularGrid<char>& A, RegularGrid<float>& L,
		RegularGrid<float>& x, RegularGrid<float>& b, RegularGrid<float>& r) {
	compute_Ax(A, L, x, r);
	op(A, b, r, r, -1.0);
}

static inline float square(float a) {
	return a * a;
}

static float A_ref(RegularGrid<char>& A, int i, int j, int k, int qi, int qj,
		int qk) {
	if (i < 0 || i > A.rows() - 1 || j < 0 || j > A.cols() - 1 || k < 0 || k > A.slices() - 1
			|| A(i,j,k) != static_cast<char>(ObjectType::FLUID))
		return 0.0;
	if (qi < 0 || qi > A.rows() - 1 || qj < 0 || qj >A.cols() - 1 || qk < 0 || qk > A.slices() - 1
			|| A(qi,qj,qk) != static_cast<char>(ObjectType::FLUID))
		return 0.0;
	return -1.0;
}

static float A_diag(RegularGrid<char>& A, RegularGrid<float>& L, int i, int j,
		int k) {
	float diag = 6.0;
	if (A(i,j,k) != static_cast<char>(ObjectType::FLUID))
		return diag;
	int q[][3] = { { i - 1, j, k }, { i + 1, j, k }, { i, j - 1, k }, { i, j
			+ 1, k }, { i, j, k - 1 }, { i, j, k + 1 } };
	for (int m = 0; m < 6; m++) {
		int qi = q[m][0];
		int qj = q[m][1];
		int qk = q[m][2];
		if (qi < 0 || qi > A.rows() - 1 || qj < 0 || qj > A.cols() - 1 || qk < 0 || qk > A.slices() - 1
				|| A(qi,qj,qk) == static_cast<char>(ObjectType::WALL))
			diag -= 1.0;
		else if (A(qi,qj,qk) == static_cast<char>(ObjectType::AIR)) {
			diag -= L(qi,qj,qk) / fmin(1.0e-6, L(i,j,k));
		}
	}
	return diag;
}

template<class T>
static float P_ref(RegularGrid<T>& P, int i, int j, int k) {
	if (i < 0 || i > P.rows() - 1 || j < 0 || j > P.cols() - 1 || k < 0 || k > P.slices() - 1
			|| P(i,j,k) != static_cast<char>(ObjectType::FLUID))
		return 0.0;
	return P(i,j,k);
}

static void buildPreconditioner(RegularGrid<double>& P, RegularGrid<float>& L,
		RegularGrid<char>& A) {
	double a = 0.25;
	FOR_EVERY_GRID_CELL(A)
		{
			if (A(i,j,k) == static_cast<char>(ObjectType::FLUID)) {
				double left = A_ref(A, i - 1, j, k, i, j, k)
						* P_ref(P, i - 1, j, k);
				double bottom = A_ref(A, i, j - 1, k, i, j, k)
						* P_ref(P, i, j - 1, k);
				double back = A_ref(A, i, j, k - 1, i, j, k)
						* P_ref(P, i, j, k - 1);
				double diag = A_diag(A, L, i, j, k);
				double e = diag - square(left) - square(bottom) - square(back);
				if (e < a * diag)
					e = diag;
				P(i,j,k) = 1.0 / sqrtf(e);
			}
		}END_FOR;
}

static void applyPreconditioner(RegularGrid<float>& z, RegularGrid<float>& r,
		RegularGrid<double>& P, RegularGrid<float>& L, RegularGrid<char>& A) {
	RegularGrid<double> q(P.rows(),P.cols(),P.slices(),P.voxelSize(),0.0);
	// Lq = r
	FOR_EVERY_GRID_CELL(q){
				if (A(i,j,k) == static_cast<char>(ObjectType::FLUID)) {
					double left = A_ref(A, i - 1, j, k, i, j, k)
							* P_ref(P, i - 1, j, k)
							* P_ref(q, i - 1, j, k);
					double bottom = A_ref(A, i, j - 1, k, i, j, k)
							* P_ref(P, i, j - 1, k)
							* P_ref(q, i, j - 1, k);
					double back = A_ref(A, i, j, k - 1, i, j, k)
							* P_ref(P, i, j, k - 1)
							* P_ref(q, i, j, k - 1);
					double t = r(i,j,k) - left - bottom - back;
					q(i,j,k) = t * P(i,j,k);
				}
	}END_FOR;

	// L^T z = q
	for (int i = q.rows() - 1; i >= 0; i--) {
		for (int j = q.cols() - 1; j >= 0; j--) {
			for (int k = q.slices() - 1; k >= 0; k--) {
				if (A(i,j,k) == static_cast<char>(ObjectType::FLUID)) {
					double right = A_ref(A, i, j, k, i + 1, j, k)
							* P_ref(P, i, j, k) * P_ref(z, i + 1, j, k);
					double top = A_ref(A, i, j, k, i, j + 1, k)
							* P_ref(P, i, j, k) * P_ref(z, i, j + 1, k);
					double front = A_ref(A, i, j, k, i, j, k + 1)
							* P_ref(P, i, j, k) * P_ref(z, i, j, k + 1);

					double t = q(i,j,k) - right - top - front;
					z(i,j,k) = t * P(i,j,k);
				}
			}
		}
	}
}

// Conjugate Gradient Method
static void conjGrad(RegularGrid<char>& A, RegularGrid<double>& P,
		RegularGrid<float>& L, RegularGrid<float>& x, RegularGrid<float>& b) {
	// Pre-allocate Memory
	openvdb::Coord dims(x.rows(),x.cols(),x.slices());
	RegularGrid<float> r(dims,x.voxelSize(),0.0);
	RegularGrid<float> z(dims,x.voxelSize(),0.0);
	RegularGrid<float> s(dims,x.voxelSize(),0.0);
	compute_Ax(A, L, x, z);                // z = applyA(x)
	op(A, b, z, r, -1.0);                  // r = b-Ax
	double error2_0 = product(A, r, r);    // error2_0 = r . r
	applyPreconditioner(z, r, P, L, A);		// Apply Conditioner z = f(r)
	copy(s, z);								// s = z
	int V=dims[0]*dims[1]*dims[2];
	double eps = 1.0e-2 * (V);
	double a = product(A, z, r);			// a = z . r
	for (int k = 0; k < V; k++) {
		compute_Ax(A, L, s, z);			// z = applyA(s)
		double alpha = a / product(A, z, s);	// alpha = a/(z . s)
		op(A, x, s, x, alpha);				// x = x + alpha*s
		op(A, r, z, r, -alpha);			// r = r - alpha*z;
		float error2 = product(A, r, r);	// error2 = r . r
		error2_0 = fmax(error2_0, error2);
		//std::cout<<"ERROR "<<error2<<" "<<error2_0<<std::endl;
		// Dump Progress
		double rate = 1.0
				- max(0.0, min(1.0, (error2 - eps) / (error2_0 - eps)));
		//std::cout<<"Laplace iteration "<<(k + 1)<<" ["<<100.0f * powf(rate, 6)<<"%]"<<std::endl;
		if (error2 <= eps)
			break;
		applyPreconditioner(z, r, P, L, A);	// Apply Conditioner z = f(r)
		double a2 = product(A, z, r);		// a2 = z . r
		double beta = a2 / a;                     // beta = a2 / a
		op(A, z, s, s, beta);				// s = z + beta*s
		a = a2;
	}
}


void laplace_solve(RegularGrid<char>& A, RegularGrid<float>& L,
		RegularGrid<float>& x, RegularGrid<float>& b) {
	RegularGrid<double> P(x.rows(),x.cols(),x.slices(),x.voxelSize());
	// Flip Divergence
	flipDivergence(b);
	buildPreconditioner(P, L, A);
	conjGrad(A, P, L, x, b);
}
}
}
