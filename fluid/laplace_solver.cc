/*
 *  solver.cpp
 *  flip3D
 *
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
		RegularGrid<float>& x, int fi, int fj, int fk, int i, int j, int k,
		int n) {
	i = min(max(0, i), n - 1);
	j = min(max(0, j), n - 1);
	k = min(max(0, k), n - 1);
	if (A[i][j][k] == FLUID)
		return x[i][j][k];
	else if (A[i][j][k] == WALL)
		return x[fi][fj][fk];
	return L[i][j][k] / fmin(1.0e-6, L[fi][fj][fk]) * x[fi][fj][fk];
}

// Ans = Ax
static void compute_Ax(RegularGrid<char>& A, RegularGrid<float>& L,
		RegularGrid<float>& x, RegularGrid<float>& ans, int n) {
	float h2 = 1.0 / (n * n);
	OPENMP_FOR FOR_EVERY_COMP(n)
		{
			if (A[i][j][k] == FLUID) {
				ans[i][j][k] = (6.0 * x[i][j][k]
						- x_ref(A, L, x, i, j, k, i + 1, j, k, n)
						- x_ref(A, L, x, i, j, k, i - 1, j, k, n)
						- x_ref(A, L, x, i, j, k, i, j + 1, k, n)
						- x_ref(A, L, x, i, j, k, i, j - 1, k, n)
						- x_ref(A, L, x, i, j, k, i, j, k + 1, n)
						- x_ref(A, L, x, i, j, k, i, j, k - 1, n)) / h2;
			} else {
				ans[i][j][k] = 0.0;
			}
		}END_FOR;
}

// ans = x^T * x
static double product(RegularGrid<char>& A, RegularGrid<float>& x,
		RegularGrid<float>& y, int n) {
	static double ans;
	ans = 0.0;
#ifdef _OPENMP
#pragma omp for reduction(+:ans)
#endif
	FOR_EVERY_COMP(n)
		{
			if (A[i][j][k] == FLUID)
				ans += x[i][j][k] * y[i][j][k];
		}END_FOR;
	return ans;
}

// x = 0
template<class T>
static void clear(RegularGrid<T>& x, int n) {
	x.fill(0.0);
}

static void flipDivergence(RegularGrid<float>& x, int n) {
	OPENMP_FOR FOR_EVERY_COMP(n)
		{
			x[i][j][k] = -x[i][j][k];
		}END_FOR;
}

// x <= y
static void copy(RegularGrid<float>& x, RegularGrid<float>& y, int n) {
	y.copyTo(x);
}

// Ans = x + a*y
static void op(RegularGrid<char>& A, RegularGrid<float>& x,
		RegularGrid<float>& y, RegularGrid<float>& ans, float a, int n) {
	RegularGrid<float> tmp(n, n, n);
	OPENMP_FOR FOR_EVERY_COMP(n)
		{
			if (A[i][j][k] == FLUID)
				tmp[i][j][k] = x[i][j][k] + a * y[i][j][k];
			else
				tmp[i][j][k] = 0.0;
		}END_FOR;
	copy(ans, tmp, n);
}

// r = b - Ax
static void residual(RegularGrid<char>& A, RegularGrid<float>& L,
		RegularGrid<float>& x, RegularGrid<float>& b, RegularGrid<float>& r,
		int n) {
	compute_Ax(A, L, x, r, n);
	op(A, b, r, r, -1.0, n);
}

static inline float square(float a) {
	return a * a;
}

static float A_ref(RegularGrid<char>& A, int i, int j, int k, int qi, int qj,
		int qk, int n) {
	if (i < 0 || i > n - 1 || j < 0 || j > n - 1 || k < 0 || k > n - 1
			|| A[i][j][k] != FLUID)
		return 0.0;
	if (qi < 0 || qi > n - 1 || qj < 0 || qj > n - 1 || qk < 0 || qk > n - 1
			|| A[qi][qj][qk] != FLUID)
		return 0.0;
	return -1.0;
}

static float A_diag(RegularGrid<char>& A, RegularGrid<float>& L, int i, int j,
		int k, int n) {
	float diag = 6.0;
	if (A[i][j][k] != FLUID)
		return diag;
	int q[][3] = { { i - 1, j, k }, { i + 1, j, k }, { i, j - 1, k }, { i, j
			+ 1, k }, { i, j, k - 1 }, { i, j, k + 1 } };
	for (int m = 0; m < 6; m++) {
		int qi = q[m][0];
		int qj = q[m][1];
		int qk = q[m][2];
		if (qi < 0 || qi > n - 1 || qj < 0 || qj > n - 1 || qk < 0 || qk > n - 1
				|| A[qi][qj][qk] == WALL)
			diag -= 1.0;
		else if (A[qi][qj][qk] == AIR) {
			diag -= L[qi][qj][qk] / fmin(1.0e-6, L[i][j][k]);
		}
	}
	return diag;
}

template<class T>
static float P_ref(RegularGrid<T>& P, int i, int j, int k, int n) {
	if (i < 0 || i > n - 1 || j < 0 || j > n - 1 || k < 0 || k > n - 1
			|| P[i][j][k] != FLUID)
		return 0.0;
	return P[i][j][k];
}

static void buildPreconditioner(RegularGrid<double>& P, RegularGrid<float>& L,
		RegularGrid<char>& A, int n) {
	clear(P, n);
	double a = 0.25;
	FOR_EVERY_COMP(n)
		{
			if (A[i][j][k] == FLUID) {
				double left = A_ref(A, i - 1, j, k, i, j, k, n)
						* P_ref(P, i - 1, j, k, n);
				double bottom = A_ref(A, i, j - 1, k, i, j, k, n)
						* P_ref(P, i, j - 1, k, n);
				double back = A_ref(A, i, j, k - 1, i, j, k, n)
						* P_ref(P, i, j, k - 1, n);
				double diag = A_diag(A, L, i, j, k, n);
				double e = diag - square(left) - square(bottom) - square(back);
				if (e < a * diag)
					e = diag;
				P[i][j][k] = 1.0 / sqrtf(e);
			}
		}END_FOR;
}

static void applyPreconditioner(RegularGrid<float>& z, RegularGrid<float>& r,
		RegularGrid<double>& P, RegularGrid<float>& L, RegularGrid<char>& A,
		int n) {
	RegularGrid<double> q(n, n, n);
	clear(q, n);

	// Lq = r
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			for (int k = 0; k < n; k++) {
				if (A[i][j][k] == FLUID) {
					double left = A_ref(A, i - 1, j, k, i, j, k, n)
							* P_ref(P, i - 1, j, k, n)
							* P_ref(q, i - 1, j, k, n);
					double bottom = A_ref(A, i, j - 1, k, i, j, k, n)
							* P_ref(P, i, j - 1, k, n)
							* P_ref(q, i, j - 1, k, n);
					double back = A_ref(A, i, j, k - 1, i, j, k, n)
							* P_ref(P, i, j, k - 1, n)
							* P_ref(q, i, j, k - 1, n);

					double t = r[i][j][k] - left - bottom - back;
					q[i][j][k] = t * P[i][j][k];
				}
			}
		}
	}

	// L^T z = q
	for (int i = n - 1; i >= 0; i--) {
		for (int j = n - 1; j >= 0; j--) {
			for (int k = n - 1; k >= 0; k--) {
				if (A[i][j][k] == FLUID) {
					double right = A_ref(A, i, j, k, i + 1, j, k, n)
							* P_ref(P, i, j, k, n) * P_ref(z, i + 1, j, k, n);
					double top = A_ref(A, i, j, k, i, j + 1, k, n)
							* P_ref(P, i, j, k, n) * P_ref(z, i, j + 1, k, n);
					double front = A_ref(A, i, j, k, i, j, k + 1, n)
							* P_ref(P, i, j, k, n) * P_ref(z, i, j, k + 1, n);

					double t = q[i][j][k] - right - top - front;
					z[i][j][k] = t * P[i][j][k];
				}
			}
		}
	}
}

// Conjugate Gradient Method
static void conjGrad(RegularGrid<char>& A, RegularGrid<double>& P,
		RegularGrid<float>& L, RegularGrid<float>& x, RegularGrid<float>& b,
		int n) {
	// Pre-allocate Memory
	RegularGrid<float> r(n, n, n);
	RegularGrid<float> z(n, n, n);
	RegularGrid<float> s(n, n, n);
	compute_Ax(A, L, x, z, n);                // z = applyA(x)
	op(A, b, z, r, -1.0, n);                  // r = b-Ax
	double error2_0 = product(A, r, r, n);    // error2_0 = r . r
	applyPreconditioner(z, r, P, L, A, n);		// Apply Conditioner z = f(r)
	copy(s, z, n);								// s = z
	double eps = 1.0e-2 * (n * n * n);
	double a = product(A, z, r, n);			// a = z . r
	dump("\n");
	for (int k = 0; k < n * n * n; k++) {
		compute_Ax(A, L, s, z, n);			// z = applyA(s)
		double alpha = a / product(A, z, s, n);	// alpha = a/(z . s)
		op(A, x, s, x, alpha, n);				// x = x + alpha*s
		op(A, r, z, r, -alpha, n);			// r = r - alpha*z;
		float error2 = product(A, r, r, n);	// error2 = r . r
		error2_0 = fmax(error2_0, error2);
		// Dump Progress
		double rate = 1.0
				- max(0.0, min(1.0, (error2 - eps) / (error2_0 - eps)));
		printf("%d Iteration %f Solved.\n", (k + 1), 100.0f * powf(rate, 6));
		if (error2 <= eps)
			break;
		applyPreconditioner(z, r, P, L, A, n);	// Apply Conditioner z = f(r)
		double a2 = product(A, z, r, n);		// a2 = z . r
		double beta = a2 / a;                     // beta = a2 / a
		op(A, z, s, s, beta, n);				// s = z + beta*s
		a = a2;
	}
}


void laplace_solve(RegularGrid<char>& A, RegularGrid<float>& L,
		RegularGrid<float>& x, RegularGrid<float>& b, int n) {
	RegularGrid<double> P(n, n, n);
	// Flip Divergence
	std::cout<<"Calculate divergence ..."<<std::endl;
	flipDivergence(b, n);

	std::cout<<"Create preconditioner ..."<<std::endl;
	buildPreconditioner(P, L, A, n);

	std::cout<<"Conjugate gradient solve ..."<<std::endl;
	conjGrad(A, P, L, x, b, n);
}
}
}
