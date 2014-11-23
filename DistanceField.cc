/**
 *       Java Image Science Toolkit
 *                  ---
 *     Multi-Object Image Segmentation
 *
 * Copyright(C) 2012, Blake Lucas (img.science@gmail.com)
 * All rights reserved.
 *
 * Center for Computer-Integrated Surgical Systems and Technology &
 * Johns Hopkins Applied Physics Laboratory &
 * The Johns Hopkins University
 *
 * Redistribution and use in source and binary forms are permitted
 * provided that the above copyright notice and this paragraph are
 * duplicated in all such forms and that any documentation,
 * advertising materials, and other materials related to such
 * distribution and use acknowledge that the software was developed
 * by the The Johns Hopkins University.  The name of the
 * University may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * @author Blake Lucas (img.science@gmail.com)
 */

#include "DistanceField.h"
#include "BinaryMinHeap.h"
using namespace std;
using namespace openvdb;
namespace imagesci {
double DistanceField::march(double Nv, double Sv, double Ev, double Wv,
		double Fv, double Bv, int Nl, int Sl, int El, int Wl, int Fl, int Bl) {

	double s, s2; /* s = a + b +c; s2 = a*a + b*b +c*c */
	double tmp;
	int count;

	s = 0;
	s2 = 0;
	count = 0;

	if (Nl == ALIVE && Sl == ALIVE) {
		tmp = std::min(Nv, Sv); /* Take the smaller one if both ALIVE */
		s += tmp;
		s2 += tmp * tmp;
		count++;
	} else if (Nl == ALIVE) {
		s += Nv; /* Else, take the ALIVE one */
		s2 += Nv * Nv;
		count++;
	} else if (Sl == ALIVE) {
		s += Sv;
		s2 += Sv * Sv;
		count++;
	}

	/*
	 * Similarly in the east-west direction to get correct approximation to
	 * the derivative in the x-direction
	 */
	if (El == ALIVE && Wl == ALIVE) {
		tmp = std::min(Ev, Wv); /* Take the smaller one if both ALIVE */
		s += tmp;
		s2 += tmp * tmp;
		count++;
	} else if (El == ALIVE) {
		s += Ev; /* Else, take the ALIVE one */
		s2 += Ev * Ev;
		count++;
	} else if (Wl == ALIVE) {
		s += Wv;
		s2 += Wv * Wv;
		count++;
	}

	/*
	 * Similarly in the front-back direction to get correct approximation to
	 * the derivative in the z-direction
	 */
	if (Fl == ALIVE && Bl == ALIVE) {
		tmp = std::min(Fv, Bv); /* Take the smaller one if both ALIVE */
		s += tmp;
		s2 += tmp * tmp;
		count++;
	} else if (Fl == ALIVE) {
		s += Fv; /* Else, take the ALIVE one */
		s2 += Fv * Fv;
		count++;
	} else if (Bl == ALIVE) {
		s += Bv;
		s2 += Bv * Bv;
		count++;
	}

	/*
	 * count must be greater than zero since there must be one ALIVE pt in
	 * the neighbors
	 */

	tmp = (s + std::sqrt((s * s - count * (s2 - 1.0f)))) / count;
	/* The larger root */
	return tmp;
}
void DistanceField::solve(RegularGrid<float>& distVol, double maxDistance) {
	int XN = vol.rows();
	int YN = vol.cols();
	int ZN = vol.slices();
	int LX, HX, LY, HY, LZ, HZ;
	short NSFlag, WEFlag, FBFlag;
	int i, j, k, koff;
	int nj, nk, ni;
	double newvalue;
	double s = 0, t = 0, w = 0;
	double result;
	static const int neighborsX[6] = { 1, 0, -1, 0, 0, 0 };
	static const int neighborsY[6] = { 0, 1, 0, -1, 0, 0 };
	static const int neighborsZ[6] = { 0, 0, 0, 0, 1, -1 };
	std::list<VoxelIndex> voxelList;
	VoxelIndex* he=nullptr;
	double Nv = 0, Sv = 0, Wv = 0, Ev = 0, Fv = 0, Bv = 0, Cv = 0;
	uint8_t Nl = 0, Sl = 0, Wl = 0, El = 0, Fl = 0, Bl = 0, Cl = 0;
	RegularGrid<uint8_t> labelVol(XN, YN, ZN, 1.0, FARAWAY);
	int countAlive = 0;
	FOR_EVERY_GRID_CELL(distVol) {
		if (vol(i, j, k) == 0) {
			distVol(i, j, k) = (0);
			labelVol(i, j, k) = (ALIVE);
			countAlive++;
		} else {
			LX = (i == 0) ? 1 : 0;
			HX = (i == (XN - 1)) ? 1 : 0;

			LY = (j == 0) ? 1 : 0;
			HY = (j == (YN - 1)) ? 1 : 0;

			LZ = (k == 0) ? 1 : 0;
			HZ = (k == (ZN - 1)) ? 1 : 0;

			NSFlag = 0;
			WEFlag = 0;
			FBFlag = 0;

			Nv = vol(i, j - 1 + LY, k);
			Sv = vol(i, j + 1 - HY, k);
			Wv = vol(i - 1 + LX, j, k);
			Ev = vol(i + 1 - HX, j, k);
			Fv = vol(i, j, k + 1 - HZ);
			Bv = vol(i, j, k - 1 + LZ);
			Cv = vol(i, j, k);
			if (Nv * Cv < 0) {
				NSFlag = 1;
				s = Nv;
			}
			if (Sv * Cv < 0) {
				if (NSFlag == 0) {
					NSFlag = 1;
					s = Sv;
				} else {
					s = (std::abs(Nv) > std::abs(Sv)) ? Nv : Sv;
				}
			}
			if (Wv * Cv < 0) {
				WEFlag = 1;
				t = Wv;
			}
			if (Ev * Cv < 0) {
				if (WEFlag == 0) {
					WEFlag = 1;
					t = Ev;
				} else {
					t = (std::abs(Ev) > std::abs(Wv)) ? Ev : Wv;
				}
			}
			if (Fv * Cv < 0) {
				FBFlag = 1;
				w = Fv;
			}
			if (Bv * Cv < 0) {
				if (FBFlag == 0) {
					FBFlag = 1;
					w = Bv;
				} else {
					w = (std::abs(Fv) > std::abs(Bv)) ? Fv : Bv;
				}
			}
			result = 0;
			if (NSFlag != 0) {
				s = Cv / (Cv - s);
				result += 1.0 / (s * s);
			}
			if (WEFlag != 0) {
				t = Cv / (Cv - t);
				result += 1.0 / (t * t);
			}
			if (FBFlag != 0) {
				w = Cv / (Cv - w);
				result += 1.0 / (w * w);
			}
			if (result == 0) {
				distVol(i, j, k)=0.0f;
				continue;
			}
			countAlive++;
			labelVol(i, j, k) = (ALIVE);
			result = std::sqrt(result);
			distVol(i, j, k) = (float) (1.0 / result);
		}
	} END_FOR;
	heap.reserve(countAlive);

	/* Initialize NarrowBand Heap */
	FOR_EVERY_GRID_CELL(distVol)
		{
			if (labelVol(i, j, k) != ALIVE) {
				continue;
			}
			for (koff = 0; koff < 6; koff++) {
				ni = i + neighborsX[koff];
				nj = j + neighborsY[koff];
				nk = k + neighborsZ[koff];
				if (nj < 0 || nj >= YN || nk < 0 || nk >= ZN || ni < 0
						|| ni >= XN) {
					continue; /* Out of computational Boundary */
				}
				if (labelVol(ni, nj, nk) != FARAWAY) {
					continue;
				}
				labelVol(ni, nj, nk) = (NBAND);
				if (nj > 0) {
					Nv = distVol(ni, nj - 1, nk);
					Nl = labelVol(ni, nj - 1, nk);
				} else {
					Nl = 0;
				}
				/* Neighbour to the south */
				if (nj < YN - 1) {
					Sv = distVol(ni, nj + 1, nk);
					Sl = labelVol(ni, nj + 1, nk);
				} else {
					Sl = 0;
				}
				/* Neighbour to the east */
				if (nk < ZN - 1) {
					Ev = distVol(ni, nj, nk + 1);
					El = labelVol(ni, nj, nk + 1);
				} else {
					El = 0;
				}
				/* Neighbour to the west */
				if (nk > 0) {
					Wv = distVol(ni, nj, nk - 1);
					Wl = labelVol(ni, nj, nk - 1);
				} else {
					Wl = 0;
				}
				/* Neighbour to the front */
				if (ni < XN - 1) {
					Fv = distVol(ni + 1, nj, nk);
					Fl = labelVol(ni + 1, nj, nk);
				} else {
					Fl = 0;
				}
				/* Neighbour to the back */
				if (ni > 0) {
					Bv = distVol(ni - 1, nj, nk);
					Bl = labelVol(ni - 1, nj, nk);
				} else {
					Bl = 0;
				}
				/*
				 * Update the value of this to-be-updated NarrowBand
				 * point
				 */
				newvalue = march(Nv, Sv, Ev, Wv, Fv, Bv, Nl, Sl, El, Wl, Fl,
						Bl);
				distVol(ni, nj, nk) = (float) (newvalue);
				voxelList.push_back(VoxelIndex(newvalue, Coord(ni, nj, nk)));
				heap.add(&voxelList.back());
			}
		};END_FOR
		imagesci::WriteToRawFile(distVol,"/home/blake/tmp/init_distanceField");
		imagesci::WriteToRawFile(vol,"/home/blake/tmp/init_vol");
		std::cout<<"Number of alive verts "<<countAlive<<" / "<<XN*YN*ZN<<std::endl;

	/*
	 * Begin Fast Marching to get the unsigned distance function inwords and
	 * outwards simultaneously since points inside and outside the contour
	 * won't interfere with each other
	 */

	while (!heap.isEmpty()) { /* There are still points not yet accepted */

		he = heap.remove();
		std::cout<<"HEAP "<<he->mValue<<std::endl;
		i = he->mIndex[0];
		j = he->mIndex[1];
		k = he->mIndex[2];
		if (he->mValue > maxDistance) {
			break;
		}
		distVol(i, j, k) = (he->mValue);
		labelVol(i, j, k) = (ALIVE);
		for (koff = 0; koff < 6; koff++) {
			ni = i + neighborsX[koff];
			nj = j + neighborsY[koff];
			nk = k + neighborsZ[koff];
			if (nj < 0 || nj >= YN || nk < 0 || nk >= ZN || ni < 0
					|| ni >= XN) {
				continue; /* Out of boundary */
			}
			if (labelVol(ni, nj, nk) == ALIVE) {
				continue; /* Don't change ALIVE neighbour */
			}
			if (nj > 0) {
				Nv = distVol(ni,nj - 1,nk);
				Nl = labelVol(ni,nj - 1,nk);
			} else {
				Nl = 0;
			}

			/* Neighbour to the south */
			if (nj < YN - 1) {
				Sv = distVol(ni,nj + 1,nk);
				Sl = labelVol(ni,nj + 1,nk);
			} else {
				Sl = 0;
			}

			/* Neighbour to the east */
			if (nk < ZN - 1) {
				Ev = distVol(ni,nj,nk + 1);
				El = labelVol(ni,nj,nk + 1);
			} else {
				El = 0;
			}

			/* Neighbour to the west */
			if (nk > 0) {
				Wv = distVol(ni,nj,nk - 1);
				Wl = labelVol(ni,nj,nk - 1);
			} else {
				Wl = 0;
			}

			/* Neighbour to the front */
			if (ni < XN - 1) {
				Fv = distVol(ni + 1,nj,nk);
				Fl = labelVol(ni + 1,nj,nk);
			} else {
				Fl = 0;
			}

			/* Neighbour to the back */
			if (ni > 0) {
				Bv = distVol(ni - 1,nj,nk);
				Bl = labelVol(ni - 1,nj,nk);
			} else {
				Bl = 0;
			}
			newvalue = march(Nv, Sv, Ev, Wv, Fv, Bv, Nl, Sl, El, Wl, Fl, Bl);
			voxelList.push_back(VoxelIndex(newvalue, Coord(ni, nj, nk)));
			VoxelIndex* vox =&voxelList.back();
			if (labelVol(ni, nj, nk) == NBAND) {
				heap.change(ni, nj, nk, vox);
			} else {
				heap.add(vox);
				labelVol(ni, nj, nk) = (NBAND);
			}
		}
	}
	OPENMP_FOR FOR_EVERY_GRID_CELL(distVol){
		if (labelVol(i, j, k) != ALIVE) {
			distVol(i, j, k) = (float) (maxDistance);
		}
		if (vol(i, j, k) < 0) {
			distVol(i, j, k) = (-distVol(i, j, k));
		}
	} END_FOR;
	heap.clear();
}
}
