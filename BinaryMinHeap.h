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
#ifndef BINARYMINHEAP_H_
#define BINARYMINHEAP_H_
#include <openvdb/openvdb.h>
#include <vector>
namespace imagesci {
template<typename ScalarT>
struct Indexable{
public:
	ScalarT mValue;
	openvdb::Coord mIndex;
	size_t mChainIndex;
	Indexable(ScalarT value,const openvdb::Coord& index):mChainIndex(0),mValue(value),mIndex(index){
	}
	void setChainIndex(size_t idx){
		mChainIndex=idx;
	}
};
template<typename ScalarT>
class BinaryMinHeap {
protected:
	typedef Indexable<ScalarT> IndexableType;
	static const int DEFAULT_CAPACITY = 100;
	std::vector<IndexableType*> mArray; // The heap array
	RegularGrid<size_t> mBackPointers;
	size_t mCurrentSize; // Number of elements in heap
public:
	BinaryMinHeap(int XN, int YN, int ZN):mBackPointers(XN,YN,ZN,1.0f,0) {
		mCurrentSize = 0;
	}
	void reserve(size_t capacity){
		mArray.resize(capacity + 2,nullptr);
	}
	bool isEmpty() {
		return mCurrentSize == 0;
	}
	IndexableType* peek() {
		if (isEmpty()) {
			throw imagesci::Exception("Empty binary heap");
		}
		return mArray[1];
	}
	size_t size() {
		return mCurrentSize;
	}
	void change(int i, int j, int k, IndexableType* x) {
		size_t index = mBackPointers(i,j,k);
		IndexableType* v = mArray[index];
		if (x != v) {
			mArray[index] = x;
			if (x->mValue<v->mValue) {
				percolateUp(index);
			} else {
				percolateDown(index);
			}
		}
	}
	void change(IndexableType node,ScalarT value) {
		change(node.mIndex[0],node.mIndex[1],node.mIndex[2], value);
	}
	void change(int i, int j, int k, ScalarT value) {
		size_t index = mBackPointers(i,j,k);
		IndexableType v = mArray[index];
		if (value<v.mValue) {
			v.setValue(value);
			percolateUp(index);
		} else {
			v.setValue(value);
			percolateDown(index);
		}
	}
	void add(IndexableType* x) {
		if (mCurrentSize + 1 == mArray.size()) {
			resize();
		}
		int hole = ++mCurrentSize;
		mArray[0] = x;
		for (; x->mValue<mArray[hole / 2]->mValue; hole /= 2) {
			mArray[hole] = mArray[hole / 2];
			mBackPointers(mArray[hole]->mIndex)=hole;
		}
		mBackPointers(x->mIndex) = hole;
		mArray[hole] = x;
	}
	IndexableType* remove() {
		IndexableType* minItem = peek();
		mArray[1] = mArray[mCurrentSize--];
		percolateDown(1);
		return minItem;
	}
	void clear() {
		mCurrentSize = 0;
		mArray.clear();
	}
protected:
	void buildHeap() {
		for (int i = mCurrentSize / 2; i > 0; i--) {
			percolateDown(i);
		}
	}


	void percolateDown(size_t parent) {
		int child;
		IndexableType* tmp = mArray[parent];
		if (tmp == nullptr) {
			return;
		}
		for (; parent * 2 <= mCurrentSize; parent = child) {
			child = parent * 2;
			if (mArray[child] == nullptr) {
				parent = child;
				break;
			}
			if (mArray[child + 1] == nullptr) {
				parent = child + 1;
				break;
			}
			if (child != mCurrentSize
					&& mArray[child + 1]->mValue<mArray[child]->mValue) {
				child++;
			}
			if (mArray[child]->mValue<tmp->mValue) {
				mArray[parent] = mArray[child];
				mBackPointers(mArray[parent]->mIndex) = parent;
			} else {
				break;
			}
		}
		mArray[parent] = tmp;
		mBackPointers(mArray[parent]->mIndex)=parent;
	}

	void percolateUp(int k) {
		int k_father;
		IndexableType* v = mArray[k];
		k_father = k / 2; /* integer divsion to retrieve its parent */
		while (k_father > 0
				&& mArray[k_father]->mValue>v->mValue) {
			mArray[k] = mArray[k_father];
			mBackPointers(mArray[k]->mIndex)=k;
			k = k_father;
			k_father = k / 2;
		}
		mArray[k] = v;
		mBackPointers(mArray[k]->mIndex)=k;
	}

	void resize() {
		mArray.resize(mArray.size()*2,nullptr);
	}


};

};
#endif
