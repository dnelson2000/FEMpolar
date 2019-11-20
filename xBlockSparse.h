
#pragma once
#ifndef _BLOCK_SPARSE_H_
#define _BLOCK_SPARSE_H_

#include "xMathUtils.h"
using namespace FEM;


#include "xVec23i.h"
#include <vector>

// Similar to ELL format, and a little bit of COO, see Michael Garland, spmv library, NVIDIA tech report
// http://www.nvidia.com/object/nvidia_research_pub_001.html

//#define USE_ELL 1
#define USE_COO 1

#ifdef USE_COO
template <class T> class BlockLookup;
#endif


template <class T, int BandSize=32, int BandSize2=64, int MaxRows=512>
class xBlockSparse
{
public:
	xBlockSparse() {}
	~xBlockSparse() {}

	int getSize() const { return m_size; };
	int getNumBlocks() const 
	{ 
#ifdef USE_COO
		return (int) m_elem.size();
#endif
#ifdef USE_ELL
		return (int) m_blockPtrs.size(); 
#endif
	}

	int getBandSize() const { return BandSize; }

	void resize(int size) { 
		m_size = size;
#ifdef USE_COO
		m_lookup.resize(size); 
#endif
	}

	void reserve(int size) 
	{
		m_index.reserve(size); 
#ifdef USE_COO
		m_elem.reserve(size); 
#endif
#ifdef USE_ELL
		m_blockPtrs.reserve(size);
#endif
	}

	T* findBlock(int i, int j) const;	
	T& getBlock(int i, int j);
	T& getBlock(int i, int j, const T& value);
	
	T* createBlock(int i, int j);
	T* createBlock(int i, int j, const T& value);
	void addBlock(int i, int j, const T& block);

	void removeBlock(int i, int j);
	void clear();

	template <class U> void blockMultiply(std::vector<U>& v, const std::vector<U>& u, float c) const;

	const Vec2i& getIndex(int i) const { return m_index[i]; }

	int maxBandedDiagonal() const;

private:
	// Similar to ELL format, and a little bit of COO, see Michael Garland, spmv library, NVIDIA tech report
#ifdef USE_ELL
	T m_block[MaxRows][BandSize2];
	T * m_exists[MaxRows][BandSize2];
	std::vector<T*> m_blockPtrs;
#endif

	int m_size;
	std::vector<Vec2i> m_index;

#ifdef USE_COO
	// COO part
	std::vector<T> m_elem;
	BlockLookup<T> m_lookup;
#endif
};


template <class T, int BandSize, int BandSize2, int MaxRows>
T* xBlockSparse<T,BandSize,BandSize2,MaxRows>::findBlock(int i, int j) const
{	
	T* res;
#ifdef USE_COO
	res = m_lookup.find(i,j);
#endif

#ifdef USE_ELL
	int c = BandSize+j-i;
	if ( c < 0 || c >= BandSize2 )
	{
		return NULL;
	}
	res = m_exists[i][BandSize+j-i];
#endif
	return res;
}


template <class T, int BandSize, int BandSize2, int MaxRows>
T* xBlockSparse<T,BandSize,BandSize2,MaxRows>::createBlock(int i, int j)
{
	T* res;
	m_index.push_back(Vec2i(i, j));
#ifdef USE_COO
	m_elem.push_back(T(float(0)));	
	res = &m_elem.back();
	m_lookup.insert(i, j, &m_elem.back());
#endif

#ifdef USE_ELL
	res = &(m_block[i][BandSize+j-i]);
	*res = T(float(0));
	m_exists[i][BandSize+j-i] = res;
	m_blockPtrs.push_back(res);
#endif
	return res; 
}

template <class T, int BandSize, int BandSize2, int MaxRows>
T* xBlockSparse<T,BandSize,BandSize2,MaxRows>::createBlock(int i, int j, const T& value)
{
	T* res;
	m_index.push_back(Vec2i(i, j));
#ifdef USE_COO
	m_elem.push_back(value);
	m_lookup.insert(i, j, &m_elem.back());
	res = &m_elem.back();
#endif

#ifdef USE_ELL
	res = &(m_block[i][BandSize+j-i]);
	*res = value;
	m_exists[i][BandSize+j-i] = res;
	m_blockPtrs.push_back(res);
#endif
	return res; 
}

template <class T, int BandSize, int BandSize2, int MaxRows>
T& xBlockSparse<T,BandSize,BandSize2,MaxRows>::getBlock(int i, int j)
{
	T* block;
#ifdef USE_COO
	block = m_lookup.find(i,j);
	if(block == NULL)
	{
		block = createBlock(i, j);
	}
#endif

#ifdef USE_ELL
	block = m_exists[i][BandSize+j-i];
	if(block == NULL)
	{
		block = createBlock(i, j);
	}
#endif
	return *block;
}

template <class T, int BandSize, int BandSize2, int MaxRows>
T& xBlockSparse<T,BandSize,BandSize2,MaxRows>::getBlock(int i, int j, const T& value)
{
	T* block;
#ifdef USE_COO
	block = m_lookup.find(i,j);
	if(block == NULL)
	{
		block = createBlock(i, j, value);
	}
#endif

#ifdef USE_ELL
	block = m_exists[i][BandSize+j-i];
	if(block == NULL)
	{
		block = createBlock(i, j, value);
	}
#endif
	return *block;
}

template <class T, int BandSize, int BandSize2, int MaxRows>
void xBlockSparse<T,BandSize,BandSize2,MaxRows>::addBlock(int i, int j, const T& value)
{
	if(i < 0 || j < 0 || i < j) // use upper diagonal only
		return;

	T* block;
#ifdef USE_COO
	block = m_lookup.find(i,j);
	if(block != NULL)
		*block += value;
	else
		createBlock(i, j, value);
#endif
#ifndef USE_ELL
	return;
#endif

#ifdef USE_ELL
	block = m_exists[i][BandSize+j-i];
	if(block != NULL)
		*block += value;
	else
		createBlock(i, j, value);
#endif
}

template <class T, int BandSize, int BandSize2, int MaxRows>
void xBlockSparse<T,BandSize,BandSize2,MaxRows>::removeBlock(int i, int j)
{
#ifdef USE_COO
	m_lookup.erase(i,j);
#endif
#ifdef USE_ELL
	m_exists[i][BandSize+j-i] = NULL;
#endif
}

template <class T, int BandSize, int BandSize2, int MaxRows>
void xBlockSparse<T,BandSize,BandSize2,MaxRows>::clear()
{
#ifdef USE_COO
	m_lookup.clear(m_index);
	m_elem.resize(0);
#endif
	
	m_index.resize(0);

#ifdef USE_ELL
	m_blockPtrs.resize(0);
	T** e0 = (T**)(&(m_exists[0][0]));
	for ( int i = 0; i < MaxRows*BandSize2; ++i ) e0[i] = (T*)NULL;
#endif
}

template <class T, int BandSize, int BandSize2, int MaxRows> template <class U>
void xBlockSparse<T,BandSize,BandSize2,MaxRows>::blockMultiply(std::vector<U>& v, const std::vector<U>& u, float scalar ) const
{
	if(m_index.empty())
		return;

	Vec2i* idx = (Vec2i*)&(m_index.front());
	U* vd = (U*)&(v.front());
	U* ud = (U*)&(u.front());

	int sz;
#ifdef USE_COO
	T* elem = (T*)&(m_elem.front());
	sz = (int) m_elem.size();
#endif
#ifdef USE_ELL
	sz = (int) m_blockPtrs.size();
#endif

	for(int i=0; i < sz; ++i )
	{
		int ii = idx[i][0];
		int jj = idx[i][1];
		bool found;

#ifdef USE_COO
		found = m_lookup.find(ii,jj);
#endif
#ifdef USE_ELL
		found = bool(m_exists[ii][BandSize+jj-ii]);
#endif
		if ( found )
		{
			T * pBlocki;
#ifdef USE_COO
			pBlocki = &(elem[i]);
#endif
#ifdef USE_ELL
			pBlocki = (m_blockPtrs[i]);
#endif

			U scratch = (*pBlocki) * ud[jj];
			vd[ii] += scratch * scalar;
			if(ii != jj)
			{
				scratch = xMath::transpose((*pBlocki)) * ud[ii]; 
				vd[jj] += scratch * scalar;
			}
		}
	}
}



template <class T, int BandSize, int BandSize2, int MaxRows>
int xBlockSparse<T,BandSize,BandSize2,MaxRows>::maxBandedDiagonal() const
{
	int banded = 0;
	for(int i = 0; i < (int) m_index.size(); ++i)
		banded = std::max(int(std::abs(int(m_index[i][0]) - int(m_index[i][1]))), banded);
	return banded;
}


#ifdef USE_COO
template <class T>
class BlockLookup
{
public:
	BlockLookup() { m_size = 0; }
	BlockLookup(int size) : m_lookup(size*size, NULL) { m_size = size; }
	~BlockLookup() {}

	T* find(int i, int j) const { return m_lookup[i*m_size + j]; }
	void insert(int i, int j, T* elem) { m_lookup[i*m_size + j] = elem; }
	void erase(int i, int j) { m_lookup[i*m_size + j] = NULL; }
	void clear() { std::fill(m_lookup.begin(), m_lookup.end(), (T*)NULL); }
	
	void clear(const Vec2i* index, int index_size)
	{
		for(int i=0; i<index_size; i++)
		{
            int idx = index[i][0]*m_size + index[i][1];
            if (idx < (int) m_lookup.size())
			    m_lookup[idx] = (T*)NULL;
		}
	}
	void clear(const std::vector<Vec2i>& index)
	{
		int sz = (int) index.size();
		if ( sz )
			clear(&index.front(), sz);
	}
	int size() const { return m_size; }

	void resize(int size) { m_size = size; m_lookup.resize(m_size*m_size); this->clear(); }

protected:
	int m_size;
	std::vector<T*> m_lookup;
};
#endif


template <class T> unsigned int xPolarDecomp(xMath::xMatrix3& res, const xMath::xMatrix3& M0, T eps);

int CGBlock( xBlockSparse<xMath::xMatrix3> &A, std::vector<xMath::Vector3> &x, const std::vector<xMath::Vector3> &b, int &max_iter, float tol);

bool CholFactor(xBlockSparse< xMath::xMatrix3, 32, 64, 512 >& L, const int banded, const float eps);

void CholSolve(std::vector< xMath::Vector3 >& x, const xBlockSparse< xMath::xMatrix3, 32, 64, 512 >& L, const std::vector< xMath::Vector3 >& b, 
			   std::vector< xMath::Vector3 >& xPass2, const int banded );


#endif // _BLOCK_SPARSE_H_
