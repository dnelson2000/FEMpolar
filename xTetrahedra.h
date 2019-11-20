
#pragma once
#ifndef _xTetrahedra_H_
#define _xTetrahedra_H_

#include "xTetEqn.h"

template <class T>
class xTetrahedra
{
public:
	xTetrahedra() {}
	virtual ~xTetrahedra() {}

	virtual void calculate(
		xBlockSparse< xMath::xMatrix3 >& dfdx,
		xBlockSparse< xMath::xMatrix3 >& dfdv,
		std::vector< xMath::Vector3 >& f,
		std::vector< xMath::Vector3 >& x,
		std::vector< xMath::Vector3 >& v,
		const std::vector< xMath::Vector3 >& x0);

	void vLumpedNodes()
	{
		m_lumpedNodes.resize(m_nNodes);
		for(int i=0; i < (int) m_lumpedNodes.size(); i++)
			m_lumpedNodes[i].clear();
		for(int i=0; i < (int) m_tets.size(); i++)
		{
			int* node = m_tets[i].getNodes();
			m_lumpedNodes[node[0]-m_startNode].push_back(i);
			m_lumpedNodes[node[1]-m_startNode].push_back(i);
			m_lumpedNodes[node[2]-m_startNode].push_back(i);
			m_lumpedNodes[node[3]-m_startNode].push_back(i);
		}
	}
	
	void updatePolar( const std::vector< xMath::Vector3 >& x, const std::vector< xMath::Vector3 >& x0 )
	{
		if(!m_bEnabled)
			return;
		
		for ( int i = 0; i < int(m_tets.size()); ++i ) {
			m_tets[i].updatePolar(x, x0);
		}
	}
	
	int getStartNode() { return m_startNode; }
	int getNumNodes() { return m_nNodes; }
	std::vector< xTetEqn<T> > & getTets() { return m_tets; }

	void vEnable(bool enabled) { m_bEnabled = enabled; }
	bool bEnabled() { return m_bEnabled; }

protected:
	bool m_bEnabled;
	std::vector< xTetEqn<T> > m_tets;
	std::vector< xTetMaterial > m_centerMaterial;
	int m_startNode;
	int m_nNodes;
	std::vector< std::vector<int> > m_lumpedNodes;
};

#endif // _xTetrahedra_H_
