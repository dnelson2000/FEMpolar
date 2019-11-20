
#include "xTetrahedra.h"
#include "xMathUtils.h"

#include <algorithm>
#include <map>
#include <set>

using namespace std;

template class xTetrahedra<float>;

typedef std::vector< xTetEqn<float> >::iterator xTetIterator;


void xTetrahedra<float>::calculate(
	xBlockSparse<xMath::xMatrix3>& dfdx,
	xBlockSparse<xMath::xMatrix3>& dfdv,
	std::vector<xMath::Vector3>& f,
	std::vector<xMath::Vector3>& x,
	std::vector<xMath::Vector3>& v,
	const std::vector<xMath::Vector3>& x0)
{
	if(!m_bEnabled)
		return;
	for(vector< xTetEqn<float> >::iterator iter=m_tets.begin(); iter != m_tets.end(); ++iter)
	{
		iter->calculate(dfdx, dfdv, f, x, v, x0);
	}
}



template <class T>
void xTetrahedra<T>::calculate(
	xBlockSparse<xMath::xMatrix3>& dfdx,
	xBlockSparse<xMath::xMatrix3>& dfdv,
	std::vector<xMath::Vector3>& f,
	std::vector<xMath::Vector3>& x,
	std::vector<xMath::Vector3>& v,
	const std::vector<xMath::Vector3>& x0)
{
	if(!m_bEnabled)
		return;

	for ( int i = 0; i < int(m_tets.size()); ++i )
	{
		m_tets[i].calculate(dfdx, dfdv, f, x, v, x0);
	}
}

