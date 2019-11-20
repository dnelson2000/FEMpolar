
#include "xPointEqn.h"
#include "xBlockSparse.h"

using namespace std;

template class xPointEqn<float>;


template <class T>
void xPointEqn<T>::calculate(
	xBlockSparse< xMath::xMatrix3 >& dfdx,
	xBlockSparse< xMath::xMatrix3 >& dfdv,
	std::vector< xMath::Vector3 >& f,
	std::vector< xMath::Vector3 >& x,
	std::vector< xMath::Vector3 >& v,
	const std::vector< xMath::Vector3 >& x0)
{
	if(!m_bEnabled) {
		return;
	}


	xMath::Vector3 xSum(0.f,0.f,0.f);
	for( int i = 0; i < m_num; ++i )
	{
		xSum += m_blends[i]*x[m_nodes[i]];
	}

	//xMath::Vector3 dv(v[m_nodes[i]]);
	//T Cdot = dv;
	float gainKp = (- m_Kp);
	//float gainKv = (- m_Kv);
	xMath::Vector3 C(xSum - m_x0);
	
	// point - point can be xi - xj with blend0 of 1.0 and blend1 of -1.0 and setting m_x0 to [ 0 0 0 ]
	// two point to two point can be blend0*xi0 + blend1*xi1 - blend2*xj0 - blend3*xj1

	for( int i = 0; i < m_num; ++i )
	{
		xMath::Vector3 fi = (m_blends[i]*gainKp) * C; // (m_blends[i]*gainKv) * Cdot);
		f[m_nodes[i]] += fi;

		for( int j = 0; j < m_num; ++j )
		if ( m_nodes[i] >= m_nodes[j] )
		{
			xMath::xMatrix3 tmp_dfdx = (m_blends[i]*m_blends[j]*gainKp) * xMath::xMatrix3::identity();
			dfdx.addBlock(m_nodes[i], m_nodes[j], tmp_dfdx);
			//xMath::xMatrix3 tmp_dfdv = (m_blends[i]*m_blends[j]*gainKv) * xMath::xMatrix3::identity();
			//dfdv.addBlock(m_nodes[i], m_nodes[j], tmp_dfdv);
		}
	}
}
