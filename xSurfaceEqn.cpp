
#include "xSurfaceEqn.h"
#include "xBlockSparse.h"

using namespace std;

template class xSurfaceEqn<float>;



template <class T>
xSurfaceEqn<T>::xSurfaceEqn( int nodes[4], T blends[4], const std::vector< xMath::Vector3 >& x0, T kp, T kv)
: m_num(4), m_normal(T(0.0)), m_x0(x0[nodes[0]]), m_Kp(kp), m_Kv(kv), m_bEnabled(true)
{
	for ( int i = 0; i < 4; ++i )
	{
		m_nodes[i] = nodes[i];
		m_blends[i] = blends[i];
	}
}


template <class T>
void xSurfaceEqn<T>::calculate(
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

	xMath::Vector3 dp(-m_x0);
	xMath::Vector3 dv(T(0.0));
	for(int ii = 0; ii < m_num; ++ii)
	{
		dp += x[m_nodes[ii]] * m_blends[ii];
		dv += v[m_nodes[ii]] * m_blends[ii];
	}
	T C = xMath::dot(dp,m_normal);
	T Cdot = xMath::dot(dv,m_normal);
	xMath::xMatrix3  JtJ(xMath::outer(m_normal,m_normal));
	for(int ii = 0; ii < m_num; ++ii)
	{
		int nai = m_nodes[ii];
		xMath::Vector3  fp = m_normal * (-m_Kp * m_blends[ii] * C);
		xMath::Vector3  fv = m_normal * (-m_Kv * m_blends[ii] * Cdot);

		f[nai] += (fp + fv);
		for(int jj = 0; jj < m_num; ++jj)
		{
			int naj = m_nodes[jj];
			if ( nai >= naj )
			{
				dfdx.addBlock(nai, naj, JtJ * (-m_Kp * m_blends[ii] * m_blends[jj]));
				dfdv.addBlock(nai, naj, JtJ * (-m_Kv * m_blends[ii] * m_blends[jj]));
			}
		}
	}
}


