
#include "xTetEqn.h"
#include "xBlockSparse.h"

template <class T> 
xTetEqn<T>::xTetEqn( int node[4], const std::vector< xMath::Vector3 >& x0, const xTetMaterial& mat)
: m_Kv(T(0.1))
{
	for(int i=0; i<4; i++)
	{
		m_node[i] = node[i];
	}

	vSetStiffness(x0, mat);
	m_R = xMath::xMatrix3::identity();
}



template <class T> 
void xTetEqn<T>::calculate(
	xBlockSparse< xMath::xMatrix3 >& dfdx,
	xBlockSparse< xMath::xMatrix3 >& dfdv,
	std::vector< xMath::Vector3 >& f,
	std::vector< xMath::Vector3 >& x,
	std::vector< xMath::Vector3 >& v,
	const std::vector< xMath::Vector3 >& x0)
{
	xMath::Vector3 xc[4];
	xMath::Vector3 vc[4];
	xMath::xMatrix3 Rt = xMath::transpose(m_R);
	for(int i=0; i<4; i++)
	{
		xc[i] = Rt*x[m_node[i]] - x0[m_node[i]];
		vc[i] = Rt*v[m_node[i]];
	}
	for(int i=0; i<4; i++)
	{
		int ni = m_node[i];
		if(ni >= 0)
		{
			xMath::Vector3 f_x(T(0.0), T(0.0), T(0.0));
			xMath::Vector3 f_v(T(0.0), T(0.0), T(0.0));
			for(int j=0; j<4; j++)
			{
				f_x -= m_K[i][j]*(xc[j] + m_Kv*vc[j]);
			}
			f[ni] += m_R*f_x;
		}
	}

	// force derivatives
	for(int i=0; i<4; i++)
	{
		int ni = m_node[i];
		if(ni >= 0)
		{
			for(int j=0; j<4; j++)
			{
				int nj = m_node[j];
				if(nj >= 0 && ni >= nj)
				{
					xMath::xMatrix3 K = -m_R*m_K[i][j]*Rt;
					dfdx.addBlock(ni, nj, K);
					dfdv.addBlock(ni, nj, K*m_Kv);
				}
			}
		}
	}
}


template class xTetEqn<float>;
