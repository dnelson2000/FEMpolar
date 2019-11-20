
#include "xSpringEqn.h"
#include "xBlockSparse.h"

using namespace std;

template class xSpringEqn<float>;

template <class T>
xSpringEqn<T>::xSpringEqn(int nodei, int nodej, T kp,	T kv, std::vector< xMath::Vector3 >& x0 ) 
: m_Kp(kp), m_Kv(kv), m_bEnabled(true)
{
	m_nodes[0] = nodei;
	m_nodes[1] = nodej;
	m_len0 = xMath::length(x0[nodei]-x0[nodej]);
}

template <class T>
void xSpringEqn<T>::vResetRestLen(std::vector< xMath::Vector3 >& xref ) 
{
	m_len0 = xMath::length(xref[m_nodes[0]]-xref[m_nodes[1]]);
}

template <class T>
void xSpringEqn<T>::calculate(
	xBlockSparse< xMath::xMatrix3 >& dfdx,
	xBlockSparse< xMath::xMatrix3 >& dfdv,
	std::vector< xMath::Vector3 >& f,
	std::vector< xMath::Vector3 >& x,
	std::vector< xMath::Vector3 >& v,
	const std::vector< xMath::Vector3 >& x0)
{
	int ia = m_nodes[0];
	int ja = m_nodes[1];

	xMath::Vector3 dx(x[m_nodes[0]] - x[m_nodes[1]]);
	T dxn = xMath::length(dx);
	dx = xMath::normalize(dx);
	T C = dxn - m_len0;
	//xMath::Vector3 fi = dx * (- m_Kp * C); // - m_Kv * Cdot);

	xMath::Vector3 dv(v[m_nodes[0]] - v[m_nodes[1]]);
	T Cdot = xMath::dot(dx,dv);
	xMath::Vector3 fi = dx * (- m_Kp * C - m_Kv * Cdot);

	/* For the derivatives formulation see Choi and Ko Siggraph 2002 
	 http://graphics.snu.ac.kr/~kjchoi/publication/cloth.pdf */

	xMath::xMatrix3 dxTdx(xMath::outer(dx,dx));
	T u = m_len0 / dxn;
	xMath::xMatrix3 D = xMath::xMatrix3::identity() * ((1.0f - u) * -m_Kp);
	xMath::xMatrix3 dfdx3 = D + dxTdx * (u * -m_Kp);

	if(ia >= 0)
	{
		f[ia] += fi;
	}
	if(ja >= 0)
	{
		f[ja] -= fi;
	}

	dfdx.addBlock(ia, ia, dfdx3);
	//dfdv.addBlock(ia, ia, dfdv3);
	dfdx.addBlock(ja, ja, dfdx3);
	//dfdv.addBlock(ja, ja, dfdv3);
	dfdx.addBlock(max(ia, ja), min(ia, ja), -dfdx3);
	//dfdv.addBlock(std::max(ia, ja), std::min(ia, ja), -dfdv3);
}

