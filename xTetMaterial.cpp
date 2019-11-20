
#include "xTetMaterial.h"
#include "xMathUtils.h"

#include <vector>
using namespace std;

xTetMaterial::xTetMaterial()
: m_E(20000.f)
, m_nu(0.45)
, m_G(20000.f/(2.0f*(1.0f+0.45)))
, m_rho(10.f)
, m_Kv(1.f)
{}

xTetMaterial::xTetMaterial(float E, float nu, float rho, float kv)
: m_E(E)
, m_nu(nu)
, m_G(E/(2.0f*(1.0f+nu)))
, m_rho(rho)
, m_Kv(kv)
{}


void xTetMaterial::getMaterialMatrix(xMath::xMatrix3& D0, xMath::xMatrix3& D1) const
{
	xMath::xMatrix3 invD0 = xMath::makexMatrix3(
		1.0f/m_E[0], -m_nu[2]/m_E[0], -m_nu[1]/m_E[2],
		-m_nu[2]/m_E[0], 1.0f/m_E[1], -m_nu[0]/m_E[1],
		-m_nu[1]/m_E[2], -m_nu[0]/m_E[1], 1.0f/m_E[2]
	);
	D0 = xMath::inverse(invD0);
	xMath::Vector3 mg(m_G[0], m_G[1], m_G[2]);
	D1 = makeDiagonal(mg);
}
