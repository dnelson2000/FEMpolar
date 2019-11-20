

#pragma once
#ifndef _TET_MATERIAL_H_
#define _TET_MATERIAL_H_

#include "xMathUtils.h"
using namespace FEM;

class xTetMaterial
{
public:
	xTetMaterial();
	xTetMaterial(float E, float nu, float rho, float kv);
	~xTetMaterial() {}

	void getMaterialMatrix(xMath::xMatrix3& D0, xMath::xMatrix3& D1) const;

	void setDensity(float rho) { m_rho = rho; }
	void setDamping(float kv) { m_Kv = kv; }

	float getDensity() const { return m_rho; }
	float getDamping() const { return m_Kv; }

	const xMath::Vector3& getYoungsModulus() const { return m_E; }
	const xMath::Vector3& getPoissonsRatio() const { return m_nu; }
	const xMath::Vector3& getShearModulus() const { return m_G; }
	void setParam(float E, float nu) { m_E = xMath::Vector3(E); m_nu = xMath::Vector3(nu); }

protected:
	xMath::Vector3 m_E;
	xMath::Vector3 m_nu;
	xMath::Vector3 m_G;
	float m_rho;
	float m_Kv;
};

#endif // _TET_MATERIAL_H_
