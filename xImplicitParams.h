
#pragma once
#ifndef _IMPLICIT_PARAMS_
#define _IMPLICIT_PARAMS_

#include "xMathUtils.h"
using namespace FEM;

#include "xTetMaterial.h"
#include "xBlockSparse.h"
#include "xMechanicalElem.h"


#include <vector>

template <class T> class xSpringEqn;
template <class T> class xTetRod;
template <class T> class xTetBrick;
template <class T> class xSpringEqn;
template <class T> class xPointEqn;
template <class T> class xSurfaceEqn;


struct MechanicsStatePOD
{
	MechanicsStatePOD();

	float fViscousDamping;
	float dt;
	
	std::vector<xPointEqn<float>*> m_pTipPt;	
	std::vector<xPointEqn<float>*> m_pManipPt;	
	std::vector<xSurfaceEqn<float>*> m_pSidePlanes;	
	std::vector<xSpringEqn<float>*> m_pSpringEqn;
	xTetBrick<float>* brick;
	
	std::vector<xMath::Vector3> x;
	std::vector<xMath::Vector3> x0;
	std::vector<xMath::Vector3> v;
	std::vector<xMath::Vector3> m;

	std::vector<xMath::Vector3> v1;
	std::vector<xMath::Vector3> f;
	xBlockSparse<xMath::xMatrix3> dfdx;
	xBlockSparse<xMath::xMatrix3> dfdv;
	xBlockSparse<xMath::xMatrix3> systemMatrix;

	std::vector<xMath::Vector3> scratch;

	void clear();
};



#endif


