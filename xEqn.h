#pragma once
#ifndef _EQN_H_
#define _EQN_H_

#include "shapesUtils/StlUtils.h"
#include "shapesUtils/Vec3.h"
#include "xMathUtils.h"

using namespace FEM;

#include <vector>

#include "xBlockSparse.h"

//!	Abstract class for constraints and forces
template <class T>
class xEqn
{
public:
	virtual ~xEqn() {}

	virtual void calculate(
		xBlockSparse< xMath::xMatrix3 >& dfdx,
		xBlockSparse< xMath::xMatrix3 >& dfdv,
		std::vector< xMath::Vector3 >& f,
		std::vector< xMath::Vector3 >& x,
		std::vector< xMath::Vector3 >& v,
		const std::vector< xMath::Vector3 >& x0) = 0;

protected:
};

#endif // _EQN_H_
