
#include "xMathUtils.h"
#include <math.h>

using namespace std;

namespace xMath {

xMath::xMatrix3 makexMatrix3( xMath::Vector3 &v0, xMath::Vector3 &v1, xMath::Vector3 &v2 )
{
	xMath::xMatrix3 rot;
	rot.setCol(0,v0);
	rot.setCol(1,v1);
	rot.setCol(2,v2);
	return rot;
}

xMath::xMatrix3 makexMatrix3( float a, float b, float c, float d, float e, float f, float g, float h, float i )
{
	xMath::xMatrix3 rot;
	xMath::Vector3 row0,row1,row2;
	row0[0] = a; row0[1] = b; row0[2] = c; 
	row1[0] = d; row1[1] = e; row1[2] = f; 
	row2[0] = g; row2[1] = h; row2[2] = i;
	rot.setRow(0,row0);
	rot.setRow(1,row1);
	rot.setRow(2,row2);
	return rot;
}


xMath::xMatrix4 makexMatrix4( float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k, float l, float m, float n, float o, float p )
{
	xMath::xMatrix4 rot;
	xMath::Vector4 row0,row1,row2,row3;
	row0[0] = a; row0[1] = b; row0[2] = c; row0[3] = d; 
	row1[0] = e; row1[1] = f; row1[2] = g; row1[3] = h; 
	row2[0] = i; row2[1] = j; row2[2] = k; row2[3] = l; 
	row3[0] = m; row3[1] = n; row3[2] = o; row3[3] = p; 
	rot.setRow(0,row0);
	rot.setRow(1,row1);
	rot.setRow(2,row2);
	rot.setRow(3,row3);
	return rot;
}

xMath::xMatrix3 makeDiagonal( xMath::Vector3 &diag )
{
	xMath::xMatrix3 rot;
	xMath::Vector3 row0,row1,row2;
	row0[0] = diag[0]; row0[1] = 0.f; row0[2] = 0.f; 
	row1[0] = 0.f; row1[1] = diag[1]; row1[2] = 0.f; 
	row2[0] = 0.f; row2[1] = 0.f; row2[2] = diag[2];
	rot.setRow(0,row0);
	rot.setRow(1,row1);
	rot.setRow(2,row2);
	return rot;
}

}

/*
 * polar decomposition : extract a rotation from current xTetrahedral configuration pose
 * see http://www.sofa-framework.org/
 * M = QS.  Also Nicholas Higham and Robert S. Schreiber,
 * Fast Polar Decomposition of An Arbitrary xxMatrix,
 * Technical Report 88-942, October 1988,
 * Department of Computer Science, Cornell University.
 * Or, see the polar decomp algorithm in Schneider, Eberly, Geometric Tools for Computer Graphics.
 * Most closely follows this work : http://www.hpl.hp.com/techreports/94/HPL-94-117.pdf
 */



template <class T>
unsigned int xPolarDecomp(xMath::xMatrix3& res, const xMath::xMatrix3& M0, T eps)
{
	xMath::xMatrix3 M = xMath::transpose(M0);
	T M_L1 = xMatrixNorm1(M);
	T M_Linf = xMatrixNormInf(M);
	T Ek_L1(0);
	unsigned int iter = 0;
	do
	{
		xMath::xMatrix3 Mi = cofactor(M);
		T det = xMath::determinant(M);
		T Mi_L1 = xMatrixNorm1(Mi);
		T Mi_Linf = xMatrixNormInf(Mi);
		T gamma = sqrtf(sqrtf((Mi_L1*Mi_Linf)/(M_L1*M_Linf))/fabsf(det));
		T g1 = gamma*T(0.5);
		T g2 = T(0.5)/(gamma*det);
		xMath::xMatrix3 Ek(M);
		M = M*g1 + Mi*g2;
		Ek -= M;
		Ek_L1 = xMatrixNorm1(Ek);
		M_L1 = xMatrixNorm1(M);
		M_Linf = xMatrixNormInf(M);
		iter++;
	} while(Ek_L1 > M_L1*eps);
	res = xMath::transpose(M);
	return iter;
}


template unsigned int xPolarDecomp(xMath::xMatrix3& Q, const xMath::xMatrix3& A, float eps);

