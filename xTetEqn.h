
#pragma once
#ifndef _TET_EQN_H_
#define _TET_EQN_H_

#include "xEqn.h"

#include "xTetMaterial.h"
#include "xMathUtils.h"
#include "xBlockSparse.h"

template <class T>
class xTetEqn : public xEqn<T>
{
public:
	xTetEqn(
		int node[4],
		const std::vector< xMath::Vector3 >& x0,
		const xTetMaterial& mat = xTetMaterial());

	~xTetEqn() {}

	void calculate(
		xBlockSparse< xMath::xMatrix3 >& dfdx,
		xBlockSparse< xMath::xMatrix3 >& dfdv,
		std::vector< xMath::Vector3 >& f,
		std::vector< xMath::Vector3 >& x,
		std::vector< xMath::Vector3 >& v,
		const std::vector< xMath::Vector3 >& x0);

	void updatePolar(const std::vector< xMath::Vector3 >& x, const std::vector< xMath::Vector3 >& x0);
	void vSetStiffness(const std::vector< xMath::Vector3 >& x0, const xTetMaterial& mat);

	int* getNodes() { return m_node; }
	const int* getNodes() const { return m_node; }

protected:
	int m_node[4];
	xMath::xMatrix3 m_K[4][4];
	T m_Kv; 
	xMath::xMatrix3 m_R;	
};



template <class T> inline
void xTetEqn<T>::updatePolar(const std::vector< xMath::Vector3 >& x, const std::vector< xMath::Vector3 >& x0)
{
	xMath::Vector3 x0_0 = x0[m_node[0]];
	xMath::Vector3 x0_1 = x0[m_node[1]];
	xMath::Vector3 x0_2 = x0[m_node[2]];
	xMath::Vector3 x0_3 = x0[m_node[3]];
	xMath::Vector3 x_0 = x[m_node[0]];
	xMath::Vector3 x_1 = x[m_node[1]];
	xMath::Vector3 x_2 = x[m_node[2]];
	xMath::Vector3 x_3 = x[m_node[3]];
	xMath::Vector3 d0 = x0_1-x0_0, d1 = x0_2-x0_0, d2 = x0_3-x0_0;
	xMath::Vector3 e0 = x_1-x_0, e1 = x_2-x_0, e2 = x_3-x_0;
	xMath::xMatrix3 P(d0,d1,d2);
	xMath::xMatrix3 Q(e0,e1,e2);
	xMath::xMatrix3 B = Q * xMath::inverse(P);
	xPolarDecomp(m_R, B, T(1e-4));
	double det = xMath::determinant(m_R);
	if(det < 0)
	{
		m_R = -m_R;
	}
}

template <class T> inline
void xTetEqn<T>::vSetStiffness(const std::vector< xMath::Vector3 >& x0, const xTetMaterial& mat)
{
	xMath::xMatrix3 D[2];
	xMath::xMatrix3 Dmat[2];
	mat.getMaterialMatrix(Dmat[0], Dmat[1]);
	D[0] = xMath::xMatrix3(Dmat[0]);
	D[1] = xMath::xMatrix3(Dmat[1]);
	xMath::Vector3 x0_0 = x0[m_node[0]];
	xMath::Vector3 x0_1 = x0[m_node[1]];
	xMath::Vector3 x0_2 = x0[m_node[2]];
	xMath::Vector3 x0_3 = x0[m_node[3]];
	xMath::Vector3 d0 = x0_1-x0_0, d1 = x0_2-x0_0, d2 = x0_3-x0_0;
	xMath::xMatrix3 V = makexMatrix3(d0,d1,d2);
	float v = fabs(xMath::determinant(V)/T(6.f));
	xMath::xMatrix4 A0 = xMath::makexMatrix4(
		x0_0[0], x0_1[0], x0_2[0], x0_3[0],
		x0_0[1], x0_1[1], x0_2[1], x0_3[1],
		x0_0[2], x0_1[2], x0_2[2], x0_3[2],
		1.0, 1.0, 1.0, 1.0);
	xMath::xMatrix4 A = xMath::inverse(A0);
	xMath::xMatrix3 B[2][4];
	xMath::xMatrix3 Bt[4][2];
	for(int i=0; i<4; i++)
	{
		xMath::Vector3 dA(A.getElem(0,i),A.getElem(1,i),A.getElem(2,i));
		B[0][i] = makeDiagonal( dA );
		B[1][i] = xMath::makexMatrix3(
			A.getElem(1,i), A.getElem(0,i), 0.0,
			0.0, A.getElem(2,i), A.getElem(1,i),
			A.getElem(2,i), 0.0, A.getElem(0,i));
		Bt[i][0] = xMath::transpose(B[0][i]);
		Bt[i][1] = xMath::transpose(B[1][i]);
		B[0][i] = D[0]*B[0][i]*v;
		B[1][i] = D[1]*B[1][i]*v;
	}
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
		{
			m_K[i][j] = xMath::xMatrix3(0.0);
			for(int k=0; k<2; k++)
			{
				m_K[i][j] += Bt[i][k]*B[k][j];
			}
		}
	}
	m_Kv = mat.getDamping();
}



#endif // _TET_EQN_H_
