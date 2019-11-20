
#pragma once
#ifndef _POINT_EQN_H_
#define _POINT_EQN_H_

#include "xEqn.h"

template <class T>
class xPointEqn : public xEqn<T>
{
public:
	xPointEqn(int nodei, T kp,	T kv, const std::vector< xMath::Vector3 >& x0 ) 
	: m_Kp(kp), m_Kv(kv), m_bEnabled(true), m_num(1)
	{
		m_nodes[0] = nodei;
		m_blends[0] = 1.f;
		m_x0 = x0[nodei];
	}

	virtual ~xPointEqn() {}

	virtual void calculate(
		xBlockSparse< xMath::xMatrix3 >& dfdx,
		xBlockSparse< xMath::xMatrix3 >& dfdv,
		std::vector< xMath::Vector3 >& f,
		std::vector< xMath::Vector3 >& x,
		std::vector< xMath::Vector3 >& v,
		const std::vector< xMath::Vector3 >& x0);

	void setPosition(const xMath::Vector3& x) { m_x0 = x; }
	const xMath::Vector3& getPosition() { return m_x0; }
	xMath::Vector3 getPosition(const std::vector< xMath::Vector3 >& x) const
	{
		xMath::Vector3 pos(0.0);
		for(int ii = 0; ii < m_num; ++ii)
		{
			pos += x[m_nodes[ii]] * m_blends[ii];
		}
		return pos;
	}

	int getNode() { return m_nodes[0]; }
	int getNumNodes() { return m_num; }

	void vSetNodes(const int node ) { m_num = 1; m_nodes[0] = node; m_blends[0] = 1.f; }
	void vSetNodes(const int node0, const int node1, const float blend0, const float blend1) { m_num = 2; m_nodes[0] = node0; m_nodes[1] = node1;  m_blends[0] = blend0; m_blends[1] = blend1; }
	void vSetNodes(const int node0, const int node1, const int node2, const float blend0, const float blend1, const float blend2) { m_num = 3; m_nodes[0] = node0; m_nodes[1] = node1;  m_nodes[2] = node2;  m_blends[0] = blend0; m_blends[1] = blend1; m_blends[2] = blend2; }
	void vSetNodes(const int node0, const int node1, const int node2, const int node3,
				 const float blend0, const float blend1, const float blend2, const float blend3) 
				{ m_num = 4; m_nodes[0] = node0; m_nodes[1] = node1;  m_nodes[2] = node2; m_nodes[3] = node3;
				  m_blends[0] = blend0; m_blends[1] = blend1; m_blends[2] = blend2; m_blends[3] = blend3; }
	void vSetNodes(const int node0, const int node1, const int node2, const int node3, const int node4, 
				 const float blend0, const float blend1, const float blend2, const float blend3, const float blend4) 
				{ m_num = 5; m_nodes[0] = node0; m_nodes[1] = node1;  m_nodes[2] = node2; m_nodes[3] = node3; m_nodes[4] = node4;
				  m_blends[0] = blend0; m_blends[1] = blend1; m_blends[2] = blend2; m_blends[3] = blend3; m_blends[4] = blend4; }

	void vSetKp(T _kp) { m_Kp = _kp; }
	void vEnable(bool enabled) { m_bEnabled = enabled; }
	bool bEnabled() { return m_bEnabled; }

protected:
	xMath::Vector3 m_x0;
	float m_blends[5];
	int m_nodes[5];
	T m_Kp;
	T m_Kv;
	bool m_bEnabled;
	int m_num;
};

#endif // _POINT_EQN_H_
