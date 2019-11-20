
#pragma once
#ifndef _SURFACE_EQN_H_
#define _SURFACE_EQN_H_

#include "xEqn.h"

template <class T>
class xSurfaceEqn : public xEqn<T>
{
public:
	xSurfaceEqn( int node, const std::vector< xMath::Vector3 >& x0, T kp, T kv)
	: m_num(1), m_normal(0.0), m_x0(x0[node]), m_Kp(kp), m_Kv(kv), m_bEnabled(true)
	{
		m_nodes[0] = node;
		m_blends[0] = T(1);
	}
	
	xSurfaceEqn( int node0, int node1, T blend, const std::vector< xMath::Vector3 >& x0, T kp, T kv)
	: m_num(2), m_normal(T(0.0)), m_x0(x0[node0]), m_Kp(kp), m_Kv(kv), m_bEnabled(true)
	{
		m_nodes[0] = node0;
		m_nodes[1] = node1;
		m_blends[0] = blend;
		m_blends[1] = T(1) - blend;
	}
	
	xSurfaceEqn( int node0, int node1, int node2, T blend0, T blend1, T blend2, const std::vector< xMath::Vector3 >& x0, T kp, T kv)
	: m_num(3), m_normal(T(0.0)), m_x0(x0[node0]), m_Kp(kp), m_Kv(kv), m_bEnabled(true)
	{
		m_nodes[0] = node0;
		m_nodes[1] = node1;
		m_nodes[2] = node2;
		m_blends[0] = blend0;
		m_blends[1] = blend1;
		m_blends[2] = blend2;
	}
	
	xSurfaceEqn( int nodes[4], T blends[4], const std::vector< xMath::Vector3 >& x0, T kp, T kv);
	xSurfaceEqn( int *nodes, T *blends, int n, const std::vector< xMath::Vector3 >& x0, T kp, T kv)
	: m_num(n), m_normal(T(0.0)), m_x0(x0[nodes[0]]), m_Kp(kp), m_Kv(kv), m_bEnabled(true)
	{
		for ( int i = 0; i < n; ++i )
		{
			m_nodes[i] = nodes[i];
			m_blends[i] = blends[i];
		}
	}

	virtual ~xSurfaceEqn() {}

	virtual void calculate(
		xBlockSparse< xMath::xMatrix3 >& dfdx,
		xBlockSparse< xMath::xMatrix3 >& dfdv,
		std::vector< xMath::Vector3 >& f,
		std::vector< xMath::Vector3 >& x,
		std::vector< xMath::Vector3 >& v,
		const std::vector< xMath::Vector3 >& x0);

	void setPosition(const xMath::Vector3& x) { m_x0 = x; }
	const xMath::Vector3& getPosition() { return m_x0; }
	
	void updateActive(const std::vector< xMath::Vector3 >& x)
	{
		xMath::Vector3 pos(0.0);
		for(int ii = 0; ii < m_num; ++ii)
		{
			pos += x[m_nodes[ii]] * m_blends[ii];
		}
		xMath::Vector3 d3 = ( pos - m_x0 );
		m_bEnabled = xMath::lengthSqr(d3) < 6.f && xMath::dot( d3,m_normal) > 0.f;
	}
	
	void setNormal(const  xMath::Vector3 & n) { m_normal = n; }
	const xMath::Vector3& getNormal() { return m_normal; }

	xMath::Vector3 getPosition(const std::vector< xMath::Vector3 >& x) const
	{
		xMath::Vector3 pos(0.0);
		for(int ii = 0; ii < m_num; ++ii)
		{
			pos += x[m_nodes[ii]] * m_blends[ii];
		}
		return pos;
	}
	
	const int * getNodes( int &num ) const { num = m_num; return m_nodes; }

	void vSetNodes(const int node0, const int node1, const int node2,
				 const float blend0, const float blend1, const float blend2) 
				{ m_num = 3; m_nodes[0] = node0; m_nodes[1] = node1;  m_nodes[2] = node2;
				  m_blends[0] = blend0; m_blends[1] = blend1; m_blends[2] = blend2; }

	void vSetNodes(const int node0, const int node1, const int node2, const int node3,
				 const float blend0, const float blend1, const float blend2, const float blend3) 
				{ m_num = 4; m_nodes[0] = node0; m_nodes[1] = node1;  m_nodes[2] = node2; m_nodes[3] = node3;
				  m_blends[0] = blend0; m_blends[1] = blend1; m_blends[2] = blend2; m_blends[3] = blend3; }

	void vSetNodes(int *_nodes, float *_blends, int n )
	{
		for ( int i = 0; i < n; ++i ) {
			m_nodes[i] = _nodes[i];
			m_blends[i] = _blends[i];
		}
		m_num = n;
	}
	
	

	void vEnable(bool enabled) { m_bEnabled = enabled; }
	bool bEnabled() { return m_bEnabled; }

protected:
	int m_nodes[6];
	int m_num;
	T m_blends[6];
	xMath::Vector3 m_x0;
	xMath::Vector3 m_normal;
	T m_Kp;
	T m_Kv;
	bool m_bEnabled;

public: 
	xMath::Vector3 m_pos;
	xMath::Vector3 m_pos_prev;
};

#endif // _SURFACE_EQN_H_
