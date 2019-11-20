
#pragma once
#ifndef _SPRING_EQN_H_
#define _SPRING_EQN_H_

#include "xEqn.h"

template <class T>
class xSpringEqn : public xEqn<T>
{
public:
	xSpringEqn( int nodei, int nodej, T kp, T kv, std::vector< xMath::Vector3 >& x0 );
	
	virtual ~xSpringEqn() {}

	virtual void calculate(
		xBlockSparse< xMath::xMatrix3 >& dfdx,
		xBlockSparse< xMath::xMatrix3 >& dfdv,
		std::vector< xMath::Vector3 >& f,
		std::vector< xMath::Vector3 >& x,
		std::vector< xMath::Vector3 >& v,
		const std::vector< xMath::Vector3 >& x0);
	
	void vResetRestLen( std::vector< xMath::Vector3 >& xref ) ;
	void vSetRestLen( float _len0 ) { m_len0 = _len0; }
	void vEnable(bool enabled) { m_bEnabled = enabled; }
	bool bEnabled() { return m_bEnabled; }
	int getNode0() { return m_nodes[0]; }
	int getNode1() { return m_nodes[1]; }

protected:
	bool m_bEnabled;
	int m_nodes[2];
	T m_len0;
	T m_Kp, m_Kv;
};

#endif // _SPRING_EQN_H_
