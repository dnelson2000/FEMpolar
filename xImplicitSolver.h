
#ifndef _FEM_SOLVER_H_
#define _FEM_SOLVER_H_
#pragma once

#include "xMathUtils.h"
using namespace FEM;

#include "Export.h"

#include "xVec23i.h"
#include "xImplicitParams.h"

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>


class xmDeclExport xImplicitSolver
{
public:
	xImplicitSolver( );
	virtual ~xImplicitSolver();	
	virtual void init();
	virtual void updateEqns();

	void calculate();
	void createEqns();
	
	void startThread(); // { m_threadPause = false; }
	void pauseThread();
	void stopThread();
	void update();

    void getNodes( int whichObj, std::vector<xMath::Vector3> &nodes, std::vector<int> &elems );
    void getNodesOnly( int whichObj, std::vector<xMath::Vector3> &nodes);
    void getNodesTris( int whichObj, std::vector<xMath::Vector3> &nodes, std::vector<int> &elems );
    void manipNode( int n, xMath::Vector3 &x);
    void manipSide( int n, xMath::Vector3 &x);
    
    std::vector<xMath::Vector3> &getDebugNodes();

//protected:
	void createThread();
	void execute();
	unsigned int m_threadId;
	int m_threadFirst;
	boost::scoped_ptr<boost::thread> m_thread;    // sparse matrix inverse thread
    
    int width, height, depth;

	volatile long m_lRead;
	volatile long m_lWrite;
	volatile long m_lReady;
	int m_bufferRead, m_bufferWrite;
	int getReadBuffer();
	int getWriteBuffer();

	float fSurfaceKpTri;
	float fSurfaceKvTri;

	MechanicsStatePOD m_state;

};

#endif // _FEM_SOLVER_H_
