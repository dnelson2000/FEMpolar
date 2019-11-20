
#include "xImplicitSolver.h"
#include "xMathUtils.h"
#include "xSurfaceEqn.h"

#include "xTetBrick.h"
#include "xSpringEqn.h"
#include "xPointEqn.h"
#include "xSurfaceEqn.h"

#include "xBlockSparse.h"
#include "xMathUtils.h"


using namespace std;

std::vector< xSurfaceEqn<float> * > g_extSrfEqns;
static double sideAng = 0;
std::vector<xMath::Vector3> debugNodes(200000,xMath::Vector3(0,0,0));
std::vector<xMath::Vector3> &xImplicitSolver::getDebugNodes() { return debugNodes; }

void xImplicitSolver::init()
{	
	//m_debugId = id;
	
	m_state.clear();
	m_state.x.clear();
	m_state.x0.clear();
	createEqns();
	m_state.brick->vEnable(true);

	m_threadFirst = 0;
	m_lRead = 0;
	m_lWrite = 1;
	m_lReady = 0;
	m_bufferRead = getReadBuffer();
	m_bufferWrite = getWriteBuffer();

	createThread();

}


void xImplicitSolver::getNodes( int whichObj, std::vector<xMath::Vector3> &nodes, std::vector<int> &elems )
{
	m_state.brick->getNodes( nodes, elems, m_state.x );
}


void xImplicitSolver::getNodesOnly( int whichObj, std::vector<xMath::Vector3> &nodes )
{
	nodes = m_state.x;
}

void xImplicitSolver::manipNode( int n, xMath::Vector3 &target )
{
	m_state.m_pManipPt[n]->setPosition( target );
}

void xImplicitSolver::manipSide( int n, xMath::Vector3 &target )
{
	sideAng = target[0];
}

void xImplicitSolver::getNodesTris( int whichObj, std::vector<xMath::Vector3> &nodes, std::vector<int> &elems )
{
	m_state.brick->getNodesTris( nodes, elems, m_state.x );
}

void xImplicitSolver::calculate()
{
	void start0(int id);
	float stop0(int id);

	int m_debugId = 0;
	start0(m_debugId);

	updateEqns();
	update();

	float msec = stop0(m_debugId);
}

void xImplicitSolver::updateEqns()
{
    
#if 1
    {
        int st  = m_state.brick->getStartNode();
        static float t = 0;
        //t = sideAng;
        t += 0.02f;
        if ( t > 1.57 ) 
            t = 1.57;
        float dev = 1.4f*sin(t);
        //float dev2 = 1.4f*sin(t*0.1);

        int cnt = m_state.m_pTipPt.size()/2;
        for (int i = width-1; i < width; i++) {
		    for (int j = 0; j < height; j++) {
			    for (int k = 0; k < depth; k++) {
                    int node = st + i*height*depth + j*depth + k;
	                xPointEqn<float> *tipPt = m_state.m_pTipPt[cnt];
	                tipPt->vEnable( true );
                    double ang = dev*1.2;
                    double r = 1.0*k;
                    double xx = r*sin(ang);
                    double yy = r*cos(ang);
	                tipPt->setPosition( xMath::Vector3((float)(width-1)+xx, (float)j, yy) );
                    ++cnt;
			    }
		    }
	    }
    }
    
    if (1)
    {
        int st  = m_state.brick->getStartNode();
        static float t = 0;
        t += 0.02;
        if ( t > 1.57 ) 
            t = 1.57;
        float dev = 1.4*sin(t);
        float dev2 = 1.4*sin(t*0.1);

        int cnt = 0;
        for (int i = 0; i < 1; i++) {
		    for (int j = 0; j < height; j++) {
			    for (int k = 0; k < depth; k++) {
                    int node = st + i*height*depth + j*depth + k;
	                xPointEqn<float> *tipPt = m_state.m_pTipPt[cnt];
	                tipPt->vEnable( true );
                    double ang = -dev*1.2;
                    double r = 1.0*k;
                    double xx = r*sin(ang);
                    double yy = r*cos(ang);
	                tipPt->setPosition( xMath::Vector3((float)0+xx, (float)j, yy) );
                    ++cnt;
			    }
		    }
	    }
    }
#endif

    for(int i=0; i < (int) m_state.m_pManipPt.size(); ++i) {
        debugNodes[i*4+0] = m_state.m_pManipPt[i]->getPosition();
        debugNodes[i*4+1] = debugNodes[i*4+0] + xMath::Vector3(0.1,0.1,0.1);
        debugNodes[i*4+2] = debugNodes[i*4+0] + xMath::Vector3(-0.1,-0.1,0.1);
        debugNodes[i*4+3] = debugNodes[i*4+0] + xMath::Vector3(0.1,0.1,-0.1);
        debugNodes[i*4+0] -= xMath::Vector3(0.1,0.1,0.1);
    }



	if(m_state.brick != NULL)
		m_state.brick->vEnable(true);

	int nb = (int) m_state.x0.size();
	m_state.v1.resize(nb,xMath::Vector3(0.f));
	m_state.f.resize(nb);
	m_state.scratch.resize(nb);

    int moreReserve = 1000;
	m_state.dfdx.resize(nb);
	m_state.dfdv.resize(nb);
	m_state.dfdx.reserve(moreReserve*nb);
	m_state.dfdv.reserve(moreReserve*nb);
	m_state.dfdx.clear();
	m_state.dfdv.clear();
    
	if(m_state.brick != NULL)
		m_state.brick->updatePolar(m_state.x, m_state.x0);


	int nLive = m_state.f.size();
    m_state.fViscousDamping = 0.0;
	
	for(int ii = 0; ii < (int) m_state.f.size(); ++ii)
	{
		m_state.f[ii] = xMath::Vector3(0);
	//	m_state.f[ii] = xMath::Vector3(0, 0, -9.81 * 100 * m_state.m[ii].getZ());	// gravity in cm/sec^2
		m_state.f[ii] += m_state.v[ii] * (-m_state.fViscousDamping); 
	}

	m_state.systemMatrix.resize(nLive);
	m_state.systemMatrix.reserve(moreReserve*nLive);
	m_state.systemMatrix.clear();
    
	if(m_state.brick != NULL)
		m_state.brick->calculate(m_state.dfdx, m_state.dfdv, m_state.f, m_state.x, m_state.v, m_state.x0);

	for(int i=0; i < (int) m_state.m_pSpringEqn.size(); ++i)
	{
		if(m_state.m_pSpringEqn[i] != NULL)
			m_state.m_pSpringEqn[i]->calculate(m_state.dfdx, m_state.dfdv, m_state.f, m_state.x, m_state.v, m_state.x0);
	}	
	for(int i=0; i < (int) m_state.m_pTipPt.size(); ++i)
	{
		if(m_state.m_pTipPt[i] != NULL)
			m_state.m_pTipPt[i]->calculate(m_state.dfdx, m_state.dfdv, m_state.f, m_state.x, m_state.v, m_state.x0);
	}
	for(int i=0; i < (int) m_state.m_pManipPt.size(); ++i)
	{
		if(m_state.m_pManipPt[i] != NULL)
			m_state.m_pManipPt[i]->calculate(m_state.dfdx, m_state.dfdv, m_state.f, m_state.x, m_state.v, m_state.x0);
	}
    for(int i=0; i < (int) m_state.m_pSidePlanes.size(); ++i)
	{
		if(m_state.m_pSidePlanes[i] != NULL)
			m_state.m_pSidePlanes[i]->calculate(m_state.dfdx, m_state.dfdv, m_state.f, m_state.x, m_state.v, m_state.x0);
	}
    for(int i=0; i < (int) g_extSrfEqns.size(); ++i)
	{
		if(g_extSrfEqns[i] != NULL)
			g_extSrfEqns[i]->calculate(m_state.dfdx, m_state.dfdv, m_state.f, m_state.x, m_state.v, m_state.x0);
	}
    

    
}


#include "windows.h"

void xImplicitSolver::update()
{
    
#ifdef WIN32
    LARGE_INTEGER start[2];
    LARGE_INTEGER stop[2];
    LARGE_INTEGER freq;
    QueryPerformanceCounter(&start[0]);
#endif

	// A = M - dt*df_dv - dt*dt*df_dx
	// v1 = inv(A)*( dt*fvec + (M-dt*df_dv)*vvec )
	//    = inv(A)*( M*vvec + dt*(fvec - df_dv*vvec ) )

	//m_state.dfdx.blockMultiply(m_state.f, m_state.v, m_state.dt);
	m_state.dfdv.blockMultiply(m_state.f, m_state.v, -1.f);
	if ( m_state.f.size() )
	{
		//float *f = (float *) &(m_state.f.front());
		xMath::Vector3 *f = (xMath::Vector3 *) &(m_state.f.front());
		for ( int i = 0; i < (int) m_state.f.size(); ++i )
			f[i] *= m_state.dt;
		for(int ii=0; ii < (int) m_state.m.size(); ++ii)
		{
			f[ii] += xMath::Vector3( m_state.m[ii][0]*m_state.v[ii][0], m_state.m[ii][1]*m_state.v[ii][1], m_state.m[ii][2]*m_state.v[ii][2] );
		}
	}

	//	Build m_state.systemMatrix for implicit solve.
	//	We modify the rhs to account for Neumann boundary conditions (i.e. velocity)
	for(int ii=0; ii < (int) m_state.m.size(); ++ii)
	{
		m_state.systemMatrix.getBlock(ii,ii) = makeDiagonal(m_state.m[ii]);
	}

    static bool wrotefile = false;
    static FILE *fil = fopen("d:\\Kscr.m","w");
        if ( !wrotefile ) {
            fprintf(fil,"K=zeros(%d);\n",m_state.f.size());
        }
	for(int i=0; i < (int) m_state.dfdx.getNumBlocks(); i++)
	{
		const Vec2i& index = m_state.dfdx.getIndex(i);
		xMath::xMatrix3 t = m_state.dfdx.getBlock(index[0], index[1]) * (-m_state.dt*m_state.dt);
		int a = index[0];
		int b = index[1];
		if ( a < b ) t = xMath::transpose(t);
		int c = max(a,b), d = min(a,b);
		m_state.systemMatrix.addBlock(c,d, t);
        if ( !wrotefile ) {
            fprintf(fil,"K(%d,%d)=%11.11f;\n",1+c,1+d,t.getElem(0,0));
        }

	}
    wrotefile=true;

	for(int i=0; i < (int) m_state.dfdv.getNumBlocks(); i++)
	{
		const Vec2i& index = m_state.dfdv.getIndex(i);
		int a = index[0];
		int b = index[1];
		int c = max(a,b), d = min(a,b);
		if (index[0] == index[1])
		{
			xMath::xMatrix3& t = m_state.dfdv.getBlock(index[0], index[1]);
			xMath::xMatrix3 mass(0);
			mass[0][0] = m_state.m[index[0]][0];
			mass[1][1] = m_state.m[index[0]][1];
			mass[2][2] = m_state.m[index[0]][2];
			m_state.systemMatrix.addBlock(c,d, (t - mass * m_state.fViscousDamping)*(-m_state.dt));
		}
		else
		{
			xMath::xMatrix3 t = m_state.dfdv.getBlock(index[0], index[1]) * (-m_state.dt);
			if ( a < b ) t = xMath::transpose(t);
			m_state.systemMatrix.addBlock( c,d, t);
		}
	}

#if 0
	int banded = m_state.systemMatrix.maxBandedDiagonal();
	CholFactor(m_state.systemMatrix, banded, float(FLT_EPSILON));

	int nLive = m_state.systemMatrix.getSize();
	vector< xMath::Vector3 > f(nLive);
	for(int ii=0; ii < nLive; ++ii)
	{
		f[ii] = m_state.f[ii];
	}
	CholSolve(m_state.scratch, m_state.systemMatrix, f, m_state.v1, banded);
#else
    float tol = 1e-15f;
    fprintf(stderr,"st cg\n");
    int max_iter = 4*m_state.f.size();
    CGBlock( m_state.systemMatrix, m_state.v1,  m_state.f, max_iter, tol);
    fprintf(stderr,"st cg\n");
#endif

    for(int ii = 0; ii < (int) m_state.v1.size(); ++ii) {
		xMath::Vector3 v1 = m_state.v1[ii];
		m_state.v[ii] = v1;
		m_state.x[ii] += m_state.dt * v1;
	}

    
#ifdef WIN32
    QueryPerformanceCounter(&stop[0]);
    QueryPerformanceFrequency(&freq);
    float msec = float( ((double)(stop[0].QuadPart - start[0].QuadPart)/(double)freq.QuadPart) * 1000.);
    fprintf( stderr, "time %f\n", msec );
#endif
}






#ifdef WIN32
// timing tests ... 
#include <windows.h>


const int READ_SYNC = 0xffff;

int xImplicitSolver::getReadBuffer()
{
	m_lRead = READ_SYNC;
	InterlockedCompareExchange( &m_lRead, m_lReady, READ_SYNC);
	return m_lRead;
}

int xImplicitSolver::getWriteBuffer()
{	
	m_lReady = m_lWrite;
	InterlockedCompareExchange( &m_lRead, m_lWrite, READ_SYNC);
	m_lWrite = ( m_lWrite + 1) % 3;
	if(m_lWrite == m_lRead)
	{
		m_lWrite = (m_lWrite + 1) % 3;
	}
	return m_lWrite;
}



LARGE_INTEGER start[2];
LARGE_INTEGER stop[2];

void start0(int id)
{
	QueryPerformanceCounter(&start[id]);
}

float stop0(int id)
{
	LARGE_INTEGER freq;
	static int iter = 0;
	if ( ++iter > 500 ) {
		QueryPerformanceCounter(&stop[id]);
		QueryPerformanceFrequency(&freq);
		float msec = float( ((double)(stop[id].QuadPart - start[id].QuadPart)/(double)freq.QuadPart) * 1000.);
		printf("time %f", msec );
		iter = 0;
		return msec;
	}
	return 0.f;
}

void stop1(int id)
{
	LARGE_INTEGER freq;
	QueryPerformanceCounter(&stop[id]);
	QueryPerformanceFrequency(&freq);
	printf("time %f", ((double)(stop[id].QuadPart - start[id].QuadPart)/(double)freq.QuadPart) * 1000.f );
}


void xImplicitSolver::execute() 
{ 
	while(1)
	{
		for ( int i = 0; i < 1; ++i )
			calculate();

		int getWritePtr();
		m_bufferWrite = getWriteBuffer();

		boost::this_thread::interruption_point();
	}
}


void xImplicitSolver::createThread()
{
	startThread();
	m_threadFirst = 1;
	if ( m_threadFirst )
	{
		m_threadFirst = 0;
#ifdef WIN32    
		 SetThreadPriority(m_thread->native_handle(), 0);
#endif
	}
}

void xImplicitSolver::pauseThread()
{
	m_thread->interrupt();
	m_thread->join();
	m_thread.reset();
}

void xImplicitSolver::stopThread()
{
	m_thread->interrupt();
	m_thread->join();
	m_thread.reset();
}

void xImplicitSolver::startThread()
{
	m_thread.reset(new boost::thread(boost::bind(&xImplicitSolver::execute, this)));
}

#endif

