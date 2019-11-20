
#include "xImplicitSolver.h"
#include "xSurfaceEqn.h"
#include "xPointEqn.h"
#include "xSurfaceEqn.h"
#include "xTetMaterial.h"
#include "xTetBrick.h"
#include "xSpringEqn.h"
#include "xMathUtils.h"
#include <sstream>

using namespace std;

xImplicitSolver::xImplicitSolver()
{
	m_threadId = 0;
}


xImplicitSolver::~xImplicitSolver()
{
	// stop thread loop .. to do
	m_state.clear();
}

extern bool faultCloseDist1[121][121];
extern float faultLocations[1000][2][2];
extern int gridFaultId[41][41];

void xImplicitSolver::createEqns()
{
    void setFaultLocations();
    setFaultLocations();

	xTetMaterial matNominal( 20000.f, 0.3f, 10.f, 1.f );    
	std::vector< xTetMaterial > matsB( 1, matNominal );

    width = 40; height = 40; depth = 7;
	m_state.brick = new xTetBrick<float>(matsB, width,height,depth);
	m_state.brick->vAddNodes(m_state.x0);
	
	m_state.x = m_state.x0;
	int size = (int) m_state.x.size();
	m_state.v.resize(size,xMath::Vector3(0.f,0.f,0.f));	
	m_state.m.resize(size,xMath::Vector3(1.f,1.f,1.f));	
    
	m_state.brick->setMass(m_state.m, m_state.x0);


    int st  = m_state.brick->getStartNode();

    for (int i = 0; i < 1; i++) {
		for (int j = 0; j < height; j++) {
			for (int k = 0; k < depth; k++) {
                int node = st + i*height*depth + j*depth + k;
#if 1
	            xPointEqn<float> *tipPt = new xPointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	            tipPt->vEnable( true );
	            m_state.m_pTipPt.push_back( tipPt );
                //m_state.m_pTipPt.back()->setPosition( xMath::Vector3((float)i, (float)j, (float)k) );

#else                
	            xSurfaceEqn<float> *tetPlane = new xSurfaceEqn<float>(node, m_state.x0, 1000000.f, 0.f );
	            tetPlane->vEnable( true );
                //tetPlane->setPosition( xMath::Vector3(0,1,0) );
                tetPlane->setNormal( xMath::Vector3(1,0,0) );
	            m_state.m_pSidePlanes.push_back( tetPlane );
#endif
			}
		}
	}
    
    
    for (int i = width-1; i < width; i++) {
		for (int j = 0; j < height; j++) {
			for (int k = 0; k < depth; k++) {
                int node = st + i*height*depth + j*depth + k;
#if 1
	            xPointEqn<float> *tipPt = new xPointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	            tipPt->vEnable( true );
	            m_state.m_pTipPt.push_back( tipPt );
                //m_state.m_pTipPt.back()->setPosition( xMath::Vector3((float)i+0.3f*j, (float)j, (float)k) );

#else                
	            xSurfaceEqn<float> *tetPlane = new xSurfaceEqn<float>(node, m_state.x0, 1000000.f, 0.f );
	            tetPlane->vEnable( true );
                //tetPlane->setPosition( xMath::Vector3(0,1,0) );
                tetPlane->setNormal( xMath::Vector3(1,0,0) );
	            m_state.m_pSidePlanes.push_back( tetPlane );
#endif

			}
		}
	}

#if 0
    for (int i = 0; i < width; i++) {
		for (int j = 0; j < 1; j++) {
			for (int k = 0; k < depth; k++) {
                int node = st + i*height*depth + j*depth + k;                
	            xSurfaceEqn<float> *tetPlane = new xSurfaceEqn<float>(node, m_state.x0, 1000000.f, 0.f );
	            tetPlane->vEnable( true );
                //tetPlane->setPosition( xMath::Vector3(0,1,0) );
                tetPlane->setNormal( xMath::Vector3(0,1,0) );
	            m_state.m_pSidePlanes.push_back( tetPlane );
			}
		}
	}
    for (int i = 0; i < width; i++) {
		for (int j = height-1; j < height; j++) {
			for (int k = 0; k < depth; k++) {
                int node = st + i*height*depth + j*depth + k;                
	            xSurfaceEqn<float> *tetPlane = new xSurfaceEqn<float>(node, m_state.x0, 1000000.f, 0.f );
	            tetPlane->vEnable( true );
                //tetPlane->setPosition( xMath::Vector3(0,1,0) );
                tetPlane->setNormal( xMath::Vector3(0,1,0) );
	            m_state.m_pSidePlanes.push_back( tetPlane );

			}
		}
	}
#endif

#if 0
    {
    int i = width/2, j = height/2,k =1;
    int node = st + i*height*depth + j*depth + k;
    PointEqn<float> *tipPt = new PointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	tipPt->vEnable( true );
	m_state.m_pManipPt.push_back( tipPt );
    m_state.m_pManipPt.back()->setPosition( m_state.m_pManipPt.back()->getPosition(m_state.x0) );
    }
    {
    int i = width/4, j = height/2,k =1;
    int node = st + i*height*depth + j*depth + k;
    PointEqn<float> *tipPt = new PointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	tipPt->vEnable( true );
	m_state.m_pManipPt.push_back( tipPt );
    m_state.m_pManipPt.back()->setPosition( m_state.m_pManipPt.back()->getPosition(m_state.x0) );
    }
    {
    int i = width/2, j = height/4,k =1;
    int node = st + i*height*depth + j*depth + k;
    PointEqn<float> *tipPt = new PointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	tipPt->vEnable( true );
	m_state.m_pManipPt.push_back( tipPt );
    m_state.m_pManipPt.back()->setPosition( m_state.m_pManipPt.back()->getPosition(m_state.x0));//+xMath::Vector3(0,0,10) );
    }
    {
    int i = 3*width/4, j = 3*height/4,k =1;
    int node = st + i*height*depth + j*depth + k;
    PointEqn<float> *tipPt = new PointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	tipPt->vEnable( true );
	m_state.m_pManipPt.push_back( tipPt );
    m_state.m_pManipPt.back()->setPosition( m_state.m_pManipPt.back()->getPosition(m_state.x0) );
    }
    {
    int i = 1*width/4, j = 1*height/4,k =1;
    int node = st + i*height*depth + j*depth + k;
    PointEqn<float> *tipPt = new PointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	tipPt->vEnable( true );
	m_state.m_pManipPt.push_back( tipPt );
    m_state.m_pManipPt.back()->setPosition( m_state.m_pManipPt.back()->getPosition(m_state.x0) );
    }
#endif

#if 0
    {
    int i = 1*width/2+3, j = 1*height/2-3,k =1;
    int node = st + i*height*depth + j*depth + k;
    PointEqn<float> *tipPt = new PointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	tipPt->vEnable( true );
	m_state.m_pManipPt.push_back( tipPt );
    m_state.m_pManipPt.back()->setPosition( m_state.m_pManipPt.back()->getPosition(m_state.x0) );
    }
    
    {
    int i = 1*width/2-4, j = 1*height/2+4,k =1;
    int node = st + i*height*depth + j*depth + k;
    PointEqn<float> *tipPt = new PointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	tipPt->vEnable( true );
	m_state.m_pManipPt.push_back( tipPt );
    m_state.m_pManipPt.back()->setPosition( m_state.m_pManipPt.back()->getPosition(m_state.x0) );
    }
    
    {
    int i = 1*width/2-7, j = 1*height/2+7,k =1;
    int node = st + i*height*depth + j*depth + k;
    PointEqn<float> *tipPt = new PointEqn<float>(node, 100000.f, 0.f, m_state.x0);
	tipPt->vEnable( true );
	m_state.m_pManipPt.push_back( tipPt );
    m_state.m_pManipPt.back()->setPosition( m_state.m_pManipPt.back()->getPosition(m_state.x0) );
    }

    for(int i=0; i < (int) m_state.m_pManipPt.size(); ++i)
	{
        fprintf(stderr,"%f %f %f\n",m_state.m_pManipPt[i]->getPosition()[0],
            m_state.m_pManipPt[i]->getPosition()[1],
            m_state.m_pManipPt[i]->getPosition()[2]);
    }
#endif
}


MechanicsStatePOD::MechanicsStatePOD() : dt(float(0.35))
{
	brick = NULL;
}


void MechanicsStatePOD::clear()
{
	 if ( brick ) delete brick;
     
	for ( int i = 0; i < int(m_pSpringEqn.size()); ++i )
		if ( m_pSpringEqn[i] ) delete m_pSpringEqn[i];
     m_pSpringEqn.clear();

	for ( int i = 0; i < int(m_pTipPt.size()); ++i )
		if ( m_pTipPt[i] ) delete m_pTipPt[i];
     m_pTipPt.clear();

}

