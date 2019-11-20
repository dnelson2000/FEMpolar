// dllmain.cpp : Defines the entry point for the DLL application.
#include "stdafx.h"

#include "xImplicitSolver.h"
#include "Export.h"

float FooPluginFunction()
{
    return 1.1f;
}


xImplicitSolver *gSolver = NULL;

extern "C" {


__declspec(dllexport) void init()
{
    gSolver = new xImplicitSolver;
    gSolver->init();
	gSolver->pauseThread();

    fprintf(stderr,"gSolver->init()\n");	
}

__declspec(dllexport) void start()
{
	gSolver->startThread();
}

__declspec(dllexport) void pause()
{
	gSolver->pauseThread();
}

__declspec(dllexport) int getNodesTris( float *x, int *elems, int *ne )
{
    std::vector<xMath::Vector3> brickNodes;
    std::vector<int> brickElems;
	gSolver->getNodesTris( 1, brickNodes, brickElems );
    for ( int i = 0; i < brickNodes.size(); ++i ) {
        x[i*3+0] = brickNodes[i][0];
        x[i*3+1] = brickNodes[i][2];
        x[i*3+2] = brickNodes[i][1];
    }
    for ( int i = 0; i < brickElems.size(); ++i ) {
        elems[i] = brickElems[i];
    }
    *ne = brickElems.size()/3;
    return brickNodes.size();
}

__declspec(dllexport) int getNodesOnly( float *x )
{
    std::vector<xMath::Vector3> brickNodes;
	gSolver->getNodesOnly( 1, brickNodes );
    for ( int i = 0; i < brickNodes.size(); ++i ) {
        x[i*3+0] = brickNodes[i][0];
        x[i*3+1] = brickNodes[i][2];
        x[i*3+2] = brickNodes[i][1];
    }
    return brickNodes.size();
}

__declspec(dllexport) int manipNode( int n, float *x )
{
	gSolver->manipNode( n, xMath::Vector3(x[0],x[1],x[2]) );
    return 0;
}
__declspec(dllexport) int manipSide( int n, float *x )
{
	gSolver->manipSide( n, xMath::Vector3(x[0],x[1],x[2]) );
    return 0;
}


}

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
					 )
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

