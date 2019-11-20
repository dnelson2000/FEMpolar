
#pragma once
#ifndef _TET_BRICK_H_
#define _TET_BRICK_H_

#include "xTetrahedra.h"
#include "xBlockSparse.h"

template <class T>
class xTetBrick : public xTetrahedra<T>
{
public:
	xTetBrick( const std::vector< xTetMaterial >& material, int widthIn, int heightIn, int depthIn)
	: width(widthIn), height(heightIn), depth(depthIn)
	{
		xTetrahedra<T>::m_centerMaterial = material;
	}

	virtual ~xTetBrick() {}

	void vAddNodes(std::vector< xMath::Vector3 >& x0)
	{
		xTetrahedra<T>::m_startNode = (int) x0.size();
		xTetrahedra<T>::m_nNodes = width*depth*height;
		x0.resize(x0.size() + xTetrahedra<T>::m_nNodes);
		vDefaultShape(x0);
		vCreateElements(x0);
		xTetrahedra<T>::vLumpedNodes();
	}

	void vDefaultShape(std::vector< xMath::Vector3 >& x0)
	{
	    for (int i = 0; i < width; i++) {
		    for (int j = 0; j < height; j++) {
			    for (int k = 0; k < depth; k++) {
#if 1
                    x0[xTetrahedra<T>::m_startNode+ i*height*depth + j*depth + k] = xMath::Vector3((float)i, (float)j, (float)k);
#else
                    double r = 10.0+0.3*i + 5.0*k;
                    double a = 0.2*i;
				    x0[xTetrahedra<T>::m_startNode+ i*height*depth + j*depth + k] = //xMath::Vector3((float)i, (float)j, (float)k);
                        xMath::Vector3(r*cos(a),(double)j,r*sin(a));
#endif
			    }
		    }
	    }
	}
	
	void vSetStiffness(const std::vector<xMath::Vector3>& x0, const std::vector<xTetMaterial> & mat)
	{
		if(!xTetrahedra<T>::m_bEnabled)
			return;
		
		for ( int i = 0; i < int(xTetrahedra<T>::m_tets.size()); ++i )
		{
			xTetrahedra<T>::m_tets[i].vSetStiffness(x0, mat[0]);
		}
	}
	
	virtual void setMass(std::vector< xMath::Vector3 >& M, const std::vector< xMath::Vector3 >& x0);
	
    void getNodes( std::vector<xMath::Vector3> &nodes, std::vector<int> &elems, const std::vector< xMath::Vector3 >& x ) {
        nodes.resize(xTetrahedra<T>::m_nNodes);
        for ( int i = 0; i < m_nNodes; ++i )
            nodes[i] = x[i + xTetrahedra<T>::m_startNode];
        elems.resize(xTetrahedra<T>::m_tets.size()*24);        
        for ( int i = 0; i < xTetrahedra<T>::m_tets.size(); ++i ) {
            int i24 = i*24;
            int *nodes = xTetrahedra<T>::m_tets[i].getNodes();
            elems[i24+0] = nodes[0]-xTetrahedra<T>::m_startNode;
            elems[i24+1] = nodes[1]-xTetrahedra<T>::m_startNode;
            elems[i24+2] = nodes[1]-xTetrahedra<T>::m_startNode;
            elems[i24+3] = nodes[2]-xTetrahedra<T>::m_startNode;
            elems[i24+4] = nodes[2]-xTetrahedra<T>::m_startNode;
            elems[i24+5] = nodes[0]-xTetrahedra<T>::m_startNode;

            elems[i24+6] = nodes[0]-xTetrahedra<T>::m_startNode;
            elems[i24+7] = nodes[2]-xTetrahedra<T>::m_startNode;
            elems[i24+8] = nodes[2]-xTetrahedra<T>::m_startNode;
            elems[i24+9] = nodes[3]-xTetrahedra<T>::m_startNode;
            elems[i24+10] = nodes[3]-xTetrahedra<T>::m_startNode;
            elems[i24+11] = nodes[0]-xTetrahedra<T>::m_startNode;

            elems[i24+12] = nodes[0]-xTetrahedra<T>::m_startNode;
            elems[i24+13] = nodes[1]-xTetrahedra<T>::m_startNode;
            elems[i24+14] = nodes[1]-xTetrahedra<T>::m_startNode;
            elems[i24+15] = nodes[3]-xTetrahedra<T>::m_startNode;
            elems[i24+16] = nodes[3]-xTetrahedra<T>::m_startNode;
            elems[i24+17] = nodes[0]-xTetrahedra<T>::m_startNode;

            elems[i24+18] = nodes[1]-xTetrahedra<T>::m_startNode;
            elems[i24+19] = nodes[2]-xTetrahedra<T>::m_startNode;
            elems[i24+20] = nodes[2]-xTetrahedra<T>::m_startNode;
            elems[i24+21] = nodes[3]-xTetrahedra<T>::m_startNode;
            elems[i24+22] = nodes[3]-xTetrahedra<T>::m_startNode;
            elems[i24+23] = nodes[1]-xTetrahedra<T>::m_startNode;

        }
    }

    void getNodesTris( std::vector<xMath::Vector3> &nodes, std::vector<int> &elems, const std::vector< xMath::Vector3 >& x ) {
        nodes.resize(xTetrahedra<T>::m_nNodes);
        for ( int i = 0; i < m_nNodes; ++i )
            nodes[i] = x[i + xTetrahedra<T>::m_startNode];
        elems.resize(xTetrahedra<T>::m_tets.size()*12);
        for ( int i = 0; i < xTetrahedra<T>::m_tets.size(); ++i ) {
            int i12 = i*12;
            int *nodes = xTetrahedra<T>::m_tets[i].getNodes();
            xMath::Vector3 mid = 0.25*(x[nodes[0]]+x[nodes[1]]+x[nodes[2]]+x[nodes[3]]);
            xMath::Vector3 n0 = xMath::cross( x[nodes[0]]-x[nodes[1]], x[nodes[0]]-x[nodes[2]] );
            xMath::Vector3 n1 = xMath::cross( x[nodes[0]]-x[nodes[2]], x[nodes[0]]-x[nodes[3]] );
            xMath::Vector3 n2 = xMath::cross( x[nodes[0]]-x[nodes[3]], x[nodes[0]]-x[nodes[1]] );
            xMath::Vector3 n3 = xMath::cross( x[nodes[1]]-x[nodes[3]], x[nodes[1]]-x[nodes[2]] );

            if ( xMath::dot( x[nodes[0]]-mid, n0 ) < 0 ) {
                elems[i12+0] = nodes[0]-xTetrahedra<T>::m_startNode;
                elems[i12+1] = nodes[1]-xTetrahedra<T>::m_startNode;
                elems[i12+2] = nodes[2]-xTetrahedra<T>::m_startNode;
            } else {
                elems[i12+0] = nodes[0]-xTetrahedra<T>::m_startNode;
                elems[i12+1] = nodes[2]-xTetrahedra<T>::m_startNode;
                elems[i12+2] = nodes[1]-xTetrahedra<T>::m_startNode;
            }

            if ( xMath::dot( x[nodes[0]]-mid, n1 ) < 0 ) {
                elems[i12+3] = nodes[0]-xTetrahedra<T>::m_startNode;
                elems[i12+4] = nodes[2]-xTetrahedra<T>::m_startNode;
                elems[i12+5] = nodes[3]-xTetrahedra<T>::m_startNode;
            } else {
                elems[i12+3] = nodes[0]-xTetrahedra<T>::m_startNode;
                elems[i12+4] = nodes[3]-xTetrahedra<T>::m_startNode;
                elems[i12+5] = nodes[2]-xTetrahedra<T>::m_startNode;
            }
            

            if ( xMath::dot( x[nodes[0]]-mid, n2 ) < 0 ) {
                elems[i12+6] = nodes[0]-xTetrahedra<T>::m_startNode;
                elems[i12+7] = nodes[3]-xTetrahedra<T>::m_startNode;
                elems[i12+8] = nodes[1]-xTetrahedra<T>::m_startNode;
            } else {
                elems[i12+6] = nodes[0]-xTetrahedra<T>::m_startNode;
                elems[i12+7] = nodes[1]-xTetrahedra<T>::m_startNode;
                elems[i12+8] = nodes[3]-xTetrahedra<T>::m_startNode;
            }
            

            if ( xMath::dot( x[nodes[1]]-mid, n3 ) < 0 ) {
                elems[i12+9] = nodes[1]-xTetrahedra<T>::m_startNode;
                elems[i12+10] = nodes[3]-xTetrahedra<T>::m_startNode;
                elems[i12+11] = nodes[2]-xTetrahedra<T>::m_startNode;
            } else {
                elems[i12+9] = nodes[1]-xTetrahedra<T>::m_startNode;
                elems[i12+10] = nodes[2]-xTetrahedra<T>::m_startNode;
                elems[i12+11] = nodes[3]-xTetrahedra<T>::m_startNode;
            }
        }

    }

protected:
    int width, height, depth;
	
	void vCreateElements(std::vector< xMath::Vector3 >& x0);
};

#endif // _TET_ROD_H_
