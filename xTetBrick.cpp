
#include <map>
#include "xTetBrick.h"
#include "xTetEqn.h"
#include "xSurfaceEqn.h"
#include "xMathUtils.h"

using namespace std;

template class xTetBrick<float>;

std::map<std::pair<int,int>,std::pair<int,int>> nodesHit;

extern bool faultCloseDist1[121][121];
extern float faultLocations[1000][2][2];
extern int gridFaultId[41][41];
extern std::vector< xSurfaceEqn<float> * > g_extSrfEqns;


double checkVol( int *nodes, std::vector< xMath::Vector3 >& x0 )
{
	xMath::Vector3 x0_0 = x0[nodes[0]];
	xMath::Vector3 x0_1 = x0[nodes[1]];
	xMath::Vector3 x0_2 = x0[nodes[2]];
	xMath::Vector3 x0_3 = x0[nodes[3]];
	xMath::Vector3 d0 = x0_1-x0_0, d1 = x0_2-x0_0, d2 = x0_3-x0_0;
	xMath::xMatrix3 V = makexMatrix3(d0,d1,d2);
	double v = fabs(xMath::determinant(V)/double(6.));
    return v;
}

bool intersectLine(xMath::Vector3 &a, xMath::Vector3 &b, xMath::Vector3 &n, xMath::Vector3 &pt, xMath::Vector3 &res)
{
    double d = (n[0]*pt[0]+n[1]*pt[1]+n[2]*pt[2]);
    xMath::Vector3 ba = b-a;
    double nDotA = xMath::dot(n, a);
    double nDotBA = xMath::dot(n, ba);
    
    double distAlong = ((d - nDotA)/nDotBA);
    double d01 = xMath::length(a-b);
    if ( distAlong < 0 || distAlong > 1.0 )
        return false;
    res = a + distAlong * ba;
    return true;
}

template <class T>
void xTetBrick<T>::vCreateElements(std::vector< xMath::Vector3 >& x0)
{
	for (int i = 0; i < width - 1; i++) {
		for (int j = 0; j < height - 1; j++) {
			for (int k = 0; k < depth - 1; k++) {

				int p0 = i*height*depth + j*depth + k;
				int p1 = p0 + 1;
				int p3 = (i + 1)*height*depth + j*depth + k;
				int p2 = p3 + 1;
				int p7 = (i + 1)*height*depth + (j + 1)*depth + k;
				int p6 = p7 + 1;
				int p4 = i*height*depth + (j + 1)*depth + k;
				int p5 = p4 + 1;


                int faultId = -1;

                int ii = i*3, jj = j*3, kk=k*3;

                bool foundOne = false;
                int fcdw = 5;
                for ( int c = -fcdw; c <= fcdw; ++c ) {
                    for ( int r = -fcdw; r <= fcdw; ++r ) {
                        if ( ii-c > 0 && jj-r > 0 && ii-c < width*3-3 && jj-r < height*3-3 ) {
                            if ( faultCloseDist1[ii-c][jj-r] ) {
                                foundOne = true;
                                break;
                            }
                        }
                    }
                    if ( foundOne ) break;
                }

                //foundOne = false;
                if ( i > 0 && j > 0 && i < width -2 && j < height-2 ) {
                    if ( foundOne )
                    {
                        bool done = false;
                        for ( int wid = 1; wid < 6; ++wid ) {
                            if ( !done )
                            for ( int c = -wid; c <= wid; ++c ) {
                                done = false;
                                for ( int r = -wid; r <= wid; ++r ) {
                                    if ( c+i >= 0 && r+j >= 0 && c+i < width && r+j < 40 )
                                    if ( gridFaultId[c+i][r+j] >= 0 ) {
                                        faultId = gridFaultId[c+i][r+j];
                                        done = true;
                                        if ( c > 2 && r > 2 ) {
                                            volatile int x=0;
                                        }
                                        if ( c > 0 ) {
                                            volatile int x=0;
                                        }
                                        //break;
                                    }
                                }
                                if ( done ) break;
                            }
                        }


                    }
                }

                    
                vector<int> indices;

				// Ensure that neighboring tetras are sharing faces
				if ((i + j + k) % 2 == 1) {
					indices.push_back(p2); indices.push_back(p1); indices.push_back(p6); indices.push_back(p3);
					indices.push_back(p6); indices.push_back(p3); indices.push_back(p4); indices.push_back(p7);
					indices.push_back(p4); indices.push_back(p1); indices.push_back(p6); indices.push_back(p5);
					indices.push_back(p3); indices.push_back(p1); indices.push_back(p4); indices.push_back(p0);
					indices.push_back(p6); indices.push_back(p1); indices.push_back(p4); indices.push_back(p3);
				} else {
					indices.push_back(p0); indices.push_back(p2); indices.push_back(p5); indices.push_back(p1);
					indices.push_back(p7); indices.push_back(p2); indices.push_back(p0); indices.push_back(p3);
					indices.push_back(p5); indices.push_back(p2); indices.push_back(p7); indices.push_back(p6);
					indices.push_back(p7); indices.push_back(p0); indices.push_back(p5); indices.push_back(p4);
					indices.push_back(p0); indices.push_back(p2); indices.push_back(p7); indices.push_back(p5);
				}
                for ( int ii = 0; ii < 5; ++ii ) {
				    int nodes[4] = {
					    indices[ii*4+0] + xTetrahedra<T>::m_startNode,
					    indices[ii*4+1] + xTetrahedra<T>::m_startNode,
					    indices[ii*4+2] + xTetrahedra<T>::m_startNode,
					    indices[ii*4+3] + xTetrahedra<T>::m_startNode };

                    if ( faultId >= 0 ) {
                        xMath::Vector3 fStart( faultLocations[faultId][0][0], faultLocations[faultId][0][1], 0 );
                        xMath::Vector3 fEnd( faultLocations[faultId][1][0], faultLocations[faultId][1][1], 0 );
                        xMath::Vector3 ptOnPlane = fStart;
                        xMath::Vector3 faultTan = fEnd-fStart;
                        faultTan = xMath::normalize(faultTan);
                        if ( faultTan[0] > 0 )
                            faultTan = -faultTan;
                        xMath::Vector3 planeNormal( -faultTan[1], faultTan[0], 0.2 );
                        if ( xMath::length(planeNormal) < 1e-4 ) {
                            fprintf(stderr,"planenormal\n");
                            continue;
                        }
                        planeNormal = xMath::normalize(planeNormal);
                        
                        
                        int nhits = 0;                        
                        int inodes[4] = { nodes[0], nodes[1], nodes[2], nodes[3] };
                        int edges[6][2] = { {nodes[0],nodes[1]}, {nodes[0],nodes[2]}, {nodes[0],nodes[3]}, {nodes[1],nodes[2]}, {nodes[1],nodes[3]}, {nodes[2],nodes[3]} };
                        int edgesHit[6][2] = { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
                        int edgesIdx[6][3] = { {0,1,0}, {0,2,0}, {0,3,0}, {1,2,0}, {1,3,0}, {2,3,0} };
                        //bool edgesHit[6] = { false, false, false, false, false, false  };
                        int vertsHit[4] = { 0, 0,0, 0 };
                        xMath::Vector3 ipt[6];
                        xMath::Vector3 iptDraw[6], iptDrawSide[2][6];
                        for ( int e = 0; e < 6; ++e ) {
                            int e0 = edges[e][0], e1 = edges[e][1];
                            if ( !intersectLine(x0[e0],x0[e1], planeNormal, ptOnPlane, ipt[e] ) )
                               continue;
                            //edgesHit[e] = true;
                            edgesHit[nhits][0]= edges[e][0];
                            edgesHit[nhits][1]= edges[e][1];
                            ++vertsHit[edgesIdx[e][0]];
                            ++vertsHit[edgesIdx[e][1]];
                            edgesIdx[e][2] = 1;
                            iptDraw[nhits] = ipt[e];
                            iptDrawSide[0][nhits] = iptDraw[nhits];
                            iptDrawSide[1][nhits] = iptDraw[nhits];
                            ++nhits;
                        }
                        
                        int vertIsOnSide[4];
                        for ( int v = 0; v < 4; ++v ) {
                            xMath::Vector3 d3 = x0[nodes[v]] - ptOnPlane;
                            vertIsOnSide[v] = xMath::dot(d3,planeNormal) < 0;
                        }
                        int newNodes[2][4];
                        if ( nhits == 3 || nhits == 4 )
                        for ( int nh = 0; nh < nhits; ++nh ) {
                            std::pair<int,int> p(edgesHit[nh][0],edgesHit[nh][1]);
                            std::map<std::pair<int,int>,std::pair<int,int>>::iterator hit = nodesHit.find(p);
                            if ( hit == nodesHit.end() ) {
                                newNodes[0][nh] = x0.size();
                                newNodes[1][nh] = x0.size()+1;
                                nodesHit[p] = std::pair<int,int>( newNodes[0][nh], newNodes[1][nh] );
                                x0.push_back(iptDraw[nh]);
                                x0.push_back(iptDraw[nh]);
                                xTetrahedra<T>::m_nNodes += 2;
                                
                                {
	                            xSurfaceEqn<float> *tetPlane = new xSurfaceEqn<float>(newNodes[0][nh], x0, 1000000.f, 0.f );
	                            tetPlane->vEnable( true );
                                tetPlane->setPosition( ptOnPlane );
                                tetPlane->setNormal( planeNormal );
	                            g_extSrfEqns.push_back( tetPlane );
                                }
                                {
	                            xSurfaceEqn<float> *tetPlane = new xSurfaceEqn<float>(newNodes[1][nh], x0, 1000000.f, 0.f );
	                            tetPlane->vEnable( true );
                                tetPlane->setPosition( ptOnPlane );
                                tetPlane->setNormal( planeNormal );
	                            g_extSrfEqns.push_back( tetPlane );
                                }

                            } else {
                                std::pair<int,int> res = nodesHit[p];
                                newNodes[0][nh] = res.first;
                                newNodes[1][nh] = res.second;
                            }
                        }

                        double veps = 0.0001;

                        if ( nhits == 3 ) {

                            int topv = -1;
                            int npv = 0;
                            int notTopV[3];
                            for ( int v = 0; v < 4; ++v ) {
                                if ( vertsHit[v] >= 3 )
                                    topv = v;
                                else 
                                    notTopV[npv++] = inodes[v];
                            }

                            double eps2 = 0.01;
                            if ( vertIsOnSide[topv] == 1 )
                                eps2 = -eps2;
                                
                            for ( int inter = 0; inter < 3; ++inter ) {
                                iptDrawSide[0][inter] += eps2*planeNormal;
                                iptDrawSide[1][inter] -= eps2*planeNormal;
                            }

                            int di = notTopV[0], ei = notTopV[1], fi = notTopV[2];
                            int news0 = vertIsOnSide[topv] ? 1 : 0;
                            int news1 = !news0;
                            int ai = newNodes[news1][0];
                            int bi = newNodes[news1][1];
                            int ci = newNodes[news1][2];
                            
                            static int addone = 0;
                            { //if (addone >=5 && addone <= 420 ) {
                                int tet[4] = { inodes[topv], newNodes[news0][0], newNodes[news0][1], newNodes[news0][2] };
                                double v = checkVol(tet, x0);
                                if ( v < veps ) {
                                    volatile int x=0;
                                }
                                if ( v > veps )
            				        xTetrahedra<T>::m_tets.push_back(xTetEqn<T>(tet, x0, xTetrahedra<T>::m_centerMaterial[0]));
                                
                                int newTets[3][4] = {
                                    { ai,ci,di,ei }, { ai,bi,ci,ei }, { ci,di,ei,fi } };
                                for ( int nt = 0; nt < 3; ++nt ) {
                                    double v = checkVol(newTets[nt], x0);
                                    if ( v < veps )
                                        continue;
            				        xTetrahedra<T>::m_tets.push_back(xTetEqn<T>(newTets[nt], x0, xTetrahedra<T>::m_centerMaterial[0]));
                                }
                            }
                            ++addone;
                        }

                        if ( nhits == 4 ) {

                            int side0verts[2] = { -1,-1 };
                            int side1verts[2] = { -1,-1 };
                            int nside = 0;
                            for ( int e = 0; e < 6; ++e ) {
                                if ( edgesIdx[e][2] == 0 ) {
                                    if ( nside == 0 ) {
                                        side0verts[0] = edgesIdx[e][0];
                                        side0verts[1] = edgesIdx[e][1];
                                    }
                                    if ( nside == 1 ) {
                                        side1verts[0] = edgesIdx[e][0];
                                        side1verts[1] = edgesIdx[e][1];
                                    }
                                    ++nside;
                                    if ( nside > 2 ) {
                                        fprintf(stderr,"nside > 2\n");
                                    }
                                }
                            }
                            
                             double eps2 = 0.01;
                            if ( vertIsOnSide[side0verts[0]] == 1 )
                                eps2 = -eps2;
                                
                            for ( int inter = 0; inter < 4; ++inter ) {
                                iptDrawSide[0][inter] += eps2*planeNormal;
                                iptDrawSide[1][inter] -= eps2*planeNormal;
                            }


                            for ( int side = 0; side < 2; ++side )
                            {
                                int ai = side0verts[0], bi = side0verts[1];
                                if ( side == 1 ) {
                                    ai = side1verts[0]; bi = side1verts[1];
                                }
                                
                                static int addone = 0;
                                { //if ( addone >=5 && addone <=111 ) {
                                    int myai = inodes[ai], mybi = inodes[bi];   
                                    int newsa = vertIsOnSide[ai] ? 1 : 0; 
                                    int newsb = vertIsOnSide[bi] ? 1 : 0;
                                    if ( newsa != newsb )
                                        fprintf(stderr,"newsa !=newsb\n");
                                    int ci = newNodes[newsa][0];
                                    int di = newNodes[newsa][1];
                                    int fi = newNodes[newsa][2];
                                    int ei = newNodes[newsa][3];
                                    
                                    int newTets[3][4] = {
                                        { mybi,di,ei,fi }, { myai,mybi,ci, side == 0 ? di : fi }, { mybi,ci,di,fi } };
                                    for ( int nt = 0; nt < 3; ++nt ) {
                                        double v = checkVol(newTets[nt], x0);
                                        if ( v < veps )
                                            continue;
            				            xTetrahedra<T>::m_tets.push_back(xTetEqn<T>(newTets[nt], x0, xTetrahedra<T>::m_centerMaterial[0]));
                                    }
                                }
                                ++addone;
                            }
                        }

                        if ( nhits ) 
                            continue;
                        //continue; // skip all, hit or not for now
                    }

				    xTetrahedra<T>::m_tets.push_back(xTetEqn<T>(nodes, x0, xTetrahedra<T>::m_centerMaterial[0]));
                }
            }

		}
    }
}


template <class T>
void xTetBrick<T>::setMass(std::vector< xMath::Vector3 >& M, const std::vector< xMath::Vector3 >& x0)
{
	if(!xTetrahedra<T>::m_bEnabled)
		return;

	M.resize(M.size(),xMath::Vector3(0.f));
	for(int i = 0; i < width*depth*height; ++i)
	{
		T V = 1;
		T m = 1;
		M[xTetrahedra<T>::getStartNode() + i] = xMath::Vector3(m);
	}
}
