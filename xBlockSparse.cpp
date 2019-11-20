
#include "xBlockSparse.h"
#include "xMathUtils.h"
#include <assert.h>

using namespace std;


float norm( const std::vector<xMath::Vector3> &vec )
{
	float res = 0.f;
	for ( int i = 0; i < int(vec.size()); ++i )
		res += xMath::dot( vec[i], vec[i] );
	return res;
}

float dot( const std::vector<xMath::Vector3> &a, const std::vector<xMath::Vector3> &b )
{
	float res = 0.f;
	for ( int i = 0; i < int(a.size()); ++i )
		res += xMath::dot( a[i], b[i] );
	return res;
}


// cg iteration, a block version of the preconditioned version
int CGBlockDiagPrec( xBlockSparse<xMath::xMatrix3> &A, std::vector<xMath::Vector3> &x, const std::vector<xMath::Vector3> &b, int &max_iter, float tol)
{
  float resid;
  std::vector<xMath::Vector3> p(x.size()), z(x.size());
  float alpha(1.f), beta(1.f), rho(1.f), rho_1(1.f);

  float normb = norm(b);
  std::vector<xMath::Vector3> r(b);
  A.blockMultiply( r, x, -1.f );

  if (normb == 0.0) 
    normb = 1;
  
  if ((resid = norm(r) / normb) <= tol) {
    tol = resid;
    max_iter = 0;
    return 0;
  }

	
  std::vector<xMath::Vector3> M(x.size());
  for ( int i = 0; i < int(M.size()); ++i )
  {
	  xMath::xMatrix3 &mat = A.getBlock(i,i);
	  xMath::xMatrix3 matd;
	  float *p = (float*)&mat;
	  float *pd = (float*)&(M[i]);
	  pd[0] = 1.f/p[0];
	  pd[1] = 1.f/p[4];
	  pd[2] = 1.f/p[8];
  }



  for (int i = 1; i <= max_iter; i++)
  {
    //z = M.solve(r);
    for ( int j = 0; j < int(x.size()); ++j )
	{
		xMath::Vector3 amult( M[j][0]*r[j][0], M[j][1]*r[j][1], M[j][2]*r[j][2] );
		z[j] = amult;		
	}

    rho = dot(r, z);
    
    if (i == 1)
      p = z;
    else {
      beta = rho / rho_1;
	  for ( int j = 0; j < int(x.size()); ++j )
		p[j] = z[j] + beta * p[j];
    }
    
	//q = A*p;
	std::vector<xMath::Vector3> q(x.size(), xMath::Vector3(0.f));
	A.blockMultiply( q, p, 1.f );
    alpha = rho / dot(p, q);
    
	for ( int j = 0; j < int(x.size()); ++j )
	{
		x[j] += alpha * p[j];
		r[j] -= alpha * q[j];
	}
    
    if ((resid = norm(r) / normb) <= tol) {
      tol = resid;
      max_iter = i;
      return 0;     
    }

    rho_1 = rho;
  }
  
  tol = resid;
  return 1;
}


// cg iteration, a block version of the preconditioned version
int CGBlock( xBlockSparse<xMath::xMatrix3> &A, std::vector<xMath::Vector3> &x, const std::vector<xMath::Vector3> &b, int &max_iter, float tol)
{
    double resid;
  std::vector<xMath::Vector3> p(x.size()), z(x.size());
  double alpha(1.f), beta(1.f), rho(1.f), rho_1(1.f);

  float normb = norm(b);
  std::vector<xMath::Vector3> r(b);
  A.blockMultiply( r, x, -1.f );

  if (normb == 0.0) 
    normb = 1;
  
  if ((resid = norm(r) / normb) <= tol) {
    tol = resid;
    max_iter = 0;
    return 0;
  }

  for (int i = 1; i <= max_iter; i++)
  {
    for ( int j = 0; j < int(x.size()); ++j )
	{
		z[j] = r[j];
	}

    rho = dot(r, z);
    
    if (i == 1)
      p = z;
    else {
      beta = rho / rho_1;
	  for ( int j = 0; j < int(x.size()); ++j )
		p[j] = z[j] + beta * p[j];
    }
    
	//q = A*p;
	std::vector<xMath::Vector3> q(x.size(), xMath::Vector3(0.f));
    A.blockMultiply( q, p, 1.f );

    alpha = rho / dot(p, q);
    
	for ( int j = 0; j < int(x.size()); ++j )
	{
		x[j] += alpha * p[j];
		r[j] -= alpha * q[j];
	}
    
    if ((resid = norm(r) / normb) <= tol) {
      tol = resid;
      max_iter = i;
      return 0;     
    }

    rho_1 = rho;
  }
  
  tol = resid;
  return 1;
}


void CholSolve( std::vector< xMath::Vector3 >& x_, const xBlockSparse< xMath::xMatrix3 >& L, const std::vector< xMath::Vector3 >& b_, std::vector< xMath::Vector3 >& xPass2, const int banded )
{
	int size = (int) b_.size();
	for ( int pass = 0; pass < 2; ++pass )
	{
		std::vector< xMath::Vector3 >& x = ( pass == 0 ? x_ : xPass2 );
		const std::vector< xMath::Vector3 >& b = ( pass == 0 ? b_ : x_ );

		if ( pass == 0 )
		{
			// x(i) = inv(A(i,i)) [ b(i) - sum(j=0..i-1) A(i,j) x(j) ]
			for(int i=0; i<size; i++)
			{
				xMath::Vector3 scratch(0.0, 0.0, 0.0);
				int st = max(0,i-banded);
				for( int j = st; j < i; ++j )
				{
					const xMath::xMatrix3* fLij = L.findBlock(i,j);
					if(fLij)
						scratch += (*fLij)*x[j];
				}
				scratch = b[i] - scratch;

				xMath::xMatrix3& Lii = *(L.findBlock(i,i));

				float elem21 = Lii.getElem(1,2);
				float elem20 = Lii.getElem(0,2);
				float elem10 = Lii.getElem(0,1);

				x[i][0] = scratch[0]/Lii.getElem(0,0);
				x[i][1] = (scratch[1] - elem10*x[i][0])/Lii.getElem(1,1);
				x[i][2] = (scratch[2] - elem20*x[i][0] - elem21*x[i][1])/Lii.getElem(2,2);
			}

		}
		else
		{
			// x(i) = inv(A(i,i)) [ b(i) - sum(j=i+1..N) A(i,j) x(j) ]
			for(int i=size-1; i>=0; i--)
			{
				xMath::Vector3 scratch(0.0, 0.0, 0.0);
				int stop = min( size, i+1+banded);
				for( int j = i+1; j < stop; ++j )
				{
					const xMath::xMatrix3* fLji = L.findBlock(j,i);
					if(fLji)
					{
						xMath::xMatrix3 Ljit(xMath::transpose(*fLji));
						scratch += Ljit*x[j];
					}
				}
				scratch = b[i] - scratch;
				//
				xMath::xMatrix3& Lii = *(L.findBlock(i,i));

				float elem21 = Lii.getElem(1,2);
				float elem20 = Lii.getElem(0,2);
				float elem10 = Lii.getElem(0,1);
				
				x[i][2] = scratch[2]/Lii.getElem(2,2);
				x[i][1] = (scratch[1] - elem21*x[i][2])/Lii.getElem(1,1);
				x[i][0] = (scratch[0] - elem20*x[i][2] - elem10*x[i][1])/Lii.getElem(0,0);
			}
		}
	}
}


/* See http://www.hpl.hp.com/techreports/98/HPL-98-114.pdf */

bool CholFactor( xBlockSparse< xMath::xMatrix3 >& L, const int banded, const float eps)
{
	int size = L.getSize();
	for(int jb = 0; jb < size; jb++)
	{
		int start = max(0, jb - banded);
		for(int kb = start; kb < jb; kb++)
		{
			const xMath::xMatrix3* pLjbkb = L.findBlock(jb, kb);
			if(pLjbkb)
			{
				xMath::xMatrix3 scratch(xMath::transpose(*pLjbkb));	
				//float *b = (float*)pLjbkb;

				int stop = min(kb + banded + 1, size);
				for(int ib = jb; ib < stop; ib++)
				{
					const xMath::xMatrix3* pLibkb = L.findBlock(ib, kb);
					if(pLibkb)
					{
						xMath::xMatrix3& Libjb = L.getBlock(ib, jb);
						Libjb -= (*pLibkb) * scratch;
						/*float *a = (float*)pLibkb;
						float *res = (float*)&Libjb;
						res[0] -= a[0]*b[0] + a[3]*b[3] + a[6]*b[6];
						res[1] -= a[1]*b[0] + a[4]*b[3] + a[7]*b[6];
						res[2] -= a[2]*b[0] + a[5]*b[3] + a[8]*b[6];
						
						res[3] -= a[0]*b[1] + a[3]*b[4] + a[6]*b[7];
						res[4] -= a[1]*b[1] + a[4]*b[4] + a[7]*b[7];
						res[5] -= a[2]*b[1] + a[5]*b[4] + a[8]*b[7];
						
						res[6] -= a[0]*b[2] + a[3]*b[5] + a[6]*b[8];
						res[7] -= a[1]*b[2] + a[4]*b[5] + a[7]*b[8];
						res[8] -= a[2]*b[2] + a[5]*b[5] + a[8]*b[8];*/
					}
				}
			}
		}

		xMath::xMatrix3& Ljbjb = *L.findBlock(jb,jb);

		float elem00 = sqrtf(Ljbjb.getElem(0,0));
		float inv00 = 1.f / elem00;
		float elem10 = Ljbjb.getElem(0,1)*inv00;
		float elem20 = Ljbjb.getElem(0,2)*inv00;
		float elem11 = sqrtf(Ljbjb.getElem(1,1) - elem10*elem10);
		float inv11 = 1.f / elem11;
		float elem21 = (Ljbjb.getElem(1,2) - elem20*elem10)*inv11;
		float elem22 = sqrtf(Ljbjb.getElem(2,2) - elem20*elem20 - elem21*elem21);
		float inv22 = 1.f / elem22;

		Ljbjb = xMath::makexMatrix3( elem00, 0.f, 0.f,		elem10, elem11, 0.f,		elem20, elem21, elem22 );

		int stop = min(jb + banded + 1, size);
		for(int ib = jb + 1; ib < stop; ib++)
		{
			xMath::xMatrix3* pLibjb = L.findBlock(ib, jb);
			if(pLibjb)
			{
				xMath::xMatrix3& Libjb = *pLibjb;

				float loc00 = Libjb.getElem(0,0)*inv00;
				float loc10 = Libjb.getElem(0,1)*inv00;
				float loc20 = Libjb.getElem(0,2)*inv00;
				float loc01 = (Libjb.getElem(1,0) - loc00*elem10)*inv11;
				float loc11 = (Libjb.getElem(1,1) - loc10*elem10)*inv11;
				float loc21 = (Libjb.getElem(1,2) - loc20*elem10)*inv11;
				float loc02 = (Libjb.getElem(2,0) - (loc00*elem20 + loc01*elem21))*inv22;
				float loc12 = (Libjb.getElem(2,1) - (loc10*elem20 + loc11*elem21))*inv22;
				float loc22 = (Libjb.getElem(2,2) - (loc20*elem20 + loc21*elem21))*inv22;

				Libjb = xMath::makexMatrix3( loc00, loc01, loc02, loc10, loc11, loc12, loc20, loc21, loc22 );

#ifdef USE_COO
				if(xMatrixNormF2(Libjb) < eps)
				{
					L.removeBlock(ib, jb);
				}
#endif
			}
		}

	}

	return true;
}


