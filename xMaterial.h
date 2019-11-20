
#ifndef _Material_h
#define _Material_h

#include "VecMatrixHelper.h"

#define ROD_DEFAULT_RADIUS 0.01

enum
{
	MATERIAL_DENSITY,
	MATERIAL_YOUNGS_STRETCH,
	MATERIAL_YOUNGS_DAMP,
	MATERIAL_YOUNGS_BEND,
	MATERIAL_YOUNGS_TWIST,
	MATERIAL_VISCOUS_DAMP,

	NUM_MATERIAL_PROPERTIES
};

class Material
{
public:
	real values[NUM_MATERIAL_PROPERTIES];
};


#endif