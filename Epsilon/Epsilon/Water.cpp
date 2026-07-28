#include "Water.h"
Water::Water(EpsilonVector surfacePosition, float density, float width, float depth) 
	:surfacePosition(surfacePosition),
	density(density),
	width(width),
	depth(depth)
{
}

Water::Water(AABB dimensions, float density)
	:dimensions(dimensions),
	density(density)
{
}
