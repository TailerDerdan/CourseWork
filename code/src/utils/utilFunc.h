#pragma once
#include "types.h"

double GetDistance(const Point& p1, const Point& p2)
{
	return std::hypot(p2.x - p1.x, p2.y - p1.y);
}
