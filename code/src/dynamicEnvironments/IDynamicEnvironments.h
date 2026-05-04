#pragma once
#include "utils/Types.h"

class IDynamicEnvironments
{
public:
	virtual PointsVector SendPredictivePoints() = 0;

	virtual ~IDynamicEnvironments() = default;
};
