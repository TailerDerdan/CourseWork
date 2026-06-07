#pragma once
#include "utils/types.h"

class IDynamicEnvironments
{
public:
	virtual std::vector<RobotState> SendPredictivePoints() = 0;

	virtual ~IDynamicEnvironments() = default;
};
