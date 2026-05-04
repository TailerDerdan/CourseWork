#pragma once
#include "dynamicEnvironments/IDynamicEnvironments.h"

class IRobot : public IDynamicEnvironments
{
	virtual void Init(const RobotState& initState, const RobotState& goalState, size_t predictionHorizon) = 0;

	virtual void Move(const InputVector& input) = 0;

	virtual RobotState GetState() const = 0;

	virtual bool IsCollisionHappen(const RobotState& myState, const RobotState& otherState) const = 0;

	virtual void PredictPath(const RobotState& myState) = 0;

	virtual PointsVector CalculateOptimalPath() = 0;

	virtual ~IRobot() = default;
};
