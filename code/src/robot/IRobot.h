#pragma once
#include "types.h"
#include "dynamicEnvironments/IDynamicEnvironments.h"

class IRobot : public IDynamicEnvironments
{
public:
	virtual void Init(const RobotState& initState, const RobotState& goalState, size_t predictionHorizon) = 0;

	virtual void Move() = 0;

	virtual RobotState GetState() const = 0;

	virtual RobotState GetGoalState() const = 0;

	virtual double CalculateCollision(const RobotState& myState, const RobotState& otherState) const = 0;

	virtual void PredictPath(const std::vector<std::vector<RobotState>>& robots) = 0;

	virtual void CalculateOptimalPath() = 0;

	virtual ~IRobot() = default;
};
