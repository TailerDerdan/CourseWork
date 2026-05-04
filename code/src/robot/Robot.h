#pragma once
#include "IRobot.h"
#include "graph/Graph.h"

class Robot : public IRobot
{
public:
	Robot() = default;

	void Init(const RobotState& initState, const RobotState& goalState, size_t predictionHorizon) override;
	void Move(const InputVector& input) override;
	RobotState GetState() const override;
	bool IsCollisionHappen(const RobotState& myState, const RobotState& otherState) const override;
	void PredictPath(const RobotState& myState) override;

	PointsVector SendPredictivePoints() override;

	PointsVector CalculateOptimalPath() override;

	~Robot() = default;

private:
	size_t GetVertexByCoordinate(const Point& state) const;
	PointsVector GetRealPosFromGraph(std::vector<size_t> path) const;
	PointsVector GetSmoothedPath(const PointsVector& path, size_t mu, double epsilon, double dt);

	double CalculateCostTracking(const RobotState& myState);
	double CalculateCostRepulsive(const RobotState& myState, std::vector<PointsVector> otherRobotsPaths);

private:
	RobotState m_goalState;
	RobotState m_currentState;
	RobotState m_initState;
	size_t m_predictionHorizon;

	PointsVector m_optimalPath;
	PointsVector m_predictPath;
	Graph& m_graph;
};
