#pragma once
#include "IRobot.h"
#include "types.h"
#include "graph/Graph.h"

class Robot : public IRobot
{
public:
	Robot(const RobotState& initState, const RobotState& goalState, size_t predictionHorizon, Graph& graph);

	void Init(const RobotState& initState, const RobotState& goalState, size_t predictionHorizon) override;
	void Move() override;
	RobotState GetState() const override;
	RobotState GetGoalState() const override;
	double CalculateCollision(const RobotState& myState, const RobotState& otherState) const override;
	void PredictPath(const std::vector<std::vector<RobotState>>& robots) override;

	std::vector<RobotState> SendPredictivePoints() override;

	void CalculateOptimalPath() override;

	~Robot() = default;

private:
	size_t GetVertexByCoordinate(const Point& state) const;
	PointsVector GetRealPosFromGraph(std::vector<size_t> path) const;
	PointsVector GetSmoothedPath(const PointsVector& path, size_t mu, double epsilon, double dt);

	double CalculateCostTracking(const RobotState& myState, const PointsVector& shortestPath, const std::vector<InputVector>& inputs, size_t currentIndex);
	double CalculateCostRepulsive(const RobotState& myState, const std::vector<std::vector<RobotState>>& otherRobotsPaths, const std::vector<InputVector>& inputs);
	double CalculateMainFunc(const RobotState& myState, const PointsVector& shortestPath, const std::vector<std::vector<RobotState>>& otherRobotsPaths,
		const std::vector<InputVector>& inputs, size_t currentIndex);

	double CalculateCollisionBetweenRobots(const RobotState& myState, const std::vector<std::vector<RobotState>>& robots, const std::vector<InputVector>& inputs);

	RobotState GetPredictRobotState(const RobotState& currentState, const InputVector& input, double time);

private:
	RobotState m_goalState;
	RobotState m_currentState;
	RobotState m_initState;
	size_t m_predictionHorizon = 0;

	PointsVector m_optimalPath;
	std::vector<RobotState> m_predictPath;

	std::vector<InputVector> m_inputsVector;
	size_t m_indexOfPath = 0;

	Graph& m_graph;
};
