#include "Robot.h"

#include "constants.h"
#include "types.h"
#include "utils/constants.h"
#include "utils/utilFunc.h"

Robot::Robot(const RobotState& initState, const RobotState& goalState, size_t predictionHorizon, Graph& graph) : m_graph(graph)
{
	Robot::Init(initState, goalState, predictionHorizon);
}

void Robot::Init(const RobotState& initState, const RobotState& goalState, size_t predictionHorizon)
{
	m_initState = initState;
	m_currentState = initState;
	m_goalState = goalState;
	m_predictionHorizon = predictionHorizon;
	CalculateOptimalPath();
	m_inputsVector.resize(m_predictionHorizon);
	m_predictPath.resize(m_predictionHorizon);
	for (size_t iter = 0; iter < m_predictionHorizon; ++iter)
	{
		m_inputsVector[iter].omega = 0;
		m_predictPath[iter] = m_currentState;
	}
}

void Robot::Move()
{
	if (m_predictPath.empty()) return;
	m_currentState = GetPredictRobotState(m_currentState, m_inputsVector[0], 1.0);

	for (size_t i = 0; i < m_inputsVector.size() - 1; ++i)
	{
		m_inputsVector[i] = m_inputsVector[i + 1];
	}
	m_inputsVector.back().omega = 0.0;

	double minDist = std::numeric_limits<double>::max();
	for (size_t iter = 0; iter < m_optimalPath.size(); ++iter)
	{
		double dx = m_currentState.position.x - m_optimalPath[iter].x;
		double dy = m_currentState.position.y - m_optimalPath[iter].y;
		double dist = dx * dx + dy * dy;

		if (dist < minDist)
		{
			minDist = dist;
			m_indexOfPath = iter;
		}
		else if (dist > minDist)
		{
			break;
		}
	}
}

RobotState Robot::GetState() const
{
	return m_currentState;
}

RobotState Robot::GetGoalState() const
{
	return m_goalState;
}

double Robot::CalculateCollision(const RobotState& myState, const RobotState& otherState) const
{
	double dx = myState.position.x - otherState.position.x;
	double dy = myState.position.y - otherState.position.y;
	double cosPhi = std::cos(otherState.phi);
	double sinPhi = std::sin(otherState.phi);

	double firstPart = (dx * cosPhi + dy * sinPhi) * (dx * cosPhi + dy * sinPhi) / ((RADIUS_A + RADIUS_ROBOT) * (RADIUS_A + RADIUS_ROBOT));
	double secondPart = (-dx * sinPhi + dy * cosPhi) * (-dx * sinPhi + dy * cosPhi) / ((RADIUS_B + RADIUS_ROBOT) * (RADIUS_B + RADIUS_ROBOT));

	return firstPart + secondPart;
}

void Robot::PredictPath(const std::vector<std::vector<RobotState>>& robots)
{
	for (size_t iter = 0; iter < COUNT_STEPS_GRADIENT; ++iter)
	{
		std::vector<double> gradient(m_inputsVector.size(), 0.0);

		for (size_t iter2 = 0; iter2 < m_inputsVector.size(); ++iter2)
		{
			double originalOmega = m_inputsVector[iter2].omega;
			m_inputsVector[iter2].omega = originalOmega + STEP_GRADIENT;
			double resultMainFuncFirst = CalculateMainFunc(m_currentState, m_optimalPath, robots, m_inputsVector, m_indexOfPath);

			m_inputsVector[iter2].omega = originalOmega - STEP_GRADIENT;
			double resultMainFuncSecond = CalculateMainFunc(m_currentState, m_optimalPath, robots, m_inputsVector, m_indexOfPath);

			m_inputsVector[iter2].omega = originalOmega;
			gradient[iter2] = (resultMainFuncFirst - resultMainFuncSecond) / (2 * STEP_GRADIENT);
		}

		for (size_t iter2 = 0; iter2 < m_inputsVector.size(); ++iter2)
		{
			double update = gradient[iter2] * STEP_DESCENT;
			// double maxUpdate = 0.05;
			// if (update > maxUpdate)
			// {
			// 	update = maxUpdate;
			// }
			// if (update < -maxUpdate)
			// {
			// 	update = -maxUpdate;
			// }
			m_inputsVector[iter2].omega -= update;
			// m_inputsVector[iter2].omega = std::atan2(std::sin(m_inputsVector[iter2].omega), std::cos(m_inputsVector[iter2].omega));
		}
	}

	std::vector<RobotState> newStates;
	RobotState state = m_currentState;
	for (auto input : m_inputsVector)
	{
		state = GetPredictRobotState(state, input, TIME_FOR_PREDICT);
		newStates.push_back(state);
	}
	m_predictPath = newStates;
}

RobotState Robot::GetPredictRobotState(const RobotState& currentState, const InputVector& input, double time)
{
	RobotState newState = currentState;
	newState.phi += input.omega * time;
	newState.position.x += input.v * time * std::cos(newState.phi);
	newState.position.y += input.v * time * std::sin(newState.phi);
	return newState;
}

std::vector<RobotState> Robot::SendPredictivePoints()
{
	return m_predictPath;
}

void Robot::CalculateOptimalPath()
{
	auto startPoint = GetVertexByCoordinate(m_initState.position);
	auto endPoint = GetVertexByCoordinate(m_goalState.position);
	auto pathFromDijkstra = m_graph.GetShortestPath(startPoint, endPoint);
	auto rawPath = GetRealPosFromGraph(pathFromDijkstra);
	m_optimalPath = GetSmoothedPath(rawPath, MU, EPSILON, DT_SMOOTH_PATH);
}

size_t Robot::GetVertexByCoordinate(const Point& pos) const
{
	size_t xChanged = std::floor(pos.x / SIZE_CELL);
	size_t yChanged = std::floor(pos.y / SIZE_CELL);

	return xChanged * COUNT_CELL_Y + yChanged;
}

PointsVector Robot::GetRealPosFromGraph(std::vector<size_t> path) const
{
	PointsVector result;
	for (unsigned long pointFromGraph : path)
	{
		size_t xPoint = pointFromGraph / COUNT_CELL_Y;
		size_t yPoint = pointFromGraph % COUNT_CELL_Y;
		Point point(xPoint * SIZE_CELL, yPoint * SIZE_CELL);
		result.push_back(point);
	}
	return result;
}

double Robot::CalculateMainFunc(
	const RobotState& myState,
	const PointsVector& shortestPath,
	const std::vector<std::vector<RobotState>>& otherRobotsPaths,
	const std::vector<InputVector>& inputs,
	size_t currentIndex)
{
	double result = 0.0;
	result += WEIGHT_COST_TRACKING * CalculateCostTracking(myState, shortestPath, inputs, currentIndex);
	result += WEIGHT_COST_REPULSIVE * CalculateCostRepulsive(myState, otherRobotsPaths, inputs);
	result += CalculateCollisionBetweenRobots(myState, otherRobotsPaths, inputs);
	return result;
}

double Robot::CalculateCostTracking(
	const RobotState& myState,
	const PointsVector& shortestPath,
	const std::vector<InputVector>& inputs,
	size_t currentIndex)
{
	if (shortestPath.empty())
	{
		return 0.0;
	}

	double result = 0.0;
	RobotState predictedState = myState;

	for (size_t iter = 0; iter < inputs.size(); ++iter)
	{
		predictedState = GetPredictRobotState(predictedState, inputs[iter], TIME_FOR_PREDICT);
		Point nextPoint;
		auto offsetIter = size_t((iter + 1) * inputs[iter].v * TIME_FOR_PREDICT);
		size_t targetIndex = currentIndex + offsetIter;
		if (targetIndex >= shortestPath.size())
		{
			nextPoint = shortestPath.back();
		}
		else
		{
			nextPoint = shortestPath[targetIndex];
		}
		double dx = predictedState.position.x - nextPoint.x;
		double dy = predictedState.position.y - nextPoint.y;
		result += dx * dx + dy * dy;
	}
	return result;
}

double Robot::CalculateCostRepulsive(
	const RobotState& myState,
	const std::vector<std::vector<RobotState>>& otherRobotsPaths,
	const std::vector<InputVector>& inputs)
{
	double result = 0.0;
	RobotState predictedState = myState;

	for (size_t iter = 0; iter < inputs.size(); ++iter)
	{
		predictedState = GetPredictRobotState(predictedState, inputs[iter], TIME_FOR_PREDICT);

		for (const auto & otherRobotsPath : otherRobotsPaths)
		{
			if (iter < otherRobotsPath.size())
			{
				auto robotPoint = otherRobotsPath[iter];
				double dx = predictedState.position.x - robotPoint.position.x;
				double dy = predictedState.position.y - robotPoint.position.y;
				result += 1 / (dx * dx + dy * dy + GAMMA_REPULSIVE);
			}
		}
	}

	return result;
}

double Robot::CalculateCollisionBetweenRobots(const RobotState& myState, const std::vector<std::vector<RobotState>>& robots, const std::vector<InputVector>& inputs)
{
	double result = 0.0;
	RobotState predictedState = myState;

	for (size_t iter = 0; iter < inputs.size(); ++iter)
	{
		predictedState = GetPredictRobotState(predictedState, inputs[iter], TIME_FOR_PREDICT);

		for (const auto & robot : robots)
		{
			if (iter < robot.size())
			{
				double collision = CalculateCollision(predictedState, robot[iter]);
				double dCol = 1 - collision;
				double resultCollision = std::max(dCol, 0.0);
				result += resultCollision * resultCollision * LARGE_LAMBDA;
			}
		}
	}

	return result;
}

PointsVector Robot::GetSmoothedPath(const PointsVector& path, size_t mu, double epsilon, double dt)
{
	if (path.size() < 2) return path;

	size_t numPoints = path.size();
	size_t numSegments = numPoints - 1;

	std::vector<double> s(numSegments);
	std::vector<double> cumS(numPoints, 0.0);

	for (size_t iter = 0; iter < numSegments; ++iter)
	{
		s[iter] = GetDistance(path[iter], path[iter+1]);
		cumS[iter+1] = cumS[iter] + s[iter];
	}

	PointsVector result;
	double totalLength = cumS.back();

	for (size_t t = 0; t < totalLength; t += dt)
	{
		size_t m = 0;
		while (m < numSegments - 1 && t > cumS[m + 1])
		{
			m++;
		}

		double sumX = 0.0;
		double sumY = 0.0;
		double totalWeight = 0.0;

		size_t endH = std::min(m + mu, numSegments - 1);

		for (size_t h = m; h <= endH; ++h)
		{
			double argMinus = (t - cumS[h+1]) / epsilon;
			double argPlus  = (-t + cumS[h]) / epsilon;

			double sigmaMinus = 1.0 / (1.0 + std::exp(argMinus));
			double sigmaPlus  = 1.0 / (1.0 + std::exp(argPlus));

			double weight = sigmaMinus * sigmaPlus;
			double localT = (t - cumS[h]) / s[h];

			Point zeta;
			zeta.x = path[h].x + localT * (path[h + 1].x - path[h].x);
			zeta.y = path[h].y + localT * (path[h + 1].y - path[h].y);

			sumX += weight * zeta.x;
			sumY += weight * zeta.y;
			totalWeight += weight;
		}

		if (totalWeight > 0.0)
		{
			result.push_back({sumX / totalWeight, sumY / totalWeight});
		}
	}

	result.push_back(path.back());
	return result;
}