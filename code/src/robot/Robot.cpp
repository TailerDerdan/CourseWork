#include "Robot.h"

#include "constants.h"
#include "types.h"
#include "utils/constants.h"
#include "utils/utilFunc.h"
#include <fstream>
#include <algorithm>

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
	CalculateOptimalPath(m_initState);
	m_inputsVector.resize(m_predictionHorizon);
	m_predictPath.resize(m_predictionHorizon);
	for (size_t iter = 0; iter < m_predictionHorizon; ++iter)
	{
		m_inputsVector[iter].omega = 0;
		m_inputsVector[iter].v = V_SPEED;
		m_predictPath[iter] = m_currentState;
	}
}

void Robot::Move()
{
	if (m_predictPath.empty()) return;
	m_currentState = GetPredictRobotState(m_currentState, m_inputsVector[0], TIME_FOR_PREDICT);

	for (size_t i = 0; i < m_inputsVector.size() - 1; ++i)
	{
		m_inputsVector[i] = m_inputsVector[i + 1];
	}
	m_inputsVector.back().omega = 0.0;
	m_inputsVector.back().v = V_SPEED;

	double minDist = std::numeric_limits<double>::max();
	size_t bestIndex = m_indexOfPath;
	size_t searchWindow = std::min(m_optimalPath.size(), m_indexOfPath + size_t(SIZE_CELL));

	for (size_t iter = m_indexOfPath; iter < searchWindow; ++iter)
	{
		double dx = m_currentState.position.x - m_optimalPath[iter].x;
		double dy = m_currentState.position.y - m_optimalPath[iter].y;
		double dist = dx * dx + dy * dy;

		if (dist < minDist)
		{
			minDist = dist;
			bestIndex = iter;
		}
	}
	m_indexOfPath = bestIndex;
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

void Robot::PredictPath(const std::vector<std::vector<RobotState>>& robots, const std::vector<std::vector<bool>>& statEnvs)
{
	CalculateOptimalPath(m_currentState);
	for (size_t iter = 0; iter < COUNT_STEPS_GRADIENT; ++iter)
	{
		std::vector<double> gradient(m_inputsVector.size(), 0.0);

		for (size_t iter2 = 0; iter2 < m_inputsVector.size(); ++iter2)
		{
			double originalOmega = m_inputsVector[iter2].omega;
			m_inputsVector[iter2].omega = originalOmega + STEP_GRADIENT;
			double resultMainFuncFirst = CalculateMainFunc(m_currentState, m_optimalPath, robots, m_inputsVector, statEnvs, m_indexOfPath);

			m_inputsVector[iter2].omega = originalOmega - STEP_GRADIENT;
			double resultMainFuncSecond = CalculateMainFunc(m_currentState, m_optimalPath, robots, m_inputsVector, statEnvs, m_indexOfPath);

			m_inputsVector[iter2].omega = originalOmega;
			gradient[iter2] = (resultMainFuncFirst - resultMainFuncSecond) / (2 * STEP_GRADIENT);
		}

		CorrectGradient(m_currentState, gradient, statEnvs, robots);
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

PointsVector Robot::GetOptimalPath()
{
	return m_optimalPath;
}

void Robot::CalculateOptimalPath(const RobotState& state)
{
	auto startPoint = GetVertexByCoordinate(state.position);
	auto endPoint = GetVertexByCoordinate(m_goalState.position);
	auto pathFromDijkstra = m_graph.GetShortestPath(startPoint, endPoint);
	auto rawPath = GetRealPosFromGraph(pathFromDijkstra);
	// m_optimalPath = GetSmoothedPath(rawPath);
	m_optimalPath = rawPath;
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
	const std::vector<std::vector<bool>>& statEnvs,
	size_t currentIndex)
{
	double result = 0.0;
	result += WEIGHT_COST_TRACKING * CalculateCostTracking(myState, shortestPath, inputs, currentIndex);
	result += CalculateCostRepulsive(myState, otherRobotsPaths, inputs);
	result += CalculateCollisionBetweenRobots(myState, otherRobotsPaths, inputs);
	result += CalculateObstacleCost(myState, inputs, statEnvs);
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

		double lookAheadDistance = inputs[iter].v * TIME_FOR_PREDICT * (iter + 1);
		size_t targetIndex = currentIndex;
		double accumulatedDist = 0.0;

		while (targetIndex < shortestPath.size() - 2 && accumulatedDist < lookAheadDistance)
		{
			double dxPath = shortestPath[targetIndex + 1].x - shortestPath[targetIndex].x;
			double dyPath = shortestPath[targetIndex + 1].y - shortestPath[targetIndex].y;
			accumulatedDist += std::sqrt(dxPath * dxPath + dyPath * dyPath);
			targetIndex++;
		}
		Point nextPoint = shortestPath[targetIndex];

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
	double safeDistance = SIZE_CELL * 2.6;

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
				double distance = std::sqrt(dx * dx + dy * dy + GAMMA_REPULSIVE);

				if (distance < safeDistance)
				{
					double clearance = distance - (RADIUS_ROBOT * 4);
					if (clearance < 0.1) clearance = 0.1;

					result += WEIGHT_COST_REPULSIVE / (clearance * clearance);
				}
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

// double Robot::CalculateObstacleCost(const RobotState& myState, const std::vector<InputVector>& inputs, const std::vector<std::vector<bool>>& statEnvs)
// {
// 	double result = 0.0;
// 	RobotState predictedState = myState;
//
// 	for (auto input : inputs)
// 	{
// 		predictedState = GetPredictRobotState(predictedState, input, TIME_FOR_PREDICT);
//
// 		if (IsCellInObstacle(predictedState.position.x, predictedState.position.y, statEnvs))
// 		{
// 			double dx = myState.position.x - predictedState.position.x;
// 			double dy = myState.position.y - predictedState.position.y;
// 			double dist = std::sqrt(dx * dx + dy * dy);
// 			result += (WEIGHT_COST_OBSTACLES * dist);
// 		}
// 	}
// 	return result;
// }

double Robot::CalculateObstacleCost(
    const RobotState& myState,
    const std::vector<InputVector>& inputs,
    const std::vector<std::vector<bool>>& statEnvs)
{
    double result = 0.0;
    RobotState predictedState = myState;

    double senseRadiusCells = 3.0;
    double minSafeDist = SIZE_CELL * 1.5;

    for (const auto& input : inputs)
    {
        predictedState = GetPredictRobotState(predictedState, input, TIME_FOR_PREDICT);

        int cx = static_cast<int>(std::floor(predictedState.position.x / SIZE_CELL));
        int cy = static_cast<int>(std::floor(predictedState.position.y / SIZE_CELL));
        int searchRadius = static_cast<int>(std::ceil(senseRadiusCells));

        double minDist = senseRadiusCells * SIZE_CELL;
        bool foundObstacle = false;

        for (int dx = -searchRadius; dx <= searchRadius; ++dx)
        {
            for (int dy = -searchRadius; dy <= searchRadius; ++dy)
            {
                int nx = cx + dx;
                int ny = cy + dy;

                if (nx >= 0 && nx < COUNT_CELL_X && ny >= 0 && ny < COUNT_CELL_Y)
                {
                    if (statEnvs[nx][ny])
                    {
                        double obsX = (nx + 0.5) * SIZE_CELL;
                        double obsY = (ny + 0.5) * SIZE_CELL;
                        double dist = std::hypot(predictedState.position.x - obsX, predictedState.position.y - obsY);

                        if (dist < minDist)
                        {
                            minDist = dist;
                            foundObstacle = true;
                        }
                    }
                }
            }
        }

        if (foundObstacle && minDist < minSafeDist)
        {
            double clearance = minDist - RADIUS_ROBOT;
            if (clearance <= 0.1) clearance = 0.1;

            result += std::exp(3.0 * (minSafeDist - clearance) / SIZE_CELL);
        }
    }
    return result;
}

void Robot::CorrectGradient(const RobotState& myState, const std::vector<double>& gradient,
	const std::vector<std::vector<bool>>& statEnvs, const std::vector<std::vector<RobotState>>& robots)
{
	auto safeInputs = m_inputsVector;
	std::vector<double> updates(m_inputsVector.size());
	for (size_t iter = 0; iter < m_inputsVector.size(); ++iter)
	{
		updates[iter] = gradient[iter] * STEP_DESCENT;

		double maxUpdate = 0.06;
		if (updates[iter] > maxUpdate) updates[iter] = maxUpdate;
		if (updates[iter] < -maxUpdate) updates[iter] = -maxUpdate;
	}

	double step = 1.0;
	bool isSafe = false;

	for (size_t attempt = 0; attempt < 30; ++attempt)
	{
		for (size_t iter = 0; iter < m_inputsVector.size(); ++iter)
		{
			m_inputsVector[iter].omega = safeInputs[iter].omega - (updates[iter] * step);
			double MAX_OMEGA = 0.3;
			if (m_inputsVector[iter].omega > MAX_OMEGA) m_inputsVector[iter].omega = MAX_OMEGA;
			if (m_inputsVector[iter].omega < -MAX_OMEGA) m_inputsVector[iter].omega = -MAX_OMEGA;
		}

		if (!CheckCollisionWithStatEnv(myState, statEnvs, robots))
		{
			isSafe = true;
			break;
		}

		step *= 0.5;
	}

	if (!isSafe)
	{
		m_inputsVector = safeInputs;
	}
}

bool Robot::CheckCollisionWithStatEnv(const RobotState& myState, const std::vector<std::vector<bool>>& statEnvs,
	const std::vector<std::vector<RobotState>>& robots)
{
	RobotState predictedState = myState;

	for (size_t iter = 0; iter < m_inputsVector.size(); ++iter)
	{
		predictedState = GetPredictRobotState(predictedState, m_inputsVector[iter], TIME_FOR_PREDICT);
		if (IsCellInObstacle(predictedState.position.x, predictedState.position.y, statEnvs))
		{
			return true;
		}
	}
	return false;
}

bool Robot::IsCellInObstacle(double x, double y, const std::vector<std::vector<bool>>& statEnvs) const
{
	int minCellX = std::max(0, static_cast<int>(std::floor((x - RADIUS_ROBOT) / SIZE_CELL)));
	int maxCellX = std::min(static_cast<int>(COUNT_CELL_X - 1), static_cast<int>(std::floor((x + RADIUS_ROBOT) / SIZE_CELL)));

	int minCellY = std::max(0, static_cast<int>(std::floor((y - RADIUS_ROBOT) / SIZE_CELL)));
	int maxCellY = std::min(static_cast<int>(COUNT_CELL_Y - 1), static_cast<int>(std::floor((y + RADIUS_ROBOT) / SIZE_CELL)));

	for (int iter = minCellX; iter <= maxCellX; ++iter)
	{
		for (int iter2 = minCellY; iter2 <= maxCellY; ++iter2)
		{
			if (statEnvs[iter][iter2])
			{
				return true;
			}
		}
	}
	return false;
}

PointsVector Robot::GetSmoothedPath(const PointsVector& path)
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

	for (size_t t = 0; t < totalLength; t += DT_SMOOTH_PATH)
	{
		size_t m = 0;
		while (m < numSegments - 1 && t > cumS[m + 1])
		{
			m++;
		}

		double sumX = 0.0;
		double sumY = 0.0;
		double totalWeight = 0.0;

		size_t endH = std::min(m + MU, numSegments - 1);

		for (size_t h = m; h <= endH; ++h)
		{
			double epsilon = s[h] * 0.3;
			double argMinus = (t - cumS[h+1]) / epsilon;
			double argPlus  = (-t + cumS[h]) / epsilon;

			double sigmaMinus = 1.0 / (1.0 + std::exp(argMinus));
			double sigmaPlus  = 1.0 / (1.0 + std::exp(argPlus));

			double weight = sigmaMinus * sigmaPlus;
			double localT = (t - cumS[h]) / s[h];
			localT = std::clamp(localT, 0.0, 1.0);

			Point zeta;
			zeta.x = path[h].x + localT * (path[h + 1].x - path[h].x);
			zeta.y = path[h].y + localT * (path[h + 1].y - path[h].y);

			sumX += weight * zeta.x;
			sumY += weight * zeta.y;
			totalWeight += weight;
		}

		if (totalWeight > 0.0)
		{
			result.emplace_back(sumX / totalWeight, sumY / totalWeight);
		}
	}

	result.push_back(path.back());
	return result;
}