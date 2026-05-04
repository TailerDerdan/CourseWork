#include "Robot.h"

#include "utils/Constants.h"
#include "utils/utilFunc.h"

void Robot::Init(const RobotState& initState, const RobotState& goalState, size_t predictionHorizon)
{
	m_initState = initState;
	m_goalState = goalState;
	m_predictionHorizon = predictionHorizon;
	m_optimalPath = CalculateOptimalPath();
}

void Robot::Move(const InputVector& input)
{
	if (m_predictPath.empty()) return;

}

RobotState Robot::GetState() const
{
	return m_currentState;
}

bool Robot::IsCollisionHappen(const RobotState& myState, const RobotState& otherState) const
{
	double dx = myState.position.x - otherState.position.x;
	double dy = myState.position.y - otherState.position.y;
	double cosPhi = std::cos(otherState.phi);
	double sinPhi = std::sin(otherState.phi);

	double firstPart = (dx * cosPhi + dy * sinPhi) * (dx * cosPhi + dy * sinPhi) / ((RADIUS_A + RADIUS_ROBOT) * (RADIUS_A + RADIUS_ROBOT));
	double secondPart = (-dx * sinPhi + dy * cosPhi) * (-dx * sinPhi + dy * cosPhi) / ((RADIUS_B + RADIUS_ROBOT) * (RADIUS_B + RADIUS_ROBOT));

	return firstPart + secondPart > 1;
}

void Robot::PredictPath(const RobotState& myState)
{

}

PointsVector Robot::SendPredictivePoints()
{
	return m_predictPath;
}

PointsVector Robot::CalculateOptimalPath()
{
	auto startPoint = GetVertexByCoordinate(m_initState.position);
	auto endPoint = GetVertexByCoordinate(m_goalState.position);
	auto pathFromDijkstra = m_graph.GetShortestPath(startPoint, endPoint);
	return GetRealPosFromGraph(pathFromDijkstra);
}

size_t Robot::GetVertexByCoordinate(const Point& pos) const
{
	size_t xChanged = std::floor(pos.x / SIZE_CELL);
	size_t yChanged = std::floor(pos.y / SIZE_CELL);

	return yChanged * COUNT_CELL_Y + xChanged;
}

PointsVector Robot::GetRealPosFromGraph(std::vector<size_t> path) const
{
	PointsVector result;
	for (size_t iter = 0; iter < path.size(); ++iter)
	{
		size_t yPoint = std::floor(path[iter] / COUNT_CELL_X);
		size_t xPoint = path[iter] - yPoint * COUNT_CELL_X;
		Point point(yPoint * SIZE_CELL, xPoint * SIZE_CELL);
		result.push_back(point);
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

	for (size_t i = 0; i < numSegments; ++i)
	{
		s[i] = GetDistance(path[i], path[i+1]);
		cumS[i+1] = cumS[i] + s[i];
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

double Robot::CalculateCostTracking(const RobotState& myState)
{

}

double Robot::CalculateCostRepulsive(const RobotState& myState, std::vector<PointsVector> otherRobotsPaths)
{

}