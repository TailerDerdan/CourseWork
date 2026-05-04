#pragma once
#include <vector>

struct Point
{
	Point();
	Point(double x, double y) : x(x), y(y) {};

	double x = 0.0;
	double y = 0.0;

	~Point() = default;
};

using PointsVector = std::vector<Point>;

struct InputVector
{
	InputVector();
	InputVector(double omega) : omega(omega) {};

	double omega = 0.0;

	~InputVector() = default;
};

struct RobotState
{
	Point position;

	double phi = 0.0;
	double theta = 0.0;

	RobotState operator=(const RobotState& rhs)
	{
		if (this == &rhs)
		{
			return *this;
		}

		position = rhs.position;
		phi = rhs.phi;
		theta = rhs.theta;
		return *this;
	}
};

struct RectInRealMap
{
	RectInRealMap();
	RectInRealMap(double x, double y, double width, double height) : x(x), y(y), width(width), height(height) {};

	double x = 0.0;
	double y = 0.0;
	double width = 0.0;
	double height = 0.0;
};