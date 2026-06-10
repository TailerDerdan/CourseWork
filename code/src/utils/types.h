#pragma once
#include <vector>

struct Point
{
	Point() = default;
	Point(double x, double y) : x(x), y(y) {};

	double x = 0.0;
	double y = 0.0;

	friend std::ostream& operator<<(std::ostream& os, const Point& p)
	{
		os << p.x << " " << p.y;
		return os;
	}

	~Point() = default;
};

using PointsVector = std::vector<Point>;

struct RectInRealMap
{
	RectInRealMap();
	RectInRealMap(double x, double y, double width, double height) : x(x), y(y), width(width), height(height) {};

	double x = 0.0;
	double y = 0.0;
	double width = 0.0;
	double height = 0.0;
};