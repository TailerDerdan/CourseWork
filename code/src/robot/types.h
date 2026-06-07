#pragma once
#include "constants.h"
#include "./utils/types.h"

struct InputVector
{
	InputVector() = default;
	InputVector(double omega) : omega(omega) {};

	double omega = 0.0;
	double v = V_SPEED;

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