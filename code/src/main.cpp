#include <iostream>
#include "./app/App.h"

int main()
{
	std::vector<RobotState> initStates = {
		RobotState{{100, 400}, 135 * M_PI / 180, 0},
		RobotState{{1200, 400}, 315 * M_PI / 180, 0},
		RobotState{{500, 100}, 315 * M_PI / 180, 0},
	};
	std::vector<RobotState> goalStates = {
		RobotState{{1200, 400}, 135 * M_PI / 180, 0},
		RobotState{{100, 400}, 315 * M_PI / 180, 0},
		RobotState{{500, 900}, 315 * M_PI / 180, 0},
	};

	App app(initStates, goalStates);
	app.RunApp();
}