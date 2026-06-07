#pragma once
#include <SFML/Graphics/RenderWindow.hpp>

#include "graph/Graph.h"
#include "robot/IRobot.h"
#include "robot/types.h"
#include "staticEnvironments/StaticEnvironments.h"

class App
{
public:
	App(const std::vector<RobotState>& initStates, const std::vector<RobotState>& goalStates);

	void RunApp();

	~App() = default;

private:
	void DrawMap();
	void DrawRobots();
	void MakeCalculations();

	std::vector<std::unique_ptr<IRobot>> m_robots;
	StaticEnvironments m_staticEnvironments;

	Graph m_graph;

	sf::RenderWindow m_window;
};
