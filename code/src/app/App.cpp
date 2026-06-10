#include "App.h"
#include "robot/Robot.h"
#include "constants.h"

#include <memory>
#include <fstream>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RectangleShape.hpp>

namespace
{
	std::array<sf::Color, 100> COLORS_FOR_ROBOTS = {
	    sf::Color::White, sf::Color::Black, sf::Color::Yellow, sf::Color::Green,
	    sf::Color::Red, sf::Color::Blue, sf::Color::Magenta, sf::Color::Cyan,
	    sf::Color(128, 128, 128), sf::Color(192, 192, 192),

	    sf::Color(255, 102, 102), sf::Color(204, 0, 0), sf::Color(153, 0, 0), sf::Color(255, 51, 153),
	    sf::Color(204, 0, 102), sf::Color(255, 153, 204), sf::Color(255, 204, 229), sf::Color(153, 51, 102),
	    sf::Color(102, 0, 51), sf::Color(255, 102, 178),

	    sf::Color(255, 128, 0), sf::Color(204, 102, 0), sf::Color(153, 76, 0), sf::Color(255, 178, 102),
	    sf::Color(255, 153, 51), sf::Color(204, 153, 102), sf::Color(153, 102, 51), sf::Color(102, 51, 0),
	    sf::Color(255, 204, 153), sf::Color(210, 105, 30),

	    sf::Color(255, 255, 102), sf::Color(204, 204, 0), sf::Color(153, 153, 0), sf::Color(255, 255, 153),
	    sf::Color(255, 215, 0), sf::Color(218, 165, 32), sf::Color(184, 134, 11), sf::Color(240, 230, 140),
	    sf::Color(189, 183, 107), sf::Color(128, 128, 0),

	    sf::Color(102, 255, 102), sf::Color(0, 204, 0), sf::Color(0, 153, 0), sf::Color(153, 255, 153),
	    sf::Color(51, 204, 51), sf::Color(0, 102, 0), sf::Color(173, 255, 47), sf::Color(124, 252, 0),
	    sf::Color(34, 139, 34), sf::Color(0, 255, 127),

	    sf::Color(102, 255, 255), sf::Color(0, 204, 204), sf::Color(0, 153, 153), sf::Color(153, 255, 255),
	    sf::Color(72, 209, 204), sf::Color(32, 178, 170), sf::Color(0, 128, 128), sf::Color(64, 224, 208),
	    sf::Color(0, 250, 154), sf::Color(175, 238, 238),

	    sf::Color(102, 102, 255), sf::Color(0, 0, 204), sf::Color(0, 0, 153), sf::Color(153, 153, 255),
	    sf::Color(65, 105, 225), sf::Color(0, 0, 128), sf::Color(135, 206, 235), sf::Color(70, 130, 180),
	    sf::Color(30, 144, 255), sf::Color(0, 191, 255),

	    sf::Color(178, 102, 255), sf::Color(102, 0, 204), sf::Color(51, 0, 102), sf::Color(204, 153, 255),
	    sf::Color(138, 43, 226), sf::Color(148, 0, 211), sf::Color(75, 0, 130), sf::Color(123, 104, 238),
	    sf::Color(218, 112, 214), sf::Color(221, 160, 221),

	    sf::Color(139, 69, 19), sf::Color(160, 82, 45), sf::Color(205, 133, 63), sf::Color(222, 184, 135),
	    sf::Color(245, 245, 220), sf::Color(255, 228, 196), sf::Color(255, 218, 185), sf::Color(250, 128, 114),
	    sf::Color(233, 150, 122), sf::Color(240, 128, 128),

	    sf::Color(255, 192, 203), sf::Color(255, 182, 193), sf::Color(255, 105, 180), sf::Color(255, 20, 147),
	    sf::Color(199, 21, 133), sf::Color(112, 128, 144), sf::Color(119, 136, 153), sf::Color(176, 196, 222),
	    sf::Color(230, 230, 250), sf::Color(245, 255, 250)
	};
}

App::App(const std::vector<RobotState>& initStates, const std::vector<RobotState>& goalStates)
{
	m_window.create(sf::VideoMode({1920, 1080}), "Multi Robot System");

	std::ifstream input("../src/static/staticEnvironments.txt");
	if (!input.is_open())
	{
		throw std::runtime_error("Can't open input file");
	}
    m_staticEnvironments.MakeStaticEnvironmentsInRealMap(input);
	m_staticEnvironments.MakeStaticEnvironmentsInGraph(COUNT_CELL_X, COUNT_CELL_Y);

	m_staticEnvironmentsInGraph = m_staticEnvironments.GetStaticEnvironmentsInGraph();
	m_drawableStatEnvInGraph = m_staticEnvironments.GetDrawableStaticEnv();

	m_graph.MakeGraph(COUNT_CELL_X, COUNT_CELL_Y, m_staticEnvironmentsInGraph);
	for (size_t iter = 0; iter < initStates.size(); iter++)
	{
		auto newRobot = std::make_unique<Robot>(
		  initStates[iter],
		  goalStates[iter],
		  PREDICTIVE_SIZE,
		  m_graph
	   );
		m_robots.push_back(std::move(newRobot));
	}
}

void App::RunApp()
{
	while (m_window.isOpen())
	{
		while (const std::optional event = m_window.pollEvent())
		{
			if (event->is<sf::Event::Closed>())
			{
				m_window.close();
			}
			else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
			{
				if (keyPressed->scancode == sf::Keyboard::Scancode::Escape)
				{
					m_window.close();
				}
			}
		}

		MakeCalculations();

		m_window.clear(sf::Color::White);
		DrawMap();
		DrawRobots();
		m_window.display();
	}
}

void App::DrawMap()
{
	sf::RectangleShape floor(sf::Vector2f(MAP_WIDTH, MAP_HEIGHT));
	floor.setFillColor(sf::Color(160, 163, 168));
	m_window.draw(floor);

	sf::VertexArray gridLines(sf::PrimitiveType::Lines);
	sf::Color gridColor(120, 120, 120);

	for (size_t iter = 0; iter < COUNT_CELL_X; iter++)
	{
		gridLines.append(sf::Vertex(sf::Vector2f(iter * SIZE_CELL, 0), gridColor));
		gridLines.append(sf::Vertex(sf::Vector2f(iter * SIZE_CELL, MAP_HEIGHT), gridColor));
	}

	for (size_t iter = 0; iter < COUNT_CELL_Y; iter++)
	{
		gridLines.append(sf::Vertex(sf::Vector2f(0, iter * SIZE_CELL), gridColor));
		gridLines.append(sf::Vertex(sf::Vector2f(MAP_WIDTH, iter * SIZE_CELL), gridColor));
	}

	sf::RectangleShape env;
	env.setSize(sf::Vector2f(SIZE_CELL, SIZE_CELL));
	env.setFillColor(sf::Color::Black);
	env.setOutlineColor(sf::Color::Red);
	for (size_t x = 0; x < COUNT_CELL_X; x++)
	{
		for (size_t y = 0; y < COUNT_CELL_Y; y++)
		{
			if (m_drawableStatEnvInGraph[x][y])
			{
				env.setPosition(sf::Vector2f(x * SIZE_CELL, y * SIZE_CELL));
				m_window.draw(env);
			}
		}
	}

	m_window.draw(gridLines);
}

void App::DrawRobots()
{
	sf::RectangleShape drawableRobot(sf::Vector2f(RADIUS_ROBOT * 4.0, RADIUS_ROBOT * 2.0));

	drawableRobot.setOutlineColor(sf::Color::Black);
	drawableRobot.setOutlineThickness(2);
	drawableRobot.setOrigin(sf::Vector2f(RADIUS_ROBOT * 2, RADIUS_ROBOT));

	sf::CircleShape drawablePoint;
	drawablePoint.setRadius(2.0f);

	sf::CircleShape goalPoint;
	goalPoint.setRadius(5.0f);

	sf::RectangleShape pointOfPath;
	pointOfPath.setFillColor(sf::Color(160, 163, 168));
	pointOfPath.setOutlineThickness(1.0f);
	pointOfPath.setSize(sf::Vector2f(SIZE_CELL, SIZE_CELL));

	size_t index = 0;
	for (auto& robot : m_robots)
	{
		auto path = robot->SendPredictivePoints();
		for (auto& point : path)
		{
			drawablePoint.setFillColor(COLORS_FOR_ROBOTS[index]);
			drawablePoint.setPosition(sf::Vector2f(point.position.x, point.position.y));
			m_window.draw(drawablePoint);
		}

		auto optimalPath = robot->GetOptimalPath();
		for (auto& point : optimalPath)
		{
			pointOfPath.setOutlineColor(COLORS_FOR_ROBOTS[index]);
			pointOfPath.setPosition(sf::Vector2f(point.x, point.y));
			m_window.draw(pointOfPath);
		}

		auto goalState = robot->GetGoalState();
		goalPoint.setFillColor(COLORS_FOR_ROBOTS[index]);
		goalPoint.setPosition(sf::Vector2f(goalState.position.x, goalState.position.y));
		m_window.draw(goalPoint);

		drawableRobot.setFillColor(COLORS_FOR_ROBOTS[index]);
		drawableRobot.setPosition(sf::Vector2f(robot->GetState().position.x, robot->GetState().position.y));
		drawableRobot.setRotation(sf::radians(robot->GetState().phi));
		m_window.draw(drawableRobot);

		index++;
	}
}

void App::MakeCalculations()
{
	std::vector<std::vector<RobotState>> robotStates;
	robotStates.reserve(m_robots.size());
	for (auto& robot : m_robots)
	{
		robotStates.push_back(robot->SendPredictivePoints());
	}
	for (auto& robot : m_robots)
	{
		robot->PredictPath(robotStates, m_staticEnvironmentsInGraph);
		robot->Move();
	}
}