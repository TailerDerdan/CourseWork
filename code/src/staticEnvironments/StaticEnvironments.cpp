#include "StaticEnvironments.h"

#include <iostream>
#include <sstream>

#include "robot/constants.h"
#include "utils/constants.h"

void StaticEnvironments::MakeStaticEnvironmentsInRealMap(std::ifstream &input)
{
	std::string line;
	while (std::getline(input, line))
	{
		try
		{
			GetRectInRealMapFromStr(line);
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << std::endl;
			return;
		}
	}
}

void StaticEnvironments::MakeStaticEnvironmentsInGraph(size_t width, size_t height)
{
	if (m_staticEnvironmentsInRealMap.empty())
	{
		std::cerr << "No static environment supplied" << std::endl;
		return;
	}
	m_staticEnvironmentsInGraph.resize(width);
	for (size_t x = 0; x < width; ++x)
	{
		m_staticEnvironmentsInGraph[x].resize(height);
	}
	for (auto & env : m_staticEnvironmentsInRealMap)
	{
		size_t xChanged = std::floor(env.x / SIZE_CELL);
		size_t yChanged = std::floor(env.y / SIZE_CELL);
		size_t widthChanged = std::floor(env.width / SIZE_CELL);
		size_t heightChanged = std::floor(env.height / SIZE_CELL);
		for (size_t x = xChanged; x < xChanged + widthChanged; ++x)
		{
			for (size_t y = yChanged; y < yChanged + heightChanged; ++y)
			{
				if (x < COUNT_CELL_X && y < COUNT_CELL_Y)
				{
					m_staticEnvironmentsInGraph[x][y] = true;
				}
			}
		}
	}

	int offset = static_cast<int>(std::ceil(RADIUS_A / SIZE_CELL)) + 1;
	m_drawableStatEnvInGraph = m_staticEnvironmentsInGraph;
	for (size_t x = 0; x < width; ++x)
	{
		for (size_t y = 0; y < height; ++y)
		{
			if (m_drawableStatEnvInGraph[x][y])
			{
				for (int dx = -offset; dx <= offset; ++dx)
				{
					for (int dy = -offset; dy <= offset; ++dy)
					{
						int nx = x + dx;
						int ny = y + dy;

						if (nx >= 0 && nx < width && ny >= 0 && ny < height)
						{
							m_staticEnvironmentsInGraph[nx][ny] = true;
						}
					}
				}
			}
		}
	}
}

void StaticEnvironments::MakeRectInRealMap(double x, double y, double width, double height)
{
	auto rect = RectInRealMap(x, y, width, height);
	m_staticEnvironmentsInRealMap.push_back(rect);
}

void StaticEnvironments::GetRectInRealMapFromStr(std::string str)
{
	std::istringstream iss(str);
	double x, y, width, height;
	if (iss >> x >> y >> width >> height)
	{
		MakeRectInRealMap(x, y, width, height);
	}
	else
	{
		throw std::invalid_argument("incorrect input static environments");
	}
}

std::vector<std::vector<bool>> StaticEnvironments::GetStaticEnvironmentsInGraph()
{
	return m_staticEnvironmentsInGraph;
}

std::vector<std::vector<bool>> StaticEnvironments::GetDrawableStaticEnv()
{
	return m_drawableStatEnvInGraph;
}