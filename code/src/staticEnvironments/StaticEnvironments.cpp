#include "StaticEnvironments.h"

#include <iostream>
#include <sstream>

#include "utils/Constants.h"

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

void StaticEnvironments::MakeStaticEnvironmentsInGraph()
{
	if (m_staticEnvironmentsInRealMap.empty())
	{
		std::cerr << "No static environment supplied" << std::endl;
		return;
	}

	for (auto iter = m_staticEnvironmentsInRealMap.begin(); iter != m_staticEnvironmentsInRealMap.end(); ++iter)
	{
		size_t xChanged = std::floor(iter->x / SIZE_CELL);
		size_t yChanged = std::floor(iter->y / SIZE_CELL);
		size_t widthChanged = std::floor(iter->width / SIZE_CELL);
		size_t heightChanged = std::floor(iter->height / SIZE_CELL);
		for (size_t y = 0; y < heightChanged; ++y)
		{
			for (size_t x = 0; x < widthChanged; ++x)
			{
				if (x >= xChanged && y >= yChanged && x <= xChanged + widthChanged && y <= yChanged + heightChanged)
				{
					m_staticEnvironmentsInGraph.at(y).at(x) = true;
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