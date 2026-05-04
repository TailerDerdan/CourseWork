#pragma once

#include <vector>
#include <fstream>

#include "utils/Types.h"

class StaticEnvironments
{
public:
	StaticEnvironments() {}

	void MakeStaticEnvironmentsInRealMap(std::ifstream &input);
	void MakeRectInRealMap(double x, double y, double width, double height);
	void MakeStaticEnvironmentsInGraph();

	~StaticEnvironments() = default;

private:
	void GetRectInRealMapFromStr(std::string str);

private:
	std::vector<RectInRealMap> m_staticEnvironmentsInRealMap;
	std::vector<std::vector<bool>> m_staticEnvironmentsInGraph;
};
