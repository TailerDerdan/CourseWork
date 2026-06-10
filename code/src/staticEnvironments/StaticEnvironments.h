#pragma once

#include <vector>
#include <fstream>

#include "utils/types.h"

class StaticEnvironments
{
public:
	StaticEnvironments() {}

	void MakeStaticEnvironmentsInRealMap(std::ifstream &input);
	void MakeRectInRealMap(double x, double y, double width, double height);
	void MakeStaticEnvironmentsInGraph(size_t width, size_t height);

	std::vector<std::vector<bool>> GetStaticEnvironmentsInGraph();
	std::vector<std::vector<bool>> GetDrawableStaticEnv();

	~StaticEnvironments() = default;

private:
	void GetRectInRealMapFromStr(std::string str);

private:
	std::vector<RectInRealMap> m_staticEnvironmentsInRealMap;
	std::vector<std::vector<bool>> m_staticEnvironmentsInGraph;
	std::vector<std::vector<bool>> m_drawableStatEnvInGraph;
};
