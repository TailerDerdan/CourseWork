#pragma once

#include <list>
#include <set>
#include <vector>

using iPair = std::pair<double, size_t>;
using AdjList = std::vector<std::vector<std::pair<size_t, double>>>;

class Graph
{
public:
	Graph();
	~Graph() = default;

	void MakeGraph(size_t width, size_t height, std::vector<std::vector<bool>> environments);
	std::vector<size_t> GetShortestPath(size_t start, size_t end) const;

private:
	AdjList m_adjList;

	size_t m_countVertex = 0;
};