#pragma once

#include <list>
#include <set>
#include <vector>

using iPair = std::pair<size_t, size_t>;
using AdjList = std::vector<std::list<iPair>>;

class Graph
{
public:
	Graph();
	~Graph() = default;

	void MakeGraph(size_t width, size_t height);
	std::vector<size_t> GetShortestPath(size_t start, size_t end) const;

private:
	AdjList m_adjList;

	size_t m_countVertex = 0;
};