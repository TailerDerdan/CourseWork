#pragma once

#include <list>
#include <set>
#include <vector>

struct Edge
{
	int weight;
	size_t uVertex;
	size_t vVertex;
};

using AdjMatrix = std::vector<std::vector<int>>;
using ListEdges = std::vector<Edge>;

using iPair = std::pair<size_t, size_t>;
using AdjList = std::vector<std::list<iPair>>;

class Graph
{
public:
	Graph();
	~Graph() = default;

	void MakeGraphFromAdjMatrix(std::istream& input);
	void MakeGraphFromListEdges(std::istream& input);

	void MakeGraphFromAdjMatrix(size_t width, size_t height, const std::vector<std::vector<bool>>& staticEnvironments);

	void TransformAdjMatrixToListEdges();
	void TransformListEdgesToAdjMatrix();

	std::vector<size_t> GetShortestPath(size_t start, size_t end) const;

	void PrintAdjMatrix(std::ostream& output) const;
	void PrintEdges(std::ostream& output) const;

private:
	[[nodiscard]] AdjList TransformAdjMatrixToAdjList() const;

	static std::vector<int> GetVectorFromStr(const std::string& row);
	static Edge GetEdgeFromStr(const std::string& row, size_t& maxVertex);


private:
	ListEdges m_edges;
	AdjMatrix m_adjMatrix;
	AdjList m_adjList;

	size_t m_countVertex = 0;
};