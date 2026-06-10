#include "Graph.h"

#include <regex>
#include <string>
#include <fstream>
#include <iostream>
#include <list>
#include <limits>

#include "utils/constants.h"

Graph::Graph() = default;

void Graph::MakeGraph(size_t width, size_t height, std::vector<std::vector<bool>> environments)
{
    size_t index = 0;
    m_adjList.clear();
    m_adjList.resize(width * height);
    m_countVertex = width * height;
    size_t weight = 1;
    double COEF_WEIGHT_FOR_DIAG = 1.414;
    for (size_t col = 0; col < width; col++)
    {
        for (size_t row = 0; row < height; row++, index++)
        {
            if (environments[col][row]) continue;
            if (col > 0 && row > 0 && !environments[col - 1][row - 1])
            {
                m_adjList[index].emplace_back(index - height - 1, weight * COEF_WEIGHT_FOR_DIAG);
            }
            if (col > 0 && !environments[col - 1][row])
            {
                m_adjList[index].emplace_back(index - height, weight);
            }
            if (col > 0 && row < height - 1 && !environments[col - 1][row + 1])
            {
                m_adjList[index].emplace_back(index - height + 1, weight * COEF_WEIGHT_FOR_DIAG);
            }

            if (row > 0 && !environments[col][row - 1])
            {
                m_adjList[index].emplace_back(index - 1, weight);
            }
            if (row < height - 1 && !environments[col][row + 1])
            {
                m_adjList[index].emplace_back(index + 1, weight);
            }

            if (col < width - 1 && row > 0 && !environments[col + 1][row - 1])
            {
                m_adjList[index].emplace_back(index + height - 1, weight * COEF_WEIGHT_FOR_DIAG);
            }
            if (col < width - 1 && !environments[col + 1][row])
            {
                m_adjList[index].emplace_back(index + height, weight);
            }
            if (col < width - 1 && row < height - 1 && !environments[col + 1][row + 1])
            {
                m_adjList[index].emplace_back(index + height + 1, weight * COEF_WEIGHT_FOR_DIAG);
            }
        }
    }
}

std::vector<size_t> Graph::GetShortestPath(size_t start, size_t end) const
{
    double INF_D = std::numeric_limits<double>::max();
    size_t INF_S = std::numeric_limits<size_t>::max();
    std::priority_queue<iPair, std::vector<iPair>, std::greater<>> pq;
    std::vector<double> dist(m_countVertex, INF_D);
    std::vector<size_t> parent(m_countVertex, INF_S);

    pq.emplace(0, start);
    dist[start] = 0;

    while (!pq.empty())
    {
        size_t uVertex = pq.top().second;
        auto currentDist = pq.top().first;
        pq.pop();

        if (currentDist > dist[uVertex]) continue;

        for (auto iter = m_adjList[uVertex].begin(); iter != m_adjList[uVertex].end(); ++iter)
        {
            size_t vVertex = iter->first;
            auto weight = iter->second;

            if (dist[vVertex] > dist[uVertex] + weight)
            {
                dist[vVertex] = dist[uVertex] + weight;
                parent[vVertex] = uVertex;
                pq.emplace(dist[vVertex], vVertex);
            }
        }
    }

    std::vector<size_t> path;
    if (dist[end] == INF_D) return path;
    for (size_t v = end; v != INF_S; v = parent[v])
    {
        path.push_back(v);
    }

    std::ranges::reverse(path);
    return path;
}