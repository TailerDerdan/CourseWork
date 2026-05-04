#include "Graph.h"

#include <regex>
#include <string>
#include <fstream>
#include <iostream>
#include <list>
#include <limits>

#include "utils/Constants.h"

Graph::Graph() = default;

void Graph::MakeGraphFromAdjMatrix(std::istream& input)
{
    std::string row;

    while (std::getline(input, row))
    {
        m_adjMatrix.push_back(GetVectorFromStr(row));
    }
    if (m_adjMatrix.size() != m_adjMatrix[0].size())
    {
        throw std::logic_error("Wrong number of vertices");
    }
    m_countVertex = m_adjMatrix.size();
}

std::vector<int> Graph::GetVectorFromStr(const std::string& row)
{
    const std::regex exp(R"(\s+)");
    std::vector<std::string> tokens(
        std::sregex_token_iterator(row.begin(), row.end(), exp, -1),
        std::sregex_token_iterator()
    );

    std::erase_if(tokens,
                  [](const std::string& s) { return s.empty(); });

    std::vector<int> rowNums;
    for (const auto& token : tokens)
    {
        int num = std::stoi(token);
        rowNums.push_back(num);
    }
    return rowNums;
}

void Graph::MakeGraphFromListEdges(std::istream& input)
{
    std::string row;
    size_t maxVertex = 0;

    while (std::getline(input, row))
    {
        m_edges.push_back(GetEdgeFromStr(row, maxVertex));
    }
    m_countVertex = maxVertex;
}

Edge Graph::GetEdgeFromStr(const std::string& row, size_t& maxVertex)
{
    const std::regex exp(R"(\s+)");
    std::vector<std::string> tokens(
        std::sregex_token_iterator(row.begin(), row.end(), exp, -1),
        std::sregex_token_iterator()
    );

    std::erase_if(tokens,
                  [](const std::string& s) { return s.empty(); });

    if (tokens.size() != 2 && tokens.size() != 3)
    {
        throw std::logic_error("Wrong number of edge");
    }

    Edge edge{};
    edge.uVertex = std::stoi(tokens[0]) - 1;
    edge.vVertex = std::stoi(tokens[1]) - 1;
    edge.weight = 0;

    if (tokens.size() == 3)
    {
        edge.weight = std::stoi(tokens[2]);
    }

    maxVertex = std::max(edge.uVertex, edge.vVertex);

    return edge;
}

void Graph::MakeGraphFromAdjMatrix(size_t width, size_t height, const std::vector<std::vector<bool>>& staticEnvironments)
{
    size_t numVertices = width * height;
    m_countVertex = numVertices;
    m_adjList.assign(numVertices, std::list<iPair>());

    int dx[] = { 0,  0, -1,  1, -1,  1, -1,  1 };
    int dy[] = { -1, 1,  0,  0, -1, -1,  1,  1 };

    for (size_t y = 0; y < height; y++)
    {
        for (size_t x = 0; x < width; x++)
        {
            if (staticEnvironments[y][x]) continue;

            size_t uVertex = y * width + x;

            for (int dir = 0; dir < 8; dir++)
            {
                int nx = x + dx[dir];
                int ny = y + dy[dir];

                if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                {
                    if (!staticEnvironments[ny][nx])
                    {
                        size_t vVertex = ny * width + nx;
                        size_t weight = (dx[dir] != 0 && dy[dir] != 0) ? MIN_WEIGHT : (MIN_WEIGHT * 14 / 10);
                        m_adjList[uVertex].push_back(std::make_pair(vVertex, weight));
                    }
                }
            }
        }
    }
}

void Graph::TransformAdjMatrixToListEdges()
{
    m_edges.clear();
    for (size_t row = 0; row < m_adjMatrix.size(); row++)
    {
        for (size_t col = row + 1; col < m_adjMatrix.size(); col++)
        {
            if (m_adjMatrix[row][col])
            {
                Edge edge{};
                edge.uVertex = row;
                edge.vVertex = col;
                edge.weight = m_adjMatrix[row][col] == 1 ? 0 : m_adjMatrix[row][col];
                m_edges.push_back(edge);
            }
        }
    }
}

void Graph::TransformListEdgesToAdjMatrix()
{
    m_adjMatrix.clear();

    size_t maxVertex = 0;
    for (const auto& edge : m_edges)
        maxVertex = std::max({maxVertex, edge.uVertex, edge.vVertex});

    m_adjMatrix.assign(maxVertex + 1, std::vector<int>(maxVertex + 1, 0));

    for (const auto&[weight, uVertex, vVertex] : m_edges)
    {
        m_adjMatrix[uVertex][vVertex] = weight ? weight : MIN_WEIGHT;
        m_adjMatrix[vVertex][uVertex] = weight ? weight : MIN_WEIGHT;
    }
}

AdjList Graph::TransformAdjMatrixToAdjList() const
{
    AdjList adjList(m_adjMatrix.size());
    for (size_t row = 0; row < m_adjMatrix.size(); row++)
    {
        for (size_t col = row + 1; col < m_adjMatrix.size(); col++)
        {
            int weight = m_adjMatrix[row][col];
            if (weight != 0)
            {
                adjList[row].push_back(std::make_pair(col, weight));
                adjList[col].push_back(std::make_pair(row, weight));
            }
        }
    }
    return adjList;
}

std::vector<size_t> Graph::GetShortestPath(size_t start, size_t end) const
{
    size_t INF = std::numeric_limits<size_t>::max();
    std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
    std::vector<size_t> dist(m_countVertex, INF);
    std::vector<size_t> parent(m_countVertex, INF);

    pq.push(std::make_pair(0, start));
    dist[start] = 0;

    while (!pq.empty())
    {
        size_t uVertex = pq.top().second;
        size_t currentDist = pq.top().first;
        pq.pop();

        if (currentDist > dist[uVertex]) continue;

        for (auto iter = m_adjList[uVertex].begin(); iter != m_adjList[uVertex].end(); iter++)
        {
            size_t vVertex = iter->first;
            auto weight = iter->second;

            if (dist[vVertex] > dist[uVertex] + weight)
            {
                dist[vVertex] = dist[uVertex] + weight;
                parent[vVertex] = uVertex;
                pq.push(std::make_pair(dist[vVertex], vVertex));
            }
        }
    }

    std::vector<size_t> path;
    if (dist[end] == INF) return path;
    for (size_t v = end; v != INF; v = parent[v])
    {
        path.push_back(v);
    }

    std::ranges::reverse(path);
    return path;
}

void Graph::PrintAdjMatrix(std::ostream& output) const
{
    for (size_t row = 0; row < m_adjMatrix.size(); row++)
    {
        for (size_t col = 0; col < m_adjMatrix.size(); col++)
        {
            output << m_adjMatrix[row][col] << " ";
        }
        output << std::endl;
    }
}
void Graph::PrintEdges(std::ostream& output) const
{
    for (const auto&[weight, uVertex, vVertex] : m_edges)
    {
        output << uVertex + 1 << " " << vVertex + 1 << " " << weight << std::endl;
    }
}