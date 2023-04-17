#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <utility>
#include <algorithm>
#include <climits>
#include <tuple>

class Node {
public:
    std::string name;
    int x, y;

    Node(const std::string& name, int x, int y) : name(name), x(x), y(y) {}

    const std::string& GetName() const {
        return name;
    }

    bool operator==(const Node& other) const {
        return name == other.name && x == other.x && y == other.y;
    }
};

std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << node.name;
    return os;
}

class Edge {
public:
    Node node1, node2;
    int weight;
    std::string type;
    std::string edgeAttribute;

    Edge(const Node& node1, const Node& node2, int weight, const std::string& type, const std::string& edgeAttribute)
        : node1(node1), node2(node2), weight(weight), type(type), edgeAttribute(edgeAttribute) {}
};

class Graph {
public:
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    void AddNode(const std::string& name, int x, int y) {
        nodes.push_back(Node(name, x, y));
    }

    Node FindNode(const std::string& name) {
        for (const Node& node : nodes) {
            if (node.name == name) {
                return node;
            }
        }
        throw std::runtime_error("Node not found");
    }

    void AddEdge(const std::string& node1Name, int node1X, int node1Y, const std::string& node2Name, int node2X, int node2Y, int weight, const std::string& type, const std::string& edgeAttribute) {
        if (!NodeExists(node1Name)) {
            AddNode(node1Name, node1X, node1Y);
        }
        if (!NodeExists(node2Name)) {
            AddNode(node2Name, node2X, node2Y);
        }
        Node node1 = FindNode(node1Name);
        Node node2 = FindNode(node2Name);
        edges.push_back(Edge(node1, node2, weight, type, edgeAttribute));
    }

    int FindNodeIndex(const std::vector<Node>& nodes, const std::string& targetNodeName) {
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i].GetName() == targetNodeName) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    Graph ExcludeEdgesByType(const std::string& type) {
        Graph newGraph;
        newGraph.nodes = nodes;

        for (const Edge& edge : edges) {
            if (edge.type != type) {
                newGraph.edges.push_back(edge);
            }
        }

        return newGraph;
    }

    Graph FilterEdges(const std::string& requiredAttribute) {
        Graph newGraph;
        newGraph.nodes = nodes;

        for (const Edge& edge : edges) {
            if (edge.edgeAttribute == requiredAttribute) {
                newGraph.edges.push_back(edge);
            }
        }

        return newGraph;
    }

    bool NodeExists(const std::string& name) {
        for (const Node& node : nodes) {
            if (node.name == name) {
                return true;
            }
        }
        return false;
    }

    std::pair<std::vector<int>, std::vector<int>> AStar(const Node& start, const Node& end);
};

std::vector<std::string> ReconstructPath(const std::vector<int>& prev, const std::vector<Node>& nodes, int startIndex, int endIndex) {
    std::vector<std::string> path;
    int currentNode = endIndex;

    while (currentNode != startIndex) {
        path.push_back(nodes[currentNode].name);
        currentNode = prev[currentNode];
        if (currentNode == -1) {
            break;
        }
    }

    if (currentNode == startIndex) {
        path.push_back(nodes[startIndex].name);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::pair<std::vector<int>, std::vector<int>> Graph::AStar(const Node& start, const Node& end) {
    int start_index = FindNodeIndex(nodes, start.name);
    int end_index = FindNodeIndex(nodes, end.name);


    std::vector<int> dist(nodes.size(), INT_MAX);
    std::vector<int> prev(nodes.size(), -1);

    dist[start_index] = 0;

    using PQElement = std::pair<int, int>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;
    pq.emplace(0, start_index);

    while (!pq.empty()) {
        int current_dist, current_node;
        std::tie(current_dist, current_node) = pq.top();
        pq.pop();

        if (current_node == end_index) {
            break;
        }

        if (current_dist > dist[current_node]) {
            continue;
        }

        for (const Edge& edge : edges) {
            if (edge.node1.name == nodes[current_node].name) {
                int next_node = FindNodeIndex(nodes, edge.node2.name);
                int candidate_dist = dist[current_node] + edge.weight;

                if (candidate_dist < dist[next_node]) {
                    dist[next_node] = candidate_dist;
                    prev[next_node] = current_node;
                    pq.emplace(candidate_dist, next_node);
                }
            }
        }
    }

    return std::make_pair(dist, prev);
}

int main() {
    Graph graph;

    // AddEdges (자동으로 노드 추가)
    graph.AddEdge("다향관", 0, 0, "명진관", 0, 0, 100, "평지", "차도");
    graph.AddEdge("명진관", 0, 0, "과학관", 0, 0, 30, "평지", "차도");
    graph.AddEdge("과학관", 0, 0, "대운동장앞", 0, 0, 20, "평지", "차도");
    graph.AddEdge("명진관", 0, 0, "법학관", 0, 0, 70, "평지", "차도");
    graph.AddEdge("다향관", 0, 0, "법학관", 0, 0, 70, "평지", "차도");
    graph.AddEdge("법학관", 0, 0, "혜화관", 0, 0, 50, "평지", "차도");
    graph.AddEdge("법학관", 0, 0, "대운동장앞", 0, 0, 170, "평지", "차도");
    graph.AddEdge("대운동장앞", 0, 0, "경영관", 0, 0, 200, "평지", "차도");
    graph.AddEdge("대운동장앞", 0, 0, "사회과학관", 0, 0, 220, "평지", "차도");
    graph.AddEdge("대운동장앞", 0, 0, "혜화관", 0, 0, 80, "평지", "차도");
    graph.AddEdge("경영관", 0, 0, "사회과학관", 0, 0, 10, "평지", "도보");
    graph.AddEdge("사회과학관", 0, 0, "혜화관", 0, 0, 30, "평지", "차도");
    graph.AddEdge("혜화관", 0, 0, "문화관", 0, 0, 45, "평지", "도보");
    graph.AddEdge("사회과학관", 0, 0, "문화관", 0, 0, 20, "평지", "도보");
    graph.AddEdge("문화관", 0, 0, "학술관", 0, 0, 20, "평지", "도보");

    std::string startNodeName = "과학관";
    std::string endNodeName = "사회과학관";

    Node startNode = graph.FindNode(startNodeName);
    Node endNode = graph.FindNode(endNodeName);

    int startIndex = graph.FindNodeIndex(graph.nodes, startNodeName);
    int endIndex = graph.FindNodeIndex(graph.nodes, endNodeName);

    // Regular search
    auto regularResult = graph.AStar(startNode, endNode);
    std::vector<int> regularDist = regularResult.first;
    std::vector<int> regularPrev = regularResult.second;
    std::vector<std::string> regularPath = ReconstructPath(regularPrev, graph.nodes, startIndex, endIndex);

    std::cout << "Regular path from " << startNode << " to " << endNode << ":\n";
    for (const std::string& node : regularPath) {
        std::cout << node << " -> ";
    }
    std::cout << "END\n\n";
    std::cout << "Total distance: " << regularDist[endIndex] << std::endl;

    // No stairs search
    Graph noStairsGraph = graph.ExcludeEdgesByType("계단");
    auto noStairsResult = noStairsGraph.AStar(startNode, endNode);
    std::vector<int> noStairsDist = noStairsResult.first;
    std::vector<int> noStairsPrev = noStairsResult.second;
    std::vector<std::string> noStairsPath = ReconstructPath(noStairsPrev, noStairsGraph.nodes, startIndex, endIndex);

    std::cout << "\nPath without stairs from " << startNode << " to " << endNode << ":\n";
    for (const std::string& node : noStairsPath) {
        std::cout << node << " -> ";
    }
    std::cout << "END\n\n";
    std::cout << "Total distance: " << noStairsDist[endIndex] << std::endl;

    // No elevators search
    Graph noElevatorsGraph = graph.ExcludeEdgesByType("엘리베이터");
    auto noElevatorsResult = noElevatorsGraph.AStar(startNode, endNode);
    std::vector<int> noElevatorsDist = noElevatorsResult.first;
    std::vector<int> noElevatorsPrev = noElevatorsResult.second;
    std::vector<std::string> noElevatorsPath = ReconstructPath(noElevatorsPrev, noElevatorsGraph.nodes, startIndex, endIndex);

    std::cout << "\nPath without elevators from " << startNode << " to " << endNode << ":\n";
    for (const std::string& node : noElevatorsPath) {
        std::cout << node << " -> ";
    }
    std::cout << "END\n\n";
    std::cout << "Total distance: " << noElevatorsDist[endIndex] << std::endl;


    // Road Only search
    Graph roadOnlyGraph = graph.FilterEdges("차도");
    auto roadOnlyResult = roadOnlyGraph.AStar(startNode, endNode);
    std::vector<int> roadOnlyDist = roadOnlyResult.first;
    std::vector<int> roadOnlyPrev = roadOnlyResult.second;
    std::vector<std::string> roadOnlyPath = ReconstructPath(roadOnlyPrev, roadOnlyGraph.nodes, startIndex, endIndex);

    std::cout << "\nPath with road only from " << startNode << " to " << endNode << ":\n";
    for (const std::string& node : roadOnlyPath) {
        std::cout << node << " -> ";
    }
    std::cout << "END\n\n";
    std::cout << "Total distance: " << roadOnlyDist[endIndex] << std::endl;

    return 0;
}

