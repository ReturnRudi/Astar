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

    // AddEdges (�ڵ����� ��� �߰�)
    graph.AddEdge("�����", 0, 0, "������", 0, 0, 100, "����", "����");
    graph.AddEdge("������", 0, 0, "���а�", 0, 0, 30, "����", "����");
    graph.AddEdge("���а�", 0, 0, "�����", 0, 0, 20, "����", "����");
    graph.AddEdge("������", 0, 0, "���а�", 0, 0, 70, "����", "����");
    graph.AddEdge("�����", 0, 0, "���а�", 0, 0, 70, "����", "����");
    graph.AddEdge("���а�", 0, 0, "��ȭ��", 0, 0, 50, "����", "����");
    graph.AddEdge("���а�", 0, 0, "�����", 0, 0, 170, "����", "����");
    graph.AddEdge("�����", 0, 0, "�濵��", 0, 0, 200, "����", "����");
    graph.AddEdge("�����", 0, 0, "��ȸ���а�", 0, 0, 220, "����", "����");
    graph.AddEdge("�����", 0, 0, "��ȭ��", 0, 0, 80, "����", "����");
    graph.AddEdge("�濵��", 0, 0, "��ȸ���а�", 0, 0, 10, "����", "����");
    graph.AddEdge("��ȸ���а�", 0, 0, "��ȭ��", 0, 0, 30, "����", "����");
    graph.AddEdge("��ȭ��", 0, 0, "��ȭ��", 0, 0, 45, "����", "����");
    graph.AddEdge("��ȸ���а�", 0, 0, "��ȭ��", 0, 0, 20, "����", "����");
    graph.AddEdge("��ȭ��", 0, 0, "�м���", 0, 0, 20, "����", "����");

    std::string startNodeName = "���а�";
    std::string endNodeName = "��ȸ���а�";

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
    Graph noStairsGraph = graph.ExcludeEdgesByType("���");
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
    Graph noElevatorsGraph = graph.ExcludeEdgesByType("����������");
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
    Graph roadOnlyGraph = graph.FilterEdges("����");
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

