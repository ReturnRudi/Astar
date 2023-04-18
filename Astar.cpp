#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <utility>
#include <algorithm>
#include <climits>
#include <tuple>

//노드의 이름과 노드의 위치가 될 픽셀값을 멤버 변수로 가진 노드 클래스
class Node {
public:
    std::string name;
    int x, y;

    Node(const std::string& name, int x, int y) : name(name), x(x), y(y) {}

    const std::string& GetName() const {
        return name;
    }
    //노드간 비교를 위한 == 오버로딩
    bool operator==(const Node& other) const {                  
        return name == other.name && x == other.x && y == other.y;
    }
};

//노드 이름 출력을 위한 << 오버로딩
std::ostream& operator<<(std::ostream& os, const Node& node) { 
    os << node.name;
    return os;
}

//양쪽 노드 객체, 가중치, 특성, 차도여부를 멤버변수로 갖는 엣지 클래스
class Edge {
public:
    Node node1, node2;
    int weight;
    std::string type;
    std::string edgeAttribute;

    Edge(const Node& node1, const Node& node2, int weight, const std::string& type, const std::string& edgeAttribute)
        : node1(node1), node2(node2), weight(weight), type(type), edgeAttribute(edgeAttribute) {}
};

//그래프 내에 존재할 노드들과 엣지들을 벡터로 가지고 있는 Graph 클래스
class Graph {
public:
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    //AddNode를 통해 노드를 그래프에 추가
    void AddNode(const std::string& name, int x, int y) {       
        nodes.push_back(Node(name, x, y));
    }

    //그래프 내에 특정 이름의 노드가 있는지 확인하는 FindNode 메소드
    Node FindNode(const std::string& name) {                   
        for (const Node& node : nodes) {
            if (node.name == name) {
                return node;
            }
        }
        throw std::runtime_error("Node not found");
    }

    //AddEdge를 통해 엣지를 그래프에 추가. 매개변수인 양쪽 노드가 그래프에 들어가있지 않는 경우 자동으로 AddNode를 해준다.
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

    //그래프 내에 노드를 저장해둔 nodes 벡터에서 특정 이름을 가진 노드를 찾아 그 인덱스를 반환하는 FindNodeIndex 메소드
    int FindNodeIndex(const std::vector<Node>& nodes, const std::string& targetNodeName) { 
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i].GetName() == targetNodeName) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    //그래프 내에 특정 특성을 가진 엣지들을 제외한 그래프를 생성하여 반환하는 ExcludeEdgesByType 메소드
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

    //그래프 내에 차도가 아닌 엣지들을 제외한 그래프를 생성하여 반환하는 FilterEdges 메소드
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

    //그래프 내에 특정 이름을 가진 노드가 있으면 true, 없으면 false를 반환해주는 NodeExists 메소드
    bool NodeExists(const std::string& name) {      
        for (const Node& node : nodes) {
            if (node.name == name) {
                return true;
            }
        }
        return false;
    }

    //Astar 알고리즘
    std::pair<std::vector<int>, std::vector<int>> AStar(const Node& start, const Node& end);   
};

//AStar의 결과인 prev 벡터를 가져와서 도착 노드부터 지나온 노드들을 거꾸로 돌아오면서 시작 노드까지 노드의 이름을 string 벡터 path에 저장(reverse를 사용해서 순서대로 저장)
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

//출발 노드부터 도착 노드까지 Astar 알고리즘을 통해 거리를 계산
//그 결과 dist에는 시작 노드에서 각 노드까지의 최단 거리가 저장되어 있고, prev에는 출발지에서 해당 노드까지 갈 때 거치는 바로 이전 노드의 인덱스가 저장되어있음.
//만약 출발지에서 도착지까지 길을 찾는데 탐색 과정에서 특정 노드를 아예 거치지 않고 도착지에 도착했다면 dist와 prev는 초기값 그대로 (INT_MAX, -1)를 가짐.
std::pair<std::vector<int>, std::vector<int>> Graph::AStar(const Node& start, const Node& end) {
    int start_index = FindNodeIndex(nodes, start.name); //그래프 객체 내의 nodes에서 출발 노드에 해당하는 노드의 인덱스를 start_index에 저장
    int end_index = FindNodeIndex(nodes, end.name); //그래프 객체 내의 nodes에서 도착 노드에 해당하는 노드의 인덱스를 end_index에 저장


    std::vector<int> dist(nodes.size(), INT_MAX);   //각 노드까지의 거리를 저장할 dist 벡터. nodes의 크기만큼 INT_MAX를 저장해서 모든 노드까지의 초기 거리를 무한으로 설정
    std::vector<int> prev(nodes.size(), -1);        //이전 노드의 인덱스를 저장할 prev 벡터. 초기에는 이전 노드를 알 수 없으므로 nodes의 크기만큼 -1을 설정

    dist[start_index] = 0;                          //출발 노드를 지정했으므로 출발 노드에서 출발 노드까지의 거리는 0으로 설정

    using PQElement = std::pair<int, int>;          //두 개의 int를 쌍으로 가지는 pair를 PQElement라고 사용. 첫번째는 현재까지의 거리를 저장, 두번째는 노드의 인덱스를 저장
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq; //Astar를 위한 우선순위 큐 pq 선언.
    //PQElement를 요소로 가지고 PQElement를 담은 벡터를 내부 컨테이너로 사용하며 std::greater<PQElement>를 이용해 거리(dist)가 작은 원소가 우선순위가 높도록 정렬
    pq.emplace(0, start_index);                     //시작 노드부터 우선순위 큐 pq에 넣는다. 시작 노드이므로 현재까지의 거리는 0이고 시작 노드의 인덱스를 저장

    while (!pq.empty()) {                           //우선순위 큐가 모두 비워질 때까지 반복
        int current_dist, current_node;
        std::tie(current_dist, current_node) = pq.top();    //우선순위 큐에 top에는 가장 거리가 작은 노드에 해당하는 값이 들어가있다. 따라서 current_dist에 가장 작은 거리, current_node에 그 노드(의 인덱스)를 저장한다.
        pq.pop();                                           //바로 윗줄에서 가장 거리가 작은 노드에 방문했으므로 해당 정보를 pop하여 없앤다.

        if (current_node == end_index) {                    //가장 최근에 방문한 노드가 도착지점이라면 알고리즘을 종료한다.
            break;
        }

        if (current_dist > dist[current_node]) {            //가장 최근에 방문한 노드에 올 때까지의 거리가 시작지점에서 그 노드까지 바로 가는 방법보다 크다면 continue하여 다음 우선순위 큐를 pop해야한다.
            continue;
        }

        for (const Edge& edge : edges) {
            if (edge.node1.name == nodes[current_node].name || edge.node2.name == nodes[current_node].name) {  //edges를 돌며 현재 노드에 연결된 엣지를 찾아 다음 노드를 선정한다.
                int next_node;
                if(edge.node1.name == nodes[current_node].name)
                    next_node = FindNodeIndex(nodes, edge.node2.name);
                if(edge.node2.name == nodes[current_node].name)
                    next_node = FindNodeIndex(nodes, edge.node1.name);
                int candidate_dist = dist[current_node] + edge.weight;                                         //연결된 엣지 반대편 노드까지의 거리에 현재 노드까지의 총 거리를 더해 총 거리를 업데이트한다.

                if (candidate_dist < dist[next_node]) {                                                        //그 총 거리가 dist에 저장된 현재 노드까지의 거리보다 작을 때에만 dist와 prev를 저장하고 우선순위 큐에 집어넣는다.
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

