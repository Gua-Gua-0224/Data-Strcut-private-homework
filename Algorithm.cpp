//
// Created by ����Դ on 24-10-13.
//
#include "Algorithm.h"
#include <unordered_set>

namespace Graph {
    namespace Algorithm {
        std::list<Vertex> GetCircuit(LGraph& graph, Vertex start) {
            // ʹ��˫���������·
            std::list<Vertex> circuit;
            Vertex head = start;  // ��ǰ��·�����
            circuit.push_back(start);  // ���������·
            auto& adjacency_list = graph.List();  // ��ȡ�ڽӱ�
            while (!adjacency_list[head].adj.empty()) {
                Vertex tail = adjacency_list[head].adj.front().dest;  // ȡ�ڽӱ��е�һ���ߵ��յ�
                // ɾ������ͼ�е�˫���
                graph.DeleteEdge(head, tail);
                graph.DeleteEdge(tail, head);
                // ���ö�������·
                circuit.push_back(tail);
                // ���µ�ǰͷ�ڵ�
                head = tail;
            }
            // ������ı����ñպϻص���㣬����һ��������·�����򷵻ز��ֻ�·
            if (circuit.front() != circuit.back()) {
                std::cerr << "Error: The graph does not form a valid circuit starting from the given vertex." << std::endl;
            }
            return circuit;
            //TODO:�Ӹ�����������һ����·
            std::cerr << "GetCircuit ��ûʵ��" << std::endl;
        }

        std::list<Vertex> EulerCircle(const LGraph& graph) {
            // ���ͼ�Ƿ�Ϊŷ��ͼ�����ж���Ķ���Ϊż����
            LGraph tmp_graph = graph;
            if (!HaveEulerCircle(tmp_graph)) {
                std::cerr << "The graph is not an Eulerian graph. It contains a vertex with odd degree.\n";
                return{};
            }
            // ��ʼ����һ����·���Ӷ��� 0 ��ʼ������ͼ�Ķ����Ŵ� 0 ��ʼ��
            std::list<Vertex> circuit = GetCircuit(tmp_graph, 0);
            // Ѱ���µ���ʼ���㣬�ϲ���·
            for (auto it = circuit.begin(); it != circuit.end(); ++it) {
                Vertex v = *it;
                if (!tmp_graph.List()[v].adj.empty()) {
                    // �����ǰ������δ���ʵıߣ���ȡ�Ӹö��������һ���»�·
                    std::list<Vertex> next_circuit = GetCircuit(tmp_graph, v);
                    // �����µĻ�·����ǰ��·��
                    it = circuit.insert(it, next_circuit.begin(), next_circuit.end());
                    --it; // �������������ص��²����·�����
                }
            }
            return circuit;
            //TODO:��ȡŷ����·,�����ʹ��GetCircuit����
            std::cerr << "EulerCircle ��ûʵ��" << std::endl;
            return {};
        }

        bool HaveEulerCircle(const LGraph& graph) {
            // ��ȡͼ�Ķ�������
            size_t vertex_count = graph.List().size();
            LGraph tmp_graph = graph;
            //�����ͨ��
            if (!IsConnected(tmp_graph)) {
                return false;
            }
            // ������ж���Ķ��Ƿ�Ϊż��
            for (size_t i = 0; i < vertex_count; ++i) {
                if (graph.List()[i].adj.size() % 2 != 0) { // �����������
                    return false;
                }
            }
            // ͼ����ŷ����·������
            return true;
            //TODO:�ж��Ƿ���ŷ����·
            std::cerr << "HaveEulerCircle ��ûʵ��" << std::endl;
            return false;
        }

        void BFSv(LGraph& graph, Vertex v, std::vector<bool>& visited) {
            //TODO:���������������ͼ
            std::queue<Vertex> q;  // ��������
            q.push(v);             // ��������
            visited[v] = true;     // ������Ϊ�ѷ���

            const auto& adjacency_list = graph.List(); // ��ȡ�ڽӱ�

            while (!q.empty()) {
                Vertex current = q.front();
                q.pop();

                // ������ǰ�ڵ�������ڽӽڵ�
                for (const auto& edge : adjacency_list[current].adj) {
                    Vertex next = edge.dest;
                    if (!visited[next]) {
                        visited[next] = true; // ���Ϊ�ѷ���
                        q.push(next);         // �������
                    }
                }
            }
            return;
            std::cerr << "BFSv ��ûʵ��" << std::endl;
        }

        bool IsConnected(LGraph& graph) {
            int n = graph.NumVertices(); // ��ȡ������
            if (n == 0) return true;     // ��ͼ����Ϊ��ͨ

            std::vector<bool> visited(n, false); // ���ʱ������
            BFSv(graph, 0, visited);            // �ӵ�һ�����㿪ʼ���й����������

            // ����Ƿ����ж��㶼�����ʹ�
            for (int i = 0; i < n; ++i) {
                if (!visited[i]) {
                    return false; // ����ж���δ�����ʣ�˵��ͼ����ͨ
                }
            }

            return true; // ���ж��㶼�����ʣ�ͼ����ͨ��
        }


        int GetShortestPath(const LGraph& graph, const std::string& vertex_name_x, const std::string& vertex_name_y) {//��Floyd�㷨ʵ��
            // ��������ڵ��Ƿ����
            if (!graph.exist_vertex(vertex_name_x)|| !graph.exist_vertex(vertex_name_y)) {
                std::cerr << "�����ڵ㲻�Ϸ�"
                    << vertex_name_x << ", " << vertex_name_y << std::endl;
                return -1;
            }
            // ��ȡ��������
            size_t n = graph.NumVertices();
            // ӳ�䶥����������ID
            const auto& vertex_map = graph.Map();
            auto it_x = vertex_map.find(vertex_name_x);
            auto it_y = vertex_map.find(vertex_name_y);
            // ��ȡ��Ӧ�Ľڵ� ID
            Vertex id_x = it_x->second;
            Vertex id_y = it_y->second;
            // ��ʼ���������
            std::vector<std::vector<int>> dist(n, std::vector<int>(n, INT_MAX));
            // ��ͼ�����еı���Ϣ����������
            const auto& adjacency_list = graph.List();
            for (Vertex i = 0; i < n; ++i) {
                dist[i][i] = 0; // ��������ľ���Ϊ0
                for (const auto& edge : adjacency_list[i].adj) {
                    dist[i][edge.dest] = edge.weight; // �ڽӱߵ�Ȩֵ
                }
            }
            // Floyd�㷨
            for (Vertex k = 0; k < n; ++k) {        // ���������м䶥��
                for (Vertex i = 0; i < n; ++i) {    // �������
                    for (Vertex j = 0; j < n; ++j) {// �����յ�
                        if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
                            dist[i][j] = std::min(dist[i][j], dist[i][k] + dist[k][j]);
                        }
                    }
                }
            }
            // ���ؽ��
            int shortest_path = dist[id_x][id_y];
            if (shortest_path == INT_MAX) {
                throw std::runtime_error("��ĳ������֮�䲻����·��");
            }
            return shortest_path;
            //TODO:��ȡ����֮������·��
            std::cerr << "GetShortestPath ��ûʵ��" << std::endl;
            return -1;
        }
        int TopologicalShortestPath(const LGraph& graph, std::vector<std::string> path) {
            // ��ȡ����ӳ��
            const auto& vertex_map = graph.Map();
            // ����������кϷ���
            if (path.empty()) {
                throw std::invalid_argument("����·��������");
            }
            for (const auto& vertex_name : path) {
                if (vertex_map.find(vertex_name) == vertex_map.end()) {
                    throw std::invalid_argument("Vertex in topological order does not exist in the graph.");
                }
            }
            // ʹ�� GetShortestPath ����������㵽ÿ����������·��
            std::string start_vertex = path[0];
            int shortest_path = INT_MAX;
            for (const auto& target_vertex : path) {
                // ֻ������㵽��ǰ�����·��
                int path_length = GetShortestPath(graph, start_vertex, target_vertex);
                if (path_length != INT_MAX) { // ���·�����ڣ��������·��
                    shortest_path = std::min(shortest_path, path_length);
                }
            }
            if (shortest_path == INT_MAX) {
                throw std::runtime_error("��������С·��");
            }
            return shortest_path;
            //TODO:��ȡ�������޵����·������������path����
            std::cerr << "TopologicalShortestPath ��ûʵ��" << std::endl;
            return -1;
        }

        std::vector<EdgeNode> MinimumSpanningTree(const LGraph& graph) {//��Kruskal�㷨ʵ��
            std::vector<EdgeNode> mst_edges; // �洢��С�������ı�
            size_t vertex_count = graph.List().size();
            size_t edge_count = graph.EdgeVertices();

            // ��ȡͼ�����б�
            std::vector<EdgeNode> all_edges = graph.SortedEdges();
            // ��ʼ�����鼯
            DSU dsu(vertex_count);
            // ���������ıߣ���Ȩֵ��С������� MST
            for (const auto& edge : all_edges) {
                // ����������㲻��ͬһ�������У����������ߵ� MST
                if (!dsu.same(edge.from, edge.dest)) {
                    mst_edges.push_back(edge);
                    dsu.unite(edge.from, edge.dest);
                }
                // ��� MST �Ѿ������� vertex_count - 1 ���ߣ�������ǰ�˳�
                if (mst_edges.size() == vertex_count - 1) {
                    break;
                }
            }
            return mst_edges;
            //TODO:������С�����������������ϵı�
            std::cerr << "MinimumSpanningTree ��ûʵ��" << std::endl;
            return {};
        }
        std::vector<EdgeNode> Visitplan(const LGraph& graph, const std::string& from, const std::string& to, const int totaltime) {
            // �������������յ��Ƿ���Ч
            const auto& vertex_map = graph.Map();
            if (vertex_map.find(from) == vertex_map.end() || vertex_map.find(to) == vertex_map.end()) {
                throw std::invalid_argument("�����յ���Ч���������롣");
            }

            // ��ʼ��
            int current_time = 0;
            int total_distance = 0;
            std::vector<EdgeNode> plan; // �������յ�����·��
            std::unordered_set<std::string> visited; // ����ѷ��ʵľ���
            std::string current_location = from;
            visited.insert(current_location); // �������ѷ���

            // ʹ��̰�Ĳ�����ѡ����һ������
            while (current_location != to) {
                const auto& adjacency_list = graph.List();
                const auto& neighbors = adjacency_list[vertex_map.at(current_location)].adj;

                // ɸѡδ���ʹ����ڽӵ㣬�����Լ۱�����
                std::vector<EdgeNode> candidates;
                for (const auto& edge : neighbors) {
                    const auto& dest_name = graph.GetVertex(edge.dest).name;
                    if (visited.find(dest_name) == visited.end()) {
                        candidates.push_back(edge);
                    }
                }

                // ���û�к�ѡ�㣬ֱ�ӽ���
                if (candidates.empty()) {
                    std::cout << "�޷��ҵ���һ����·����ֹͣ�滮��" << std::endl;
                    break;
                }

                // ���Լ۱������Լ۱� = ����ι�ʱ�� / ·��
                std::sort(candidates.begin(), candidates.end(), [&](const EdgeNode& a, const EdgeNode& b) {
                    int time_a = graph.GetVertex(a.dest).visitTime;
                    int time_b = graph.GetVertex(b.dest).visitTime;
                    return static_cast<double>(time_a) / a.weight > static_cast<double>(time_b) / b.weight;
                    });

                // ѡ���Լ۱���ߵĺ�ѡ��
                const EdgeNode& next_edge = candidates.front();
                const auto& next_name = graph.GetVertex(next_edge.dest).name;

                // ���㵽����һ�����������ʱ��
                int travel_time = next_edge.weight / 60; // ·��ʱ�� (��λ������)
                int visit_time = graph.GetVertex(next_edge.dest).visitTime;

                // �������һ�������ᳬʱ����ֹͣ�滮
                if (current_time + travel_time + visit_time > totaltime) {
                    break;
                }

                // ���¹滮��Ϣ
                current_time += travel_time + visit_time;
                total_distance += next_edge.weight;
                plan.push_back(next_edge);
                visited.insert(next_name);
                current_location = next_name;

                // ����ѵ����յ㣬��ֹͣ
                if (current_location == to) {
                    break;
                }
            }

            // ���������
            std::cout << "���ι滮��ɣ���ʱ�䣺" << current_time << " ���ӣ��ܾ��룺" << total_distance << " �ס�" << std::endl;
            if (current_location != to) {
                std::cout << "ע�⣺δ�ܵ����յ� \"" << to << "\"�����ʵ������滮ʱ�䡣" << std::endl;
            }
            else {
                std::cout << "�滮·�߳ɹ������յ� \"" << to << "\"��" << std::endl;
            }

            return plan;
        }



        void SuggestEdges(LGraph& graph) {//�鿴��Щ��δ��ͨ
            int n = graph.NumVertices(); // ��ȡͼ�ж�������
            if (n == 0) {
                std::cout << "ͼΪ�գ�û�е���Ҫ���ӡ�" << std::endl;
                return;
            }

            // ��ʼ�����鼯�����ڱ����ͨ��
            DSU dsu(n);

            // ����ÿ���ߣ�����ͨ�Ķ���ϲ�
            const auto& adjacency_list = graph.List();
            for (int i = 0; i < n; ++i) {
                for (const auto& edge : adjacency_list[i].adj) {
                    int from = i;
                    int to = edge.dest;
                    dsu.unite(from, to);
                }
            }

            // �ҳ���ͨ�����ĸ��ڵ�
            std::unordered_set<int> components;
            for (int i = 0; i < n; ++i) {
                components.insert(dsu.find(i));
            }

            // ���ֻ��һ����ͨ��������ͼ����ͨ
            if (components.size() == 1) {
                std::cout << "ͼ����ͨ�ģ�û����Ҫ��ӵıߡ�" << std::endl;
                return;
            }

            // ��ʾ�û���Ҫ�����Щ��
            std::vector<int> component_roots(components.begin(), components.end());
            std::cout << "ͼ����ͨ����Ҫ���±�����ͨͼ��" << std::endl;

            std::vector<std::pair<int, int>> edges_to_add; // ���ڱ�����Ҫ��ӵı�

            for (size_t i = 1; i < component_roots.size(); ++i) {
                int u = component_roots[0];   // ѡ��һ���̶�����ͨ����
                int v = component_roots[i];  // ��������ͨ������������
                auto u_info = graph.GetVertex(u);
                auto v_info = graph.GetVertex(v);
                std::cout << "������ӱߣ�\"" << u_info.name << "\" - \"" << v_info.name << "\"" << std::endl;
                edges_to_add.emplace_back(u, v);
            }

            // ��ʾ�û�ѡ���Ƿ��Զ�������Щ��
            char choice;
            std::cout << "�Ƿ��ɳ����Զ�������Щ�ߣ�(y/n): ";
            std::cin >> choice;

            if (choice == 'y') {
                for (const auto& edge : edges_to_add) {
                    int weight = 1 + rand() % 200; // ���� 1 �� 200 �����Ȩ��
                    auto u_info = graph.GetVertex(edge.first);
                    auto v_info = graph.GetVertex(edge.second);
                    graph.InsertEdge(u_info.name, v_info.name, weight); // ��ӱߵ�ͼ��
                    std::cout << "���Զ���ӱߣ�\"" << u_info.name << "\" - \"" << v_info.name << "\"��Ȩ�أ�" << weight << std::endl;
                }
                std::cout << "ͼ��ͨ���������ɵı���ͨ��" << std::endl;
            }
            else {
                std::cout << "����ݽ����ֶ������Щ�ߡ�" << std::endl;
            }
        }

    }
}