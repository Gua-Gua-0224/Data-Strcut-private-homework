#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <cassert>
#include "LGraph.h"
#include "Algorithm.h"
#include "LocationInfo.h"
#include "GraphException.h"

using Graph::LGraph;
using Graph::VertInfo;
using namespace Graph::Algorithm;

// Hint: �������д����Ŀ�����������ļ���������backup�ļ����л�ȡԭʼ�ļ�
// TODO: ����ݿ�ִ���ļ���csv��λ����д�ļ�·��
const std::string nodes_path{ "nodestext.csv" };
const std::string edges_path{ "edges1text.csv" };

struct edge {
    std::string from;
    std::string to;
    int length;
};

void init(LGraph& graph);

std::vector<VertInfo> read_nodes(const std::string& nodes_path);

std::vector<edge> read_edges(const std::string& edges_path);

void store_nodes(const std::string& nodes_path, const LGraph& graph);

void store_edges(const std::string& edges_path, const LGraph& graph);

void show_all_nodes(const LGraph& graph);

void show_all_edges(const LGraph& graph);

int main() {
    LGraph graph(false); //��ʼ��һ������ͼ��
    init(graph);
    while (true) {
        int choice{ 0 };
        std::cout << "��ӭʹ��У԰����ϵͳ��" << std::endl
            << "��ѡ����Ҫ���еĲ�����" << std::endl
            << "1.������ز���" << std::endl
            << "2.����ز���" << std::endl
            << "3.���ļ������¼��ص����" << std::endl
            << "4.�Ƿ����ŷ��ͨ·" << std::endl
            << "5.��������������̾���" << std::endl
            << "6.����С������" << std::endl
            << "7.�����������ʱ�����·��" << std::endl
            << "8.���һ�����·��" << std::endl
            << "9.�˳�����" << std::endl
            << "���������ǰ�����֣�";
        std::cin >> choice;
        if (choice == 1) { // ������ز���
            std::cout << "������ز�����" << std::endl
                << "1.����ض�������Ϣ" << std::endl
                << "2.������ж�����Ϣ" << std::endl
                << "3.���һ������" << std::endl
                << "4.ɾ��һ������" << std::endl
                << "5.����һ�����������ʱ��" << std::endl
                << "6.�洢���㵽�ļ���" << std::endl
                << "7.��������ض����Ͷ�����Ϣ"<< std::endl
                << "8.�ص���һ���˵�" << std::endl
                << "���������ǰ�����֣�";
            std::cin >> choice;

            // ��������ض����Ͷ�����Ϣ
            if (choice == 7) {
                std::string name;
                std::cout << "�������������ƣ�";
                std::cin >> name;
                try {
                    const auto& vertex_map = graph.Map();
                    bool found = false;
                    std::cout << "����Ϊ \"" << name << "\" �Ķ�����Ϣ���£�" << std::endl;
                    for (const auto& pair : vertex_map) {
                        const auto& vertex_info = graph.GetVertex(pair.second);
                        if (vertex_info.type == name) {
                            found = true;
                            std::cout << "��������: " << vertex_info.name 
                                << "�������ͣ�" << vertex_info.type 
                                << "��������ʱ�䣺" << vertex_info.visitTime << std::endl;
                        }
                    }
                    if (!found) {
                        throw Graph::GraphException("���Ͳ�����");
                    }
                }
                catch (Graph::GraphException& e) {
                    std::cout << "����Ϊ \"" << name << "\" �����Ͳ�����" << std::endl;
                }
            }
            if (choice == 1) {
                std::string name;
                std::cout << "�����붥�����ƣ�";
                std::cin >> name;
                try {
                    VertInfo v{ graph.GetVertex(name) };
                    std::cout << "�������ƣ�" << v.name << std::endl
                        << "�������ͣ�" << v.type << std::endl
                        << "��������ʱ�䣺" << v.visitTime << std::endl;

                }
                catch (Graph::GraphException& e) {
                    std::cout << "����Ϊ " << name << " �Ķ��㲻����" << std::endl;
                }
            }
            else if (choice == 2) {
                std::cout << "��������, ��������, ��������ʱ�䣺" << std::endl;
                show_all_nodes(graph);
            }
            else if (choice == 3) {
                std::cout << "�����붥�����ƣ��������ͣ���������ʱ��(�ո�ָ�)��";
                std::string name, type;
                int visit_time;
                std::cin >> name >> type >> visit_time;
                try {
                    graph.InsertVertex({ name, visit_time, type });
                }
                catch (Graph::GraphException& e) {
                    std::cout << e.what() << std::endl;
                }
            }
            else if (choice == 4) {
                std::cout << "�����붥�����ƣ�";
                std::string name;
                std::cin >> name;
                try {
                    graph.DeleteVertex({ name, 0, "" });
                }
                catch (Graph::GraphException& e) {
                    std::cout << e.what() << std::endl;
                }
            }
            else if (choice == 5) {
                std::cout << "�����붥�����ƺ��µ�����ʱ�䣨�ո�ָ�����";
                int time;
                std::string name;
                std::cin >> name>>time;
                graph.UpdateVertex(name, time);
            }
            else if (choice == 6) {
                std::cout << "���ڽ�����洢��test_nodes.csv��..." << std::endl;
                store_nodes("test_nodes.csv", graph);
                std::cout << "�洢�ɹ���" << std::endl;
            }
        }
        else if (choice == 2) { // ����ز���
            std::cout << "����ز�����" << std::endl
                << "1.����ض�����Ϣ" << std::endl
                << "2.������б���Ϣ" << std::endl
                << "3.���һ����" << std::endl
                << "4.ɾ��һ����" << std::endl
                << "5.�޸�һ���ߵ�Ȩֵ" << std::endl
                << "6.�洢�ߵ��ļ���" << std::endl
                << "7.����ض������б���Ϣ" << std::endl
                << "8.�ص���һ���˵�" << std::endl
                << "���������ǰ�����֣�";
            std::cin >> choice;
            if (choice == 1) {
                std::string from, to;
                std::cout << "������ߵ�������յ�(�ո�ָ�)��";
                std::cin >> from >> to;
                try {
                    Graph::GElemSet w{ graph.GetEdge(from, to) };
                    std::cout << from << " <---> " << to << " ����Ϊ��" << w << std::endl;
                }
                catch (Graph::GraphException& e) {
                    std::cout << from << " <---> " << to << " �����ڣ�" << std::endl;
                }
            }
            else if (choice == 2) {
                std::cout << "���, �յ�, ���룺" << std::endl;
                show_all_edges(graph);
            }
            else if (choice == 3) {
                std::cout << "������������ƣ��յ����ƣ�����(�ո�ָ�)��";
                std::string from, to;
                int length;
                std::cin >> from >> to >> length;
                graph.InsertEdge(from, to, length);
            }
            else if (choice == 4) {
                std::cout << "������������ƣ��յ�����(�ո�ָ�)��";
                std::string from, to;
                std::cin >> from >> to;
                graph.DeleteEdge(from, to); // ɾ��һ����
            }
            else if (choice == 5) {
                std::cout << "������������ƣ��յ�����,�Ѿ����µ�Ȩֵ(�ո�ָ�)";
                std::string from, to;
                int weight;
                std::cin >> from >> to>>weight;
                graph.UpdateEdge(from, to, weight); // ���±�
            }
            else if (choice == 6) {
                std::cout << "���ڽ��ߴ洢��test_edges.csv�ļ���..." << std::endl;
                store_edges("test_edges.csv", graph);
                std::cout << "�洢�ɹ���" << std::endl;
            }
            else if (choice == 7) {
                std::string location_name;
                std::cout << "������ص����ƣ�";
                std::cin >> location_name;

                try {
                    const auto& edges = graph.GetEdgesFromLocation(location_name);
                    if (edges.empty()) {
                        std::cout << "�ص� \"" << location_name << "\" û����صĵ�·��" << std::endl;
                    }
                    else {
                        std::cout << "�ص� \"" << location_name << "\" ��صĵ�·���£�" << std::endl;
                        for (const auto& edge : edges) {
                            auto dest_info = graph.GetVertex(edge.dest);
                            std::cout << location_name << " <---> " << dest_info.name << " ����Ϊ��" << edge.weight << std::endl;
                        }
                    }
                }
                catch (Graph::GraphException& e) {
                    std::cout << "����Ϊ \"" << location_name << "\" �ĵص㲻����" << std::endl;
                }
            }

        }
        else if (choice == 3) {// ���ļ������¼��ص����
            init(graph);
        }
        else if (choice == 4) { // �Ƿ����ŷ��ͨ·
            auto res = IsConnected(graph);
            auto circle = HaveEulerCircle(graph);
            std::cout << ( res ? "���е���ͨ" : "���е㲻��ͨ") << std::endl;
            std::cout << (circle ? "����ŷ����·" : "������ŷ����·") << std::endl;

        }
        else if (choice == 5) { // ��������������̾���
            std::cout << "�����������ص㣬ʹ�ÿո�ֿ���" << std::endl;
            std::string x, y;
            std::cin >> x >> y;
            try {
                std::cout << x << "��" << y << "֮�����̾���Ϊ��" << GetShortestPath(graph, x, y) << std::endl;

            }
            catch (Graph::GraphException& e) {
                std::cout << "���ҵ�����յľ���" << std::endl;
            }
        }
        else if (choice == 6) { // ����С������
            if (IsConnected(graph)) {
                std::cout << "��С�������ĵ����£�";
                auto res = MinimumSpanningTree(graph);
                int sum = 0;
                for (auto& item : res) {
                    const auto& e = edge{ graph.List()[item.from].data.name, graph.List()[item.dest].data.name, item.weight };
                    std::cout << e.from << "," << e.to << "," << e.length << std::endl;
                    sum += e.length;
                }
                std::cout << "��Ȩ��Ϊ" << sum << std::endl;
            }
            else {
                std::cout << "ͼ����ͨ" << std::endl;
                SuggestEdges(graph);
            }
        }
        else if (choice == 7) { // �����������ʱ�����·��
            std::cout << "��������ϣ���������򣬵�һ��һ��nΪ���г��ȣ��ڶ���n���ص�Ϊ��������" << std::endl;
            int n;
            std::vector<std::string> list;
            std::cin >> n;
            while (n--) {
                std::string x;
                std::cin >> x;
                list.push_back(x);
            }
            std::cout << "���·��Ϊ" << TopologicalShortestPath(graph, list) << std::endl;
        }
        else if (choice == 8) {
            std::cout << "���������ƻ��Ŀ�ʼ�����յ��Լ�Ԥ������ʱ�䣨�ÿո������:" << std::endl;
            int totaltime;
            std::string from, to;
            std::cin >> from >> to >> totaltime;

            auto res = Visitplan(graph, from, to, totaltime);

            if (res.empty()) {
                std::cout << "�޷����������ƻ��������������㡢�յ��Ԥ��ʱ���Ƿ����" << std::endl;
            }
            else {
                int total_time_spent = 0;
                int total_distance = 0;

                std::cout << "�����ƻ����£�" << std::endl;

                for (const auto& edge : res) {
                    const auto& from_vertex = graph.GetVertex(edge.from).name;
                    const auto& to_vertex = graph.GetVertex(edge.dest).name;
                    std::cout << from_vertex << " -> " << to_vertex
                        << " ����: " << edge.weight << " ʱ��: "
                        << graph.GetVertex(edge.dest).visitTime << " ����" << std::endl;

                    total_time_spent += edge.weight/60 + graph.GetVertex(edge.dest).visitTime;
                    total_distance += edge.weight;
                }

                std::cout << "��ʱ�仨��: " << total_time_spent << " ����" << std::endl;
                std::cout << "��·��: " << total_distance << " ��" << std::endl;
            }
}
        else {
            std::cout << "��л����ʹ�ã��ټ���\n �����ߣ�\\lky1433223/ \\Voltline/" << std::endl;
            break;
        }
    }
    return 0;
}

/* ���ļ��ж�ȡ������Ϣ
 * @param nodes_path: �����ļ�·��
 * @return: ������ȡ�������ж����std::vector
 */
std::vector<VertInfo> read_nodes(const std::string& nodes_path) {
    std::ifstream fin(nodes_path);              // �����ļ�����������
    assert(fin.good());                         // ���ԣ�ͨ���ļ���������good()��������ļ��Ƿ����
    std::vector<VertInfo> nodes;                // ��ʱ�洢������Ϣ
    std::string line;                           // ͨ��std::getline��ȡ����ÿһ�У���ʱ�洢���ַ���line��
    while (std::getline(fin, line)) {       // ͨ��std::getline(std::istream&, std::string&)����ȡ��������һ��
        for (auto& c : line) {
            if (c == ',') c = ' ';              // csv�ļ��Զ��ŷָ���Ϊ�˷������ʹ���ַ������������������滻Ϊ�ո�
        }
        std::istringstream sin(line);           // ����std::istringstream���Ի�ȡ����Ϊ���ݹ���һ���ַ���������
        std::string name, type;                 // �ļ�ÿ���еĶ���������������
        int visit_time;                         // �ļ���ÿ�еĶ����Ƽ�����ʱ��
        sin >> name >> type >> visit_time;      // �����ѹ�����ַ�������������ȡ��Ӧ�ļ�������
        nodes.push_back({ name, visit_time, type });  // ������Ҫ���ص�std::vector��
    }
    fin.close();                                // �ļ�ʹ����Ϻ������ʹ��close���ʹر��������������ļ���
    return nodes;
}

/* ���ļ��ж�ȡ����Ϣ
 * @param edges_path: ���ļ�·��
 * @return: ������ȡ�������бߵ�std::vector
 */
std::vector<edge> read_edges(const std::string& edges_path) {
    std::ifstream fin(edges_path);              // �����ļ�����������
    assert(fin.good());                         // ���ԣ�ͨ���ļ���������good()��������ļ��Ƿ����
    std::vector<edge> edges;                // ��ʱ�洢������Ϣ
    std::string line;                           // ͨ��std::getline��ȡ����ÿһ�У���ʱ�洢���ַ���line��
    while (std::getline(fin, line)) {       // ͨ��std::getline(std::istream&, std::string&)����ȡ��������һ��
        for (auto& c : line) {
            if (c == ',') c = ' ';              // csv�ļ��Զ��ŷָ���Ϊ�˷������ʹ���ַ������������������滻Ϊ�ո�
        }
        std::istringstream sin(line);           // ����std::istringstream���Ի�ȡ����Ϊ���ݹ���һ���ַ���������
        std::string from, dest;                 // �ļ�ÿ���е���㣬�յ�
        int weight;                         // �ļ��������ص��Ȩ��
        sin >> from >> dest >> weight;      // �����ѹ�����ַ�������������ȡ��Ӧ�ļ�������
        edges.push_back({ from, dest, weight});  // ������Ҫ���ص�std::vector��
    }
    fin.close();                                // �ļ�ʹ����Ϻ������ʹ��close���ʹر��������������ļ���
    return edges;
}

void init(LGraph& graph) {
    std::vector<VertInfo> nodes{ read_nodes(nodes_path) };        // ͨ��read_nodes������ö�����Ϣ
    std::vector<edge> edges{ read_edges(edges_path) };            // ͨ��read_edges������ñ���Ϣ
    for (auto& v : nodes) {
        std::string name, type;
        int visit_time;
        name = v.name;
        visit_time = v.visitTime;
        type = v.type;
        graph.InsertVertex({ name, visit_time, type });
        // TODO: ��ͼgraph�в��붥��
    }
    for (auto& e : edges) {
        std::string from, to;
        int length;
        from = e.from;
        to = e.to;
        length = e.length;
        graph.InsertEdge(from, to, length);
        // TODO: ��ͼgraph�в����
    }
}

/* ��ӡĿǰ�����ж���
 * @param graph: ͼ
 */
void show_all_nodes(const LGraph& graph) {
    const auto& graph_list{ graph.List() };// ��ȡ�ڽӱ�
    const auto& deleted_nodes{ graph.Deleted() }; //��ñ�ɾ���ı�
    for (const auto& nn : graph_list) {
        // ����ڵ�������ɾ�������У�������
        if (deleted_nodes.find(nn.data.name) != deleted_nodes.end()) {
            continue;
        }
        const auto& n = VertInfo{ nn.data.name, nn.data.visitTime, nn.data.type };
        std::cout << n.name << "," << n.type << "," << n.visitTime << std::endl;
    }
    return;
    // TODO: ʵ�ִ�ӡ���ж�����Ϣ
    // TODO: ÿ�������ʽ��������,��������,�Ƽ�����ʱ��
}

/* ��ӡĿǰ�����б�
 * @param graph: ͼ
 */
void show_all_edges(const LGraph& graph) {
    // LGraph��ֻ�ṩ�˻�ȡ�ڽӱ�ͻ�ȡ��������ıߵķ���
    // Ϊ���ܹ���ȡ��ÿ���ߵ�Ԫ��Ϣ��������Ҫ������������������Ϣ�Ļ�ȡ
    const auto& graph_edges{ graph.SortedEdges() };       // ��ȡ��������ı�
    const auto& graph_list{ graph.List() };               // ��ȡ�ڽӱ�
    for (const auto& en : graph_edges) {                 // �����ߣ�ͨ���±�����ȡ��Ӧ�Ķ�����Ϣ
        const auto& e = edge{ graph_list[en.from].data.name, graph_list[en.dest].data.name, en.weight };
        std::cout << e.from << "," << e.to << "," << e.length << std::endl;
    }
}

/* ��������Ϣ�洢���ļ���
 * @param nodes_path: �����ļ�·��
 * @param graph: ͼ
 */
void store_nodes(const std::string& nodes_path, const LGraph& graph) {
    std::ofstream fout(nodes_path);                                         // ͨ��Ŀ¼�����ļ������
    const auto& deleted_nodes{ graph.Deleted() };
    const auto& graph_list{ graph.List() };                                   // ͨ��LGraph�����List()��������ڽӱ�
    for (const auto& n : graph_list) {                                       // �����ڽӱ��е�ÿ������
        /*if (deleted_nodes.find(n.data.name) != deleted_nodes.end()) {       //������ɾ���ĵ�
            continue;}*/
        const auto& v = n.data;                                             // ��ö����е�Ԫ����
        fout << v.name << "," << v.type << "," << v.visitTime << std::endl; // ���ն���洢�ĸ�ʽ���������뵽�ļ���
    }
    fout.close();                                                           // ��ʹ�����κ��ļ����󣬶�Ҫʹ��close�����ر�
}

/* ������Ϣ�洢���ļ���
 * @param edges_path: ���ļ�·��
 * @param graph: ͼ
 */
void store_edges(const std::string& edges_path, const LGraph& graph) {
    std::ofstream fout(edges_path);  // �����ļ������
    const auto& graph_edges{ graph.SortedEdges() };       // ��ȡ��������ı�
    const auto& graph_list{ graph.List() };               // ��ȡ�ڽӱ�
    for (const auto& en : graph_edges) {
        const std::string& from_name = graph_list[en.from].data.name;
        const std::string& to_name = graph_list[en.dest].data.name;
        const int weight = en.weight;
        fout << from_name << "," << to_name << "," << weight << std::endl;
    }
    fout.close();  // �ر��ļ���
    // TODO: ��ο�store_nodes��������������ʾ�����edges.csv�ļ�������
    // TODO: �Լ�show_all_edges�����л�ȡ�ߵ�Ԫ��Ϣ�ķ�ʽ���store_edges�����Ķ���
}