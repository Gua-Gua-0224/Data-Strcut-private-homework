//
// Created by ����Դ on 24-5-16.
//

#include "LGraph.h"

namespace Graph {
    LGraph::LGraph(bool directed)
        : n_verts(0), m_edges(0), directed(directed), ver_list(std::vector<HeadNode>()) {}

    bool LGraph::exist_vertex(const std::string& name) const {
        if (vertex_map.find(name) != vertex_map.end()) {//true��ʾ�Ѿ�����name�ڵ�
            return true;
        }
        //TODO:�ж��Ƿ����name���ƵĽڵ�
        return false;
    }

    bool LGraph::exist_edge(const std::string& vertex_x_name, const std::string& vertex_y_name) const {
        auto it_x = vertex_map.find(vertex_x_name);
        auto it_y = vertex_map.find(vertex_y_name);
        if (it_x == vertex_map.end() || it_y == vertex_map.end()){//��һ���ڵ㲻����
            return true;
        }
        //TODO:�ж��Ƿ����x-y�ı�
        return false;//�����ڵ㶼����
    }

    void LGraph::InsertVertex(const LocationInfo& vertex_info) {
        // ���ص����Ƿ��Ѿ�����
        int flag = exist_vertex(vertex_info.name);
        if (flag) {
            std::cerr << "Vertex \"" << vertex_info.name << "\" already exists.\n";
            return ;
        }
        //����²���Ķ���֮ǰ�ֱ�ɾ����������� ɾ�����еļ�¼
        if (deleted.find(vertex_info.name) != deleted.end()) {
            deleted.erase(vertex_info.name);
        }
        // �����µĶ���ID
        Vertex new_id = n_verts;
        // �����ڽӱ�
        ver_list.emplace_back(vertex_info);
        // ���µص�����ID��ӳ��
        vertex_map[vertex_info.name] = new_id;
        // ���¶�����
        ++n_verts;
        return;
        //TODO:����ڵ㣬�ڵ���Ϣ��LocationInfo�����
        std::cerr << "LGraph::InsertVertex(const LocationInfo &vertex_info) ��ûʵ��" << std::endl;
    }

    void LGraph::DeleteVertex(const LocationInfo& vertex_info) {
       //���ɾ�����Ƿ����
       int flag = exist_vertex(vertex_info.name);
       if (!flag) {
           std::cerr << "Vertex \"" << vertex_info.name << "\" don't exist.\n";
           return;
       }
       auto it = vertex_map.find(vertex_info.name);
        // ��ȡ����ID
        Vertex vertex_id = it->second;
        // ���ڽӱ���ɾ�����������б�
        ver_list[vertex_id].adj.clear();
        // ������������ڽӱ���ɾ����ö��������ı�
        for (auto& head_node : ver_list) {
            head_node.adj.remove_if([vertex_id](const EdgeNode& edge) {
                return edge.dest == vertex_id;
                });
        }
        // ��ӳ�����Ƴ��ö���
        vertex_map.erase(it);
        // ��������Ϊ��ɾ��
        deleted.insert(vertex_info.name);
        // ���¶�����
        --n_verts;
        return;
        //TODO:ɾ���ڵ㣬�ڵ���Ϣ��LocationInfo�����
        std::cerr << "LGraph::DeleteVertex(const LocationInfo &vertex_info) ��ûʵ��" << std::endl;
    }

    void LGraph::UpdateVertex(const std::string& name, int new_time) {
        // ȷ�Ͼɽڵ��Ƿ����
        auto it = vertex_map.find(name);
        if (it == vertex_map.end()) {
            std::cerr << "Vertex \"" << name << "\" does not exist." << std::endl;
            return;
        }
        Vertex vertex_id = it->second;
        // ���½ڵ�����ʱ��
        ver_list[vertex_id].data.visitTime = new_time;
        std::cout << "�ڵ�������" << std::endl;
        return;
        //TODO:���½ڵ㣬��/�ɽڵ����Ϣ��LocationInfo�����
        std::cerr << "LGraph::UpdateVertex(const LocationInfo &old_info, LocationInfo &new_info) ��ûʵ��" << std::endl;
    }

    VertInfo LGraph::GetVertex(const std::string& name) const {
        int flag = exist_vertex(name);
        if (!flag) {
            std::cerr << "Vertex \"" << name << "\" don't exist.\n";
            return{};
        }
        //���ؼ�ֵ��
        auto it = vertex_map.find(name);
        //TODO:��ȡ�ڵ㣬�ɽڵ�����ѯ�ڵ���Ϣ
        return ver_list[it->second].data;
        std::cerr << "LGraph::GetVertex(const std::string &name) ��ûʵ��" << std::endl;
    }

    // �����ȡͬ���Ͷ�����Ϣ�ĺ���
    VertInfo LGraph::GetsametypeVertex(const std::string& name) const {
        // ��ȡӳ���
        const auto& vertex_map = this->Map();

        // �������ж��㣬��������ƥ�������
        for (const auto& pair : vertex_map) {
            const auto& vertex_info = this->GetVertex(pair.second);
            if (vertex_info.type == name) {
                return vertex_info;
            }
        }
        // ���δ�ҵ����׳��쳣
        throw std::out_of_range("���Ͳ�����");
    }

    VertInfo LGraph::GetVertex(const Vertex vertex) const {
        // ���ڵ�ID�Ƿ�����Ч��Χ��
        if (vertex >= ver_list.size() || vertex < 0) {
            throw std::out_of_range("���ID������Ч��Χ��");
        }
        // ���ض�Ӧ�Ķ�����Ϣ
        return ver_list[vertex].data;
        //TODO:��ȡ�ڵ㣬�ɽڵ��ID��ѯ
        std::cerr << "LGraph::GetVertex(const Vertex vertex) ��ûʵ��" << std::endl;
    }

    void LGraph::InsertEdge(const std::string& vertex_x_name, const std::string& vertex_y_name, GElemSet weight) {
        //TODO:�����
           // ������������Ƿ����
        auto it_x = vertex_map.find(vertex_x_name);
        auto it_y = vertex_map.find(vertex_y_name);
        int flag = exist_edge(vertex_x_name, vertex_y_name);
        if (flag) {
            std::cerr << "One or both vertices do not exist: \""
                << vertex_x_name << "\", \"" << vertex_y_name << "\".\n";
            return;
        }
        Vertex x = it_x->second;
        Vertex y = it_y->second;
        // ����ߵ�x���ڽӱ�
        ver_list[x].adj.emplace_back(x, y, weight);
        // ���������ͼ��������y���ڽӱ������Ӧ�ı�
        if (!directed) {
            ver_list[y].adj.emplace_back(y, x, weight);
        }
        // ���±���
        ++m_edges;
        return;
        std::cerr << "LGraph::InsertEdge(const std::string &vertex_x_name, const std::string &vertex_y_name, GElemSet weight) ��ûʵ��" << std::endl;
    }

    void LGraph::DeleteEdge(const std::string& vertex_x_name, const std::string& vertex_y_name) {
        // ��������ڵ��Ƿ����
        int flag = exist_edge(vertex_x_name, vertex_y_name);
        auto it_x = vertex_map.find(vertex_x_name);
        auto it_y = vertex_map.find(vertex_y_name);
        if (flag) {
            std::cerr << "�����ڵ㲻�Ϸ�"
                << vertex_x_name << ", " << vertex_y_name << std::endl;
            return;
        }
        // ��ȡ��Ӧ�Ľڵ� ID
        Vertex vertex_x = it_x->second;
        Vertex vertex_y = it_y->second;
        // ɾ�� vertex_x ���ڽӱ���ָ�� vertex_y �ı�
        auto& tmpx = ver_list[vertex_x].adj;
        tmpx.remove_if([vertex_y](const EdgeNode& edge) {
            return edge.dest == vertex_y;
            });

        // ���������ͼ��ɾ�� vertex_y ���ڽӱ���ָ�� vertex_x �ı�
        if (!directed) {
            auto& tmpy = ver_list[vertex_y].adj;
            tmpy.remove_if([vertex_x](const EdgeNode& edge) {
                return edge.dest == vertex_x;
                });
        }

        // ���±���
        --m_edges;
        return;
        //TODO:ɾ���ߣ��������ڵ���ȷ��һ����
        std::cerr << "LGraph::DeleteEdge(const std::string &vertex_x_name, const std::string &vertex_y_name) ��ûʵ��" << std::endl;

    }

    void LGraph::DeleteEdge(Vertex vertex_x, Vertex vertex_y) {
        // ���ڵ� ID �Ƿ���Ч
        if (vertex_x >= ver_list.size() || vertex_y >= ver_list.size() ||
            vertex_x < 0 || vertex_y < 0) {
            std::cerr << "Invalid vertex IDs: " << vertex_x << ", " << vertex_y << std::endl;
            return;
        }

        // ɾ�� vertex_x ���ڽӱ���ָ�� vertex_y �ı�
        auto& tmpx = ver_list[vertex_x].adj;
        tmpx.remove_if([vertex_y](const EdgeNode& edge) {
            return edge.dest == vertex_y;
            });

        // ���������ͼ��ɾ�� vertex_y ���ڽӱ���ָ�� vertex_x �ı�
        if (!directed) {
            auto& tmpy = ver_list[vertex_y].adj;
            tmpy.remove_if([vertex_x](const EdgeNode& edge) {
                return edge.dest == vertex_x;
                });
        }
        // ���±���
        --m_edges;
        return;
        //TODO:ɾ���ߣ��������ڵ�IDȷ��һ����
        std::cerr << "LGraph::DeleteEdge(Vertex vertex_x, Vertex vertex_y) ��ûʵ��" << std::endl;
    }


    void LGraph::UpdateEdge(const std::string& vertex_x_name, const std::string& vertex_y_name, GElemSet new_weight) {
        // ȷ�Ͻڵ����
        int flag = exist_edge(vertex_x_name, vertex_y_name);
        auto it_x = vertex_map.find(vertex_x_name);
        auto it_y = vertex_map.find(vertex_y_name);
        if (flag) {
            std::cerr << "�����ڵ㲻�Ϸ�"
                << vertex_x_name << ", " << vertex_y_name << std::endl;
            return;
        }
        Vertex vertex_x = it_x->second;
        Vertex vertex_y = it_y->second;
        bool updated = false;
        // �����ڽӱ��еı�Ȩ��
        for (auto& edge : ver_list[vertex_x].adj) {
            if (edge.dest == vertex_y) {
                edge.weight = new_weight;
                updated = true;
                break; 
            }
        }
        std::cout << "�߸������" << std::endl;
        return;
        //TODO:���±ߣ��������ڵ���ȷ��һ����
        std::cerr << "LGraph::UpdateEdge(const std::string &vertex_x_name, const std::string &vertex_y_name, GElemSet new_weight) ��ûʵ��" << std::endl;
    }

    GElemSet LGraph::GetEdge(const std::string& vertex_x_name, const std::string& vertex_y_name) const {
        int flag = exist_edge(vertex_x_name, vertex_y_name);
        auto it_x = vertex_map.find(vertex_x_name);
        auto it_y = vertex_map.find(vertex_y_name);
        if (flag) {
            std::cerr << "�����ڵ㲻�Ϸ�"
                << vertex_x_name << ", " << vertex_y_name << std::endl;
            return NIL;
        }
        // ��ȡ��Ӧ�Ľڵ� ID
        Vertex vertex_x = it_x->second;
        Vertex vertex_y = it_y->second;
        // �� vertex_x ���ڽ������в���ָ�� vertex_y �ı�
        const auto& adj_list_x = ver_list[vertex_x].adj;
        for (const auto& edge : adj_list_x) {
            if (edge.dest == vertex_y) {
                return edge.weight; // �����ҵ��ıߵ�Ȩ��
            }
        }
        // ���������ͼ��û���ҵ������ vertex_y ���ڽ������ԳƱߣ�
        if (!directed) {
            const auto& adj_list_y = ver_list[vertex_y].adj;
            for (const auto& edge : adj_list_y) {
                if (edge.dest == vertex_x) {
                    return edge.weight; // �����ҵ��ĶԳƱߵ�Ȩ��
                }
            }
        }
        // ����߲����ڣ����� NIL
        return NIL;
        //TODO:��ȡ�ߵ���Ϣ
        std::cerr << "LGraph::GetEdge(const std::string &vertex_x_name, const std::string &vertex_y_name) ��ûʵ��" << std::endl;
    }

    // ���尴�ص���Ϣ������ص�·�ĺ���
    std::vector<EdgeNode> LGraph::GetEdgesFromLocation(const std::string& location_name) const {
        // ��ȡӳ���
        const auto& vertex_map = this->Map();

        // ���ص��Ƿ����
        auto it = vertex_map.find(location_name);
        if (it == vertex_map.end()) {
            throw  std::out_of_range("�ص㲻����");
        }
        // ��ȡ�õص��Ӧ�Ķ���
        Vertex vertex_id = it->second;
        const auto& adjacency_list = this->List();

        // ת�� std::list<EdgeNode> �� std::vector<EdgeNode>
        std::vector<EdgeNode> edges(adjacency_list[vertex_id].adj.begin(), adjacency_list[vertex_id].adj.end());

        return edges;
    }


    std::vector<EdgeNode> LGraph::SortedEdges(std::function<bool(const GElemSet&, const GElemSet&)> cmp) const {
        // ����һ���洢���бߵ�����
        std::vector<EdgeNode> edges;

        // ����ÿ��������ڽ�����
        for (size_t i = 0; i < ver_list.size(); ++i) {
            for (const auto& edge : ver_list[i].adj) {
                // ��������ͼ�������ظ��ߣ�ֻ����һ����
                if (!directed && i > edge.dest) {
                    continue;
                }
                edges.push_back(edge); // ���߼���������
            }
        }
        // ����Ȩ��С��������
        std::sort(edges.begin(), edges.end(), [](const EdgeNode& e1, const EdgeNode& e2) {
            return e1.weight < e2.weight;
            });
        return edges; // ���������ı��б�
        //TODO:��ȡ����Ȩ�͸���������������б�
        std::cerr << "LGraph::SortedEdges(std::function<bool(const GElemSet &, const GElemSet &)> cmp) ��ûʵ��" << std::endl;
        return std::vector<EdgeNode>();
    }


}