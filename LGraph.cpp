//
// Created by 刘凯源 on 24-5-16.
//

#include "LGraph.h"

namespace Graph {
    LGraph::LGraph(bool directed)
        : n_verts(0), m_edges(0), directed(directed), ver_list(std::vector<HeadNode>()) {}

    bool LGraph::exist_vertex(const std::string& name) const {
        if (vertex_map.find(name) != vertex_map.end()) {//true表示已经存在name节点
            return true;
        }
        //TODO:判断是否存在name名称的节点
        return false;
    }

    bool LGraph::exist_edge(const std::string& vertex_x_name, const std::string& vertex_y_name) const {
        auto it_x = vertex_map.find(vertex_x_name);
        auto it_y = vertex_map.find(vertex_y_name);
        if (it_x == vertex_map.end() || it_y == vertex_map.end()){//有一个节点不存在
            return true;
        }
        //TODO:判断是否存在x-y的边
        return false;//两个节点都存在
    }

    void LGraph::InsertVertex(const LocationInfo& vertex_info) {
        // 检查地点名是否已经存在
        int flag = exist_vertex(vertex_info.name);
        if (flag) {
            std::cerr << "Vertex \"" << vertex_info.name << "\" already exists.\n";
            return ;
        }
        //如果新插入的顶点之前又被删除过，则清除 删除表中的记录
        if (deleted.find(vertex_info.name) != deleted.end()) {
            deleted.erase(vertex_info.name);
        }
        // 分配新的顶点ID
        Vertex new_id = n_verts;
        // 更新邻接表
        ver_list.emplace_back(vertex_info);
        // 更新地点名到ID的映射
        vertex_map[vertex_info.name] = new_id;
        // 更新顶点数
        ++n_verts;
        return;
        //TODO:插入节点，节点信息由LocationInfo类给出
        std::cerr << "LGraph::InsertVertex(const LocationInfo &vertex_info) 还没实现" << std::endl;
    }

    void LGraph::DeleteVertex(const LocationInfo& vertex_info) {
       //检查删除点是否存在
       int flag = exist_vertex(vertex_info.name);
       if (!flag) {
           std::cerr << "Vertex \"" << vertex_info.name << "\" don't exist.\n";
           return;
       }
       auto it = vertex_map.find(vertex_info.name);
        // 获取顶点ID
        Vertex vertex_id = it->second;
        // 从邻接表中删除关联的所有边
        ver_list[vertex_id].adj.clear();
        // 在其他顶点的邻接表中删除与该顶点相连的边
        for (auto& head_node : ver_list) {
            head_node.adj.remove_if([vertex_id](const EdgeNode& edge) {
                return edge.dest == vertex_id;
                });
        }
        // 从映射中移除该顶点
        vertex_map.erase(it);
        // 将顶点标记为已删除
        deleted.insert(vertex_info.name);
        // 更新顶点数
        --n_verts;
        return;
        //TODO:删除节点，节点信息由LocationInfo类给出
        std::cerr << "LGraph::DeleteVertex(const LocationInfo &vertex_info) 还没实现" << std::endl;
    }

    void LGraph::UpdateVertex(const std::string& name, int new_time) {
        // 确认旧节点是否存在
        auto it = vertex_map.find(name);
        if (it == vertex_map.end()) {
            std::cerr << "Vertex \"" << name << "\" does not exist." << std::endl;
            return;
        }
        Vertex vertex_id = it->second;
        // 更新节点的浏览时间
        ver_list[vertex_id].data.visitTime = new_time;
        std::cout << "节点跟新完毕" << std::endl;
        return;
        //TODO:更新节点，新/旧节点的信息由LocationInfo类给出
        std::cerr << "LGraph::UpdateVertex(const LocationInfo &old_info, LocationInfo &new_info) 还没实现" << std::endl;
    }

    VertInfo LGraph::GetVertex(const std::string& name) const {
        int flag = exist_vertex(name);
        if (!flag) {
            std::cerr << "Vertex \"" << name << "\" don't exist.\n";
            return{};
        }
        //返回键值对
        auto it = vertex_map.find(name);
        //TODO:获取节点，由节点名查询节点信息
        return ver_list[it->second].data;
        std::cerr << "LGraph::GetVertex(const std::string &name) 还没实现" << std::endl;
    }

    // 定义获取同类型顶点信息的函数
    VertInfo LGraph::GetsametypeVertex(const std::string& name) const {
        // 获取映射表
        const auto& vertex_map = this->Map();

        // 遍历所有顶点，查找名称匹配的类型
        for (const auto& pair : vertex_map) {
            const auto& vertex_info = this->GetVertex(pair.second);
            if (vertex_info.type == name) {
                return vertex_info;
            }
        }
        // 如果未找到，抛出异常
        throw std::out_of_range("类型不存在");
    }

    VertInfo LGraph::GetVertex(const Vertex vertex) const {
        // 检查节点ID是否在有效范围内
        if (vertex >= ver_list.size() || vertex < 0) {
            throw std::out_of_range("检查ID不在有效范围内");
        }
        // 返回对应的顶点信息
        return ver_list[vertex].data;
        //TODO:获取节点，由节点的ID查询
        std::cerr << "LGraph::GetVertex(const Vertex vertex) 还没实现" << std::endl;
    }

    void LGraph::InsertEdge(const std::string& vertex_x_name, const std::string& vertex_y_name, GElemSet weight) {
        //TODO:插入边
           // 检查两个顶点是否存在
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
        // 插入边到x的邻接表
        ver_list[x].adj.emplace_back(x, y, weight);
        // 如果是无向图，还需在y的邻接表插入相应的边
        if (!directed) {
            ver_list[y].adj.emplace_back(y, x, weight);
        }
        // 更新边数
        ++m_edges;
        return;
        std::cerr << "LGraph::InsertEdge(const std::string &vertex_x_name, const std::string &vertex_y_name, GElemSet weight) 还没实现" << std::endl;
    }

    void LGraph::DeleteEdge(const std::string& vertex_x_name, const std::string& vertex_y_name) {
        // 检查两个节点是否存在
        int flag = exist_edge(vertex_x_name, vertex_y_name);
        auto it_x = vertex_map.find(vertex_x_name);
        auto it_y = vertex_map.find(vertex_y_name);
        if (flag) {
            std::cerr << "两个节点不合法"
                << vertex_x_name << ", " << vertex_y_name << std::endl;
            return;
        }
        // 获取对应的节点 ID
        Vertex vertex_x = it_x->second;
        Vertex vertex_y = it_y->second;
        // 删除 vertex_x 的邻接表中指向 vertex_y 的边
        auto& tmpx = ver_list[vertex_x].adj;
        tmpx.remove_if([vertex_y](const EdgeNode& edge) {
            return edge.dest == vertex_y;
            });

        // 如果是无向图，删除 vertex_y 的邻接表中指向 vertex_x 的边
        if (!directed) {
            auto& tmpy = ver_list[vertex_y].adj;
            tmpy.remove_if([vertex_x](const EdgeNode& edge) {
                return edge.dest == vertex_x;
                });
        }

        // 更新边数
        --m_edges;
        return;
        //TODO:删除边，由两个节点名确定一条边
        std::cerr << "LGraph::DeleteEdge(const std::string &vertex_x_name, const std::string &vertex_y_name) 还没实现" << std::endl;

    }

    void LGraph::DeleteEdge(Vertex vertex_x, Vertex vertex_y) {
        // 检查节点 ID 是否有效
        if (vertex_x >= ver_list.size() || vertex_y >= ver_list.size() ||
            vertex_x < 0 || vertex_y < 0) {
            std::cerr << "Invalid vertex IDs: " << vertex_x << ", " << vertex_y << std::endl;
            return;
        }

        // 删除 vertex_x 的邻接表中指向 vertex_y 的边
        auto& tmpx = ver_list[vertex_x].adj;
        tmpx.remove_if([vertex_y](const EdgeNode& edge) {
            return edge.dest == vertex_y;
            });

        // 如果是无向图，删除 vertex_y 的邻接表中指向 vertex_x 的边
        if (!directed) {
            auto& tmpy = ver_list[vertex_y].adj;
            tmpy.remove_if([vertex_x](const EdgeNode& edge) {
                return edge.dest == vertex_x;
                });
        }
        // 更新边数
        --m_edges;
        return;
        //TODO:删除边，由两个节点ID确定一条边
        std::cerr << "LGraph::DeleteEdge(Vertex vertex_x, Vertex vertex_y) 还没实现" << std::endl;
    }


    void LGraph::UpdateEdge(const std::string& vertex_x_name, const std::string& vertex_y_name, GElemSet new_weight) {
        // 确认节点存在
        int flag = exist_edge(vertex_x_name, vertex_y_name);
        auto it_x = vertex_map.find(vertex_x_name);
        auto it_y = vertex_map.find(vertex_y_name);
        if (flag) {
            std::cerr << "两个节点不合法"
                << vertex_x_name << ", " << vertex_y_name << std::endl;
            return;
        }
        Vertex vertex_x = it_x->second;
        Vertex vertex_y = it_y->second;
        bool updated = false;
        // 更新邻接表中的边权重
        for (auto& edge : ver_list[vertex_x].adj) {
            if (edge.dest == vertex_y) {
                edge.weight = new_weight;
                updated = true;
                break; 
            }
        }
        std::cout << "边跟新完毕" << std::endl;
        return;
        //TODO:更新边，由两个节点名确定一条边
        std::cerr << "LGraph::UpdateEdge(const std::string &vertex_x_name, const std::string &vertex_y_name, GElemSet new_weight) 还没实现" << std::endl;
    }

    GElemSet LGraph::GetEdge(const std::string& vertex_x_name, const std::string& vertex_y_name) const {
        int flag = exist_edge(vertex_x_name, vertex_y_name);
        auto it_x = vertex_map.find(vertex_x_name);
        auto it_y = vertex_map.find(vertex_y_name);
        if (flag) {
            std::cerr << "两个节点不合法"
                << vertex_x_name << ", " << vertex_y_name << std::endl;
            return NIL;
        }
        // 获取对应的节点 ID
        Vertex vertex_x = it_x->second;
        Vertex vertex_y = it_y->second;
        // 在 vertex_x 的邻接链表中查找指向 vertex_y 的边
        const auto& adj_list_x = ver_list[vertex_x].adj;
        for (const auto& edge : adj_list_x) {
            if (edge.dest == vertex_y) {
                return edge.weight; // 返回找到的边的权重
            }
        }
        // 如果是无向图且没有找到，检查 vertex_y 的邻接链表（对称边）
        if (!directed) {
            const auto& adj_list_y = ver_list[vertex_y].adj;
            for (const auto& edge : adj_list_y) {
                if (edge.dest == vertex_x) {
                    return edge.weight; // 返回找到的对称边的权重
                }
            }
        }
        // 如果边不存在，返回 NIL
        return NIL;
        //TODO:获取边的信息
        std::cerr << "LGraph::GetEdge(const std::string &vertex_x_name, const std::string &vertex_y_name) 还没实现" << std::endl;
    }

    // 定义按地点信息查找相关道路的函数
    std::vector<EdgeNode> LGraph::GetEdgesFromLocation(const std::string& location_name) const {
        // 获取映射表
        const auto& vertex_map = this->Map();

        // 检查地点是否存在
        auto it = vertex_map.find(location_name);
        if (it == vertex_map.end()) {
            throw  std::out_of_range("地点不存在");
        }
        // 获取该地点对应的顶点
        Vertex vertex_id = it->second;
        const auto& adjacency_list = this->List();

        // 转换 std::list<EdgeNode> 到 std::vector<EdgeNode>
        std::vector<EdgeNode> edges(adjacency_list[vertex_id].adj.begin(), adjacency_list[vertex_id].adj.end());

        return edges;
    }


    std::vector<EdgeNode> LGraph::SortedEdges(std::function<bool(const GElemSet&, const GElemSet&)> cmp) const {
        // 创建一个存储所有边的向量
        std::vector<EdgeNode> edges;

        // 遍历每个顶点的邻接链表
        for (size_t i = 0; i < ver_list.size(); ++i) {
            for (const auto& edge : ver_list[i].adj) {
                // 对于无向图，避免重复边（只保留一条）
                if (!directed && i > edge.dest) {
                    continue;
                }
                edges.push_back(edge); // 将边加入结果向量
            }
        }
        // 按边权从小到大排序
        std::sort(edges.begin(), edges.end(), [](const EdgeNode& e1, const EdgeNode& e2) {
            return e1.weight < e2.weight;
            });
        return edges; // 返回排序后的边列表
        //TODO:获取按边权和给定规则排序的所有边
        std::cerr << "LGraph::SortedEdges(std::function<bool(const GElemSet &, const GElemSet &)> cmp) 还没实现" << std::endl;
        return std::vector<EdgeNode>();
    }


}