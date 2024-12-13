//
// Created by 刘凯源 on 24-10-13.
//
#include "Algorithm.h"
#include <unordered_set>

namespace Graph {
    namespace Algorithm {
        std::list<Vertex> GetCircuit(LGraph& graph, Vertex start) {
            // 使用双向链表构造回路
            std::list<Vertex> circuit;
            Vertex head = start;  // 当前回路的起点
            circuit.push_back(start);  // 将起点加入回路
            auto& adjacency_list = graph.List();  // 获取邻接表
            while (!adjacency_list[head].adj.empty()) {
                Vertex tail = adjacency_list[head].adj.front().dest;  // 取邻接表中第一条边的终点
                // 删除无向图中的双向边
                graph.DeleteEdge(head, tail);
                graph.DeleteEdge(tail, head);
                // 将该顶点加入回路
                circuit.push_back(tail);
                // 更新当前头节点
                head = tail;
            }
            // 如果最后的边正好闭合回到起点，则是一个完整回路，否则返回部分回路
            if (circuit.front() != circuit.back()) {
                std::cerr << "Error: The graph does not form a valid circuit starting from the given vertex." << std::endl;
            }
            return circuit;
            //TODO:从给定点出发获得一条回路
            std::cerr << "GetCircuit 还没实现" << std::endl;
        }

        std::list<Vertex> EulerCircle(const LGraph& graph) {
            // 检查图是否为欧拉图（所有顶点的度数为偶数）
            LGraph tmp_graph = graph;
            if (!HaveEulerCircle(tmp_graph)) {
                std::cerr << "The graph is not an Eulerian graph. It contains a vertex with odd degree.\n";
                return{};
            }
            // 初始化第一个回路，从顶点 0 开始（假设图的顶点编号从 0 开始）
            std::list<Vertex> circuit = GetCircuit(tmp_graph, 0);
            // 寻找新的起始顶点，合并回路
            for (auto it = circuit.begin(); it != circuit.end(); ++it) {
                Vertex v = *it;
                if (!tmp_graph.List()[v].adj.empty()) {
                    // 如果当前顶点有未访问的边，获取从该顶点出发的一个新回路
                    std::list<Vertex> next_circuit = GetCircuit(tmp_graph, v);
                    // 插入新的回路到当前回路中
                    it = circuit.insert(it, next_circuit.begin(), next_circuit.end());
                    --it; // 调整迭代器，回到新插入回路的起点
                }
            }
            return circuit;
            //TODO:获取欧拉回路,你可以使用GetCircuit函数
            std::cerr << "EulerCircle 还没实现" << std::endl;
            return {};
        }

        bool HaveEulerCircle(const LGraph& graph) {
            // 获取图的顶点数量
            size_t vertex_count = graph.List().size();
            LGraph tmp_graph = graph;
            //检查连通性
            if (!IsConnected(tmp_graph)) {
                return false;
            }
            // 检查所有顶点的度是否为偶数
            for (size_t i = 0; i < vertex_count; ++i) {
                if (graph.List()[i].adj.size() % 2 != 0) { // 如果度是奇数
                    return false;
                }
            }
            // 图满足欧拉回路的条件
            return true;
            //TODO:判断是否有欧拉回路
            std::cerr << "HaveEulerCircle 还没实现" << std::endl;
            return false;
        }

        void BFSv(LGraph& graph, Vertex v, std::vector<bool>& visited) {
            //TODO:广度优先搜索整个图
            std::queue<Vertex> q;  // 创建队列
            q.push(v);             // 将起点入队
            visited[v] = true;     // 标记起点为已访问

            const auto& adjacency_list = graph.List(); // 获取邻接表

            while (!q.empty()) {
                Vertex current = q.front();
                q.pop();

                // 遍历当前节点的所有邻接节点
                for (const auto& edge : adjacency_list[current].adj) {
                    Vertex next = edge.dest;
                    if (!visited[next]) {
                        visited[next] = true; // 标记为已访问
                        q.push(next);         // 将其入队
                    }
                }
            }
            return;
            std::cerr << "BFSv 还没实现" << std::endl;
        }

        bool IsConnected(LGraph& graph) {
            int n = graph.NumVertices(); // 获取顶点数
            if (n == 0) return true;     // 空图被视为连通

            std::vector<bool> visited(n, false); // 访问标记数组
            BFSv(graph, 0, visited);            // 从第一个顶点开始进行广度优先搜索

            // 检查是否所有顶点都被访问过
            for (int i = 0; i < n; ++i) {
                if (!visited[i]) {
                    return false; // 如果有顶点未被访问，说明图不连通
                }
            }

            return true; // 所有顶点都被访问，图是连通的
        }


        int GetShortestPath(const LGraph& graph, const std::string& vertex_name_x, const std::string& vertex_name_y) {//用Floyd算法实现
            // 检查两个节点是否存在
            if (!graph.exist_vertex(vertex_name_x)|| !graph.exist_vertex(vertex_name_y)) {
                std::cerr << "两个节点不合法"
                    << vertex_name_x << ", " << vertex_name_y << std::endl;
                return -1;
            }
            // 获取顶点数量
            size_t n = graph.NumVertices();
            // 映射顶点名到顶点ID
            const auto& vertex_map = graph.Map();
            auto it_x = vertex_map.find(vertex_name_x);
            auto it_y = vertex_map.find(vertex_name_y);
            // 获取对应的节点 ID
            Vertex id_x = it_x->second;
            Vertex id_y = it_y->second;
            // 初始化距离矩阵
            std::vector<std::vector<int>> dist(n, std::vector<int>(n, INT_MAX));
            // 将图中已有的边信息填入距离矩阵
            const auto& adjacency_list = graph.List();
            for (Vertex i = 0; i < n; ++i) {
                dist[i][i] = 0; // 自身到自身的距离为0
                for (const auto& edge : adjacency_list[i].adj) {
                    dist[i][edge.dest] = edge.weight; // 邻接边的权值
                }
            }
            // Floyd算法
            for (Vertex k = 0; k < n; ++k) {        // 遍历所有中间顶点
                for (Vertex i = 0; i < n; ++i) {    // 遍历起点
                    for (Vertex j = 0; j < n; ++j) {// 遍历终点
                        if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
                            dist[i][j] = std::min(dist[i][j], dist[i][k] + dist[k][j]);
                        }
                    }
                }
            }
            // 返回结果
            int shortest_path = dist[id_x][id_y];
            if (shortest_path == INT_MAX) {
                throw std::runtime_error("在某两个点之间不存在路径");
            }
            return shortest_path;
            //TODO:获取两点之间的最短路径
            std::cerr << "GetShortestPath 还没实现" << std::endl;
            return -1;
        }
        int TopologicalShortestPath(const LGraph& graph, std::vector<std::string> path) {
            // 获取顶点映射
            const auto& vertex_map = graph.Map();
            // 检查拓扑序列合法性
            if (path.empty()) {
                throw std::invalid_argument("拓扑路径不存在");
            }
            for (const auto& vertex_name : path) {
                if (vertex_map.find(vertex_name) == vertex_map.end()) {
                    throw std::invalid_argument("Vertex in topological order does not exist in the graph.");
                }
            }
            // 使用 GetShortestPath 函数计算起点到每个顶点的最短路径
            std::string start_vertex = path[0];
            int shortest_path = INT_MAX;
            for (const auto& target_vertex : path) {
                // 只计算起点到当前顶点的路径
                int path_length = GetShortestPath(graph, start_vertex, target_vertex);
                if (path_length != INT_MAX) { // 如果路径存在，更新最短路径
                    shortest_path = std::min(shortest_path, path_length);
                }
            }
            if (shortest_path == INT_MAX) {
                throw std::runtime_error("不存在最小路径");
            }
            return shortest_path;
            //TODO:获取拓扑受限的最短路径，拓扑序由path给出
            std::cerr << "TopologicalShortestPath 还没实现" << std::endl;
            return -1;
        }

        std::vector<EdgeNode> MinimumSpanningTree(const LGraph& graph) {//用Kruskal算法实现
            std::vector<EdgeNode> mst_edges; // 存储最小生成树的边
            size_t vertex_count = graph.List().size();
            size_t edge_count = graph.EdgeVertices();

            // 获取图中所有边
            std::vector<EdgeNode> all_edges = graph.SortedEdges();
            // 初始化并查集
            DSU dsu(vertex_count);
            // 遍历排序后的边，按权值从小到大加入 MST
            for (const auto& edge : all_edges) {
                // 如果两个顶点不在同一个集合中，加入这条边到 MST
                if (!dsu.same(edge.from, edge.dest)) {
                    mst_edges.push_back(edge);
                    dsu.unite(edge.from, edge.dest);
                }
                // 如果 MST 已经包含了 vertex_count - 1 条边，可以提前退出
                if (mst_edges.size() == vertex_count - 1) {
                    break;
                }
            }
            return mst_edges;
            //TODO:计算最小生成树，并返回树上的边
            std::cerr << "MinimumSpanningTree 还没实现" << std::endl;
            return {};
        }
        std::vector<EdgeNode> Visitplan(const LGraph& graph, const std::string& from, const std::string& to, const int totaltime) {
            // 检查输入的起点和终点是否有效
            const auto& vertex_map = graph.Map();
            if (vertex_map.find(from) == vertex_map.end() || vertex_map.find(to) == vertex_map.end()) {
                throw std::invalid_argument("起点或终点无效，请检查输入。");
            }

            // 初始化
            int current_time = 0;
            int total_distance = 0;
            std::vector<EdgeNode> plan; // 保存最终的旅游路线
            std::unordered_set<std::string> visited; // 标记已访问的景点
            std::string current_location = from;
            visited.insert(current_location); // 标记起点已访问

            // 使用贪心策略逐步选择下一个景点
            while (current_location != to) {
                const auto& adjacency_list = graph.List();
                const auto& neighbors = adjacency_list[vertex_map.at(current_location)].adj;

                // 筛选未访问过的邻接点，并按性价比排序
                std::vector<EdgeNode> candidates;
                for (const auto& edge : neighbors) {
                    const auto& dest_name = graph.GetVertex(edge.dest).name;
                    if (visited.find(dest_name) == visited.end()) {
                        candidates.push_back(edge);
                    }
                }

                // 如果没有候选点，直接结束
                if (candidates.empty()) {
                    std::cout << "无法找到进一步的路径，停止规划。" << std::endl;
                    break;
                }

                // 按性价比排序：性价比 = 景点参观时间 / 路程
                std::sort(candidates.begin(), candidates.end(), [&](const EdgeNode& a, const EdgeNode& b) {
                    int time_a = graph.GetVertex(a.dest).visitTime;
                    int time_b = graph.GetVertex(b.dest).visitTime;
                    return static_cast<double>(time_a) / a.weight > static_cast<double>(time_b) / b.weight;
                    });

                // 选择性价比最高的候选点
                const EdgeNode& next_edge = candidates.front();
                const auto& next_name = graph.GetVertex(next_edge.dest).name;

                // 计算到达下一个景点所需的时间
                int travel_time = next_edge.weight / 60; // 路程时间 (单位：分钟)
                int visit_time = graph.GetVertex(next_edge.dest).visitTime;

                // 如果到下一个景点后会超时，则停止规划
                if (current_time + travel_time + visit_time > totaltime) {
                    break;
                }

                // 更新规划信息
                current_time += travel_time + visit_time;
                total_distance += next_edge.weight;
                plan.push_back(next_edge);
                visited.insert(next_name);
                current_location = next_name;

                // 如果已到达终点，则停止
                if (current_location == to) {
                    break;
                }
            }

            // 最后输出结果
            std::cout << "旅游规划完成，总时间：" << current_time << " 分钟，总距离：" << total_distance << " 米。" << std::endl;
            if (current_location != to) {
                std::cout << "注意：未能到达终点 \"" << to << "\"，请适当调整规划时间。" << std::endl;
            }
            else {
                std::cout << "规划路线成功到达终点 \"" << to << "\"。" << std::endl;
            }

            return plan;
        }



        void SuggestEdges(LGraph& graph) {//查看哪些边未联通
            int n = graph.NumVertices(); // 获取图中顶点数量
            if (n == 0) {
                std::cout << "图为空，没有点需要连接。" << std::endl;
                return;
            }

            // 初始化并查集，用于标记连通性
            DSU dsu(n);

            // 遍历每条边，将连通的顶点合并
            const auto& adjacency_list = graph.List();
            for (int i = 0; i < n; ++i) {
                for (const auto& edge : adjacency_list[i].adj) {
                    int from = i;
                    int to = edge.dest;
                    dsu.unite(from, to);
                }
            }

            // 找出连通分量的根节点
            std::unordered_set<int> components;
            for (int i = 0; i < n; ++i) {
                components.insert(dsu.find(i));
            }

            // 如果只有一个连通分量，则图已连通
            if (components.size() == 1) {
                std::cout << "图是连通的，没有需要添加的边。" << std::endl;
                return;
            }

            // 提示用户需要添加哪些边
            std::vector<int> component_roots(components.begin(), components.end());
            std::cout << "图不连通，需要以下边以连通图：" << std::endl;

            std::vector<std::pair<int, int>> edges_to_add; // 用于保存需要添加的边

            for (size_t i = 1; i < component_roots.size(); ++i) {
                int u = component_roots[0];   // 选择一个固定的连通分量
                int v = component_roots[i];  // 将其他连通分量与其连接
                auto u_info = graph.GetVertex(u);
                auto v_info = graph.GetVertex(v);
                std::cout << "建议添加边：\"" << u_info.name << "\" - \"" << v_info.name << "\"" << std::endl;
                edges_to_add.emplace_back(u, v);
            }

            // 提示用户选择是否自动生成这些边
            char choice;
            std::cout << "是否由程序自动生成这些边？(y/n): ";
            std::cin >> choice;

            if (choice == 'y') {
                for (const auto& edge : edges_to_add) {
                    int weight = 1 + rand() % 200; // 生成 1 至 200 的随机权重
                    auto u_info = graph.GetVertex(edge.first);
                    auto v_info = graph.GetVertex(edge.second);
                    graph.InsertEdge(u_info.name, v_info.name, weight); // 添加边到图中
                    std::cout << "已自动添加边：\"" << u_info.name << "\" - \"" << v_info.name << "\"，权重：" << weight << std::endl;
                }
                std::cout << "图已通过程序生成的边连通。" << std::endl;
            }
            else {
                std::cout << "请根据建议手动添加这些边。" << std::endl;
            }
        }

    }
}