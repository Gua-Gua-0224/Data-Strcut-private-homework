//
// Created by 刘凯源 on 24-10-13.
//

#ifndef CAMPUSNAVIGATION_ALGORITHM_H
#define CAMPUSNAVIGATION_ALGORITHM_H

#include "LGraph.h"

namespace Graph {
    namespace Algorithm {
        class DSU {//实现并查集操作
        private:
            std::vector<int> parent, rank;
        public:
            // 并查集构造函数，初始化n个元素
            explicit DSU(int n) : parent(n), rank(n, 0) {
                for (int i = 0; i < n; ++i) {
                    parent[i] = i; // 每个节点的初始父节点是其自身
                }
            }

            // 查找根节点并路径压缩
            int find(int x) {
                if (parent[x] != x) { // 如果x的父节点不是其自身
                    parent[x] = find(parent[x]); // 递归查找根节点，并路径压缩
                }
                return parent[x];
                // TODO: 查找元素的根节点，并完成路径压缩
            }

            // 合并两个集合
            void unite(int x, int y) {
                int rootX = find(x); // 找到x的根节点
                int rootY = find(y); // 找到y的根节点

                if (rootX != rootY) { // 如果两个节点不在同一个集合中
                    if (rank[rootX] < rank[rootY]) {
                        parent[rootX] = rootY; // 将较小的树连接到较大的树上
                    }
                    else if (rank[rootX] > rank[rootY]) {
                        parent[rootY] = rootX;
                    }
                    else {
                        parent[rootY] = rootX; // 任意连接，并增加根节点的rank
                        rank[rootX]++;
                    }
                }
                // TODO: 合并集合
            }

            // 检查两个节点是否属于同一集合
            bool same(int x, int y){
                int rootX = find(x); // 找到x的根节点
                int rootY = find(y); // 找到y的根节点
                if (rootX==rootY) {
                    return true;
                }
                return false; // TODO: 检查两个节点是否属于同一个集合
            }
        };

        std::list<Vertex> EulerCircle(const LGraph& graph); //计算欧拉回路
        bool HaveEulerCircle(const LGraph& graph); //判断是否存在欧拉回路
        bool IsConnected(LGraph& graph); //判断图是否联通
        int GetShortestPath(const LGraph& graph, const std::string& vertex_name_x,
        const std::string& vertex_name_y); //计算单源最短路径
        int TopologicalShortestPath(const LGraph& graph, std::vector<std::string> path); //计算拓扑受限的最短路径
        void SuggestEdges(LGraph& graph);//找出未联通的边
        std::vector<EdgeNode> MinimumSpanningTree(const LGraph& graph); //计算最小生成树
        std::vector<EdgeNode> Visitplan(const LGraph& graph, const std::string& from, const std::string& to,const int totaltime);
    }//找出一条旅游路径
}
#endif //CAMPUSNAVIGATION_ALGORITHM_H
