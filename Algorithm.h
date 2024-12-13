//
// Created by ����Դ on 24-10-13.
//

#ifndef CAMPUSNAVIGATION_ALGORITHM_H
#define CAMPUSNAVIGATION_ALGORITHM_H

#include "LGraph.h"

namespace Graph {
    namespace Algorithm {
        class DSU {//ʵ�ֲ��鼯����
        private:
            std::vector<int> parent, rank;
        public:
            // ���鼯���캯������ʼ��n��Ԫ��
            explicit DSU(int n) : parent(n), rank(n, 0) {
                for (int i = 0; i < n; ++i) {
                    parent[i] = i; // ÿ���ڵ�ĳ�ʼ���ڵ���������
                }
            }

            // ���Ҹ��ڵ㲢·��ѹ��
            int find(int x) {
                if (parent[x] != x) { // ���x�ĸ��ڵ㲻��������
                    parent[x] = find(parent[x]); // �ݹ���Ҹ��ڵ㣬��·��ѹ��
                }
                return parent[x];
                // TODO: ����Ԫ�صĸ��ڵ㣬�����·��ѹ��
            }

            // �ϲ���������
            void unite(int x, int y) {
                int rootX = find(x); // �ҵ�x�ĸ��ڵ�
                int rootY = find(y); // �ҵ�y�ĸ��ڵ�

                if (rootX != rootY) { // ��������ڵ㲻��ͬһ��������
                    if (rank[rootX] < rank[rootY]) {
                        parent[rootX] = rootY; // ����С�������ӵ��ϴ������
                    }
                    else if (rank[rootX] > rank[rootY]) {
                        parent[rootY] = rootX;
                    }
                    else {
                        parent[rootY] = rootX; // �������ӣ������Ӹ��ڵ��rank
                        rank[rootX]++;
                    }
                }
                // TODO: �ϲ�����
            }

            // ��������ڵ��Ƿ�����ͬһ����
            bool same(int x, int y){
                int rootX = find(x); // �ҵ�x�ĸ��ڵ�
                int rootY = find(y); // �ҵ�y�ĸ��ڵ�
                if (rootX==rootY) {
                    return true;
                }
                return false; // TODO: ��������ڵ��Ƿ�����ͬһ������
            }
        };

        std::list<Vertex> EulerCircle(const LGraph& graph); //����ŷ����·
        bool HaveEulerCircle(const LGraph& graph); //�ж��Ƿ����ŷ����·
        bool IsConnected(LGraph& graph); //�ж�ͼ�Ƿ���ͨ
        int GetShortestPath(const LGraph& graph, const std::string& vertex_name_x,
        const std::string& vertex_name_y); //���㵥Դ���·��
        int TopologicalShortestPath(const LGraph& graph, std::vector<std::string> path); //�����������޵����·��
        void SuggestEdges(LGraph& graph);//�ҳ�δ��ͨ�ı�
        std::vector<EdgeNode> MinimumSpanningTree(const LGraph& graph); //������С������
        std::vector<EdgeNode> Visitplan(const LGraph& graph, const std::string& from, const std::string& to,const int totaltime);
    }//�ҳ�һ������·��
}
#endif //CAMPUSNAVIGATION_ALGORITHM_H
