#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();
                // your code here:
                int n = v->Neighbors().size();
                float u = (n == 3) ? 3.0f / 16.0f : 3.0f / (8.0f * n);
                glm::vec3 updated_v = (1 - n * u ) * prev_mesh.Positions[i];
                for(int j=0;j < neighbors.size();j++){
                    updated_v += prev_mesh.Positions[neighbors[j]] * u;
                }
                curr_mesh.Positions.push_back(updated_v);

            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                       = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 p1 = prev_mesh.Positions[e->From()];
                    glm::vec3 p2 = prev_mesh.Positions[e->To()];
                    glm::vec3 p_new = (p1 + p2)/2.0f;
                    curr_mesh.Positions.push_back(p_new);

                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 p1 = prev_mesh.Positions[e->From()];
                    glm::vec3 p2 = prev_mesh.Positions[e->To()];
                    glm::vec3 p3 = prev_mesh.Positions[e->OppositeVertex()];
                    glm::vec3 p4 = prev_mesh.Positions[e->TwinOppositeVertex()];
                    glm::vec3 p_new = (p1 + p2) * 3.0f/8.0f + (p3 + p4) * 1.0f/8.0f;
                    curr_mesh.Positions.push_back(p_new);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    {v0, m2, m1},{m2, v1, m0},{m1, m0, v2},{m0, m1, m2}
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input; 
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });
        //输出的 output.TexCoords 应该保存每个顶点的 UV 坐标

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        //找一个顶点作为起始
        int n = input.Positions.size();
        int start = 0; //起始点
        for(int i = 0; i < n; i++){
            DCEL::VertexProxy const * v = G.Vertex(i);
            if (v->OnBoundary()) {
                start = i;
                break;
            }
        }
        //从start开始绕一圈，顺着找
        int last = -1;
        int current = start;
        std::vector<int> boundary;
        boundary.push_back(start);
        while (last == -1 || current != start) {
            DCEL::VertexProxy const * v = G.Vertex(current);
            if (last == -1 || v->BoundaryNeighbors().first == last) {
                last = current;
                current = v->BoundaryNeighbors().second;
            } else {
                last = current;
                current = v->BoundaryNeighbors().first;
            }
            boundary.push_back(current);
        }
        int boundary_num = boundary.size();
        int vertex_num   = input.Positions.size();
        std::vector<glm::vec2> uv;
        for (int i = 0; i < vertex_num; ++i) {
            uv.push_back(glm::vec2(0, 0)); // uv坐标初始化
        }
        float t = 1.0 / boundary_num; //平均权重
        float pi = acos(-1);
        for (int i = 0; i < boundary_num; ++i) {
            uv[boundary[i]].x = 0.5 + 0.5 * cos(2 * pi * t * i); //圆，边界在[0,1]之间
            uv[boundary[i]].y = 0.5 + 0.5 * sin(2 * pi * t * i);
        }
        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            std::vector<glm::vec2> tmp;
            for (int i = 0; i < vertex_num; ++i) {
                tmp.push_back(uv[i]); //全部复制到tmp中，在tmp中处理
            }
            for (int i = 0; i < vertex_num; ++i) {
                DCEL::VertexProxy const * v = G.Vertex(i); // get vertex with index i

                if (v->OnBoundary()) continue; //检查边界点
                std::vector<uint32_t> Neighbor  = v->Neighbors();
                int  neighbor_size = v->Neighbors().size();
                float lambda = 1.0 / neighbor_size; //简单起见可以使用 入ij=1/ni 的平均权重
                uv[i]  = glm::vec2(0, 0);
                for (int j = 0; j < neighbor_size; j++)
                    uv[i] += lambda * tmp[Neighbor[j]];
            }
        }

        //输出的 output Mesh 的 TexCoords 应该保存每个顶点的 UV 坐标
        output = input;
        for (int i = 0; i < n; i++) {
            output.TexCoords.push_back(uv[i]);
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Kp matrix of the face f.
        auto UpdateQ {
            [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Kp;
                // your code here:
                //三角形f的三个顶点为abc，求坐标
                glm::vec3 vertex_a = output.Positions[f->VertexIndex(0)];
                glm::vec3 vertex_b = output.Positions[f->VertexIndex(1)];
                glm::vec3 vertex_c = output.Positions[f->VertexIndex(2)];

                // 计算法向量 n = (a - b) x (a - c)
                glm::vec3 n = glm::normalize(glm::cross(vertex_b - vertex_a, vertex_c - vertex_a));

                //过abc的平面方程为ax+by+cz+d=0,且满足a^2+b^2+c^2=1
                float a = n.x;
                float b = n.y;
                float c = n.z;
                // 计算平面方程中的 d 值，d = -n · a
                float d = -glm::dot(n, vertex_a);

                float p[4] = { a, b, c, d };
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        Kp[i][j] = p[i] * p[j];
                    }
                }
                return Kp;
            }
        };

        // The struct to record contraction info.
        struct ContractionPair {
            DCEL::HalfEdge const * edge;            // which edge to contract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ContractionPair struct.
        static constexpr auto MakePair {
            [] (DCEL::HalfEdge const * edge,
                glm::vec3 const & p1,
                glm::vec3 const & p2,
                glm::mat4 const & Q
            ) -> ContractionPair {
                // your code here:
                //判断edge是否有效
                if (edge == nullptr) {
                    //return什么？？
                    return ContractionPair{nullptr, glm::vec4(0), std::numeric_limits<float>::infinity()};
                }

                //计算最优收缩点 构建Q2
                glm::mat4 Q2 = Q;
                for (int j = 0; j < 3; ++j) {
                    Q2[3][j] = 0;
                }
                Q2[3][3] = 1;

                glm::vec4 targetPosition;
                float cost;
                if (std::fabs(glm::determinant(Q2)) > 0.001) {
                    //Q2可逆：最优收缩点为逆矩阵第4列
                    Q2 = glm::inverse(Q2);
                    targetPosition = glm::vec4(Q2[0][3], Q2[1][3], Q2[2][3], Q2[3][3]);
                    cost = glm::dot(targetPosition, Q * targetPosition); //cost=v^T Q v
                }
                else{
                    //Q2不可逆：取中点
                    glm::vec4 position1 = glm::vec4(p1.x, p1.y, p1.z, 1);
                    glm::vec4 position2 = glm::vec4(p2.x, p2.y, p2.z, 1);
                    targetPosition = glm::vec4((p1.x+p2.x)/2.0f, (p1.y+p2.y)/2.0f,(p1.z+p2.z)/2.0f, 1);
                    cost = glm::dot(targetPosition, Q * targetPosition); //cost=v^T Q v
                   /* float cost1 = glm::dot(position1, Q * position1); 
                    float cost2 = glm::dot(position2, Q * position2); 
                    if (cost1 < cost) {
                        targetPosition = position1;
                        cost = cost1;
                    }
                    if (cost2 < cost){
                        targetPosition = position2;
                        cost = cost2;
                    }
                */
                }

                return ContractionPair{edge, targetPosition, cost};
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ContractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Kf:       $Kf[idx]$ is the Kp matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map; 
        std::vector<ContractionPair>                  pairs; 
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Kf(G.NumOfFaces(),    glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q                 = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Kf[G.IndexOf(f)]       = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the contractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsContractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the contractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsContractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the contractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the contract result
            // ring:   the edge ring of vertex v1
            ContractionPair & top    = pairs[min_idx];
            auto               v1     = top.edge->From();
            auto               v2     = top.edge->To();
            auto               result = G.Contract(top.edge); //对边 e 进行坍缩操作，删除 e->To() 并更新 e->From() 的拓扑关系
            auto               ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The contraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Kf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Kf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Kp matrix for $e->Face()$.
                glm::mat4 Kp_new = UpdateQ(e->Face());
                //     2. According to the difference between the old Kp (in $Kf$) and the new Kp (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                glm::mat4 Kp_diff = Kp_new - Kf[G.IndexOf(e->Face())];
                Qv[e->From()] += Kp_diff;
                Qv[e->To()] += Kp_diff;
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                Qv[v1] += Kp_new;
                //     4. Update $Kf$.
                Kf[G.IndexOf(e->Face())] = Kp_new;
            }

            // Finally, as the Q matrix changed, we should update the relative $ContractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:
            
            //Changed_vertex：所有Q改变的点的index
            std::vector<uint32_t> Changed_vertex = G.Vertex(v1)->Neighbors();
            Changed_vertex.push_back(v1);

            //遍历pairs，寻找以Changed_vertex中的点为顶点的pairs
            for(int i = 0; i < pairs.size(); i++){
                if (pairs[i].edge == nullptr) continue;

                for(int j = 0; j < Changed_vertex.size(); j++){
                    if(pairs[i].edge->To()  == Changed_vertex[j] || pairs[i].edge->From() == Changed_vertex[j]){
                        uint32_t p1_index = pairs[i].edge->From();
                        uint32_t p2_index = pairs[i].edge->To();
                        glm::vec3 p1_pos = output.Positions[p1_index];
                        glm::vec3 p2_pos = output.Positions[p2_index];
                        glm::mat4 Q_updated = Qv[p1_index] + Qv[p2_index];
                        DCEL::HalfEdge const * e = pairs[i].edge;
                        pairs[i] = MakePair(e, p1_pos, p2_pos, Q_updated);
                        //break;
                    }
                }
            }

/*
            for (auto e : ring) {
                auto v1 = e->From();
                auto v2 = e->To();

                // 检查是否有 ContractionPair 关联到这些顶点
                auto edgeIdx = G.IndexOf(e);
                if (pair_map.find(edgeIdx) != pair_map.end()) {
                   std::size_t pairIdx = pair_map[edgeIdx];
                   if (pairs[pairIdx].edge != nullptr) {
                       // 获取顶点 v1 和 v2 的位置
                        glm::vec3 p1 = output.Positions[v1];
                        glm::vec3 p2 = output.Positions[v2];
                        // 重新生成收缩对，因为 Q 矩阵已更新
                        glm::mat4 Q_combined = Qv[v1] + Qv[v2];
                        pairs[pairIdx] = MakePair(e, p1, p2, Q_combined);
                    }
                }
            }*/
        }


        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                //已知3顶点，求两边向量
                //glm::vec3 v1Angle = glm::normalize(v1 - vAngle);
                //glm::vec3 v2Angle = glm::normalize(v2 - vAngle);
                glm::vec3 v1Angle = v1 - vAngle;
                glm::vec3 v2Angle = v2 - vAngle;

                // 计算夹角的余切
                float dotProduct = glm::dot(v1Angle, v2Angle); // 计算点积
                float crossProductLength = glm::length(glm::cross(v1Angle, v2Angle)); // 计算叉积的长度

                // 余切 = dot / cross 的长度
                //求解余切值时注意对异常余切值的处理（极大、极小）并检查任何可能除以零的操作以保证数值稳定
                crossProductLength = std::max(crossProductLength, 1e-6f);
                
                float cotangent = dotProduct / crossProductLength;

                // 进一步限制cotangent值的范围，防止数值过大或过小影响计算
                cotangent = std::clamp(cotangent, -900.0f, 900.0f);

                
                if (std::isnan(cotangent)) {
                std::cout << "Warning: Computation resulted in NaN" << std::endl;
                cotangent = 0.0f; // 或者其他合适的默认值
                }

                return cotangent;
                
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        //记录 from to这条边对应的边 由点索引边
        //from to 这条边是(unsigned long long) e->From() * oldv_cnt + e->To())，类似二维数组
        int vertex_num = input.Positions.size();
        std::map<unsigned long long, DCEL::HalfEdge  const *> point_to_edge; 
        for (DCEL::HalfEdge const * e : G.Edges()) {
            point_to_edge[(unsigned long long) e->From() * vertex_num + e->To()] = e;
            if (e->From() == 0 && e->To() == 1) {
                DCEL::HalfEdge const * tmp_e = e;
            }
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;

         
            //对每一个顶点
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                glm::vec3 vertex_sum = glm::vec3(0,0,0);
                float w_sum = 0.0f;
                float w;

                // your code here: curr_mesh.Positions[i] = ...
                DCEL::VertexProxy const * v   = G.Vertex(i);
                glm::vec3 v_vec = prev_mesh.Positions[i];

   
                //对每一个相邻的点
                std::vector<uint32_t> Neighbor = v->Neighbors();//相邻点索引
                for (int j = 0; j < Neighbor.size(); ++j) {
                    if (useUniformWeight) {
                        w = 1;
                    } else {//使用 Cotangent Laplacian 时 wij=cotaij+cotbij
                        //找vertexi,j之间的边（如果没有就反向）
                        DCEL::HalfEdge const * e=point_to_edge[(unsigned long long)i * vertex_num + Neighbor[j]];
                        if (!e)//如果没有(i,Neighbor[j]）这条边就找它的pair
                        e = point_to_edge[(unsigned long long) Neighbor[j] * vertex_num + i];

                        //找到四个顶点坐标
                        glm::vec3 v0 = prev_mesh.Positions[e->From()];
                        glm::vec3 v1 = prev_mesh.Positions[e->OppositeVertex()];
                        glm::vec3 v2 = prev_mesh.Positions[e->To()];
                        glm::vec3 v3 = prev_mesh.Positions[e->TwinOppositeVertex()];
                        float w = GetCotangent(v0, v1, v2) + GetCotangent(v0, v3, v2);
                        w = std::max(std::min(w, 900.0f), 0.0f);//w大了有洞
    
                    }

                    w_sum += w;
                    vertex_sum += w * prev_mesh.Positions[Neighbor[j]];
                }

                    glm::vec3 v_new = vertex_sum / w_sum;
                    curr_mesh.Positions[i] = (1 - lambda) * v_vec + lambda * v_new;
                
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/
    //为了用std::map映射点坐标与序号建立struct
    struct Vec3 {   
    float x, y, z;

    // 重载 < 运算符，用于在 std::map 中进行比较
    bool operator<(const Vec3& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
        }
    };

    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        //这三个参数定义了以 grid_min 为起始点、每个格子长度为 dx ，XYZ 三个方向都向正方向延伸 n 个格子的背景网格
        // your code here:

        //为了判断点有没有重复生成建立映射
        std::map<Vec3, int> point_to_index;

        //遍历每个cube
        for(int x = 0; x < n; x++ ){
            for(int y = 0; y < n; y++){
                for(int z = 0; z < n; z++){
                    //对于每个cube：从v0开始判断每个顶点正负
                    //v0位置
                    float x0 = grid_min.x + dx * x;
                    float y0 = grid_min.y + dx * y;
                    float z0 = grid_min.z + dx * z;
                    glm::vec3 v0 = {x0, y0, z0};

                    //1. 为网格结构的边建立查询其上有无 mesh 顶点的数据结构
                    //首先需要计算一个 8 位的二进制数 v ，第 i 位为 1 或 0 分别表示在立方体的第 i 个顶点处 sdf 为正或负； 
                    uint32_t point_state = 0;

                    for(int i = 0; i < 8; i++){
                        //假设立方体 v0 顶点的位置为 (x0,y0,z0) ，则第 i 个顶点 vi 的位置为：
                        glm::vec3 vi = glm::vec3(x0 + (i & 1) * dx, y0 + ((i >> 1) & 1) * dx, z0 + (i >> 2) * dx);
                        if (sdf(vi) <= 0) {
                            point_state |= (1 << i);
                        }
                    }

                    //2. 逐网格判断哪些边上有 mesh 的顶点；
                    //c_EdgeStateTable[v] 输出一个 12 位的二进制数，第 i 位表示立方体的第 i 条边上有无 mesh 的顶点；
                    uint32_t edge_state = c_EdgeStateTable[point_state];
                    
                    //记录这个cube上第j条边上生成的顶点的全局序号
                    int edge_vertex[12];
                    //遍历每一条边->如果有顶点，则线性插值求顶点坐标
                    for(int j = 0; j < 12; j++){
                        //如果有顶点
                        if (edge_state & (1 << j)) {
                            //求该边的起始点和终点坐标
                            //第 j 条边 ej 的方向为 unit(j >> 2) ，其中 unit() 表示第几个方向的基矢， 
                            glm::vec3 unit[3] = { glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1) };
                            //第 j 条边的起始点为 ：v0 + dx * (j & 1) * unit(((j >> 2) + 1) % 3) + dx * ((j >> 1) & 1) * unit(((j >> 2) + 2) % 3)
                            glm::vec3 start = v0 + dx * (j & 1) * unit[((j >> 2) + 1) % 3] + dx * ((j >> 1) & 1) * unit[((j >> 2) + 2) % 3];
                            glm::vec3 end = start + unit[j >> 2] * dx;

                            //线性插值计算交点坐标：P = P1 + (V – V1)·(P2 – P1)/(V2 – V1) 1start,2end
                            glm::vec3 p = start + (0 - sdf(start)) * (end - start)/(sdf(end) - sdf(start));

                            
                            //*判断这个点有没有生成过
                            int p_index = 0; //p在index里的序号
                            //如果生成过了：
                            if (point_to_index.count(Vec3(p.x,p.y,p.z))) {
                                p_index  = point_to_index[Vec3(p.x, p.y, p.z)];
                                edge_vertex[j] = p_index;
                            } 
                            else {
                                output.Positions.push_back(p);
                                p_index = output.Positions.size() - 1; //p点序号为在output.Positions里的顺序
                                point_to_index[Vec3(p.x, p.y, p.z)] = p_index;
                                edge_vertex[j] = p_index;
                            }
                        }
                    }

                    //3. 逐网格连接相应边上 mesh 的顶点，组成三角 mesh
                    //c_EdgeOrdsTable[point_state][j]：point_state->第j个三角形的三条边的cube内序号
                    //->通过edge_vertex[j]找到它们的全局序号
                    for (int j = 0; c_EdgeOrdsTable[point_state][j] != -1; j++) {
                        output.Indices.push_back(edge_vertex[c_EdgeOrdsTable[point_state][j]]);
                    }
                }
            }
        }
        
         
        
    }
} // namespace VCX::Labs::GeometryProcessing
