#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"
#include <cmath>
#include <numbers>


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            // 将局部偏移量扩展为4D齐次坐标
            glm::vec4 localOffset = glm::vec4(ik.JointLocalOffset[i], 1.0f);

            // 父关节的全局位置
            glm::vec4 parentGlobalPos = glm::vec4(ik.JointGlobalPosition[i - 1], 1.0f);

            // 通过父关节的全局旋转矩阵变换局部偏移
            localOffset = glm::mat4_cast(ik.JointGlobalRotation[i - 1]) * localOffset;

            // 更新全局位置
            //GlobalRotation(child) = GlobalRotation(parent)*LocalRotation(child)
            //GlobalPosition(child) = GlobalPosition(parent) + GlobalRotation(parent)*LocalOffset(child)
 
            ik.JointGlobalPosition[i] = glm::vec3(parentGlobalPos + localOffset);
            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i - 1] * ik.JointLocalRotation[i];

        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        int CCDIKIteration = 0;
        for (CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            //从最后一个关节开始遍历每个关节
            for(int i = ik.JointLocalOffset.size()-1; i >= 0; i--){
                glm::vec3 currentEndEffectorPos = ik.EndEffectorPosition();
                glm::vec3 jointGlobalPos = ik.JointGlobalPosition[i];

                //目标：让现在关节、最后一个关节endEffector、目标位置endPosition 共线
                //现在关节到最后一个关节的向量：
                glm::vec3 toEndEffector = glm::normalize(currentEndEffectorPos - jointGlobalPos);
                //现在关节到目标位置的向量：
                glm::vec3 toEndPosition = glm::normalize(EndPosition - jointGlobalPos);

                //让两个向量共线：计算所需旋转
                glm::quat rotationDelta = glm::rotation(toEndEffector, toEndPosition);
                //更新该关节的旋转：新增旋转*原旋转量
                ik.JointLocalRotation[i] = rotationDelta * ik.JointLocalRotation[i];

                //更新i关节后的关节
                ForwardKinematics(ik, i);
            } 
        }//ccd循环结束
        //std::cout<<"CCDIKIteration: "<<CCDIKIteration<<std::endl;
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        int IKIteration = 0;
        for (IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) { //从倒数第二个节点开始
                // your code here
                //方向：自身原本位置指向backward更新过的子节点
                glm::vec3 directionBackward = glm::normalize(backward_positions[i + 1] - ik.JointGlobalPosition[i]);
                backward_positions[i] = backward_positions[i + 1] - directionBackward * ik.JointOffsetLength[i+1];
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                //方向：自身原本位置（即上轮backward处理后的）指向forward处理后的父节点的位置
                glm::vec3 direction = glm::normalize(forward_positions[i] - backward_positions[i + 1]);
                forward_positions[i + 1] = forward_positions[i] - direction * ik.JointOffsetLength[i+1];

            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }//IKIteration结束
        //std::cout<<"IKIteration: "<<IKIteration<<std::endl;

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
/*        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index = 0;
        for (int i = 0; i < nums; i++) {
            float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
            float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
        }
        custom->resize(index);
        return custom;
*/
                /************** subtask 4: 自定义函数 ****************/
       //heart curve parameter: x= -sqr2 * (sint)^3, y = 2cost-(cost)^2-(cost)^3
        int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index = 0;
        for (int i = 0; i < nums; i++) {
            float t = i * 2 * std::numbers::pi / nums ; //范围0-2pi
            float sint = std::sin(t);
            float cost = std::cos(t);
            float x_val = -std::sqrt(2.0) * sint * sint * sint;
            float y_val = 2 * cost - cost * cost - cost * cost * cost;
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(1.0f - 0.5f * x_val, 0.0f, 0.5f * y_val + 0.5f );
        }
        custom->resize(index);
        return custom;

    }

/************************ spring mass *********************/
    //将一个 std::vector<glm::vec3> 类型的数据转换为 Eigen::VectorXf 类型
    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    //将一个 Eigen::VectorXf 类型的数据转换为 std::vector<glm::vec3> 类型
    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    //根据一组 Eigen::Triplet<float> 数据构建一个 Eigen 的稀疏矩阵
    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
    /*   //显示欧拉
        //1. 时间步初始化
        int const steps = 1000;
        float const ddt = dt / steps; 
        for (std::size_t s = 0; s < steps; s++) {
            //2. 力的初始化
            std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
            //3. 计算弹簧力和阻尼力
            for (auto const spring : system.Springs) {
                auto const p0 = spring.AdjIdx.first; //弹簧两端粒子的索引
                auto const p1 = spring.AdjIdx.second;
                glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];//位置差
                glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0]; //速度差
                glm::vec3 const e01 = glm::normalize(x01); //受力方向，即位置差方向
                //受力：弹性力=劲度系数*形变量*方向，阻尼力=阻尼系数*（v01*e01）e01
                glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
                forces[p0] += f;
                forces[p1] -= f;
            }
            //对每个点更新
            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                //跳过固定点
                if (system.Fixed[i]) continue; 
                //求a以更新v，求v以更新位置
                //f=重力+force
                //重力作用方向是沿着 Y 轴的负方向（X轴：水平向右，Y轴：垂直向上，Z轴：水平，指向屏幕外（或内）
                system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
                system.Positions[i] += system.Velocities[i] * ddt;
            }
        */ 
        //隐式欧拉
/*      //1. 时间步初始化
        int const steps = 1000;
        float h = dt / steps; 
        for (std::size_t s = 0; s < steps; s++) {
            int  n  = system.Positions.size();//粒子数

            //将三维坐标展成一维
            Eigen::VectorXf x0      = Eigen::VectorXf::Zero(3 * n);//创建坐标：创建一个3*n的矩阵，所有元素初始化为0
            Eigen::VectorXf v0      = Eigen::VectorXf::Zero(3 * n);//创建速度
            Eigen::VectorXf gravity = Eigen::VectorXf::Zero(3 * n);//创建重力

            for (int i = 0; i < 3 * n; i++) {
                x0[i] = system.Positions[i / 3][i % 3]; //初始化坐标
                v0[i] = system.Velocities[i / 3][i % 3]; //初始化速度
            }

            //重力：y轴
            for (int i = 0; i < n; ++i) {
                gravity[3 * i + 1] = -system.Gravity;
            }

            //表示y（不考虑阻尼，外力=重力）
            Eigen::VectorXf y = x0 + v0 * h + (gravity / system.Mass) * h * h;

            //根据牛顿迭代法，x1 = y
            Eigen::VectorXf x1 = y;

            //根据牛顿迭代法，g(x1) = eval Object（x1）
            float g1 = g(system, x1, y, h); 

            //牛顿迭代: for k = iter
            for (int k = 0; k < 3; k++) {
                //g(x)梯度 = eval Gradient(x)
                Eigen::VectorXf grad_g = nabla_g(system, x1, y, h);

                //g(x)梯度^2 = eval Hessian(x)
                Eigen::SparseMatrix<float> grad2_g(3 * n, 3 * n);
                nabla2_g(grad2_g, system, grad_g, x1, y, h);//grad2_g返回黑塞矩阵

                //dx = -g(x)梯度^2^-1 * g(x)梯度
                Eigen::VectorXf dx = -cal_inverse(grad2_g, n) * grad_g;

                Eigen::VectorXf x2 = Eigen::VectorXf::Zero(3 * n);

                float beta  = 0.95;
                float alpha = 1.0f / beta;
                float gamma = 1e-3;
                float g2    = 0;

                //repeat部分
                while (g2 > g1 + gamma * alpha * (grad_g.transpose() * dx)[0] + 1e-4){
                    alpha = beta * alpha;
                    x2    = x1 + dx * alpha;
                    g2    = g(system, x2, y, h);
                }

                x1 = x2;
                g1 = g2;
            }
            
            //v = (xn+1 -xn)/h
            Eigen::VectorXf v1 = (x1 - x0) / h;
            for (int i = 0; i < n; i++) {
                if (system.Fixed[i]) continue;
                system.Positions[i]  = { x1[i * 3], x1[i * 3 + 1], x1[i * 3 + 2] };
                system.Velocities[i] = { v1[i * 3], v1[i * 3 + 1], v1[i * 3 + 2] };
            }
         }
         */ 

        //1. 时间步初始化
        int const steps = 100;
        float h = dt / steps; 
        for (std::size_t s = 0; s < steps; s++) {
            int  n  = system.Positions.size();//粒子数

            //构造X矩阵
            //Eigen::VectorXf x0      = Eigen::VectorXf::Zero(3 * n);//创建坐标：创建一个3*n的矩阵，所有元素初始化为0
            //Eigen::VectorXf v0      = Eigen::VectorXf::Zero(3 * n);//创建速度
            
            /*for (int i = 0; i < 3 * n; i++) {
                x0[i] = system.Positions[i / 3][i % 3]; //初始化坐标
                v0[i] = system.Velocities[i / 3][i % 3]; //初始化速度
            }
*/          
            //构造X矩阵
            Eigen::VectorXf x0 = glm2eigen(system.Positions);
            Eigen::VectorXf v0 = glm2eigen(system.Velocities);

            //重力：y轴
            Eigen::VectorXf gravity = Eigen::VectorXf::Zero(3 * n);//创建重力
            for (int i = 0; i < n; ++i) 
                gravity[3 * i + 1] = -system.Gravity;

            //表示y（不考虑阻尼，外力=重力）
            Eigen::VectorXf y = x0 + v0 * h + (gravity / system.Mass) * h * h;

            //求Hg(x)
            Eigen::SparseMatrix<float> grad2_g(3 * n, 3 * n);
            std::vector<Eigen::Triplet<float>> coefficients;//存储triplets
            //质量矩阵的对角项:M/h^2
            for (int i = 0; i < 3 * n; i++)
                coefficients.push_back(Eigen::Triplet(i, i, system.Mass / (h * h)));
            //第二项：H(xk)
            for (auto const spring : system.Springs) {
                int p0 = spring.AdjIdx.first;
                int p1 = spring.AdjIdx.second;

                glm::vec3       pos1 = glm::vec3(x0[p1 * 3], x0[p1 * 3 + 1], x0[p1 * 3 + 2]);
                glm::vec3       pos0 = glm::vec3(x0[p0 * 3], x0[p0 * 3 + 1], x0[p0 * 3 + 2]);
                glm::vec3 const x01  = pos1 - pos0; 
                float length = glm::length(x01); //长度
                //计算Hessian
                glm::mat3 H(0.0f);
                for (int i = 0; i < 3; i++)
                    H[i][i] = system.Stiffness * (1.0f - spring.RestLength / length);
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                         H[i][j] += system.Stiffness * spring.RestLength * x01[i] * x01[j] / (length * length * length);
                //放进稀疏矩阵
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        if (glm::abs(H[i][j]) > 1e-5f) {
                            coefficients.push_back(Eigen::Triplet(p0 * 3 + i, p1 * 3 + j, -H[i][j]));
                            coefficients.push_back(Eigen::Triplet(p1 * 3 + i, p0 * 3 + j, -H[i][j]));
                            coefficients.push_back(Eigen::Triplet(p0 * 3 + i, p0 * 3 + j, H[i][j]));
                            coefficients.push_back(Eigen::Triplet(p1 * 3 + i, p1 * 3 + j, H[i][j]));
                        }
            }
            grad2_g = CreateEigenSparseMatrix(3*n, coefficients);

            //g(x)梯度 = eval Gradient(x)
            //第一项：(x-y)M/h^2
            Eigen::VectorXf grad_g = (x0 - y) * system.Mass / (h * h); 
            //第二项：弹性势能梯度 k*形变*e
            for (auto const spring : system.Springs) {
                auto const      p0   = spring.AdjIdx.first;
                auto const      p1   = spring.AdjIdx.second;
                glm::vec3       pos1 = glm::vec3(x0[p1 * 3], x0[p1 * 3 + 1], x0[p1 * 3 + 2]);
                glm::vec3       pos0 = glm::vec3(x0[p0 * 3], x0[p0 * 3 + 1], x0[p0 * 3 + 2]);
                glm::vec3 const x01  = pos1 - pos0;
                glm::vec3 const e01  = glm::normalize(x01);                                             //单位向量
                glm::vec3       f    = system.Stiffness * (glm::length(x01) - spring.RestLength) * e01; // f=kx
                for (int i = 0; i < 3; i++) {
                    grad_g[p0 * 3 + i] -= f[i];
                    grad_g[p1 * 3 + i] += f[i];
                }
            }

            //解方程：Hg(x) * （xk+1-xk) = -g(x)梯度
            Eigen::VectorXf x1 = ComputeSimplicialLLT(grad2_g, -grad_g);
            x1 += x0;

            //v = (xn+1 -xn)/h
            Eigen::VectorXf v1 = (x1 - x0) / h;
            for (int i = 0; i < n; i++) {
                if (system.Fixed[i]) continue;
                system.Positions[i]  = { x1[i * 3], x1[i * 3 + 1], x1[i * 3 + 2] };
                system.Velocities[i] = { v1[i * 3], v1[i * 3 + 1], v1[i * 3 + 2] };
            }

            //为什么这样没反应@@
/*          for (int i = 0; i < n; i++) {
                if (system.Fixed[i]) {
                    x1[i*3] = system.Positions[i].x;
                    x1[i*3+1] = system.Positions[i].y;
                    x1[i*3+2] = system.Positions[i].z;
                }
            }
            system.Positions = eigen2glm(x1);
            system.Velocities = eigen2glm(v1);
*/
        }//每个时间步循环结束  
    }//Void AdvanceMassSpringSystem
}
