#pragma once
#include <Eigen/Dense>          // FTC 官方 SDK 已内置 Eigen 3.4
using Vec3  = Eigen::Vector3d;
using Mat33 = Eigen::Matrix3d;

class RobotEKF {
public:
    RobotEKF() {
        x_.setZero();               // 初始 pose
        P_ = Mat33::Identity() * 1.0; // 1 m 初始不确定度
    }

    /* 1. 预测：用底盘速度向前推模型
       vX     : 机器人坐标系前进速度  (m/s)
       omega  : 角速度                (rad/s)
       dt     : 采样周期              (s)   */
    void predict(double vX, double omega, double dt) {
        double theta = x_(2);
        // 线性化状态转移矩阵
        Mat33 F = Mat33::Identity();
        F(0,2) = -vX * sin(theta) * dt;
        F(1,2) =  vX * cos(theta) * dt;

        // 过程噪声  Q 可调
        Mat33 Q = Mat33::Zero();
        Q(0,0) = 0.02 * dt;   // x
        Q(1,1) = 0.02 * dt;   // y
        Q(2,2) = 0.005 * dt;  // theta

        // 预测状态
        x_(0) += vX * cos(theta) * dt;
        x_(1) += vX * sin(theta) * dt;
        x_(2) += omega * dt;

        // 预测协方差
        P_ = F * P_ * F.transpose() + Q;
    }

    /* 2. 更新：把视觉 pose 当绝对观测
       z     : 视觉给出的场地位姿 [x,y,θ]
       R_std : 3 元素向量，本次视觉的标准差 [x,y,θ]   */
    void update(const Vec3& z, const Vec3& R_std) {
        Mat33 R = R_std.cwiseProduct(R_std).asDiagonal(); // 协方差
        Mat33 H = Mat33::Identity();                      // 观测模型 h(x)=x
        Vec3  y = z - x_;                                 // 新息

        // 角度归一化到 [-pi,pi]
        y(2) = std::remainder(y(2), 2*M_PI);

        Mat33 S = H * P_ * H.transpose() + R;
        Mat33 K = P_ * H.transpose() * S.inverse();

        x_ += K * y;
        P_  = (Mat33::Identity() - K * H) * P_;
    }

    const Vec3& pose() const { return x_; }  // 返回最优估计
private:
    Vec3  x_;   // [x,y,θ]
    Mat33 P_;   // 误差协方差
};