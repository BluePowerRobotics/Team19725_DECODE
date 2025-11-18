package org.firstinspires.ftc.teamcode.utility;
import org.ejml.simple.SimpleMatrix;

public class RobotEKF {
    private SimpleMatrix x;   // [x, y, θ]
    private SimpleMatrix P;   // 误差协方差 3x3

    public RobotEKF() {
        // 初始 pose [x, y, θ]
        x = new SimpleMatrix(3, 1);

        // 初始协方差 1 m 不确定度
        P = SimpleMatrix.identity(3).scale(1.0);
    }

    /**
     * 1. 预测：用底盘速度向前推模型
     * @param vX 机器人坐标系前进速度 (m/s)
     * @param omega 角速度 (rad/s)
     * @param dt 采样周期 (s)
     */
    public void predict(double vX, double omega, double dt) {
        double theta = x.get(2);

        // 线性化状态转移矩阵
        SimpleMatrix F = SimpleMatrix.identity(3);
        F.set(0, 2, -vX * Math.sin(theta) * dt);
        F.set(1, 2, vX * Math.cos(theta) * dt);

        // 过程噪声 Q 可调
        SimpleMatrix Q = new SimpleMatrix(3, 3);
        Q.set(0, 0, 0.02 * dt);   // x
        Q.set(1, 1, 0.02 * dt);   // y
        Q.set(2, 2, 0.005 * dt);  // theta

        // 预测状态
        double newX = x.get(0) + vX * Math.cos(theta) * dt;
        double newY = x.get(1) + vX * Math.sin(theta) * dt;
        double newTheta = x.get(2) + omega * dt;

        x.set(0, newX);
        x.set(1, newY);
        x.set(2, newTheta);

        // 预测协方差: P = F * P * F^T + Q
        SimpleMatrix F_P = F.mult(P);
        SimpleMatrix F_P_FT = F_P.mult(F.transpose());
        P = F_P_FT.plus(Q);
    }

    /**
     * 2. 更新：把视觉 pose 当绝对观测
     * @param z 视觉给出的场地位姿 [x, y, θ]
     * @param R_std 3 元素向量，本次视觉的标准差 [x, y, θ]
     */
    public void update(double[] z, double[] R_std) {
        // 创建观测向量
        SimpleMatrix zMatrix = new SimpleMatrix(3, 1);
        zMatrix.set(0, z[0]);
        zMatrix.set(1, z[1]);
        zMatrix.set(2, z[2]);

        // 创建观测噪声协方差 R
        SimpleMatrix R = new SimpleMatrix(3, 3);
        for (int i = 0; i < 3; i++) {
            R.set(i, i, R_std[i] * R_std[i]);
        }

        // 观测模型 H = I
        SimpleMatrix H = SimpleMatrix.identity(3);

        // 新息: y = z - x
        SimpleMatrix y = zMatrix.minus(x);

        // 角度归一化到 [-pi, pi]
        double angle = y.get(2);
        y.set(2, remainder(angle, 2 * Math.PI));

        // 新息协方差: S = H * P * H^T + R
        SimpleMatrix H_P = H.mult(P);
        SimpleMatrix H_P_HT = H_P.mult(H.transpose());
        SimpleMatrix S = H_P_HT.plus(R);

        // 卡尔曼增益: K = P * H^T * S^-1
        SimpleMatrix P_HT = P.mult(H.transpose());
        SimpleMatrix K = P_HT.mult(S.invert());

        // 状态更新: x = x + K * y
        SimpleMatrix Ky = K.mult(y);
        x = x.plus(Ky);

        // 协方差更新: P = (I - K * H) * P
        SimpleMatrix I = SimpleMatrix.identity(3);
        SimpleMatrix KH = K.mult(H);
        SimpleMatrix I_KH = I.minus(KH);
        P = I_KH.mult(P);
    }

    /**
     * 返回最优估计位姿 [x, y, θ]
     */
    public double[] getPose() {
        return new double[]{x.get(0), x.get(1), x.get(2)};
    }

    /**
     * 手动设置位姿（用于初始化或重置）
     */
    public void setPose(double x, double y, double theta) {
        this.x.set(0, x);
        this.x.set(1, y);
        this.x.set(2, theta);
    }

    /**
     * 获取协方差矩阵（用于调试）
     */
    public SimpleMatrix getCovariance() {
        return P.copy();
    }

    /**
     * 实现类似 C++ std::remainder 的功能
     */
    private double remainder(double dividend, double divisor) {
        return dividend - Math.round(dividend / divisor) * divisor;
    }
}