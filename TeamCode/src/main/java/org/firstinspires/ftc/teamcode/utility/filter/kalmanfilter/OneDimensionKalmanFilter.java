package org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.jama.Matrix;

@Config
public class OneDimensionKalmanFilter {
    long lastUpdateTime = System.nanoTime();
    double lastEsPosition = 0.0;
    double lastEsVelocity = 0.0;
    double lastWheelPosition = 0.0;

    // F矩阵将在updateF()中设置
    Matrix F;

    // 状态协方差矩阵 - 初始不确定性较小
    Matrix P = new Matrix(new double[][]{
            {0.01, 0.0},   // 位置初始方差
            {0.0, 0.001}   // 速度初始方差
    });

    // 观测矩阵
    Matrix H = new Matrix(new double[][]{
            {1.0, 0.0}
    });

    // Dashboard可调参数
    public static double q_pos = 1e-4;  // 位置过程噪声
    public static double q_vel = 1e-6;  // 速度过程噪声
    public static double R = 0.01 * 0.01;  // 测量噪声

    // 过程噪声协方差矩阵
    Matrix Q = new Matrix(new double[][]{
            {q_pos, 0.0},
            {0.0, q_vel}
    });

    // 使用正确的匀速模型F
    private void updateF(double dt) {
        this.F = new Matrix(new double[][]{
                {1.0, dt},
                //todo 0.95可能也需要动态，原因详见与lby的聊天
                {0.0, 0.95}  // 轻微衰减，更鲁棒
        });
    }

    public OneDimensionKalmanFilter(double initialPosition, double initialVelocity) {
        lastEsPosition = initialPosition;
        lastEsVelocity = initialVelocity;
        lastWheelPosition = initialPosition;
        // 初始化为单位矩阵，第一次更新时会修正
        this.F = Matrix.identity(2, 2);
    }
    /// <summary>
    /// 更新卡尔曼滤波结果
    /// </summary>
    /// <param name="wheelPosition">随动轮当前位置</param>
    /// <param name="measurementPosition">视觉位置，如果没有输入 则请输入Double.NaN</param>
    /// <returns>新的估计位置</returns>
    public PosVelTuple Update(double wheelPosition, double measurementPosition) {
        long currentTime = System.nanoTime();
        double deltaTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        // === 关键修正1：正确的时间转换（纳秒→秒）===
        double dtSeconds = deltaTime / 1_000_000_000.0;  // 或者 1e9

        double deltaPosition = wheelPosition - lastWheelPosition;
        lastWheelPosition = wheelPosition;

        // === 关键修正2：使用正确的F矩阵 ===
        updateF(dtSeconds);

        Matrix X_=new Matrix(new double[][]{
                {lastEsPosition+deltaPosition},
                {deltaPosition/dtSeconds}
        });

        // === 关键修正4：动态更新Q矩阵的数值（因为q_pos和q_vel可能已通过Dashboard调整）===
        // 但是对模型本身没有影响，只是方便了dashboard热调参
        Q.set(0, 0, q_pos);
        Q.set(1, 1, q_vel);

        if (Double.isNaN(measurementPosition)) {
            // 没有视觉测量，只更新预测
            lastEsPosition = X_.get(0, 0);
            lastEsVelocity = X_.get(1, 0);
            // 注意：这里也需要更新协方差P！
            Matrix P_ = F.times(P).times(F.transpose()).plus(Q);
            P = P_;
            return new PosVelTuple(lastEsPosition, lastEsVelocity);
        }

        // 有视觉测量，进行完整的卡尔曼更新
        Matrix P_ = F.times(P).times(F.transpose()).plus(Q);
        Matrix K = P_.times(H.transpose()).times(1.0 /
                (H.times(P_).times(H.transpose()).get(0, 0) + R));

        Matrix X = X_.plus(K.times(
                new Matrix(new double[][]{{measurementPosition}})
                        .minus(H.times(X_))
        ));

        P = (Matrix.identity(2, 2).minus(K.times(H))).times(P_);

        lastEsPosition = X.get(0, 0);
        lastEsVelocity = X.get(1, 0);

        return new PosVelTuple(lastEsPosition, lastEsVelocity);
    }

    // 重置滤波器（在比赛开始时调用）
    public void reset(double position, double velocity) {
        lastEsPosition = position;
        lastEsVelocity = velocity;
        lastWheelPosition = position;
        lastUpdateTime = System.nanoTime();
        P = new Matrix(new double[][]{
                {0.01, 0.0},
                {0.0, 0.001}
        });
        this.F = Matrix.identity(2, 2);
    }
}