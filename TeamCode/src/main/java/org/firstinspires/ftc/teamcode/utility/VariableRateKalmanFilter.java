package org.firstinspires.ftc.teamcode.utility;
import org.apache.commons.math3.linear.*;

public class VariableRateKalmanFilter {
    // 状态向量: [x, y, vx, vy, ax, ay]^T
    private RealVector state; // 6x1
    private RealMatrix covariance; // 6x6

    // 系统矩阵
    private RealMatrix stateTransitionMatrix; // 6x6
    private RealMatrix processNoiseCovariance; // 6x6
    private RealMatrix observationMatrix; // 4x6

    // 传感器配置
    private SensorConfig chassisConfig;
    private SensorConfig visionConfig;

    // 时间管理
    private double lastUpdateTime;
    private Double lastChassisTime;
    private Double lastVisionTime;

    // 视觉权重参数
    private double referenceDistance;
    private double maxTrustDistance;

    public VariableRateKalmanFilter(double referenceDistance, double maxTrustDistance) {
        this.referenceDistance = referenceDistance;
        this.maxTrustDistance = maxTrustDistance;

        initializeMatrices();
        initializeSensorConfigs();
    }

    private void initializeMatrices() {
        // 初始化状态向量 [x, y, vx, vy, ax, ay]
        state = new ArrayRealVector(6);

        // 初始化协方差矩阵
        covariance = MatrixUtils.createRealIdentityMatrix(6).scalarMultiply(1000);

        // 状态转移矩阵
        stateTransitionMatrix = MatrixUtils.createRealIdentityMatrix(6);

        // 过程噪声协方差矩阵
        processNoiseCovariance = MatrixUtils.createRealMatrix(new double[][] {
                {0.01, 0, 0, 0, 0, 0},
                {0, 0.01, 0, 0, 0, 0},
                {0, 0, 0.05, 0, 0, 0},
                {0, 0, 0, 0.05, 0, 0},
                {0, 0, 0, 0, 0.1, 0},
                {0, 0, 0, 0, 0, 0.1}
        });

        // 观测矩阵 - 观测位置和速度，不观测加速度
        observationMatrix = MatrixUtils.createRealMatrix(new double[][] {
                {1, 0, 0, 0, 0, 0},
                {0, 1, 0, 0, 0, 0},
                {0, 0, 1, 0, 0, 0},
                {0, 0, 0, 1, 0, 0}
        });

        lastUpdateTime = getCurrentTime();
    }

    private void initializeSensorConfigs() {
        // 底盘配置
        RealMatrix chassisNoise = MatrixUtils.createRealMatrix(new double[][] {
                {0.1, 0, 0, 0},
                {0, 0.1, 0, 0},
                {0, 0, 0.3, 0},
                {0, 0, 0, 0.3}
        });
        this.chassisConfig = new SensorConfig(chassisNoise, 1.0);

        // 视觉配置
        RealMatrix visionNoise = MatrixUtils.createRealMatrix(new double[][] {
                {0.05, 0, 0, 0},
                {0, 0.05, 0, 0},
                {0, 0, 0.2, 0},
                {0, 0, 0, 0.2}
        });
        this.visionConfig = new SensorConfig(visionNoise, 1.0);
    }

    /**
     * 根据时间间隔更新状态转移矩阵
     */
    private void updateStateTransitionMatrix(double deltaTime) {
        double dt = deltaTime;
        double dt2 = dt * dt / 2;

        stateTransitionMatrix = MatrixUtils.createRealMatrix(new double[][] {
                {1, 0, dt, 0, dt2, 0},
                {0, 1, 0, dt, 0, dt2},
                {0, 0, 1, 0, dt, 0},
                {0, 0, 0, 1, 0, dt},
                {0, 0, 0, 0, 1, 0},
                {0, 0, 0, 0, 0, 1}
        });
    }

    /**
     * 根据时间间隔更新过程噪声
     */
    private void updateProcessNoise(double deltaTime) {
        double dt = deltaTime;
        double positionNoise = 0.01 * (1 + dt * 10);
        double velocityNoise = 0.05 * (1 + dt * 5);
        double accelerationNoise = 0.1 * (1 + dt * 2);

        processNoiseCovariance = MatrixUtils.createRealMatrix(new double[][] {
                {positionNoise, 0, 0, 0, 0, 0},
                {0, positionNoise, 0, 0, 0, 0},
                {0, 0, velocityNoise, 0, 0, 0},
                {0, 0, 0, velocityNoise, 0, 0},
                {0, 0, 0, 0, accelerationNoise, 0},
                {0, 0, 0, 0, 0, accelerationNoise}
        });
    }

    /**
     * 预测步骤
     */
    public void predict(double deltaTime) {
        if (deltaTime <= 0) {
            return;
        }

        // 更新系统矩阵
        updateStateTransitionMatrix(deltaTime);
        updateProcessNoise(deltaTime);

        // 状态预测: x = F * x
        state = stateTransitionMatrix.operate(state);

        // 协方差预测: P = F * P * F^T + Q
        RealMatrix F = stateTransitionMatrix;
        RealMatrix FT = stateTransitionMatrix.transpose();
        covariance = F.multiply(covariance).multiply(FT).add(processNoiseCovariance);

        lastUpdateTime = getCurrentTime();
    }

    /**
     * 自动预测到当前时间
     */
    private void predictToCurrentTime() {
        double currentTime = getCurrentTime();
        double deltaTime = currentTime - lastUpdateTime;
        predict(deltaTime);
    }

    /**
     * 根据传感器数据时间戳进行预测
     */
    private void predictToTime(double sensorTime) {
        double deltaTime = sensorTime - lastUpdateTime;
        if (deltaTime > 0) {
            predict(deltaTime);
        }
    }

    /**
     * 更新步骤 - 底盘数据
     */
    public void updateChassis(double x, double y, double vx, double vy, double timestamp) {
        updateChassis(x, y, vx, vy, timestamp, 1.0);
    }

    public void updateChassis(double x, double y, double vx, double vy, double timestamp, double confidence) {
        predictToTime(timestamp);

        RealVector measurement = new ArrayRealVector(new double[]{x, y, vx, vy});
        double chassisWeight = chassisConfig.baseWeight * confidence;

        updateWithMeasurement(measurement, chassisConfig.noiseCovariance, chassisWeight);
        lastChassisTime = timestamp;
    }

    /**
     * 更新步骤 - 视觉数据
     */
    public void updateVision(double x, double y, double vx, double vy,
                             double timestamp, double distanceToReference) {
        updateVision(x, y, vx, vy, timestamp, distanceToReference, 1.0);
    }

    public void updateVision(double x, double y, double vx, double vy,
                             double timestamp, double distanceToReference, double confidence) {
        predictToTime(timestamp);

        RealVector measurement = new ArrayRealVector(new double[]{x, y, vx, vy});
        double distanceWeight = calculateVisionWeight(distanceToReference);
        double visionWeight = visionConfig.baseWeight * distanceWeight * confidence;

        updateWithMeasurement(measurement, visionConfig.noiseCovariance, visionWeight);
        lastVisionTime = timestamp;
    }

    /**
     * 根据距离计算视觉权重
     */
    private double calculateVisionWeight(double currentDistance) {
        if (currentDistance <= referenceDistance) {
            return 1.0;
        } else if (currentDistance >= maxTrustDistance) {
            return 0.1;
        } else {
            double normalizedDistance = (currentDistance - referenceDistance) /
                    (maxTrustDistance - referenceDistance);
            return Math.exp(-2 * normalizedDistance);
        }
    }

    /**
     * 通用的更新步骤
     */
    private void updateWithMeasurement(RealVector measurement, RealMatrix observationNoise, double weight) {
        if (weight <= 0.01) {
            return;
        }

        // 调整观测噪声基于权重
        RealMatrix adjustedObservationNoise = observationNoise.scalarMultiply(1.0 / Math.max(weight, 0.01));

        // 计算卡尔曼增益: K = P * H^T * (H * P * H^T + R)^-1
        RealMatrix H = observationMatrix;
        RealMatrix HT = observationMatrix.transpose();
        RealMatrix innovationCovariance = H.multiply(covariance).multiply(HT).add(adjustedObservationNoise);

        RealMatrix kalmanGain = covariance.multiply(HT)
                .multiply(new LUDecomposition(innovationCovariance).getSolver().getInverse());

        // 更新状态: x = x + K * (z - H * x)
        RealVector measurementResidual = measurement.subtract(H.operate(state));
        state = state.add(kalmanGain.operate(measurementResidual));

        // 更新协方差: P = (I - K * H) * P
        RealMatrix identity = MatrixUtils.createRealIdentityMatrix(6);
        RealMatrix KH = kalmanGain.multiply(H);
        covariance = identity.subtract(KH).multiply(covariance);
    }

    /**
     * 获取当前状态的预测结果
     */
    public PredictionResult getCurrentState() {
        predictToCurrentTime();
        return new PredictionResult(
                state.getEntry(0), // x
                state.getEntry(1), // y
                state.getEntry(2), // vx
                state.getEntry(3)  // vy
        );
    }

    /**
     * 预测未来某个时间点的状态
     */
    public PredictionResult predictFuture(double deltaTime) {
        if (deltaTime <= 0) {
            return getCurrentState();
        }

        // 保存当前状态
        RealVector savedState = state.copy();
        RealMatrix savedCovariance = covariance.copy();

        // 预测到未来时间
        predict(deltaTime);
        PredictionResult result = new PredictionResult(
                state.getEntry(0), state.getEntry(1), state.getEntry(2), state.getEntry(3)
        );

        // 恢复状态
        state = savedState;
        covariance = savedCovariance;

        return result;
    }

    /**
     * 获取传感器健康状态
     */
    public SensorHealth getSensorHealth() {
        double currentTime = getCurrentTime();
        boolean chassisHealthy = lastChassisTime != null && (currentTime - lastChassisTime) < 1.0;
        boolean visionHealthy = lastVisionTime != null && (currentTime - lastVisionTime) < 1.0;

        return new SensorHealth(chassisHealthy, visionHealthy, lastChassisTime, lastVisionTime);
    }

    private double getCurrentTime() {
        return System.currentTimeMillis() / 1000.0;
    }

    // Getters
    public double[] getState() {
        return state.toArray();
    }

    public void setState(double x, double y, double vx, double vy, double ax, double ay) {
        state.setEntry(0, x);
        state.setEntry(1, y);
        state.setEntry(2, vx);
        state.setEntry(3, vy);
        state.setEntry(4, ax);
        state.setEntry(5, ay);
    }

    /**
     * 传感器配置类
     */
    private static class SensorConfig {
        RealMatrix noiseCovariance;
        double baseWeight;

        SensorConfig(RealMatrix noiseCovariance, double baseWeight) {
            this.noiseCovariance = noiseCovariance;
            this.baseWeight = baseWeight;
        }
    }

    /**
     * 预测结果类
     */
    public static class PredictionResult {
        public final double x, y, velocityX, velocityY;

        public PredictionResult(double x, double y, double velocityX, double velocityY) {
            this.x = x;
            this.y = y;
            this.velocityX = velocityX;
            this.velocityY = velocityY;
        }

        @Override
        public String toString() {
            return String.format("Position: (%.3f, %.3f), Velocity: (%.3f, %.3f)",
                    x, y, velocityX, velocityY);
        }
    }

    /**
     * 传感器健康状态类
     */
    public static class SensorHealth {
        public final boolean chassisHealthy;
        public final boolean visionHealthy;
        public final Double lastChassisTime;
        public final Double lastVisionTime;

        public SensorHealth(boolean chassisHealthy, boolean visionHealthy,
                            Double lastChassisTime, Double lastVisionTime) {
            this.chassisHealthy = chassisHealthy;
            this.visionHealthy = visionHealthy;
            this.lastChassisTime = lastChassisTime;
            this.lastVisionTime = lastVisionTime;
        }
    }
}