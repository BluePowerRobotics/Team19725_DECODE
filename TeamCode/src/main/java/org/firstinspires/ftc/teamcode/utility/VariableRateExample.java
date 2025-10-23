package org.firstinspires.ftc.teamcode.utility;

public class VariableRateExample {
    public static void main(String[] args) {
        // 初始化滤波器
        VariableRateKalmanFilter filter = new VariableRateKalmanFilter(2.0, 10.0);

        // 设置初始状态
        filter.setState(0, 0, 0.1, 0.1, 0, 0);

        double currentTime = System.currentTimeMillis() / 1000.0;

        // 模拟不固定帧率的传感器数据
        for (int i = 0; i < 100; i++) {
            currentTime = System.currentTimeMillis() / 1000.0;

            // 模拟不规则的传感器更新
            if (i % 3 == 0) { // 底盘大约33Hz
                double chassisX = 0.1 * i + Math.random() * 0.1;
                double chassisY = 0.1 * i + Math.random() * 0.1;
                double chassisVx = 0.1 + Math.random() * 0.05;
                double chassisVy = 0.1 + Math.random() * 0.05;

                filter.updateChassis(chassisX, chassisY, chassisVx, chassisVy, currentTime);
            }

            if (i % 5 == 0) { // 视觉大约20Hz
                double visionX = 0.1 * i + Math.random() * 0.05;
                double visionY = 0.1 * i + Math.random() * 0.05;
                double visionVx = 0.1 + Math.random() * 0.03;
                double visionVy = 0.1 + Math.random() * 0.03;
                double distanceToReference = 1.0 + i * 0.08;

                filter.updateVision(visionX, visionY, visionVx, visionVy,
                        currentTime, distanceToReference);
            }

            // 获取当前状态
            VariableRateKalmanFilter.PredictionResult result = filter.getCurrentState();
            VariableRateKalmanFilter.SensorHealth health = filter.getSensorHealth();

            System.out.printf("Frame %d: %s | Chassis: %s, Vision: %s%n",
                    i, result, health.chassisHealthy, health.visionHealthy);

            // 预测未来0.1秒的状态
            VariableRateKalmanFilter.PredictionResult future = filter.predictFuture(0.1);
            System.out.printf("Future prediction: %s%n", future);

            // 不固定的延迟
            try {
                Thread.sleep(15 + (int)(Math.random() * 10));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}