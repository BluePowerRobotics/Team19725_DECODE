package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.VariableRateKalmanFilter;

@Config
@TeleOp
@Disabled
public class VariableRateExample  extends LinearOpMode {
    public static double vx = 1;
    public static double vy = 1;
    public static double kchassispos = 0.1;
    public static double kchassisvel = 0.05;
    public static double kvisionpos = 0.05;
    public static double kvisionvel = 0.025;
    private double chassisVx;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 初始化滤波器
        VariableRateKalmanFilter filter = new VariableRateKalmanFilter(2.0, 10.0);


        // 设置初始状态
        filter.setState(0, 0, vx, vy, 0, 0);
        waitForStart();

        double currentTime = System.currentTimeMillis() / 1000.0;

        // 模拟不固定帧率的传感器数据
        for (int i = 0; i < 10000; i++) {
            currentTime = System.currentTimeMillis() / 1000.0;

            // 模拟不规则的传感器更新
            if (i % 5 == 0) { // 底盘大约20Hz
                double chassisX = 0.1 * i + kchassispos;
                double chassisY = 0.1 * i + kchassispos;
                double chassisVx = 0.1 + kchassisvel;
                double chassisVy = 0.1 + kchassisvel;
                telemetry.addData("chassisX", chassisX);
                telemetry.addData("chassisY", chassisY);
                filter.updateChassis(chassisX, chassisY, chassisVx, chassisVy, currentTime);
            }

            if (i % 5 == 0) { // 视觉大约20Hz
                double visionX = 0.1 * i + kvisionpos;
                double visionY = 0.1 * i + kvisionpos;
                double visionVx = 0.1 + kvisionvel;
                double visionVy = 0.1 + kvisionvel;
                double distanceToReference = 10000000 + 1.0 + i * 0.08;
                telemetry.addData("VisionX", visionX);
                telemetry.addData("VisionY", visionY);
                filter.updateVision(visionX, visionY, visionVx, visionVy,
                        currentTime, distanceToReference);
            }

            // 获取当前状态
            VariableRateKalmanFilter.PredictionResult result = filter.getCurrentState();
            VariableRateKalmanFilter.SensorHealth health = filter.getSensorHealth();

//            System.out.printf("Frame %d: %s | Chassis: %s, Vision: %s%n",
//                    i, result, health.chassisHealthy, health.visionHealthy);

            // 预测未来0.1秒的状态
            VariableRateKalmanFilter.PredictionResult future = filter.predictFuture(0.1);
            //System.out.printf("Future prediction: %s%n", future);
            telemetry.addData("future x", future.x);
            telemetry.addData("future y", future.y);
            telemetry.addData("future velx", future.velocityX);
            telemetry.addData("future velY", future.velocityY);
            telemetry.update();

            // 不固定的延迟
            sleep(20 + (int)(Math.random() * 10));
        }
    }
}