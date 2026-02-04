package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive_Kalman;

/**
 * 卡尔曼融合定位测试OpMode，用于测试和验证基于卡尔曼滤波器的融合定位系统。
 * 
 * 此OpMode允许用户通过游戏手柄控制机器人移动，
 * 并实时显示和可视化三种不同定位系统的位置和姿态信息：
 * 1. 卡尔曼融合定位（KalmanFusionLocalizer）
 * 2. AprilTag视觉定位
 * 3. PinPoint轮式定位
 * 帮助调试和验证融合定位系统的准确性。
 */
@TeleOp
public class LocalizationTest_KalmanFusion extends LinearOpMode {
    /**
     * 起始X坐标
     */
    public static double startX = -64.8;
    
    /**
     * 起始Y坐标
     */
    public static double startY = 17.6;
    
    /**
     * 起始朝向（弧度）
     */
    public static double startHeading = 0;
    
    /**
     * 运行OpMode的主要方法。
     * 
     * 此方法初始化MecanumDrive_Kalman对象，
     * 然后进入主循环，处理游戏手柄输入，更新姿态估计，
     * 并显示和可视化三种不同定位系统的机器人状态。
     * 
     * @throws InterruptedException 如果线程被中断
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化多重遥测，同时输出到手机和FtcDashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // 设置起始姿态
        final Pose2d START_POSE = new Pose2d(startX, startY, startHeading);
        
        // 创建MecanumDrive_Kalman实例，使用卡尔曼融合定位
        MecanumDrive_Kalman drive_kalman = new MecanumDrive_Kalman(hardwareMap, START_POSE);
        
        // 等待开始信号
        waitForStart();

        // 主循环
        while (opModeIsActive()) {
            // 设置驱动功率，根据游戏手柄输入控制机器人移动
            drive_kalman.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,  // 前进/后退
                            -gamepad1.left_stick_x   // 左右平移
                    ),
                    -gamepad1.right_stick_x  // 旋转
            ));

            // 更新姿态估计
            drive_kalman.updatePoseEstimate();

            // 获取卡尔曼融合定位的姿态
            Pose2d pose_kalman = drive_kalman.localizer.getPose();
            // 显示卡尔曼融合定位的姿态信息
            telemetry.addData("kalmanx", pose_kalman.position.x);
            telemetry.addData("kalmany", pose_kalman.position.y);
            telemetry.addData("kalmanheading (deg)", Math.toDegrees(pose_kalman.heading.toDouble()));

            // 获取AprilTag视觉定位的姿态
            Pose2d pose_apriltag = drive_kalman.poseData.AprilTagPose;
            // 显示AprilTag视觉定位的姿态信息
            telemetry.addData("apriltagx", pose_apriltag.position.x);
            telemetry.addData("apriltagy", pose_apriltag.position.y);
            telemetry.addData("apriltagheading (deg)", Math.toDegrees(pose_apriltag.heading.toDouble()));

            // 获取PinPoint轮式定位的姿态
            Pose2d pose_wheel = drive_kalman.poseData.PinPointPose;
            // 显示PinPoint轮式定位的姿态信息
            telemetry.addData("x", pose_wheel.position.x);
            telemetry.addData("y", pose_wheel.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose_wheel.heading.toDouble()));
            telemetry.update();

            // 创建遥测数据包，用于在FtcDashboard上可视化机器人
            TelemetryPacket packet = new TelemetryPacket();
            
            // 绘制PinPoint轮式定位的机器人（蓝色）
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose_wheel);
            
            // 绘制卡尔曼融合定位的机器人（青色）
            packet.fieldOverlay().setStroke("#4E9999");
            Drawing.drawRobot(packet.fieldOverlay(), pose_kalman);
            
            // 绘制AprilTag视觉定位的机器人（绿色）
            packet.fieldOverlay().setStroke("#19725C");
            Drawing.drawRobot(packet.fieldOverlay(), pose_apriltag);
            
            // 发送遥测数据包
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
