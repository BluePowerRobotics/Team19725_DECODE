package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;

/**
 * 定位测试OpMode，用于测试和验证机器人的定位系统。
 * 
 * 此OpMode允许用户通过游戏手柄控制机器人移动，
 * 并实时显示和可视化机器人的位置和姿态信息，
 * 帮助调试和验证定位系统的准确性。
 */
public class LocalizationTest extends LinearOpMode {
    /**
     * 运行OpMode的主要方法。
     * 
     * 此方法根据配置的驱动类型（MecanumDrive或TankDrive）初始化相应的驱动对象，
     * 然后进入主循环，处理游戏手柄输入，更新姿态估计，并显示和可视化机器人状态。
     * 
     * @throws InterruptedException 如果线程被中断
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化多重遥测，同时输出到手机和FtcDashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // 初始化实例遥测
        telemetry = InstanceTelemetry.init(telemetry);
        // 设置起始姿态
        final Pose2d START_POSE = new Pose2d(-64.8, 17.6, 0);
        
        // 根据配置的驱动类型创建相应的驱动对象
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            // 创建MecanumDrive实例
            MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

            // 等待开始信号
            waitForStart();

            // 主循环
            while (opModeIsActive()) {
                // 设置驱动功率，根据游戏手柄输入控制机器人移动
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,  // 前进/后退
                                -gamepad1.left_stick_x   // 左右平移
                        ),
                        -gamepad1.right_stick_x  // 旋转
                ));

                // 更新姿态估计
                drive.updatePoseEstimate();

                // 获取当前姿态
                Pose2d pose = drive.localizer.getPose();
                // 显示姿态信息
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                // 创建遥测数据包，用于在FtcDashboard上可视化机器人
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                // 绘制机器人
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                // 发送遥测数据包
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            // 创建TankDrive实例
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            // 等待开始信号
            waitForStart();

            // 主循环
            while (opModeIsActive()) {
                // 设置驱动功率，根据游戏手柄输入控制机器人移动
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,  // 前进/后退
                                0.0                     // 坦克驱动不支持左右平移
                        ),
                        -gamepad1.right_stick_x  // 旋转
                ));

                // 更新姿态估计
                drive.updatePoseEstimate();

                // 获取当前姿态
                Pose2d pose = drive.localizer.getPose();
                // 显示姿态信息
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                // 创建遥测数据包，用于在FtcDashboard上可视化机器人
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                // 绘制机器人
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                // 发送遥测数据包
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            // 如果驱动类型不支持，抛出异常
            throw new RuntimeException();
        }
    }
}
