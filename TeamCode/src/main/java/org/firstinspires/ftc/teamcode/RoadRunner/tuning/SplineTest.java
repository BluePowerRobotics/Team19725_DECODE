package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;

/**
 * 样条测试OpMode，用于测试和验证机器人的样条轨迹跟踪能力。
 * 
 * 此OpMode让机器人按照预定义的样条轨迹移动，
 * 测试机器人是否能够准确跟踪曲线轨迹，
 * 适用于MecanumDrive和TankDrive两种驱动类型。
 */
public final class SplineTest extends LinearOpMode {
    /**
     * 运行OpMode的主要方法。
     * 
     * 此方法根据配置的驱动类型（MecanumDrive或TankDrive）初始化相应的驱动对象，
     * 然后让机器人按照预定义的样条轨迹移动，
     * 测试机器人的轨迹跟踪能力。
     * 
     * @throws InterruptedException 如果线程被中断
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // 设置起始姿态
        Pose2d beginPose = new Pose2d(0, 0, 0);
        
        // 根据配置的驱动类型创建相应的驱动对象
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            // 创建MecanumDrive实例
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            // 等待开始信号
            waitForStart();

            // 运行样条轨迹
            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(24, 24), Math.PI / 2)  // 样条移动到(24, 24)，朝向π/2
                        .splineTo(new Vector2d(0, 48), Math.PI)        // 样条移动到(0, 48)，朝向π
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            // 创建TankDrive实例
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            // 等待开始信号
            waitForStart();

            // 运行样条轨迹
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)  // 样条移动到(30, 30)，朝向π/2
                            .splineTo(new Vector2d(0, 60), Math.PI)        // 样条移动到(0, 60)，朝向π
                            .build());
        } else {
            // 如果驱动类型不支持，抛出异常
            throw new RuntimeException();
        }
    }
}
