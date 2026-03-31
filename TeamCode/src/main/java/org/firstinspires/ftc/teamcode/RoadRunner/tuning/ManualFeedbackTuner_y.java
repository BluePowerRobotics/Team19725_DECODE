package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.TwoDeadWheelLocalizer;

/**
 * 手动Y轴反馈调优OpMode，用于手动调整和测试机器人的Y轴控制器参数。
 * 
 * 此OpMode让机器人在Y轴方向上来回移动，
 * 允许用户观察和调整Y轴控制器的性能，
 * 适用于MecanumDrive和TankDrive两种驱动类型。
 */
@Config
public final class ManualFeedbackTuner_y extends LinearOpMode {
    /**
     * 测试移动距离（单位：英寸）
     */
    public static double DISTANCE = 48;
    
    /**
     * 等待时间（单位：秒）
     */
    public static double TIME = 1;

    /**
     * 运行OpMode的主要方法。
     * 
     * 此方法根据配置的驱动类型（MecanumDrive或TankDrive）初始化相应的驱动对象，
     * 检查里程计轮位置是否已设置，然后让机器人在Y轴方向上来回移动，
     * 以便用户观察和调整Y轴控制器的性能。
     * 
     * @throws InterruptedException 如果线程被中断
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // 根据配置的驱动类型创建相应的驱动对象
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            // 创建MecanumDrive实例
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            
            // 检查里程计轮位置是否已设置
            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            
            // 等待开始信号
            waitForStart();

            // 主循环
            while (opModeIsActive()) {
                // 运行横向移动动作
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(0, DISTANCE))  // 横向移动到指定Y坐标
                            .waitSeconds(TIME)                   // 等待指定时间
                            .strafeTo(new Vector2d(0, 0))         // 横向移动回起始位置
                            .waitSeconds(TIME)                   // 等待指定时间
                            .build());
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            // 创建TankDrive实例
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            // 检查里程计轮位置是否已设置
            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            
            // 等待开始信号
            waitForStart();

            // 主循环
            while (opModeIsActive()) {
                // 运行Y轴移动动作
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToY(DISTANCE)  // 移动到指定Y坐标
                            .lineToY(0)         // 移动回起始位置
                            .build());
            }
        } else {
            // 如果驱动类型不支持，抛出异常
            throw new RuntimeException();
        }
    }
}
