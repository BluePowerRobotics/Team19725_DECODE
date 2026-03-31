package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

/**
 * 手动反馈调优OpMode，用于手动调整和测试机器人的反馈控制器参数。
 * 
 * 此OpMode让机器人在指定距离上来回移动，
 * 允许用户观察和调整控制器的性能，
 * 适用于MecanumDrive和TankDrive两种驱动类型。
 */
@Config
public final class ManualFeedbackTuner extends LinearOpMode {
    /**
     * 测试移动距离（单位：英寸）
     */
    public static double DISTANCE = 64;
    
    /**
     * 手动控制标志
     */
    boolean mannual = false;
    
    /**
     * 运行OpMode的主要方法。
     * 
     * 此方法根据配置的驱动类型（MecanumDrive或TankDrive）初始化相应的驱动对象，
     * 检查里程计轮位置是否已设置，然后让机器人在指定距离上来回移动，
     * 同时允许用户通过游戏手柄进行手动控制。
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
            
            // 创建动作运行器
            ActionRunner actionRunner = new ActionRunner();
            
            // 主循环
            while (opModeIsActive()) {
                // 如果动作运行器空闲，添加来回移动的动作
                if(!actionRunner.isBusy())
                    actionRunner.add(
                            drive.actionBuilder(new Pose2d(0, 0, 0))
                                    .lineToX(DISTANCE)  // 向前移动指定距离
                                    .lineToX(0)         // 返回到起始位置
                                    .build());
                
                // 如果游戏手柄空闲，更新动作运行器
                if(gamepad1.atRest())
                    actionRunner.update();
                // 否则，手动控制机器人
                else
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,  // 前进/后退
                                    -gamepad1.left_stick_x   // 左右平移
                            ),
                            -gamepad1.right_stick_x  // 旋转
                    ));
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
                // 运行来回移动的动作
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)  // 向前移动指定距离
                            .lineToX(0)         // 返回到起始位置
                            .build());
            }
        } else {
            // 如果驱动类型不支持，抛出异常
            throw new RuntimeException();
        }
    }
}
