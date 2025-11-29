package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.controllers.Sweeper.Sweeper_PID;
import org.firstinspires.ftc.teamcode.controllers.Trigger;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;

import java.io.FileWriter;
import java.io.IOException;

@Autonomous
@Disabled
@Config
public class TEST_ACTIONS extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    Trigger trigger;
    Sweeper_PID sweeper;
    ShooterAction shooterAction;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterAction = new ShooterAction(hardwareMap, telemetry);
        trigger = new Trigger(hardwareMap);
        trigger.close();
        sweeper = new Sweeper_PID(hardwareMap,telemetry,"sweeperMotor", true);
        waitForStart();

        if (isStopRequested()) return;


        telemetry.addData("000",0);
        telemetry.update();
        Actions.runBlocking(sweeper.SweeperBack());
        telemetry.addData("SWEEPER EAT", 1);
        telemetry.update();
//        sweeper.Eat();
        sleep(1000);
//        telemetry.addData("Trigger OPEN", 2);
//        telemetry.update();
//        trigger.open();
//        sleep(1000);
//        telemetry.addData("STOP", 3);
//        telemetry.update();
////        sweeper.stop();
//        trigger.close();
//        sleep(5000);
//        telemetry.addData("Speed", shooterAction.getCurrent_speed1());
//
//        //todo add UP
//        Actions.runBlocking(
//                new SequentialAction(
//                        shooterAction.SpeedUp(ShooterAction.targetSpeed_low)
//                )
//        );
//
//        telemetry.addData("SPEEDUP COMPLETED", 4);
//        telemetry.update();
//        Actions.runBlocking(
//                new SequentialAction(
//                        shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
//                )
//        );


//        //todo add UP
//        new SequentialAction(
//                shooterAction.SpeedUp(ShooterAction.targetSpeed_low),
//                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
//        );
        try (FileWriter writer = new FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(drive.localizer.getPose().position.x + "," + drive.localizer.getPose().position.y + "," + drive.localizer.getPose().heading.toDouble());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
