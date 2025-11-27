package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.controllers.Sweeper.Sweeper_PID;
import org.firstinspires.ftc.teamcode.controllers.Trigger;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;

import java.io.FileWriter;
import java.io.IOException;

@Autonomous
@Config
public class Auto_BlueBig_3_open_3 extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    ShooterAction shooterAction;
    Sweeper_PID sweeper;
    Trigger trigger;
    public static int INTAKE_END_Y1 = -54;
    public static int INTAKE_END_Y2 = -62;
    public static int OPEN_GATE_Y = -60;

    public static final Pose2d START_POSE = new Pose2d(-64.8, -17.6, 0);
    public static final Vector2d SHOOT_POSE = new Vector2d(-24, -24);
    public static final Vector2d INTAKE_START1 = new Vector2d(-12, -24);
    public static final Vector2d INTAKE_END1 = new Vector2d(-12, INTAKE_END_Y1);
    public static final Vector2d OPEN_START = new Vector2d(0, -48);
    public static final Vector2d OPEN_END = new Vector2d(0, OPEN_GATE_Y);
    public static final Vector2d INTAKE_START2 = new Vector2d(12, -24);
    public static final Vector2d INTAKE_END2 = new Vector2d(12, INTAKE_END_Y2);
    public static double SHOOT_HEADING = Math.PI / 4;
    public static double EAT_HEADING = -Math.PI / 2;
    public static double collectWait = 1;
    public static double openGateWait = 1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterAction = new ShooterAction(hardwareMap, telemetry);
        sweeper = new Sweeper_PID(hardwareMap, telemetry, "sweeperMotor", true);
        trigger = new Trigger(hardwareMap);
        drive = new MecanumDrive(hardwareMap, START_POSE);

        waitForStart();

        if (isStopRequested()) return;

        Action shootPreloadAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                shootPreloadAction,
                shooterAction.SpeedUp(ShooterAction.targetSpeed_low)
        ));

        trigger.open();
        sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
        Actions.runBlocking(new SequentialAction(
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
        ));
        trigger.close();

        Action intakeAction1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START1, EAT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                intakeAction1
        ));

        Action collectAction1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_END1)
                .waitSeconds(collectWait)
                .build();

        sweeper.Sweep(Sweeper_PID.EatVel);
        Actions.runBlocking(new SequentialAction(
                collectAction1
        ));
        sweeper.Sweep(0);
        sweeper.SweeperBack();

        Action openGateAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(OPEN_START)
                .strafeTo(OPEN_END)
                .waitSeconds(openGateWait)
                .strafeTo(OPEN_START)
                .build();

        Actions.runBlocking(new SequentialAction(
                openGateAction
        ));

        Action returnToShootAction1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                returnToShootAction1,
                shooterAction.SpeedUp(ShooterAction.targetSpeed_low)
        ));

        trigger.open();
        sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
        Actions.runBlocking(new SequentialAction(
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
        ));
        trigger.close();

//        Action intakeAction2 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(INTAKE_START2, EAT_HEADING)
//                .build();
//
//        Actions.runBlocking(new SequentialAction(
//                intakeAction2
//        ));
//
//        Action collectAction2 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeTo(INTAKE_END2)
//                .waitSeconds(collectWait)
//                .build();
//
//        sweeper.Eat();
//        Actions.runBlocking(new SequentialAction(
//                collectAction2
//        ));
//        sweeper.stop();
//
//        Action returnToShootAction2 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeTo(INTAKE_START2)
//                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
//                .build();
//
//        Actions.runBlocking(new SequentialAction(
//                returnToShootAction2,
//                shooterAction.SpeedUp(ShooterAction.targetSpeed_low)
//        ));
//
//        trigger.open();
//        sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
//        Actions.runBlocking(new SequentialAction(
//                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
//        ));
//        trigger.close();

        try (FileWriter writer = new FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(drive.localizer.getPose().position.x + "," + drive.localizer.getPose().position.y + "," + drive.localizer.getPose().heading.toDouble());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}