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
import org.firstinspires.ftc.teamcode.controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.controllers.Trigger;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;

import java.io.FileWriter;
import java.io.IOException;

@Autonomous
@Config
public class Auto_RedBig1 extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    ShooterAction shooterAction;
    Sweeper sweeper;
    Trigger trigger;
    public static int INTAKE_END_Y = 54;

    public static final Pose2d START_POSE = new Pose2d(-64.8, 17.6, 0);
    public static final Vector2d SHOOT_POSE = new Vector2d(-24, 24);
    public static final Vector2d INTAKE_START = new Vector2d(-12, 24);
    public static final Vector2d INTAKE_END = new Vector2d(-12, INTAKE_END_Y);
    public static double SHOOT_HEADING = -Math.PI / 4;
    public static double EAT_HEADING = Math.PI / 2;
    public static double waitSeconds = 1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterAction = new ShooterAction(hardwareMap, telemetry);
        sweeper = new Sweeper(hardwareMap);
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
        sweeper.GiveArtifact();
        Actions.runBlocking(new SequentialAction(
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
        ));
        trigger.close();

        Action intakeAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START, EAT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                intakeAction
        ));

        Action collectAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_END)
                .waitSeconds(waitSeconds)
                .build();

        sweeper.Eat();
        Actions.runBlocking(new SequentialAction(
                collectAction
        ));
        sweeper.stop();

        Action returnToShootAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                returnToShootAction,
                shooterAction.SpeedUp(ShooterAction.targetSpeed_low)
        ));

        trigger.open();
        sweeper.GiveArtifact();
        Actions.runBlocking(new SequentialAction(
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
        ));
        trigger.close();

        try (FileWriter writer = new FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(drive.localizer.getPose().position.x + "," + drive.localizer.getPose().position.y + "," + drive.localizer.getPose().heading.toDouble());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}