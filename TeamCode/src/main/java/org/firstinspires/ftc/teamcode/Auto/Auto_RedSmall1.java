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
import org.firstinspires.ftc.teamcode.controllers.Sweeper;
import org.firstinspires.ftc.teamcode.controllers.Trigger;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;

@Autonomous
@Config
public class Auto_RedSmall1 extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    ShooterAction shooterAction;
    Sweeper sweeper;
    Trigger trigger;
    public static int INTAKE_END_Y = 55;

    public static final Pose2d START_POSE = new Pose2d(64.8, 17.6, Math.PI);
    public static final Vector2d SHOOT_POSE = new Vector2d(60, 12);
    public static final Vector2d INTAKE_START = new Vector2d(36, 24);
    public static final Vector2d INTAKE_END = new Vector2d(36, INTAKE_END_Y);
    public static final double SHOOT_HEADING = -Math.PI / 4;
    public static final double EAT_HEADING = Math.PI / 2;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterAction = new ShooterAction(hardwareMap, telemetry);
        sweeper = new Sweeper(hardwareMap);
        trigger = new Trigger(hardwareMap);
        drive = new MecanumDrive(hardwareMap, START_POSE);

        Action shootPreloadAction = drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Action intakeAction = drive.actionBuilder(new Pose2d(SHOOT_POSE, SHOOT_HEADING))
                .strafeToLinearHeading(INTAKE_START, EAT_HEADING)
                .build();

        Action collectAction = drive.actionBuilder(new Pose2d(INTAKE_START, EAT_HEADING))
                .strafeTo(INTAKE_END)
                .build();

        Action returnToShootAction = drive.actionBuilder(new Pose2d(INTAKE_END, EAT_HEADING))
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        waitForStart();

        if (isStopRequested()) return;

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

        sweeper.Eat();
        Actions.runBlocking(new SequentialAction(
                intakeAction,
                collectAction
        ));
        sweeper.stop();

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
    }
}