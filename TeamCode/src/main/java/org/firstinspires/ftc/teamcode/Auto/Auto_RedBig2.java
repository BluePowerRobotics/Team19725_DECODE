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
public class Auto_RedBig2 extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    ShooterAction shooterAction;
    Sweeper sweeper;
    Trigger trigger;
    public static int INTAKE_END_Y = 55;
    public static int OPEN_GATE_Y = 60;

    public static final Pose2d START_POSE = new Pose2d(-64.8, 17.6, 0);
    public static final Vector2d SHOOT_POSE = new Vector2d(-24, 24);
    public static final Vector2d INTAKE_START1 = new Vector2d(-12, 24);
    public static final Vector2d INTAKE_END1 = new Vector2d(-12, INTAKE_END_Y);
    public static final Vector2d INTAKE_START2 = new Vector2d(12, 24);
    public static final Vector2d INTAKE_END2 = new Vector2d(12, INTAKE_END_Y);
    public static final Vector2d OPEN_GATE = new Vector2d(0, OPEN_GATE_Y);
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

        Action openGateAction = drive.actionBuilder(new Pose2d(SHOOT_POSE, SHOOT_HEADING))
                .strafeTo(OPEN_GATE)
                .build();

        Action intakeAction1 = drive.actionBuilder(new Pose2d(SHOOT_POSE, SHOOT_HEADING))
                .strafeToLinearHeading(INTAKE_START1, EAT_HEADING)
                .build();

        Action collectAction1 = drive.actionBuilder(new Pose2d(INTAKE_START1, EAT_HEADING))
                .strafeTo(INTAKE_END1)
                .build();

        Action returnToShootAction1 = drive.actionBuilder(new Pose2d(INTAKE_END1, EAT_HEADING))
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Action intakeAction2 = drive.actionBuilder(new Pose2d(SHOOT_POSE, SHOOT_HEADING))
                .strafeToLinearHeading(INTAKE_START2, EAT_HEADING)
                .build();

        Action collectAction2 = drive.actionBuilder(new Pose2d(INTAKE_START2, EAT_HEADING))
                .strafeTo(INTAKE_END2)
                .build();

        Action returnToShootAction2 = drive.actionBuilder(new Pose2d(INTAKE_END2, EAT_HEADING))
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

        Actions.runBlocking(new SequentialAction(
                openGateAction
        ));

        sweeper.Eat();
        Actions.runBlocking(new SequentialAction(
                intakeAction1,
                collectAction1
        ));
        sweeper.stop();

        Actions.runBlocking(new SequentialAction(
                returnToShootAction1,
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
                intakeAction2,
                collectAction2
        ));
        sweeper.stop();

        Actions.runBlocking(new SequentialAction(
                returnToShootAction2,
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