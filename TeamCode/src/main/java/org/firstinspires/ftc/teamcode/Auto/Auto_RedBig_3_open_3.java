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
public class Auto_RedBig_3_open_3 extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    ShooterAction shooterAction;
    Sweeper_PID sweeper;
    Trigger trigger;
    public static int INTAKE_END_Y = 52;
    public static int OPEN_GATE_Y = 60;

    public static double endx = 0;
    public static double endy = 20;
    public static final Vector2d Big_End = new Vector2d(endx, endy);

    public static final Pose2d START_POSE = new Pose2d(-64.8, 17.6, 0);
    public static final Vector2d SHOOT_POSE = new Vector2d(-24, 24);
    public static final Vector2d INTAKE_START = new Vector2d(-12, 24);
    public static final Vector2d INTAKE_END = new Vector2d(-12, INTAKE_END_Y);
    public static final Vector2d OPEN_START = new Vector2d(-2, 48);
    public static final Vector2d OPEN_END = new Vector2d(-2, OPEN_GATE_Y);
    public static double SHOOT_HEADING = -Math.PI / 4;
    public static double EAT_HEADING = Math.PI / 2;
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

        Action intakeAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START, EAT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                intakeAction
        ));

        Action collectAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_END)
                .waitSeconds(collectWait)
                .build();

        sweeper.Sweep(Sweeper_PID.EatVel);
        Actions.runBlocking(new SequentialAction(
                collectAction
        ));
        sweeper.Sweep(0);

        Action openGateAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(OPEN_START)
                .strafeTo(OPEN_END)
                .waitSeconds(openGateWait)
                .strafeTo(OPEN_START)
                .build();

        Actions.runBlocking(new SequentialAction(
                openGateAction
        ));

        Action returnToShootAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                returnToShootAction,
                sweeper.SweeperBack()
        ));
        sleep(500);
        Actions.runBlocking(new SequentialAction(
                shooterAction.SpeedUp(ShooterAction.targetSpeed_low)
        ));

        trigger.open();
        sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
        Actions.runBlocking(new SequentialAction(
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
        ));
        trigger.close();


        Action endAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(Big_End)
                .build();
        Actions.runBlocking(
                endAction
        );

        try (FileWriter writer = new FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(drive.localizer.getPose().position.x + "," + drive.localizer.getPose().position.y + "," + drive.localizer.getPose().heading.toDouble());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}