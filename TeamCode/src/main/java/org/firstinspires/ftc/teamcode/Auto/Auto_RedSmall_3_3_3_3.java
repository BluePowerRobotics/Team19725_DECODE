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
import org.firstinspires.ftc.teamcode.controllers.chassis.ChassisController;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;

import java.io.FileWriter;
import java.io.IOException;

@Autonomous
@Config
public class Auto_RedSmall_3_3_3_3 extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    ShooterAction shooterAction;
    Sweeper_PID sweeper;
    Trigger trigger;
    public static double endx = 24;
    public static double endy = 20;
    public static int INTAKE_END_Y = 62;
    public static double front = ChassisController.PARAMS.FrontToCenterInch;
    public static double back = ChassisController.PARAMS.BackToCenterInch;
    public static double side = ChassisController.PARAMS.SideToCenterInch;
    public static final Vector2d Small_End = new Vector2d(endx, endy);
    public static final Pose2d START_POSE = new Pose2d(72-front, 24-side, 0);
    public static final Vector2d SHOOT_POSE = new Vector2d(60, 12);
    public static final Vector2d INTAKE_START1 = new Vector2d(36, 72-side);
    public static final Vector2d INTAKE_END1 = new Vector2d(72-front, 72-side);
    public static final Vector2d INTAKE_START2 = new Vector2d(36, 24);
    public static final Vector2d INTAKE_END2 = new Vector2d(36, INTAKE_END_Y);
    public static final Vector2d INTAKE_START3 = new Vector2d(12, 24);
    public static final Vector2d INTAKE_END3 = new Vector2d(12, INTAKE_END_Y);
    public static double SHOOT_HEADING = -0.4993467217;
    public static double EAT_HEADING = Math.PI / 2;
    public static double END_HEADING = -Math.PI / 2;
    public static double collectWait = 1;

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
                shooterAction.SpeedUp(ShooterAction.targetSpeed_high)
        ));

        trigger.open();
        sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
        Actions.runBlocking(new SequentialAction(
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_high)
        ));
        trigger.close();

        Action intakeAction1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START1, 0)
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

        Action returnToShootAction1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_START1)
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                returnToShootAction1,
                sweeper.SweeperBack()
        ));
        sleep(500);
        Actions.runBlocking(new SequentialAction(
                shooterAction.SpeedUp(ShooterAction.targetSpeed_high)
        ));

        trigger.open();
        sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
        Actions.runBlocking(new SequentialAction(
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_high)
        ));
        trigger.close();

        Action intakeAction2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START2, EAT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                intakeAction2
        ));

        Action collectAction2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_END2)
                .waitSeconds(collectWait)
                .build();

        sweeper.Sweep(Sweeper_PID.EatVel);
        Actions.runBlocking(new SequentialAction(
                collectAction2
        ));
        sweeper.Sweep(0);

        Action returnToShootAction2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_START2)
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                returnToShootAction2,
                sweeper.SweeperBack()
        ));
        sleep(500);
        Actions.runBlocking(new SequentialAction(
                shooterAction.SpeedUp(ShooterAction.targetSpeed_high)
        ));

        trigger.open();
        sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
        Actions.runBlocking(new SequentialAction(
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_high)
        ));
        trigger.close();

        Action intakeAction3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START3, EAT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                intakeAction3
        ));

        Action collectAction3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_END3)
                .waitSeconds(collectWait)
                .build();

        sweeper.Sweep(Sweeper_PID.EatVel);
        Actions.runBlocking(new SequentialAction(
                collectAction3
        ));
        sweeper.Sweep(0);

        Action returnToShootAction3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_START3)
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                returnToShootAction3,
                sweeper.SweeperBack()
        ));
        sleep(500);
        Actions.runBlocking(new SequentialAction(
                shooterAction.SpeedUp(ShooterAction.targetSpeed_high)
        ));

        trigger.open();
        sweeper.Sweep(Sweeper_PID.GiveTheArtifactVel);
        Actions.runBlocking(new SequentialAction(
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_high)
        ));
        trigger.close();

        Action endAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(Small_End, END_HEADING)
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