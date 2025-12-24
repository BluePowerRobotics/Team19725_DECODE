package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
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
public class Auto_BlueSmall_3_3_3 extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    ShooterAction shooterAction;
    Sweeper_PID sweeper;
    Trigger trigger;
    public static double endx = 24;
    public static double endy = -20;
    public static int INTAKE_START_X = 56;
    public static int INTAKE_END_Y = -62;
    public static double front = ChassisController.PARAMS.FrontToCenterInch;
    public static double side = ChassisController.PARAMS.SideToCenterInch;
    public static final Vector2d Small_End = new Vector2d(endx, endy);
    public static final Pose2d START_POSE = new Pose2d(72-front, side-24, 0);
    public static final Vector2d SHOOT_POSE = new Vector2d(60, -12);
    public static final Vector2d INTAKE_START1 = new Vector2d(36, -24);
    public static final Vector2d INTAKE_END1 = new Vector2d(36, INTAKE_END_Y);
    public static final Vector2d INTAKE_START2_1 = new Vector2d(48, -48);
    public static final Vector2d INTAKE_END2_1 = new Vector2d(52.7, -60);
    public static final Vector2d INTAKE_START2_2 = new Vector2d(INTAKE_START_X, -48);
    public static final Vector2d INTAKE_END2_2 = new Vector2d(INTAKE_START_X + 4.7, -60);
    public static double SHOOT_HEADING = 0.4993467217;
    public static double EAT_HEADING1 = -Math.PI / 2;
    public static double EAT_HEADING2 = -1.197494436;
    public static double END_HEADING = Math.PI / 2;
    public static double collectWait = 1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterAction = new ShooterAction(hardwareMap, telemetry);
        sweeper = new Sweeper_PID(hardwareMap, telemetry, "sweeperMotor", false);
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
        Actions.runBlocking(new RaceAction(
                sweeper.SweeperAction(Sweeper_PID.GiveTheArtifactVel),
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_high)
        ));
        trigger.close();
        sweeper.Sweep(0);

        Action intakeAction1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START1, EAT_HEADING1)
                .build();
        Actions.runBlocking(
                intakeAction1
        );

        Action collectAction1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_END1)
                .waitSeconds(collectWait)
                .build();

        Actions.runBlocking(new RaceAction(
                sweeper.SweeperAction(Sweeper_PID.EatVel),
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
        //sleep(500);
        Actions.runBlocking(new SequentialAction(
                shooterAction.SpeedUp(ShooterAction.targetSpeed_high)
        ));

        trigger.open();
        Actions.runBlocking(new RaceAction(
                sweeper.SweeperAction(Sweeper_PID.GiveTheArtifactVel),
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_high)
        ));
        trigger.close();
        sweeper.Sweep(0);

        Action intakeAction2_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START2_1, EAT_HEADING2)
                .build();
        Actions.runBlocking(
                intakeAction2_1
        );

        Action collectAction2_1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_END2_1)
                .waitSeconds(collectWait)
                .build();

        Actions.runBlocking(new RaceAction(
                sweeper.SweeperAction(Sweeper_PID.EatVel),
                collectAction2_1
        ));

        Action intakeAction2_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START2_2, EAT_HEADING2)
                .build();
        Actions.runBlocking(
                intakeAction2_2
        );

        Action collectAction2_2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_END2_2)
                .waitSeconds(collectWait)
                .build();

        Actions.runBlocking(new RaceAction(
                sweeper.SweeperAction(Sweeper_PID.EatVel),
                collectAction2_2
        ));
        sweeper.Sweep(0);

        Action returnToShootAction2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Actions.runBlocking(new SequentialAction(
                returnToShootAction2,
                sweeper.SweeperBack()
        ));
        //sleep(500);
        Actions.runBlocking(new SequentialAction(
                shooterAction.SpeedUp(ShooterAction.targetSpeed_high)
        ));

        trigger.open();
        Actions.runBlocking(new RaceAction(
                sweeper.SweeperAction(Sweeper_PID.GiveTheArtifactVel),
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_high)
        ));
        trigger.close();
        sweeper.Sweep(0);

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