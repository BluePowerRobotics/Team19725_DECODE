package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
public class Auto_RedBig_3_3 extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    ShooterAction shooterAction;
    Sweeper_PID sweeper;
    Trigger trigger;
    public static double endx = 0;
    public static double endy = 40;
    public static int INTAKE_END_Y = 52;
    public static double front = ChassisController.PARAMS.FrontToCenterInch;
    public static double side = ChassisController.PARAMS.SideToCenterInch;
    public static final Vector2d Big_End = new Vector2d(endx, endy);
    public static final Pose2d START_POSE = new Pose2d(front-71.6, 48-side, 0);
    public static final Vector2d SHOOT_POSE = new Vector2d(-24, 24);
    public static final Vector2d INTAKE_START = new Vector2d(-12, 24);
    public static final Vector2d INTAKE_END = new Vector2d(-12, INTAKE_END_Y);
    public static double SHOOT_HEADING = -Math.PI / 4;
    public static double EAT_HEADING = Math.PI / 2;
    public static double END_HEADING = -Math.PI / 2;
    public static double CollectWait = 1;

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
        Actions.runBlocking(new RaceAction(
                sweeper.SweeperAction(Sweeper_PID.GiveTheArtifactVel),
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
        ));
        trigger.close();
        sweeper.Sweep(0);

        Action intakeAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(INTAKE_START, EAT_HEADING)
                .build();
        Actions.runBlocking(
                intakeAction
        );

        Action collectAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(INTAKE_END)
                .waitSeconds(CollectWait)
                .build();

        Actions.runBlocking(new RaceAction(
                sweeper.SweeperAction(Sweeper_PID.EatVel),
                collectAction
        ));
        sweeper.Sweep(0);

        Action returnToShootAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(SHOOT_POSE, SHOOT_HEADING)
                .build();

        Actions.runBlocking(new ParallelAction(
                returnToShootAction,
                sweeper.SweeperBack()
        ));
        //sleep(500);
        Actions.runBlocking(new SequentialAction(
                shooterAction.SpeedUp(ShooterAction.targetSpeed_low)
        ));

        trigger.open();
        Actions.runBlocking(new RaceAction(
                sweeper.SweeperAction(Sweeper_PID.GiveTheArtifactVel),
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
        ));
        trigger.close();
        sweeper.Sweep(0);

        Action endAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(Big_End, END_HEADING)
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