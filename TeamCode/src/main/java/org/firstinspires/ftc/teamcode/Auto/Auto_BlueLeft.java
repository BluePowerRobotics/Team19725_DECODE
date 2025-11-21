package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.controllers.shooter.ShooterAction;

import java.io.FileWriter;
import java.io.IOException;

@Autonomous
@Config
public class Auto_BlueLeft  extends LinearOpMode {
    public Pose2d FinalPose;
    MecanumDrive drive;
    AprilTagDetector aprilTagDetector;
    ShooterAction shooterAction;
    public static final Pose2d START_POSE = new Pose2d(31, -60, 0);
    public static final Vector2d SCAN_POSE = new Vector2d(24, -24);
    public static final double ShootHeading = Math.PI / 4;
    public static final Vector2d SHOOT_POSE = new Vector2d(24, -24);
    public static final double IntakeHeading = Math.PI;
    public static final Vector2d INTAKE_POSE_PPG = new Vector2d(24, -12);
    public static final Vector2d INTAKEFINISH_POSE_PPG = new Vector2d(60, -12);
    public static final Vector2d INTAKE_POSE_PGP = new Vector2d(24, 12);
    public static final Vector2d INTAKEFINISH_POSE_PGP = new Vector2d(60, 12);
    public static final Vector2d INTAKE_POSE_GPP = new Vector2d(24, 36);
    public static final Vector2d INTAKEFINISH_POSE_GPP = new Vector2d(60, 36);

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterAction = new ShooterAction(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, START_POSE);
        TrajectoryActionBuilder scan_builder = drive.actionBuilder(START_POSE)
                .strafeTo(SCAN_POSE);
        Action scan_action = scan_builder.build();

        TrajectoryActionBuilder shoot1_builder = drive.actionBuilder(START_POSE)
                .turnTo(ShootHeading);
        Action shoot1_action = shoot1_builder.build();

        TrajectoryActionBuilder intake_PPG_builder = shoot1_builder.endTrajectory().fresh()
                .turnTo(IntakeHeading)
                .strafeTo(INTAKE_POSE_PPG);
        Action intake_PPG_action = intake_PPG_builder.build();
        TrajectoryActionBuilder intakefinish_PPG_builder = intake_PPG_builder.endTrajectory().fresh()
                .strafeTo(INTAKEFINISH_POSE_PPG);
        Action intakefinish_PPG_action = intakefinish_PPG_builder.build();

        TrajectoryActionBuilder intake_PGP_builder = shoot1_builder.endTrajectory().fresh()
                .turnTo(IntakeHeading)
                .strafeTo(INTAKE_POSE_PGP);
        Action intake_PGP_action = intake_PGP_builder.build();
        TrajectoryActionBuilder intakefinish_PGP_builder = intake_PGP_builder.endTrajectory().fresh()
                .strafeTo(INTAKEFINISH_POSE_PGP);
        Action intakefinish_PGP_action = intakefinish_PGP_builder.build();

        TrajectoryActionBuilder intake_GPP_builder = shoot1_builder.endTrajectory().fresh()
                .turnTo(IntakeHeading)
                .strafeTo(INTAKE_POSE_GPP);
        Action intake_GPP_action = intake_GPP_builder.build();
        TrajectoryActionBuilder intakefinish_GPP_builder = intake_GPP_builder.endTrajectory().fresh()
                .strafeTo(INTAKEFINISH_POSE_GPP);
        Action intakefinish_GPP_action = intakefinish_GPP_builder.build();

        TrajectoryActionBuilder shoot_builder_PPG = intakefinish_PPG_builder.endTrajectory().fresh()
                .strafeToLinearHeading(SHOOT_POSE, ShootHeading);
        Action shoot_action_PPG = shoot_builder_PPG.build();
        TrajectoryActionBuilder shoot_builder_PGP = intakefinish_PGP_builder.endTrajectory().fresh()
                .strafeToLinearHeading(SHOOT_POSE, ShootHeading);
        Action shoot_action_PGP = shoot_builder_PGP.build();
        TrajectoryActionBuilder shoot_builder_GPP = intakefinish_GPP_builder.endTrajectory().fresh()
                .strafeToLinearHeading(SHOOT_POSE, ShootHeading);
        Action shoot_action_GPP = shoot_builder_GPP.build();


        aprilTagDetector.init(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        new SequentialAction(scan_action);
        AprilTagDetector.MOTIFTYPE motif = aprilTagDetector.decodeAprilTag();
        telemetry.addData("Detected motif", motif);
        telemetry.update();
        new SequentialAction(shoot1_action);
        //todo add UP
        new SequentialAction(
                shooterAction.SpeedUp(ShooterAction.targetSpeed_low),
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
        );
        if(motif == AprilTagDetector.MOTIFTYPE.PPG){
            new SequentialAction(intake_PPG_action, intakefinish_PPG_action, shoot_action_PPG);
        }else if(motif == AprilTagDetector.MOTIFTYPE.PGP){
            new SequentialAction(intake_PGP_action, intakefinish_PGP_action, shoot_action_PGP);
        }else{ //GPP
            new SequentialAction(intake_GPP_action, intakefinish_GPP_action, shoot_action_GPP);
        }
        //todo add UP
        new SequentialAction(
                shooterAction.SpeedUp(ShooterAction.targetSpeed_low),
                shooterAction.ShootThreeArtifacts(ShooterAction.targetSpeed_low)
        );
        try (FileWriter writer = new FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(drive.localizer.getPose().position.x + "," + drive.localizer.getPose().position.y + "," + drive.localizer.getPose().heading.toDouble());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
