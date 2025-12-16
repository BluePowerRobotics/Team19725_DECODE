package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.KalmanFusionLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive_Kalman;
@TeleOp
@Disabled
public class LocalizationTest_KalmanFusion extends LinearOpMode {
    public static double startX = -64.8;
    public static double startY = 17.6;
    public static double startHeading = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final Pose2d START_POSE = new Pose2d(startX, startY, startHeading);
            MecanumDrive_Kalman drive_kalman = new MecanumDrive_Kalman(hardwareMap, START_POSE);
            waitForStart();

            while (opModeIsActive()) {
                drive_kalman.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive_kalman.updatePoseEstimate();

                Pose2d pose_kalman = drive_kalman.localizer.getPose();
                telemetry.addData("kalmanx", pose_kalman.position.x);
                telemetry.addData("kalmany", pose_kalman.position.y);
                telemetry.addData("kalmanheading (deg)", Math.toDegrees(pose_kalman.heading.toDouble()));


                Pose2d pose_apriltag = drive_kalman.poseData.AprilTagPose;
                telemetry.addData("apriltagx", pose_apriltag.position.x);
                telemetry.addData("apriltagy", pose_apriltag.position.y);
                telemetry.addData("apriltagheading (deg)", Math.toDegrees(pose_apriltag.heading.toDouble()));


                Pose2d pose_wheel = drive_kalman.poseData.PinPointPose;
                telemetry.addData("x", pose_wheel.position.x);
                telemetry.addData("y", pose_wheel.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose_wheel.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose_wheel);
                packet.fieldOverlay().setStroke("#4E9999");
                Drawing.drawRobot(packet.fieldOverlay(), pose_kalman);
                packet.fieldOverlay().setStroke("#19725C");
                Drawing.drawRobot(packet.fieldOverlay(), pose_apriltag);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
    }
}
