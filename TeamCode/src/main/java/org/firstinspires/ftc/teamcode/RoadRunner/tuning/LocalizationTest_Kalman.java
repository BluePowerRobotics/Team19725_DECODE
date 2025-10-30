package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.utility.kalmanfilter.OneDimensionKalmanFilter;
import org.firstinspires.ftc.teamcode.utility.kalmanfilter.PosVelTuple;

@TeleOp
public class LocalizationTest_Kalman extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AprilTagDetector aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.init(hardwareMap);
        OneDimensionKalmanFilter filter_x=new OneDimensionKalmanFilter(0.0, 0.0);
        OneDimensionKalmanFilter filter_y=new OneDimensionKalmanFilter(0.0, 0.0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();


            long last=System.nanoTime();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

                Pose2d apriltag_pose = aprilTagDetector.getPose().pose;

                long n = System.nanoTime();
                double dlt = n-last;
                last=n;


                double wheelPosition_x = pose.position.x;
                double wheelPosition_y = pose.position.y;

                double measuredPosition_x = apriltag_pose.position.x;
                double measuredPosition_y = apriltag_pose.position.y;

                PosVelTuple result_x=filter_x.Update(wheelPosition_x, measuredPosition_x);
                PosVelTuple result_y=filter_y.Update(wheelPosition_y, measuredPosition_y);

                telemetry.addData("wheel_pos_x", wheelPosition_x);
                telemetry.addData("wheel_pos_y", wheelPosition_y);
                telemetry.addData("apriltag_pos_x", measuredPosition_x);
                telemetry.addData("apriltag_pos_y", measuredPosition_y);
                telemetry.addData("result_x", result_x.position);
                telemetry.addData("result_y", result_y.position);

                telemetry.update();
                Pose2d resultPose = new Pose2d(result_x.position, result_y.position, pose.heading.toDouble());
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), resultPose);
                packet.fieldOverlay().setStroke("#BBBBBB");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
