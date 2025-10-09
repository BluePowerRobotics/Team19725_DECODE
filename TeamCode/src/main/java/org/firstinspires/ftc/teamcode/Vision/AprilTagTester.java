package org.firstinspires.ftc.teamcode.Vision;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;

@TeleOp
public class AprilTagTester extends LinearOpMode {
    AprilTagDetector aprilTagDetector = new AprilTagDetector();
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        double t = System.currentTimeMillis();
        aprilTagDetector.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("MOTIF", aprilTagDetector.decodeAprilTag());
            if (aprilTagDetector.getPose().id!=-1) {
                telemetry.addData("getPose", aprilTagDetector.getPose().pose.toString());
                telemetry.addData("getpitch", aprilTagDetector.getPose().pitchToCameraInDEG);
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), aprilTagDetector.getPose().pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.addData("getrange", aprilTagDetector.getPose().distanceToCameraInINCH);
            }
            telemetry.addData("FPS", 1000 / (System.currentTimeMillis() - t));
            t = System.currentTimeMillis();
            telemetry.update();

        }
    }

}
