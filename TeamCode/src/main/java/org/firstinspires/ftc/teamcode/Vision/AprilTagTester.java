package org.firstinspires.ftc.teamcode.Vision;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
            telemetry.addData("getPose", aprilTagDetector.getPose()[0].pose.toString());
            telemetry.addData("getpitch", aprilTagDetector.getPose()[0].pitchToCameraInDEG);
            telemetry.addData("getrange", aprilTagDetector.getPose()[0].distanceToCameraInINCH);
            telemetry.addData("FPS", 1000 / (System.currentTimeMillis() - t));
            t = System.currentTimeMillis();
            telemetry.update();

        }
    }

}
