package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.controllers.Sweeper;

@TeleOp(name = "sweeperTest", group = "TEST")
public class SweeperTest extends LinearOpMode {
    Sweeper sweeper;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sweeper = new Sweeper(hardwareMap);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
//                drive.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                                -gamepad1.left_stick_y,
//                                -gamepad1.left_stick_x
//                        ),
//                        -gamepad1.right_stick_x
//                ));


                if(gamepad1.a){
                    sweeper.Eat();
                }
                if(gamepad1.b){
                    sweeper.GiveArtifact();
                }
                telemetry.addData("sweeperPower", sweeper.getPower());
//                drive.updatePoseEstimate();
//
//                Pose2d pose = drive.localizer.getPose();
//                telemetry.addData("x", pose.position.x);
//                telemetry.addData("y", pose.position.y);
//                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//                telemetry.update();

//                TelemetryPacket packet = new TelemetryPacket();
//                packet.fieldOverlay().setStroke("#3F51B5");
//                Drawing.drawRobot(packet.fieldOverlay(), pose);
//                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {

        } else {
            throw new RuntimeException();
        }
    }
}