package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.controllers.chassis.model.MoveAction;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@Config
@TeleOp(name = "ChassisControlTester", group = "TEST")
public class ChassisControlTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ChassisController chassis = new ChassisController(hardwareMap,new Point2D(0,0),0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //chassis.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y-gamepad2.left_stick_y; // 前后
            double strafe =-gamepad1.left_stick_x; // 左右
            double rotate = gamepad1.right_stick_x; // 旋转
            chassis.gamepadInput(strafe, drive, rotate);
            if(gamepad1.xWasReleased()) chassis.exchangeNoHeadMode();
            if(gamepad1.yWasReleased()) {
                MoveAction builder = new MoveAction.Builder()
                        .setStartPoint(chassis.robotPosition.getData().getPosition(DistanceUnit.MM))
                        .setStartRadian(chassis.robotPosition.getData().headingRadian)
                        .setTargetPoint(new Point2D(0,0))
                        .setTargetRadian(0)
                        .build();
                chassis.setTargetPoint(builder);
            }
            telemetry.addData("y-power",drive);
            telemetry.addData("x-power",strafe);
            telemetry.addData("r-power",rotate);
            telemetry.addData("NoHeadModeStartError:",chassis.noHeadModeStartError);
            telemetry.addData("NoHeadMode",chassis.useNoHeadMode?"NoHead":"Manual");
            telemetry.addData("RunMode",chassis.runningToPoint?"RUNNING_TO_POINT":"MANUAL");
            telemetry.addData("",chassis.robotPosition.getData().toString());
            //telemetry.update();

            //mecanumDrive.updatePoseEstimate();
            Pose2d pose = chassis.robotPosition.mecanumDrive.localizer.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
