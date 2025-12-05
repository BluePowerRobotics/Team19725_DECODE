package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@Config
@TeleOp(name = "ChassisControlTester", group = "TEST")
public class ChassisControlTester extends LinearOpMode {
    long lastNanoTime=0;
    @Override
    public void runOpMode() throws InterruptedException {
        ChassisController chassis = new ChassisController(hardwareMap);
        chassis.robotPosition.setMinUpdateIntervalMs(1);//todo 测试更小的时间间隔是否能带来更好的效果
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.init(telemetry);
        //chassis.init(hardwareMap);
        waitForStart();
        lastNanoTime=System.nanoTime();
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y-gamepad1.right_stick_y; // 前后
            double strafe = gamepad1.left_stick_x; // 左右
            double rotate =-gamepad1.right_stick_x; // 旋转——正为逆时针旋转
            chassis.gamepadInput(strafe, drive, rotate);
            if(gamepad1.xWasReleased()) chassis.exchangeNoHeadMode();
            if(gamepad1.yWasReleased()) {
                chassis.setTargetPoint(new Pose2d(new Vector2d(0,0),Rotation2d.fromDouble(0)));
            }
            telemetry.addData("FPS",1000000000.0/(System.nanoTime()-lastNanoTime));
            telemetry.addData("y-power",drive);
            telemetry.addData("x-power",strafe);
            telemetry.addData("r-power",rotate);
            telemetry.addData("NoHeadModeStartError:",chassis.noHeadModeStartError);
            telemetry.addData("NoHeadMode",chassis.useNoHeadMode?"NoHead":"Manual");
            telemetry.addData("RunMode",chassis.runningToPoint?"RUNNING_TO_POINT":"MANUAL");
            telemetry.addData("Position",chassis.robotPosition.getData().getPosition(DistanceUnit.MM).toString());
            telemetry.update();
            lastNanoTime = System.nanoTime();

            Pose2d pose = chassis.robotPosition.mecanumDrive.localizer.getPose();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}