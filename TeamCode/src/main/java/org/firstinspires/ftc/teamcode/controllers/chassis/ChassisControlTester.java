package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "ChassisControlTester", group = "TEST")
public class ChassisControlTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ChassisController chassis = new ChassisController(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //chassis.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y-gamepad2.left_stick_y; // 前后
            double strafe = gamepad1.left_stick_x; // 左右
            double rotate = gamepad1.right_stick_x; // 旋转
            chassis.gamepadInput(strafe, drive, rotate);
            if(gamepad1.xWasReleased()) chassis.exchangeNoHeadMode();
            telemetry.addData("y-power",drive);
            telemetry.addData("x-power",strafe);
            telemetry.addData("r-power",rotate);
            telemetry.addData("NoHeadModeStartError:",chassis.noHeadModeStartError);
            telemetry.addData("Mode",chassis.useNoHeadMode?"NoHead":"Manual");
            telemetry.addData("",chassis.robotPosition.getData().toString());
            telemetry.update();
        }
    }
}
