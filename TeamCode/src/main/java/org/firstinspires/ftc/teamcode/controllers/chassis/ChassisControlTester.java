package org.firstinspires.ftc.teamcode.controllers.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "ChassisControlTester", group = "Test")
public class ChassisControlTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //ChassisController chassis = ChassisController.getInstance();
        //chassis.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y; // 前后
            double strafe = gamepad1.left_stick_x; // 左右
            double rotate = gamepad1.right_stick_x; // 旋转


        }
    }
}
