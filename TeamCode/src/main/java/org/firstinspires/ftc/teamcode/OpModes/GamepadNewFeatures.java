package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class GamepadNewFeatures extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("test gamepad1 press and release features(所有按键均有此功能，这里只测试肩键和ABXY）", "Press start to begin");
        telemetry.update();
        waitForStart();
        while( opModeIsActive() ) {
            telemetry.addData("Gamepad 1 Left Bumper Pressed", gamepad1.leftBumperWasPressed());
            telemetry.addData("Gamepad 1 Left Bumper Released", gamepad1.leftBumperWasReleased());
            telemetry.addData("Gamepad 1 Left Bumper Status", gamepad1.left_bumper);

            // Add an empty line to seperate the buttons in telemetry
            telemetry.addLine();

            // Add the status of the Gamepad 1 Right Bumper
            telemetry.addData("Gamepad 1 Right Bumper Pressed", gamepad1.rightBumperWasPressed());
            telemetry.addData("Gamepad 1 Right Bumper Released", gamepad1.rightBumperWasReleased());
            telemetry.addData("Gamepad 1 Right Bumper Status", gamepad1.right_bumper);
            // 添加Gamepad 1 A、B、X、Y按键的按下和释放状态
            telemetry.addData("Gamepad 1 A Pressed", gamepad1.aWasPressed());
            telemetry.addData("Gamepad 1 A Released", gamepad1.aWasReleased());
            telemetry.addData("Gamepad 1 A Status", gamepad1.a);

            telemetry.addData("Gamepad 1 B Pressed", gamepad1.bWasPressed());
            telemetry.addData("Gamepad 1 B Released", gamepad1.bWasReleased());
            telemetry.addData("Gamepad 1 B Status", gamepad1.b);

            telemetry.addData("Gamepad 1 X Pressed", gamepad1.xWasPressed());
            telemetry.addData("Gamepad 1 X Released", gamepad1.xWasReleased());
            telemetry.addData("Gamepad 1 X Status", gamepad1.x);

            telemetry.addData("Gamepad 1 Y Pressed", gamepad1.yWasPressed());
            telemetry.addData("Gamepad 1 Y Released", gamepad1.yWasReleased());
            telemetry.addData("Gamepad 1 Y Status", gamepad1.y);

            // Add a note that the telemetry is only updated every 2 seconds
            telemetry.addLine("\nTelemetry FPS = 20");

            // Update the telemetry on the DS screen
            
            telemetry.update();
            sleep(50);
        }
    }
}
