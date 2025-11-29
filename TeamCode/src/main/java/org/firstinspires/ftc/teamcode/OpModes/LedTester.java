package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.controllers.BlinkinLedController;
import org.firstinspires.ftc.teamcode.controllers.LedPreset;

@TeleOp(name = "LED Controller Tester")
@Disabled
public class LedTester extends OpMode {

    private BlinkinLedController ledController;

    @Override
    public void init() {
        RevBlinkinLedDriver blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        ledController = new BlinkinLedController(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (gamepad1.b) {
            ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (gamepad1.y) {
            ledController.setPreset(LedPreset.GREEN);
        } else if (gamepad1.x) {
            ledController.setPreset(LedPreset.ORANGE);
        }


        if (gamepad1.leftBumperWasPressed()) {
            ledController.setPreviousPreset();
            //colornumber=colornumber-1;
        } else if  (gamepad1.rightBumperWasPressed()) {
            ledController.setNextPreset();
            //colornumber=colornumber+1;
        }




        telemetry.addData("LED Status", ledController.getCurrentPattern().toString());
//        telemetry.addData("colornumber",colornumber);
        telemetry.update();
    }
}