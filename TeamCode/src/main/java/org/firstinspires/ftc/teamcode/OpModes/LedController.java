package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.utility.BlinkinLedController;

@TeleOp(name = "LED Controller Example")
public class LedController extends OpMode {

    private BlinkinLedController ledController;

    @Override
    public void init() {
        RevBlinkinLedDriver blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        ledController = new BlinkinLedController(blinkin);

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
            ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else if (gamepad1.x) {
            ledController.turnOff();
        }
        //这里，打，什么，数字，对应，哪一个pattern。群里有表自己找。

        if (gamepad1.leftBumperWasPressed()) {
            ledController.setColorprevious();
            //colornumber=colornumber-1;
        } else if  (gamepad1.rightBumperWasPressed()) {
            ledController.setColornext();
            //colornumber=colornumber+1;
        }




        telemetry.addData("LED Status", ledController.getCurrentPattern().toString());
//        telemetry.addData("colornumber",colornumber);
        telemetry.update();
    }
}