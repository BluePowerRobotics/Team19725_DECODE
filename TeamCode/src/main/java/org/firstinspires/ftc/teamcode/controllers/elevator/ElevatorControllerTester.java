package org.firstinspires.ftc.teamcode.controllers.elevator;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "ElevatorControllerTester", group = "TEST")
public class ElevatorControllerTester extends LinearOpMode {
    public static double BalancePower=0;
    boolean directControl=false;
    enum TestMode{
        ControllerTest,
        FunctionTest;
        // 缓存枚举数组以避免重复调用values()
        private static final TestMode[] VALUES = values();

        public TestMode next() {
            // 计算下一个位置（循环）
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case ControllerTest:
                    return "Controller Test";
                case FunctionTest:
                    return "Function Test";
                default:
                    return super.toString();
            }
        }
    }
    int CP=0;
    TestMode testMode=TestMode.ControllerTest;
    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorController elevatorController = new ElevatorController(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.aWasReleased()){
                testMode=testMode.next();
            }
            switch (testMode) {
                case FunctionTest:
                    elevatorController.elevatorMotor.setPower(-gamepad1.right_stick_y);
                    break;
                case ControllerTest:
                    if (gamepad1.right_stick_y != 0) {
                        directControl = true;
                        elevatorController.setPower(BalancePower - gamepad1.right_stick_y);
                    } else {
                        if (directControl) {
                            directControl = false;
                            CP = elevatorController.elevatorMotor.getCurrentPosition();
                        }
                        elevatorController.setPosition(CP);
                    }
                    break;
            }
            telemetry.addData("gamePad1:RY",gamepad1.right_stick_y);
            telemetry.addData("true_pos",elevatorController.elevatorMotor.getCurrentPosition());
            telemetry.addData("CurrentPosition",CP);
            telemetry.addData("TestMode",testMode.toString());
            telemetry.addData("directControl",directControl);
            telemetry.addData("BalancePower",BalancePower);
            telemetry.update();
        }
    }
}
