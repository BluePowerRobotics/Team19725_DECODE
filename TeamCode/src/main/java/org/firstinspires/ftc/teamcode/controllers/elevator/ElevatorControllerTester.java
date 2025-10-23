package org.firstinspires.ftc.teamcode.controllers.elevator;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "ElevatorControllerTester", group = "TEST")
public class ElevatorControllerTester extends LinearOpMode {
    public static double BalancePower=0;
    boolean directControl=false;
    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorController elevatorController = new ElevatorController(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.right_stick_y!=0) {
                directControl=true;
                elevatorController.setPower(BalancePower-gamepad1.right_stick_y);
            }else{
                if(directControl){
                    directControl=false;
                    elevatorController.setPosition(elevatorController.elevatorMotor.getCurrentPosition());
                }
            }
            telemetry.addData("gamePad1:RY",gamepad1.right_stick_y);
            telemetry.addData("directControl",directControl);
            telemetry.addData("BalancePower",BalancePower);
            telemetry.update();
        }
    }
}
