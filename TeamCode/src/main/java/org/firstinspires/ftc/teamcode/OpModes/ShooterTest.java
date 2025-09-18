package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.controllers.Shooter;


@TeleOp
@Config
public class ShooterTest extends LinearOpMode {
    public Shooter shooter;
    public static double initSpeed = 300;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap, telemetry);
        waitForStart();
        double targetSpeed = 0;
        while (opModeIsActive()) {
            if(gamepad1.a){
                targetSpeed = initSpeed;
            }
            if(gamepad1.b){
                targetSpeed = 0;
            }
            if(gamepad1.xWasPressed()){
                targetSpeed -= 50;
            }
            if(gamepad1.yWasPressed()){
                targetSpeed += 50;
            }
            shooter.shoot(targetSpeed);
            telemetry.update();
        }
    }
}
